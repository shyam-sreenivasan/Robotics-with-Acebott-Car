# Robotics with Acebott Car

Fun robotics projects with the Acebott smart car (ESP32-based).

## Project Structure

```
robot/                  # Arduino firmware
  robot.ino             # main sketch
  motor.h / motor.cpp   # low-level motor driver
  motion.h / motion.cpp # differential drive + smoothing
  comm.h / comm.cpp     # communication (WiFi or Serial)
  secrets.h             # WiFi credentials (gitignored — see below)
  secrets.h.example     # template for secrets.h
keyboard-control.py     # PC control over Serial
keyboard-wifi-control.py# PC control over WiFi
roam.py                 # autonomous roaming with obstacle avoidance
find-robot.py           # scan the LAN for the car's IP
requirements.txt        # Python dependencies
ros2/
  person_follower/      # ROS 2 package: follow a person with the iPhone camera
```

## Firmware Setup

### 1. WiFi credentials

```bash
cp robot/secrets.h.example robot/secrets.h
```

Edit `robot/secrets.h` and fill in your network name and password.

### 2. Choose communication mode

In [robot/comm.h](robot/comm.h), the default is WiFi:

```cpp
#define COMM_WIFI  // comment out to use Serial
```

Comment out that line to switch to Serial.

### 3. Flash the ESP32

Open the `robot/` folder in Arduino IDE and upload to the board. The IP address will print to the Serial Monitor after connecting.

## Python Control

### Install dependencies

```bash
pip install -r requirements.txt
```

### Keyboard control over WiFi

Edit `keyboard-wifi-control.py` and set `ESP_IP` to the IP shown in Serial Monitor, then:

```bash
python keyboard-wifi-control.py
```

### Keyboard control over Serial

```bash
sudo python keyboard-control.py
```

> `sudo` is required on Linux because the `keyboard` library reads raw input events.

### Controls

| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn left |
| `D` | Turn right |

## Person Following (ROS 2)

`ros2/person_follower/` tracks a person in the iPhone camera stream and
drives the car to follow them. Requires ROS 2 Humble and the
[Conduit](https://github.com/) iPhone bridge publishing
`/conduit/camera/front/image_raw/compressed`.

### Pipeline

```
/conduit/camera/... -> person_tracker -> /target_person
                                              |
                                       follow_controller
                                              |
                                       /follow_cmd_vel
                                              |
                                        acebott_bridge --TCP 1234--> car
```

### Build

The package lives in this repo but is built from a colcon workspace, via
a symlink:

```bash
ln -s ~/workspace/Robotics-with-Acebott-Car/ros2/person_follower \
      ~/sensorstream_ws/src/person_follower

cd ~/sensorstream_ws
colcon build --packages-select person_follower
source install/setup.bash
```

Python dependencies (into the same interpreter ROS uses — a venv would
hide `rclpy`):

```bash
pip install --user ultralytics "numpy<2" "setuptools<80" "packaging>=23" lap
pip uninstall -y opencv-python   # conflicts with the ROS-shipped cv2
```

### Run

Every terminal needs both workspaces sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/sensorstream_ws/install/setup.bash
```

**1. Start Conduit.** Open the app on the iPhone and start the
`sensorstream_driver` node on the laptop. Confirm frames are arriving
before going further:

```bash
ros2 topic hz /conduit/camera/front/image_raw/compressed
```

Roughly 30 Hz means the camera path is healthy. If the topic exists but
shows no rate, Conduit is not publishing — see Troubleshooting.

**2. Power the car and find its IP.** DHCP tends to move it:

```bash
./venv/bin/python find-robot.py
```

Or read it from the Arduino Serial Monitor (115200 baud) at boot.

**3. Launch the follower:**

```bash
ros2 launch person_follower follow.launch.py esp_ip:=<car-ip>
```

Step into frame. The log shows a five second countdown, then `Following
now`, and the car starts tracking.

Stop with Ctrl-C — the bridge sends `0,0` on exit, and the firmware also
halts on its own after 300 ms of silence.

### Running nodes individually

Useful for isolating a problem. Each in its own terminal:

```bash
# camera only, no detection — is Conduit working?
ros2 run person_follower camera_viewer

# detection only, no tracking or motion (Stage 1)
ros2 run person_follower person_detector

# tracking with persistent IDs, publishes /target_person (Stage 2)
ros2 run person_follower person_tracker

# turns /target_person into velocity commands, no car needed
ros2 run person_follower follow_controller

# talks to the car; nothing else drives it
ros2 run person_follower acebott_bridge --ros-args -p esp_ip:=<car-ip>
```

The tracker and detector open an OpenCV window: green box is the locked
target, orange boxes are other people. **Q** or **Esc** quits, **T**
forces a re-lock onto the nearest person.

### Testing without the car

The controller can be verified with no hardware at all:

```bash
ros2 run person_follower follow_controller
ros2 topic echo /follow_cmd_vel
```

Walk around in front of the camera and watch the numbers. Person to your
left gives `angular.z > 0`; too far away gives `linear.x > 0`.

You can also replay a recorded session instead of using the live camera:

```bash
ros2 bag play ~/rosbags/<bag-dir>
```

### Recording a run

```bash
ros2 bag record /conduit/camera/front/image_raw/compressed \
                /conduit/camera/front/camera_info \
                /target_person /follow_cmd_vel /cmd_vel
```

### Troubleshooting

| Symptom | Cause |
|---|---|
| `camera_viewer` hangs on "Waiting for frames" | Conduit is not running. `ros2 topic info <topic>` will show `Publisher count: 0`. |
| `Could not reach car ... No route to host` | Car is off, or DHCP moved it. Re-run `find-robot.py`. |
| Car turns away from you instead of toward | `w_scale` sign — see Sign conventions below. |
| Car reverses when it should approach | `target_box_h` is below your actual box height. Echo `/target_person` and raise it. |
| Car drives but never turns, or vice versa | Expected in `pivot` mode: it does one or the other, never both. |
| Tracker keeps re-locking onto new IDs | Detection is dropping out. Try `model:=yolov8s.pt` or a lower `conf`. |

If a node behaves as though your code changes did nothing, check for a
stale process from an earlier run:

```bash
ros2 node list
pkill -f person_tracker
```

### Key parameters

| Argument | Default | Meaning |
|---|---|---|
| `esp_ip` | `10.76.211.120` | Car's IP address — **usually needs overriding** |
| `target_box_h` | `500.0` | Follow distance, as person box height in px. **Higher = follows closer** |
| `start_delay` | `5.0` | Seconds after first detection before moving. `0` disables |
| `drive_mode` | `pivot` | `pivot`, `pid`, or `blended` (see below) |
| `linear_enabled` | `true` | `false` = rotate in place only, no forward motion |

Turning:

| Argument | Default | Meaning |
|---|---|---|
| `turn_speed` | `0.6` | Turn magnitude at `full_turn_deg` of error |
| `min_turn` | `0.35` | Turn magnitude just outside the deadzone. Must clear the firmware's 0.05 deadband |
| `full_turn_deg` | `40.0` | Bearing error at which the turn reaches `turn_speed` |
| `turn_slew` | `1.2` | Max change in turn command per second; lower is smoother |
| `pid_kp` / `pid_ki` / `pid_kd` | `1.1` / `0.15` / `0.35` | Gains, used only when `drive_mode:=pid` |

Tracking:

| Argument | Default | Meaning |
|---|---|---|
| `smoothing` | `0.6` | EMA on bearing and box height. Higher = smoother but laggier |
| `confirm_frames` | `5` | Frames a re-acquired target must persist before the car acts on it |
| `reacquire_secs` | `3.0` | How long to stay locked on a target that has vanished |
| `distance_deadzone_px` | `45.0` | Tolerance around `target_box_h` before moving |

Detector settings are node parameters rather than launch arguments:

```bash
ros2 run person_follower person_tracker --ros-args \
  -p model:=yolov8s.pt -p conf:=0.3
```

To calibrate the follow distance: stand where you want the car to hold
station, run `ros2 topic echo /target_person`, and use the magnitude of
`y` as `target_box_h`.

### Why `pivot` mode

All four motors share a single PWM line (see `motorMove()` in
`robot/motor.cpp`), so the car cannot run one side slower than the other.
A blended `v,w` just drops the weaker side under the firmware deadband and
the car spins instead of curving. `pivot` mode therefore turns in place or
drives straight, never both — the same approach `roam.py` uses.

### Sign conventions

`/cmd_vel` uses standard ROS signs (+x forward, +z left). The car's own
protocol differs, so `acebott_bridge` applies `w_scale = -1.0`: `roam.py`'s
`turn_for()` documents "direction is -1 for left, +1 for right". Forward
(`v`) is **not** inverted. Keep these flips in the bridge only, so every
other node can use normal ROS conventions.
