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
requirements.txt        # Python dependencies
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
