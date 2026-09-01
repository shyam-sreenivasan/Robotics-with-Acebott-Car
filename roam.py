"""Autonomous roaming controller for the Acebott car.

Runs entirely on the PC: reads ultrasonic telemetry over WiFi and streams
back v,w velocity commands.

Behaviour, per decision point:
  - 80% keep going straight
  - 20% turn, to a random heading between 0 and 90 degrees, left or right
  - independently, 10% of turns become a full 360 degree spin

Obstacle avoidance preempts all of the above: when the filtered distance
drops below OBSTACLE_CM the car backs up, scans left and right with the
sensor servo, and commits to whichever side is more open.

Run with:  sudo ./venv/bin/python roam.py
Press Q at any time to stop. Ctrl-C also stops the car safely.
"""

import csv
import random
import socket
import threading
import time
from datetime import datetime

import keyboard

# ---------------- connection ----------------
ESP_IP = "10.0.0.18"   # 🔧 same IP as your other scripts
PORT = 1234

# ---------------- tuning ----------------
# Thresholds picked from the logged sensor data: under 30cm accounted for
# only ~4% of samples while 40cm caught ~20%, so 30 marks a genuine
# obstacle without firing at normal open-room distances.
OBSTACLE_CM = 30.0       # back away below this
CAUTION_CM = 50.0        # slow down below this

CRUISE_SPEED = 0.8       # forward v while roaming
TURN_SPEED = 0.7         # |w| while turning
REVERSE_SPEED = -0.2     # v while backing away from an obstacle

# Open-loop turn calibration: no encoders, so a turn is "hold w for N ms".
# Tune this one number if the car consistently over- or under-rotates.
MS_PER_DEGREE = 7.0

STRAIGHT_MIN_S = 1.5     # how long to hold a straight leg
STRAIGHT_MAX_S = 4.0

# ---------------- drift correction ----------------
# The firmware drives all four motors from a single PWM pin, so a small
# steady w does NOT curve the car -- both sides just get the same duty.
# Correcting drift therefore needs brief pulses of real turning, large
# enough that motion.cpp flips one side out of forward.
#
# DRIFT_TRIM is in "correction pulses per second of straight driving".
# Positive values steer left (use when the car drifts right).
# Set to 0.0 to disable. Tune with:  ./venv/bin/python roam.py --calibrate
DRIFT_TRIM = 1.2
# During a pulse we also drop v. motion.cpp computes one PWM as
# max(|v-w|,|v+w|)*255, so cruising at 0.8 with w=-0.9 would ask for 433 --
# well past the 255 that analogWrite accepts. v=0.4/w=-0.5 rotates the car
# at pwm 229 while keeping some forward motion.
CORRECTION_V = 0.4
CORRECTION_W = -0.5      # negative = left; use when the car drifts right
CORRECTION_PULSE_S = 0.06

REVERSE_S = 0.7          # how long to back up after hitting an obstacle
SERVO_SETTLE_S = 0.35    # let the head stop moving before trusting a reading

# Stay in manual this long after the last keypress before auto resumes.
RESUME_DELAY_S = 1.5

# Servo angles used when scanning for an escape route.
SERVO_LEFT = 150
SERVO_CENTER = 90
SERVO_RIGHT = 30

LOG_PATH = f"roam-log-{datetime.now():%Y%m%d-%H%M%S}.csv"


class Telemetry:
    """Background reader for the firmware's 10Hz telemetry stream."""

    def __init__(self, sock):
        self.sock = sock
        self.latest = None
        self.recent = []          # last few distances, for median filtering
        self.running = True
        self.lock = threading.Lock()
        self.rows = []

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        buf = ""
        self.sock.settimeout(0.5)
        while self.running:
            try:
                chunk = self.sock.recv(1024).decode(errors="replace")
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            *lines, buf = buf.split("\n")
            for line in lines:
                p = line.strip().split(",")
                if len(p) != 9 or p[0] != "T":
                    continue
                try:
                    row = {
                        "host_time": time.time(),
                        "esp_ms": int(p[1]),
                        "distance_cm": float(p[2]),
                        "line_l": int(p[3]),
                        "line_m": int(p[4]),
                        "line_r": int(p[5]),
                        "v": float(p[6]),
                        "w": float(p[7]),
                        "servo_deg": int(p[8]),
                    }
                except ValueError:
                    continue
                with self.lock:
                    self.latest = row
                    self.rows.append(row)
                    # Only real readings feed the filter. A -1 means the echo
                    # timed out, which means nothing is in range -- treating
                    # that as "0cm, obstacle!" would be exactly backwards.
                    if row["distance_cm"] > 0:
                        self.recent.append(row["distance_cm"])
                        del self.recent[:-3]

    def distance(self):
        """Median of the last 3 valid readings, or None if we have none yet.

        The raw stream is mostly stable but throws occasional 40cm+ spikes;
        a 3-sample median rejects those without adding real latency.
        """
        with self.lock:
            if not self.recent:
                return None
            return sorted(self.recent)[len(self.recent) // 2]

    def stop(self):
        self.running = False


def manual_input():
    """Current WASD state, or None if the user isn't touching the keys."""
    v = 0.0
    w = 0.0
    if keyboard.is_pressed("w"):
        v = 1.0
    elif keyboard.is_pressed("s"):
        v = -1.0
    if keyboard.is_pressed("a"):
        w = -1.0
    elif keyboard.is_pressed("d"):
        w = 1.0
    return None if (v == 0.0 and w == 0.0) else (v, w)


class ManualOverride(Exception):
    """Raised inside hold() so the autonomous behaviour unwinds immediately."""


class Roamer:
    def __init__(self, sock, telem):
        self.sock = sock
        self.telem = telem
        self.servo_deg = SERVO_CENTER
        self.stats = {"straight": 0, "turn": 0, "spin": 0,
                      "avoid": 0, "manual": 0}

    # ---------------- low-level output ----------------

    def drive(self, v, w):
        self.sock.send(f"{v},{w}\n".encode())

    def aim(self, deg):
        if deg != self.servo_deg:
            self.sock.send(f"S,{deg}\n".encode())
            self.servo_deg = deg

    def hold(self, v, w, seconds, watch_obstacle=True, correct_drift=True):
        """Drive at (v, w) for a while, refreshing the firmware watchdog.

        The firmware stops the car if it hears nothing for 300ms, so we must
        keep sending. Returns True if it ran the full duration, False if an
        obstacle cut it short.
        """
        end = time.time() + seconds
        next_correction = time.time() + (1.0 / DRIFT_TRIM if DRIFT_TRIM else 1e9)

        while time.time() < end:
            if keyboard.is_pressed("q"):
                raise KeyboardInterrupt
            if manual_input():
                raise ManualOverride

            # Straight-line drift correction. Only while actually driving
            # straight -- never during a deliberate turn.
            if (correct_drift and DRIFT_TRIM and w == 0.0 and v > 0
                    and time.time() >= next_correction):
                pulse_end = time.time() + CORRECTION_PULSE_S
                while time.time() < pulse_end:
                    if manual_input():
                        raise ManualOverride
                    self.drive(CORRECTION_V, CORRECTION_W)
                    time.sleep(0.02)
                next_correction = time.time() + 1.0 / DRIFT_TRIM

            self.drive(v, w)
            if watch_obstacle and v > 0:
                d = self.telem.distance()
                if d is not None and d < OBSTACLE_CM:
                    return False
            time.sleep(0.05)
        return True

    # ---------------- behaviours ----------------

    def turn_for(self, degrees, direction):
        """Open-loop turn. direction is -1 for left, +1 for right."""
        duration = (degrees * MS_PER_DEGREE) / 1000.0
        # Don't watch for obstacles while turning in place: the whole point
        # of a turn is usually to get away from one we can already see.
        self.hold(0.0, direction * TURN_SPEED, duration,
                  watch_obstacle=False, correct_drift=False)

    def scan(self):
        """Look left and right, return (left_cm, right_cm).

        Falls back to 0.0 for a side that reads out of range, which is a lie
        in the safe direction -- we'd rather under-estimate clearance.
        """
        self.drive(0.0, 0.0)

        self.aim(SERVO_LEFT)
        self._settle()
        left = self.telem.distance() or 0.0

        self.aim(SERVO_RIGHT)
        self._settle()
        right = self.telem.distance() or 0.0

        self.aim(SERVO_CENTER)
        self._settle()
        return left, right

    def _settle(self):
        """Wait for the servo to stop moving, staying responsive to the user.

        A plain sleep here would make the car ignore WASD for the length of
        a three-position scan, which is exactly when you most want to grab it.
        """
        end = time.time() + SERVO_SETTLE_S
        while time.time() < end:
            if keyboard.is_pressed("q"):
                raise KeyboardInterrupt
            if manual_input():
                raise ManualOverride
            self.drive(0.0, 0.0)   # hold still and feed the watchdog
            time.sleep(0.05)

    def avoid(self):
        """Obstacle recovery: stop, back up, scan, turn toward open space."""
        self.stats["avoid"] += 1
        print("\n  obstacle -> backing up and scanning")

        self.drive(0.0, 0.0)
        time.sleep(0.2)
        self.hold(REVERSE_SPEED, 0.0, REVERSE_S,
                  watch_obstacle=False, correct_drift=False)
        self.drive(0.0, 0.0)

        left, right = self.scan()
        direction = -1 if left > right else 1
        side = "left" if direction < 0 else "right"
        print(f"  left={left:.0f}cm right={right:.0f}cm -> turning {side}")

        self.turn_for(random.uniform(60, 120), direction)

    def manual_drive(self):
        """Hand control to the user until they stop pressing keys.

        Stays in manual for RESUME_DELAY_S after the last keypress, so
        briefly releasing a key between nudges doesn't hand control back
        to the roamer mid-correction.
        """
        self.stats["manual"] += 1
        print("\n  [MANUAL] driving by hand -- release keys to resume roaming")

        last_input = time.time()
        while True:
            if keyboard.is_pressed("q"):
                raise KeyboardInterrupt

            cmd = manual_input()
            if cmd:
                last_input = time.time()
                self.drive(*cmd)
            else:
                if time.time() - last_input > RESUME_DELAY_S:
                    break
                self.drive(0.0, 0.0)

            # Aim the sensor head by hand too, same keys as sensor-log.py.
            if keyboard.is_pressed("j"):
                self.aim(min(180, self.servo_deg + 3))
            elif keyboard.is_pressed("l"):
                self.aim(max(0, self.servo_deg - 3))
            elif keyboard.is_pressed("k"):
                self.aim(SERVO_CENTER)

            time.sleep(0.05)

        self.drive(0.0, 0.0)
        self.aim(SERVO_CENTER)
        print("  [AUTO] resuming")

    def step(self):
        """One decision: mostly go straight, sometimes turn."""
        roll = random.random()

        if roll < 0.80:
            self.stats["straight"] += 1
            duration = random.uniform(STRAIGHT_MIN_S, STRAIGHT_MAX_S)
            # Ease off the throttle when something is ahead but not yet close
            # enough to count as an obstacle.
            d = self.telem.distance()
            speed = CRUISE_SPEED
            if d is not None and d < CAUTION_CM:
                speed = CRUISE_SPEED * 0.6
            print(f"\rstraight {duration:.1f}s at v={speed:.1f}"
                  f"   (dist {d if d else '--'})      ", end="", flush=True)
            if not self.hold(speed, 0.0, duration):
                self.avoid()
        else:
            # 10% of turns escalate to a full spin.
            if random.random() < 0.10:
                self.stats["spin"] += 1
                direction = random.choice([-1, 1])
                print("\r360 spin                                    ")
                self.turn_for(360, direction)
            else:
                self.stats["turn"] += 1
                degrees = random.uniform(0, 90)
                direction = random.choice([-1, 1])
                side = "left" if direction < 0 else "right"
                print(f"\rturn {degrees:.0f} deg {side}                    ")
                self.turn_for(degrees, direction)


def calibrate(sock, telem):
    """Drive straight for 4s at a few DRIFT_TRIM values so you can watch.

    There is no heading sensor on this car, so the measurement is your eyes:
    run it in a clear space and note which value tracks straightest.
    """
    global DRIFT_TRIM
    roamer = Roamer(sock, telem)
    candidates = [0.0, 0.8, 1.2, 1.8, 2.5]

    print("Calibration: 4s straight runs. Watch which drifts least.")
    print("Re-position the car by hand between runs.\n")

    for trim in candidates:
        DRIFT_TRIM = trim
        input(f"  DRIFT_TRIM = {trim}  -- press Enter to run...")
        try:
            roamer.hold(CRUISE_SPEED, 0.0, 4.0, watch_obstacle=False)
        except (KeyboardInterrupt, ManualOverride):
            pass
        for _ in range(3):
            sock.send(b"0.0,0.0\n")
            time.sleep(0.05)
        print("    done.\n")

    print("Set DRIFT_TRIM at the top of roam.py to whichever ran straightest.")
    print("If every value still drifted right, raise the range or increase")
    print(f"CORRECTION_PULSE_S (currently {CORRECTION_PULSE_S}s).")


def main():
    import sys

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ESP_IP, PORT))
    print(f"Connected to {ESP_IP}:{PORT}")
    print(f"Obstacle threshold: {OBSTACLE_CM:.0f}cm.")
    print("WASD takes manual control at any time (auto resumes "
          f"{RESUME_DELAY_S}s after you let go).")
    print("JKL aim the sensor head while in manual.  Q stops.\n")

    telem = Telemetry(sock)
    telem.start()

    # Wait for the first readings so we never make a decision blind.
    deadline = time.time() + 3.0
    while telem.distance() is None and time.time() < deadline:
        time.sleep(0.1)
    if telem.distance() is None:
        print("No telemetry -- check the firmware is running. Aborting.")
        telem.stop()
        sock.close()
        return

    if "--calibrate" in sys.argv:
        try:
            calibrate(sock, telem)
        finally:
            telem.stop()
            sock.close()
        return

    roamer = Roamer(sock, telem)
    try:
        roamer.aim(SERVO_CENTER)
        while True:
            if keyboard.is_pressed("q"):
                break
            try:
                roamer.step()
            except ManualOverride:
                # Abandon whatever the roamer was doing and hand over.
                roamer.manual_drive()
    except KeyboardInterrupt:
        pass
    finally:
        telem.stop()
        try:
            # Explicit stop, rather than relying on the firmware watchdog.
            for _ in range(3):
                sock.send(b"0.0,0.0\n")
                time.sleep(0.05)
            sock.send(f"S,{SERVO_CENTER}\n".encode())
            time.sleep(0.1)
        except OSError:
            pass
        sock.close()

        print("\n\nStopped.")
        s = roamer.stats
        print(f"  straight legs: {s['straight']}   turns: {s['turn']}   "
              f"360 spins: {s['spin']}   avoidances: {s['avoid']}   "
              f"manual takeovers: {s['manual']}")

        if telem.rows:
            with open(LOG_PATH, "w", newline="") as f:
                wr = csv.DictWriter(f, fieldnames=list(telem.rows[0].keys()))
                wr.writeheader()
                wr.writerows(telem.rows)
            print(f"  logged {len(telem.rows)} samples to {LOG_PATH}")


if __name__ == "__main__":
    main()
