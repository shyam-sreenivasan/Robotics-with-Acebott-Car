"""Drive the car with WASD over WiFi while logging sensor telemetry to CSV.

The firmware streams one "T,..." line every 100ms. We read those on a
background thread so keypress sending never blocks on the socket.

Run with:  sudo ./venv/bin/python sensor-log.py
"""

import csv
import socket
import threading
import time
from datetime import datetime

import keyboard

# 🔧 same IP you set in keyboard-wifi-control.py
ESP_IP = "10.0.0.18"
PORT = 1234

CSV_PATH = f"sensor-log-{datetime.now():%Y%m%d-%H%M%S}.csv"

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ESP_IP, PORT))
print(f"Connected to {ESP_IP}:{PORT}")
print(f"Logging to {CSV_PATH}")
print("WASD to drive.  J/L aim the sensor head, K re-centers it.  Q to quit.\n")

servo_deg = 90
last_servo_sent = None

rows = []
latest = None
running = True


def reader():
    """Parse telemetry lines off the socket until the main thread stops us."""
    global latest
    buf = ""
    sock.settimeout(0.5)
    while running:
        try:
            chunk = sock.recv(1024).decode(errors="replace")
        except socket.timeout:
            continue
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
        # Hold the trailing partial line until its newline arrives.
        *lines, buf = buf.split("\n")
        for line in lines:
            parts = line.strip().split(",")
            if len(parts) != 9 or parts[0] != "T":
                continue
            try:
                row = {
                    "host_time": time.time(),
                    "esp_ms": int(parts[1]),
                    "distance_cm": float(parts[2]),
                    "line_l": int(parts[3]),
                    "line_m": int(parts[4]),
                    "line_r": int(parts[5]),
                    "v": float(parts[6]),
                    "w": float(parts[7]),
                    "servo_deg": int(parts[8]),
                }
            except ValueError:
                continue
            rows.append(row)
            latest = row


t = threading.Thread(target=reader, daemon=True)
t.start()

try:
    while True:
        if keyboard.is_pressed("q"):
            break

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

        sock.send(f"{v},{w}\n".encode())

        # Aim the sensor head. Sent only on change, so we don't spam the
        # servo with an identical angle 20 times a second.
        if keyboard.is_pressed("j"):
            servo_deg = min(180, servo_deg + 3)
        elif keyboard.is_pressed("l"):
            servo_deg = max(0, servo_deg - 3)
        elif keyboard.is_pressed("k"):
            servo_deg = 90

        if servo_deg != last_servo_sent:
            sock.send(f"S,{servo_deg}\n".encode())
            last_servo_sent = servo_deg

        if latest:
            d = latest["distance_cm"]
            dist = "  out of range" if d < 0 else f"{d:6.1f} cm"
            print(
                f"\rdist {dist}  servo {latest['servo_deg']:3d}deg   "
                f"line L{latest['line_l']:4d} "
                f"M{latest['line_m']:4d} R{latest['line_r']:4d}   "
                f"v={latest['v']:+.1f} w={latest['w']:+.1f}   "
                f"({len(rows)} samples)",
                end="",
                flush=True,
            )

        time.sleep(0.05)
finally:
    running = False
    # Coast to a stop rather than relying on the 300ms firmware timeout.
    try:
        sock.send(b"0.0,0.0\n")
        time.sleep(0.1)
    except OSError:
        pass
    sock.close()
    t.join(timeout=1.0)

    if rows:
        with open(CSV_PATH, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\n\nWrote {len(rows)} samples to {CSV_PATH}")
    else:
        print("\n\nNo telemetry received — check that the firmware was re-flashed.")
