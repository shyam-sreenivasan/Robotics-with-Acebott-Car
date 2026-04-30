import serial
import keyboard
import time

ser = serial.Serial('/dev/ttyUSB1', 115200)
time.sleep(2)

while True:
    v = 0.0
    w = 0.0

    if keyboard.is_pressed('w'):
        v = 1.0
    elif keyboard.is_pressed('s'):
        v = -1.0

    if keyboard.is_pressed('a'):
        w = -1.0
    elif keyboard.is_pressed('d'):
        w = 1.0

    msg = f"{v},{w}\n"
    print(msg, end='')
    ser.write(msg.encode())
    time.sleep(0.05)
