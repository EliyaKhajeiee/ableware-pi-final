"""Interactive terminal control — press keys to send commands to Arduino.
Run:  python3 control.py
"""
import serial
import sys
import tty
import termios
import time

PORT = "/dev/cu.usbmodem1401"
BAUD = 115200

COMMANDS = {
    'u': (b"U\n", "UP"),
    'd': (b"D\n", "DOWN"),
    's': (b"S\n", "STOP"),
    'e': (b"E\n", "EMERGENCY STOP"),
    'r': (b"R\n", "RESUME"),
    'q': (None,   "QUIT"),
}

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

print(f"Connecting to Arduino on {PORT}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"ERROR: {e}")
    raise SystemExit(1)

time.sleep(2)
boot = ser.read(ser.in_waiting or 1).decode(errors='ignore').strip()
if boot:
    print(f"Arduino: {boot}")

print("\nControls:  U=up  D=down  S=stop  E=estop  R=resume  Q=quit\n")

while True:
    key = getch().lower()
    if key not in COMMANDS:
        continue
    data, label = COMMANDS[key]
    if data is None:
        print("\nQuit.")
        break
    ser.write(data)
    ser.flush()
    print(f"  → {label}")

ser.close()
