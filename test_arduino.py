"""Quick sanity test for Arduino serial connection.
Run from project root:  python3 test_arduino.py
"""
import serial
import time

PORT = "/dev/cu.usbmodem1401"
BAUD = 115200
PAUSE = 1.5  # seconds between commands

def send(ser, cmd, label):
    ser.write(cmd)
    ser.flush()
    print(f"  → sent {label!r}  ({cmd!r})")
    time.sleep(PAUSE)

print(f"\nOpening {PORT} at {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"ERROR: could not open port — {e}")
    raise SystemExit(1)

time.sleep(2)  # Arduino resets on serial open; wait for it to boot

# drain any boot message
boot_msg = ser.read(ser.in_waiting or 1)
if boot_msg:
    print(f"  Arduino says: {boot_msg.decode(errors='ignore').strip()}")

print("\n--- Test sequence ---")
input("Press ENTER to send UP (actuators should extend slightly)...")
send(ser, b"U\n", "U")

input("Press ENTER to send DOWN (actuators should retract slightly)...")
send(ser, b"D\n", "D")

input("Press ENTER to send UP again...")
send(ser, b"U\n", "U")

input("Press ENTER to send STOP...")
send(ser, b"S\n", "S")

input("Press ENTER to send EMERGENCY STOP...")
send(ser, b"E\n", "E")

input("Press ENTER to send RESUME...")
send(ser, b"R\n", "R")

print("\nTest complete. Close with Ctrl+C or wait...")
ser.close()
