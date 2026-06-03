import glob, subprocess, time

HEX = "/Users/eliyakhajeie/Library/Caches/arduino/sketches/2619736C2FFA864CB4E8D376F0AF0190/actuator_driver.ino.hex"
AVRDUDE = "/Users/eliyakhajeie/Library/Arduino15/packages/arduino/tools/avrdude/8.0.0-arduino1/bin/avrdude"
CONF = "/Users/eliyakhajeie/Library/Arduino15/packages/arduino/tools/avrdude/8.0.0-arduino1/etc/avrdude.conf"

ports = sorted(glob.glob('/dev/cu.usbmodem*'))
if not ports:
    print("Arduino not found. Plug it in first.")
    exit(1)

port = ports[0]
print(f"Using port: {port}")
print()
print("Starting upload... PRESS THE RED RESET BUTTON when you see attempt 1 or 2 below.")
print()

result = subprocess.run([
    AVRDUDE, f"-C{CONF}", "-patmega328p", "-carduino",
    f"-P{port}", "-b115200", "-D",
    f"-Uflash:w:{HEX}:i"
], text=True)

if result.returncode == 0:
    print("\nSUCCESS! Sketch uploaded!")
else:
    print("\nFailed. Try pressing reset earlier next time.")
