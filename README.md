# Ableware

Voice + UI + hardware controlled assistive lift system. A Raspberry Pi listens for wake word + command and sends it over WebSocket to a hub running on your laptop. The hub forwards commands to the Arduino (which drives two linear actuators), the simulation, and updates the dashboard in real time. The dashboard also has manual buttons for direct control without the Pi.

---

## Architecture

```
Pi (mic)
  → wake word (OpenWakeWord)
  → speech recognition (Vosk)
  → WebSocket → Hub :8000
                  → Arduino (serial /dev/cu.usbmodem*)  ← drives both linear actuators
                  → Dashboard (browser, port 5173)
                  → Simulation Stub :8001
```

The hub is the single point of control — Pi, dashboard, and voice all route through it. The Arduino is the real hardware backend; the simulation mirrors the state visually.

---

## Hardware (Arduino branch)

### Wiring
| Arduino Pin | Motor Driver |
|---|---|
| 7 | Actuator 1 IN1 |
| 8 | Actuator 1 IN2 |
| 9 | Actuator 2 IN3 |
| 10 | Actuator 2 IN4 |
| 11 | IR Receiver signal |

Enable pins on the driver board are jumpered HIGH (always enabled). Motor power (12V) must be connected to the driver board separately from the Arduino USB.

### Serial protocol
The hub sends single-character commands at **115200 baud**, newline-terminated:

| Char | Action |
|---|---|
| `U` | Both actuators extend (up) for 200ms |
| `D` | Both actuators retract (down) for 200ms |
| `S` | Stop |
| `E` | Emergency stop |
| `R` | Resume (clear e-stop) |

The IR remote also works in parallel — codes `0x09` (up), `0x07` (down), `0x1C` (stop).

### Flashing the sketch
```bash
arduino-cli compile --fqbn arduino:avr:uno arduino/actuator_driver
arduino-cli upload -p /dev/cu.usbmodem1401 --fqbn arduino:avr:uno arduino/actuator_driver
```

Find your port with: `ls /dev/cu.usb*`

---

## What runs where

| Component | Machine | Command |
|---|---|---|
| Simulation stub | Laptop | `cd lift_actuator_sim && python3 -m uvicorn simulation_stub:app --port 8001` |
| Hub | Laptop | `python3 -m uvicorn server.main:app --host 0.0.0.0 --port 8000` |
| Frontend | Laptop | `cd frontend && npm run dev` → open http://localhost:5173 |
| Voice client | Pi | `python3 voice/main.py` |

**Start order:** simulation → hub → frontend → Pi (Pi is optional).

---

## Setup

### First time (laptop)

```bash
pip3 install fastapi "uvicorn[standard]" httpx pydantic pyyaml pyserial
cd frontend && npm install
```

### Arduino port
Edit `config.yaml` and set `arduino.port` to your device:
```yaml
arduino:
  enabled: true
  port: "/dev/cu.usbmodem1401"   # check with: ls /dev/cu.usb*
  baud: 115200
```

### Raspberry Pi (voice client)

```bash
sudo apt install portaudio19-dev espeak-ng
pip3 install -r voice/requirements.txt
```

Models needed in `voice/models/`:
- `vosk-model-small-en-us-0.15/` — from https://alphacephei.com/vosk/models
- `ableware_wakeword.onnx` — custom wake word model

Say **"Ableware"** to wake, then say a command: **up, down, start, stop, left, right**.

---

## Position control (floor / ceiling)

The system tracks actuator position as a step counter:

- **Floor** = 4 steps above physical bottom (hard limit — DOWN is blocked here)
- **Ceiling** = fully extended (no software limit; physical end-stop handles it)

**HOME button** (purple, top of controls): automatically retracts the actuators to the physical bottom, then extends 4 steps to the floor position. Press this once on startup before using UP/DOWN.

The step badge in the UI shows current position: `Step 0 from floor` = at floor, `Step 3 from floor` = 3 UP presses above floor.

---

## Dashboard connection badges

| Badge | Color | Meaning |
|---|---|---|
| Hub | Green | WebSocket to server connected |
| Pi | Green | Raspberry Pi voice client connected |
| Arduino | Blue | Serial connection to Arduino established |

---

## Config reference (`config.yaml`)

| Key | What it does |
|---|---|
| `server.ip` | LAN IP of the laptop running the hub |
| `server.hub_port` | Hub WebSocket port (default 8000) |
| `server.sim_port` | Simulation stub port (default 8001) |
| `arduino.enabled` | Set false to run without hardware |
| `arduino.port` | Serial port for Arduino |
| `arduino.baud` | Baud rate (default 115200) |
| `actuator.step_size` | Simulation step size per UP/DOWN (metres) |
| `voice.wake_word_threshold` | Wake word confidence threshold (0–1) |

---

## Troubleshooting

**Arduino badge gray** — Check `arduino.port` in config.yaml matches `ls /dev/cu.usb*`. Arduino must be plugged in before starting the hub.

**Actuators don't move** — Motor driver needs its own 12V power supply separate from Arduino USB. Check that the power LED on the driver board is lit.

**Pi badge gray** — Pi can't reach the hub. Check `server.ip` in config.yaml matches your laptop's LAN IP. Both must be on the same network.

**Hub badge gray** — Hub isn't running, or browser is on a different port. Use http://localhost:5173.

**DOWN button grayed out** — System is at the floor position (step 0). Press HOME first if position tracking is off, or press UP to go higher.

**"Unsupported upgrade request" in server logs** — Run `pip3 install "uvicorn[standard]"` to add WebSocket support.
