# Ableware

Voice-controlled assistive lift system. A Raspberry Pi listens for wake word + command, sends it over WebSocket to a hub running on your laptop, and the hub forwards it to the simulation and updates the dashboard in real time.

---

## Architecture

```
Pi (mic)
  → wake word (OpenWakeWord)
  → speech recognition (Vosk)
  → WebSocket → Hub :8000
                  → Dashboard (browser)
                  → Simulation Stub :8001
```

The hub is the only thing that talks to both the Pi and the simulation. The Pi and simulation never talk directly.

---

## What runs where

| Component | Machine | How to start |
|---|---|---|
| Hub + Dashboard | Laptop | `python3 -m uvicorn server.main:app --host 0.0.0.0 --port 8000` |
| Simulation stub | Laptop | `python3 -m uvicorn simulation_stub:app --port 8001` (run from `lift_actuator_sim/`) || Voice client | Pi | `python3 voice/main.py` |

Start order: simulation → hub → Pi client.

The hub serves the built dashboard at `http://localhost:8000`. No separate frontend server needed.

---

## Setup

### Laptop (hub + dashboard)

```bash
cd ~/ableware-pi
pip3 install fastapi "uvicorn[standard]" httpx pydantic pyyaml
```

Edit `config.yaml` and set `server.ip` to your laptop's LAN IP. The Pi needs to reach this address.

### Raspberry Pi (voice client)

```bash
sudo apt install portaudio19-dev espeak-ng
pip3 install -r voice/requirements.txt
```

You need two models in `voice/models/`:
- `vosk-model-small-en-us-0.15/` — download from https://alphacephei.com/vosk/models
- `ableware_wakeword.onnx` — your custom wake word model

If the wake word model is missing the system falls back to always-listening mode.

Edit `config.yaml`:
```yaml
server:
  ip: "192.168.1.XX"   # your laptop's IP, not the Pi's
  hub_port: 8000
```

Run:
```bash
python3 voice/main.py
```

Say "Ableware" to wake it, then say a command: **up, down, start, stop, left, right**.

---

## Working on the simulation (no Pi needed)

If you're working on the simulation side and don't have a Pi, you don't need one. The dashboard has manual control buttons that send the same commands the Pi would send.

1. Start the simulation stub:
```bash
cd lift_actuator_sim
python3 -m uvicorn simulation_stub:app --port 8001
```

2. Start the hub:
```bash
python3 -m uvicorn server.main:app --host 0.0.0.0 --port 8000
```

3. Open `http://localhost:8000` in a browser.

Use the UP / DOWN / START / STOP buttons in the dashboard. Commands go through the same hub → simulation path that voice commands use. The Pi badge will show disconnected — that's fine.

The simulation stub is in `lift_actuator_sim/simulation_stub.py`. It exposes:
- `POST /command` — accepts `{"command": "UP"}` etc.
- `GET /state` — returns current actuator + controller state

The hub polls `/state` every 250ms and pushes updates to the dashboard over WebSocket.

---

## Config reference (`config.yaml`)

| Key | What it does |
|---|---|
| `server.ip` | LAN IP of the laptop running the hub. Pi uses this to connect. |
| `server.hub_port` | Hub WebSocket port (default 8000) |
| `server.sim_port` | Simulation stub port (default 8001) |
| `voice.wake_word_threshold` | How confident the model needs to be before triggering (0–1) |
| `voice.listen_timeout_s` | Seconds to wait for a command after wake word before giving up |
| `actuator.step_size` | How far UP/DOWN moves the actuator per command (metres) |

---

## Troubleshooting

**Pi badge gray in dashboard** — Pi can't reach the hub. Check `server.ip` in `config.yaml` matches your laptop's IP. Both devices need to be on the same network.

**Hub badge gray in dashboard** — Hub isn't running, or you're accessing the dashboard from a non-localhost address on port 5173 (Vite dev server). Use `http://localhost:8000` instead.

**"Command queued (not connected)"** on Pi — Pi recognised the command but couldn't send it. Hub isn't reachable. You'll hear an error beep.

**No wake word detection** — If `models/ableware_wakeword.onnx` is missing, the system runs in bypass mode (always listening). Check the Pi logs on startup.
