from __future__ import annotations

import asyncio
import logging
import os
import time
from typing import Optional

import httpx

from server.arduino_serial import get_arduino
from server.models import CommandMessage, SimulationState, StateUpdate
from server.state import get_state
from server.ws_manager import get_manager

logger = logging.getLogger(__name__)

SIM_URL = os.environ.get("SIM_URL", "http://localhost:8001")

_VALID_COMMANDS = {"START", "STOP", "UP", "DOWN", "LEFT", "RIGHT", "HOME", "SET_FLOOR"}

# HOME sequence: retract this many times to guarantee physical bottom,
# then extend HOME_STEPS to reach the floor position.
_RETRACT_PULSES = 25
_HOME_STEPS = 4
_PULSE_GAP = 0.35  # seconds between each pulse (> Arduino PULSE_MS of 200ms)

_MAX_STEPS = 7  # ceiling — UP blocked above this


async def route_command(msg: CommandMessage) -> None:
    """Validate command → enforce limits → forward to Arduino + sim → broadcast."""
    cmd = msg.command.upper()
    if cmd not in _VALID_COMMANDS:
        logger.warning("Ignoring unknown command: %s", cmd)
        return

    app_state = get_state()

    # Block everything except STOP while homing
    if app_state.is_homing and cmd != "STOP":
        logger.info("Homing in progress — ignoring %s", cmd)
        return

    # Floor enforcement: block DOWN at step 0 when homed
    if cmd == "DOWN" and app_state.is_homed and app_state.position_steps <= 0:
        logger.info("At floor — blocking DOWN")
        return

    # Ceiling enforcement: block UP at max steps when homed
    if cmd == "UP" and app_state.is_homed and app_state.position_steps >= _MAX_STEPS:
        logger.info("At ceiling (%d) — blocking UP", _MAX_STEPS)
        return

    app_state.record_command(cmd, msg.source, msg.timestamp or time.time())

    arduino = get_arduino()

    if cmd == "HOME":
        asyncio.create_task(_run_home_sequence())
        return

    if cmd == "SET_FLOOR":
        app_state.position_steps = 0
        app_state.is_homed = True
        logger.info("Floor set at current position")
        await _broadcast_state()
        return

    # Forward to Arduino
    if arduino and arduino.connected:
        if cmd == "UP":
            arduino.up()
            app_state.position_steps += 1
        elif cmd == "DOWN":
            arduino.down()
            app_state.position_steps -= 1
        elif cmd == "STOP":
            arduino.emergency_stop()
        elif cmd == "START":
            arduino.resume()

    # Forward to simulation stub
    try:
        async with httpx.AsyncClient(timeout=2.0) as client:
            resp = await client.post(f"{SIM_URL}/command", json={"command": cmd})
            resp.raise_for_status()
    except Exception as exc:
        logger.warning("Sim stub unreachable: %s", exc)

    sim_state = await _fetch_sim_state()
    if sim_state:
        app_state.update_sim_state(sim_state)

    await _broadcast_state()


async def _run_home_sequence() -> None:
    """Retract to physical bottom then extend HOME_STEPS to reach floor position."""
    app_state = get_state()
    arduino = get_arduino()

    if not arduino or not arduino.connected:
        logger.warning("HOME requested but Arduino not connected")
        return

    app_state.is_homing = True
    app_state.is_homed = False
    await _broadcast_state()
    logger.info("HOME: retracting (%d pulses)...", _RETRACT_PULSES)

    for _ in range(_RETRACT_PULSES):
        arduino.down()
        await asyncio.sleep(_PULSE_GAP)

    logger.info("HOME: extending to floor (%d pulses)...", _HOME_STEPS)
    for _ in range(_HOME_STEPS):
        arduino.up()
        await asyncio.sleep(_PULSE_GAP)

    app_state.position_steps = 0
    app_state.is_homed = True
    app_state.is_homing = False
    logger.info("HOME complete — at floor (position_steps=0)")
    await _broadcast_state()


async def _fetch_sim_state() -> Optional[SimulationState]:
    try:
        async with httpx.AsyncClient(timeout=2.0) as client:
            resp = await client.get(f"{SIM_URL}/state")
            resp.raise_for_status()
            return SimulationState(**resp.json())
    except Exception as exc:
        logger.warning("Sim state fetch failed: %s", exc)
        return None


async def _broadcast_state() -> None:
    app_state = get_state()
    manager = get_manager()
    arduino = get_arduino()
    update = StateUpdate(
        last_command=app_state.last_command,
        last_command_source=app_state.last_command_source,
        simulation_state=app_state.simulation_state,
        command_history=list(app_state.history),
        pi_connected=manager.pi_connected,
        arduino_connected=arduino.connected if arduino else False,
        position_steps=app_state.position_steps,
        is_homed=app_state.is_homed,
        is_homing=app_state.is_homing,
    )
    await manager.broadcast_to_dashboards(update.model_dump_json())
