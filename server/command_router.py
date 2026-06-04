from __future__ import annotations

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

SIM_URL = os.environ.get("SIM_URL", "http://127.0.0.1:8001")

_VALID_COMMANDS = {"START", "STOP", "UP", "DOWN", "LEFT", "RIGHT"}

_MIN_STEPS = 0
_MAX_STEPS = 7


def _describe_http_error(exc: Exception) -> str:
    text = str(exc).strip()
    if text:
        return f"{type(exc).__name__}: {text}"
    return type(exc).__name__


async def route_command(msg: CommandMessage) -> None:
    """Validate command → send to Arduino + sim → broadcast state."""
    cmd = msg.command.upper()
    if cmd not in _VALID_COMMANDS:
        logger.warning("Ignoring unknown command: %s", cmd)
        return

    app_state = get_state()

    app_state.record_command(cmd, msg.source, msg.timestamp or time.time())

    hardware_should_move = False
    if cmd == "UP":
        if app_state.position_steps >= _MAX_STEPS:
            logger.info("At ceiling - skipping step UP")
        else:
            app_state.position_steps += 1
            hardware_should_move = True
    elif cmd == "DOWN":
        if app_state.position_steps <= _MIN_STEPS:
            logger.info("At floor - skipping step DOWN")
        else:
            app_state.position_steps -= 1
            hardware_should_move = True

    # Send to Arduino (hardware)
    arduino = get_arduino()
    if arduino and arduino.connected:
        logger.warning("Arduino connected — sending %s", cmd)
        if cmd == "UP":
            if hardware_should_move:
                arduino.up()
        elif cmd == "DOWN":
            if hardware_should_move:
                arduino.down()
        elif cmd == "STOP":
            arduino.emergency_stop()
        elif cmd == "START":
            arduino.resume()
    else:
        logger.warning("Arduino NOT connected - skipping hardware for %s (arduino=%s connected=%s)",
                       cmd, arduino, arduino.connected if arduino else "N/A")

    # Forward to the simulation API. In the normal workflow this is wheelchair_sim_3d.py --serve.
    try:
        async with httpx.AsyncClient(timeout=2.0, trust_env=False) as client:
            resp = await client.post(f"{SIM_URL}/command", json={"command": cmd})
            resp.raise_for_status()
        app_state.sim_connected = True
    except Exception as exc:
        logger.warning("Simulation unreachable at %s: %s", SIM_URL, _describe_http_error(exc))
        app_state.sim_connected = False

    # Fetch updated sim state
    sim_state = await _fetch_sim_state()
    if sim_state:
        app_state.update_sim_state(sim_state)
        app_state.sim_connected = True

    await _broadcast_state()


async def _fetch_sim_state() -> Optional[SimulationState]:
    try:
        async with httpx.AsyncClient(timeout=2.0, trust_env=False) as client:
            resp = await client.get(f"{SIM_URL}/state")
            resp.raise_for_status()
            get_state().sim_connected = True
            return SimulationState(**resp.json())
    except Exception as exc:
        logger.warning("Sim state fetch failed at %s/state: %s", SIM_URL, _describe_http_error(exc))
        get_state().sim_connected = False
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
        sim_connected=app_state.sim_connected,
        arduino_connected=arduino.connected if arduino else False,
        position_steps=app_state.position_steps,
    )
    await manager.broadcast_to_dashboards(update.model_dump_json())
