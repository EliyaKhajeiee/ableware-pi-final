from __future__ import annotations

import logging
import os
import time
from typing import Optional

import httpx

from server.models import CommandMessage, SimulationState, StateUpdate
from server.state import get_state
from server.ws_manager import get_manager

logger = logging.getLogger(__name__)

SIM_URL = os.environ.get("SIM_URL", "http://localhost:8001")

_VALID_COMMANDS = {"START", "STOP", "UP", "DOWN", "LEFT", "RIGHT"}


async def route_command(msg: CommandMessage) -> None:
    """Validate command → forward to sim stub → update state → broadcast."""
    cmd = msg.command.upper()
    if cmd not in _VALID_COMMANDS:
        logger.warning("Ignoring unknown command: %s", cmd)
        return

    app_state = get_state()
    app_state.record_command(cmd, msg.source, msg.timestamp or time.time())

    # Forward to simulation stub
    try:
        async with httpx.AsyncClient(timeout=2.0) as client:
            resp = await client.post(f"{SIM_URL}/command", json={"command": cmd})
            resp.raise_for_status()
    except Exception as exc:
        logger.warning("Sim stub unreachable: %s", exc)

    # Fetch updated sim state
    sim_state = await _fetch_sim_state()
    if sim_state:
        app_state.update_sim_state(sim_state)

    await _broadcast_state()


async def _fetch_sim_state() -> Optional[SimulationState]:
    try:
        async with httpx.AsyncClient(timeout=2.0) as client:
            resp = await client.get(f"{SIM_URL}/state")
            resp.raise_for_status()
            return SimulationState(**resp.json())
    except Exception:
        return None


async def _broadcast_state() -> None:
    app_state = get_state()
    manager = get_manager()
    update = StateUpdate(
        last_command=app_state.last_command,
        last_command_source=app_state.last_command_source,
        simulation_state=app_state.simulation_state,
        command_history=list(app_state.history),
        pi_connected=manager.pi_connected,
    )
    await manager.broadcast_to_dashboards(update.model_dump_json())
