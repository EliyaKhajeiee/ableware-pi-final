from __future__ import annotations

import asyncio
import json
import logging
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from server.command_router import _broadcast_state, _fetch_sim_state, route_command
from server.models import CommandMessage, HealthResponse
from server.state import get_state
from server.ws_manager import get_manager

logger = logging.getLogger(__name__)

app = FastAPI(title="Ableware Command Hub", version="1.0.0")

_STATE_POLL_INTERVAL = 0.25  # seconds


async def _state_poll_loop() -> None:
    """Continuously poll the sim stub and broadcast to dashboards."""
    while True:
        await asyncio.sleep(_STATE_POLL_INTERVAL)
        manager = get_manager()
        if not manager.dashboard_clients:
            continue
        sim_state = await _fetch_sim_state()
        if sim_state:
            get_state().update_sim_state(sim_state)
            await _broadcast_state()


@app.on_event("startup")
async def _startup() -> None:
    asyncio.create_task(_state_poll_loop())


@app.websocket("/ws/pi")
async def ws_pi(ws: WebSocket) -> None:
    manager = get_manager()
    app_state = get_state()
    await manager.connect_pi(ws)
    app_state.pi_connected = True
    await _broadcast_state()
    try:
        while True:
            raw = await ws.receive_text()
            try:
                data = json.loads(raw)
                msg = CommandMessage(**data)
                if msg.type == "command":
                    await route_command(msg)
            except Exception as exc:
                logger.warning("Bad message from Pi: %s — %s", raw, exc)
    except WebSocketDisconnect:
        pass
    finally:
        manager.disconnect_pi()
        app_state.pi_connected = False
        await _broadcast_state()


@app.websocket("/ws/dashboard")
async def ws_dashboard(ws: WebSocket) -> None:
    manager = get_manager()
    await manager.connect_dashboard(ws)
    # Send current state immediately so the dashboard doesn't wait for a command
    await _broadcast_state()
    try:
        while True:
            raw = await ws.receive_text()
            try:
                data = json.loads(raw)
                msg = CommandMessage(**data)
                if msg.type == "command":
                    await route_command(msg)
            except Exception as exc:
                logger.warning("Bad message from dashboard: %s — %s", raw, exc)
    except WebSocketDisconnect:
        pass
    finally:
        manager.disconnect_dashboard(ws)


@app.get("/health", response_model=HealthResponse)
async def health() -> HealthResponse:
    return HealthResponse(pi_connected=get_manager().pi_connected)


# Serve built frontend (production only — dev uses vite dev server)
_dist = Path(__file__).parent.parent / "frontend" / "dist"
if _dist.exists():
    app.mount("/", StaticFiles(directory=str(_dist), html=True), name="static")
