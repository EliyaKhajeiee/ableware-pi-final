from __future__ import annotations

import logging
from typing import Optional, Set

from fastapi import WebSocket

logger = logging.getLogger(__name__)


class ConnectionManager:
    def __init__(self) -> None:
        self.pi_ws: Optional[WebSocket] = None
        self.dashboard_clients: Set[WebSocket] = set()

    @property
    def pi_connected(self) -> bool:
        return self.pi_ws is not None

    async def connect_pi(self, ws: WebSocket) -> None:
        await ws.accept()
        self.pi_ws = ws
        logger.info("Pi connected")

    def disconnect_pi(self) -> None:
        self.pi_ws = None
        logger.info("Pi disconnected")

    async def connect_dashboard(self, ws: WebSocket) -> None:
        await ws.accept()
        self.dashboard_clients.add(ws)
        logger.info("Dashboard connected (%d total)", len(self.dashboard_clients))

    def disconnect_dashboard(self, ws: WebSocket) -> None:
        self.dashboard_clients.discard(ws)
        logger.info("Dashboard disconnected (%d remaining)", len(self.dashboard_clients))

    async def broadcast_to_dashboards(self, message: str) -> None:
        dead: Set[WebSocket] = set()
        for ws in self.dashboard_clients:
            try:
                await ws.send_text(message)
            except Exception:
                dead.add(ws)
        self.dashboard_clients -= dead


_manager = ConnectionManager()


def get_manager() -> ConnectionManager:
    return _manager
