"""
WebSocket client for the Ableware Pi → Command Hub connection.

Features:
- Auto-reconnect with exponential back-off (max 30s)
- Local send queue (deque maxlen=20) drained on reconnect
- Caller awaits send_command(); returns True on success, False on failure
"""

import asyncio
import json
import logging
import time
from collections import deque
from typing import Optional

import websockets
from websockets.exceptions import ConnectionClosed, WebSocketException

logger = logging.getLogger(__name__)

_BACKOFF_BASE = 1.0   # seconds
_BACKOFF_MAX = 30.0   # seconds
_QUEUE_MAX = 20


class PiWebSocketClient:
    """
    Maintains a persistent WebSocket connection to the Command Hub.

    Run `start()` as an asyncio background task; it handles connect/reconnect
    automatically. Use `send_command()` to dispatch commands.
    """

    def __init__(self, uri: str):
        """
        Args:
            uri: WebSocket URI, e.g. "ws://192.168.1.100:8000/ws/pi"
        """
        self.uri = uri
        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._connected = asyncio.Event()
        self._queue: deque = deque(maxlen=_QUEUE_MAX)
        self._running = False

    @property
    def is_connected(self) -> bool:
        return self._ws is not None

    async def start(self) -> None:
        """
        Persistent connection loop — run as a background asyncio task.
        Reconnects with exponential back-off on any failure.
        """
        self._running = True
        backoff = _BACKOFF_BASE

        while self._running:
            try:
                logger.info("Connecting to Command Hub at %s …", self.uri)
                async with websockets.connect(self.uri, ping_interval=20, ping_timeout=10) as ws:
                    self._ws = ws
                    self._connected.set()
                    backoff = _BACKOFF_BASE  # reset on successful connect
                    logger.info("Connected to Command Hub.")

                    await self._drain_queue()
                    await self._receive_loop(ws)

            except (ConnectionClosed, WebSocketException, OSError) as exc:
                logger.warning("WS disconnected: %s. Reconnecting in %.1fs …", exc, backoff)
            except Exception as exc:  # noqa: BLE001
                logger.error("Unexpected WS error: %s. Reconnecting in %.1fs …", exc, backoff)
            finally:
                self._ws = None
                self._connected.clear()

            if self._running:
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, _BACKOFF_MAX)

    async def stop(self) -> None:
        """Gracefully shut down the connection loop."""
        self._running = False
        if self._ws and not self._ws.closed:
            await self._ws.close()

    async def send_command(self, command: str, source: str = "voice") -> bool:
        """
        Send a command to the Command Hub.

        If the socket is not currently connected the message is queued locally
        and will be sent on the next successful reconnect.

        Args:
            command: One of START/STOP/UP/DOWN/LEFT/RIGHT (will be uppercased).
            source:  "voice" or "manual".

        Returns:
            True if sent immediately, False if queued or dropped.
        """
        payload = json.dumps({
            "type": "command",
            "command": command.upper(),
            "timestamp": time.time(),
            "source": source,
        })

        if self.is_connected:
            try:
                await self._ws.send(payload)
                logger.info("Sent command: %s (%s)", command, source)
                return True
            except Exception as exc:  # noqa: BLE001
                logger.warning("Send failed (%s), queuing: %s", exc, command)

        # Queue for later delivery
        self._queue.append(payload)
        logger.debug("Queued command (%d in queue): %s", len(self._queue), command)
        return False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    async def _receive_loop(self, ws: websockets.WebSocketClientProtocol) -> None:
        """Consume (and log) any messages sent from the server."""
        async for message in ws:
            try:
                data = json.loads(message)
                logger.debug("Hub → Pi: %s", data)
            except json.JSONDecodeError:
                logger.debug("Hub → Pi (raw): %s", message)

    async def _drain_queue(self) -> None:
        """Send any queued commands now that we are connected."""
        while self._queue and self.is_connected:
            payload = self._queue.popleft()
            try:
                await self._ws.send(payload)
                logger.info("Drained queued command: %s", payload)
            except Exception as exc:  # noqa: BLE001
                logger.warning("Failed to drain queued command: %s", exc)
                self._queue.appendleft(payload)
                break
