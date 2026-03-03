from __future__ import annotations

import time
from collections import deque
from typing import Deque, Optional

from server.models import HistoryEntry, SimulationState


class AppState:
    def __init__(self) -> None:
        self.last_command: Optional[str] = None
        self.last_command_source: Optional[str] = None
        self.simulation_state: SimulationState = SimulationState()
        self.history: Deque[HistoryEntry] = deque(maxlen=50)
        self.pi_connected: bool = False

    def record_command(self, command: str, source: str, timestamp: float) -> None:
        self.last_command = command
        self.last_command_source = source
        self.history.appendleft(
            HistoryEntry(command=command, source=source, timestamp=timestamp)
        )

    def update_sim_state(self, state: SimulationState) -> None:
        self.simulation_state = state


_state = AppState()


def get_state() -> AppState:
    return _state
