from __future__ import annotations

import time
from typing import List, Optional

from pydantic import BaseModel, Field


class CommandMessage(BaseModel):
    type: str = "command"
    command: str
    timestamp: float = Field(default_factory=time.time)
    source: str = "manual"


class SimulationState(BaseModel):
    position: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0
    pwm: float = 0.0
    emergency_stopped: bool = False
    stalled: bool = False
    target_position: float = 0.0
    at_target: bool = True
    is_stable: bool = True


class HistoryEntry(BaseModel):
    command: str
    source: str
    timestamp: float


class StateUpdate(BaseModel):
    type: str = "state_update"
    last_command: Optional[str] = None
    last_command_source: Optional[str] = None
    simulation_state: SimulationState = Field(default_factory=SimulationState)
    command_history: List[HistoryEntry] = Field(default_factory=list)
    pi_connected: bool = False


class HealthResponse(BaseModel):
    pi_connected: bool
    status: str = "ok"
