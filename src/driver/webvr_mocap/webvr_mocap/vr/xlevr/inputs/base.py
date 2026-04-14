"""Base classes and data structures for input providers."""

import asyncio
import contextlib
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any, Literal

import numpy as np


class ControlMode(Enum):
    """Control modes for the teleoperation system."""

    POSITION_CONTROL = "position"
    IDLE = "idle"


@dataclass
class ControlGoal:
    """High-level control goal message sent from input providers."""

    arm: Literal["left", "right"]
    mode: ControlMode | None = None  # Control mode (None = no mode change)
    target_position: np.ndarray | None = None  # 3D position in robot coordinates
    wrist_roll_deg: float | None = None  # Wrist roll angle in degrees
    wrist_flex_deg: float | None = None  # Wrist flex (pitch) angle in degrees
    gripper_closed: bool | None = None  # Gripper state (None = no change)

    # Additional data for debugging/monitoring
    metadata: dict[str, Any] | None = None


class BaseInputProvider(ABC):
    """Abstract base class for input providers."""

    def __init__(self, command_queue: asyncio.Queue):
        self.command_queue = command_queue
        self.is_running = False

    @abstractmethod
    async def start(self):
        """Start the input provider."""
        pass

    @abstractmethod
    async def stop(self):
        """Stop the input provider."""
        pass

    async def send_goal(self, goal: ControlGoal):
        """Send a control goal to the command queue."""
        # Best effort: ignore queue full or other put() errors.
        with contextlib.suppress(Exception):
            await self.command_queue.put(goal)
