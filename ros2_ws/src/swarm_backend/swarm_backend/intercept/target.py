"""Where target measurements come from: a TargetSource gives the latest Fix of the
target, whoever measured it (the camera detector, intercept/inbox_detector.py, or the
stand-in detector of the evaluation, intercept/target_detector.py).
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass

from swarm_backend.config import Vector3


@dataclass(frozen=True)
class Fix:
    time: float  # seconds, on the clock of whoever consumes the fix
    source: str  # who measured it, e.g. 'camera'


@dataclass(frozen=True)
class PositionFix(Fix):
    """A measured position, e.g. from a drone's own depth camera."""
    position: Vector3  # world NED
    std: float  # metres, on every axis


class TargetSource(ABC):

    @abstractmethod
    def latest(self) -> Fix | None:
        """The most recent fix, or None if there has not been one."""
