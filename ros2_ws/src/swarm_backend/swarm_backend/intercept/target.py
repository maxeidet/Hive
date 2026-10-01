"""Where target measurements come from.

The chase does not care who measured the target: today it is the base station's
radar (mocked, see intercept/radar.py); later a drone's own depth camera or lidar
will lock on once it has visual contact. Each is a TargetSource, and the chase uses
the first source in its list that has a fresh fix.
"""
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from swarm_backend.config import Vector3


@dataclass(frozen=True)
class Fix:
    time: float  # seconds, on the clock of whoever consumes the fix
    source: str  # 'radar', later 'depth', 'lidar', ...


@dataclass(frozen=True)
class PositionFix(Fix):
    """A measured position, e.g. from a drone's own depth camera."""
    position: Vector3  # world NED
    std: float  # metres, on every axis


@dataclass(frozen=True)
class RadarFix(Fix):
    """One radar return, as the radar measures it (polar, from the radar)."""
    radar: Vector3  # where the radar is, world NED
    range: float  # metres, straight line from the radar
    azimuth: float  # radians from north, clockwise (toward east)
    elevation: float  # radians above the horizon
    radial_velocity: float  # m/s along the line of sight, positive = moving away
    range_std: float
    azimuth_std: float  # radians
    elevation_std: float  # radians
    radial_velocity_std: float

    @property
    def position(self) -> Vector3:
        """The measured position in world NED (elevation used for the height)."""
        horizontal = self.range * math.cos(self.elevation)
        rx, ry, rz = self.radar
        return (rx + horizontal * math.cos(self.azimuth),
                ry + horizontal * math.sin(self.azimuth),
                rz - self.range * math.sin(self.elevation))


class TargetSource(ABC):

    @abstractmethod
    def latest(self) -> Fix | None:
        """The most recent fix, or None if there has not been one."""
