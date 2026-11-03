"""Where a defender's gimbal camera looks while it searches (planning report, Base
Surveillance). The detection evaluation asks a GimbalScan for each defender's aim at
every image; the static-ring sector sweep is the baseline. No ROS here.
"""
from abc import ABC, abstractmethod

from swarm_backend.config import DetectConfig, Vector3


class GimbalScan(ABC):

    @abstractmethod
    def aim(self, t: float, drone: int, position: Vector3) -> tuple[float, float]:
        """(pitch, yaw) degrees for defender `drone` at `position` at time `t`: pitch
        from the horizon (negative = down), yaw from north."""


class SectorScan(GimbalScan):
    """Baseline (report, Base Surveillance > Baseline: static ring): each defender
    sweeps its outward sector (180 deg, centred on the direction away from the base)
    in a fixed tempo, holding each camera position for `scan_dwell` seconds, with
    positions one field of view apart.

    Placeholder: to be written with the surveillance component."""

    def __init__(self, config: DetectConfig, base: Vector3):
        raise NotImplementedError

    def aim(self, t: float, drone: int, position: Vector3) -> tuple[float, float]:
        raise NotImplementedError
