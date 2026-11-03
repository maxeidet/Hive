"""Where a defender's gimbal camera looks while it searches (planning report, Base
Surveillance). The detection evaluation asks a GimbalScan for each defender's aim at
every image; the static-ring sector sweep is the baseline. No ROS here.
"""
import math
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

    With the 2.0 rad camera that is two positions, 57 deg either side of outward,
    which together see 229 deg. The sweep goes back and forth (0, 1, .., n-1, n-2, .., 1),
    the camera level (pitch 0). Its vertical field of view is about +-41 deg, so a
    hostile far above or below a defender drops out of view when it is close."""

    SECTOR_DEG = 180.0

    def __init__(self, config: DetectConfig, base: Vector3):
        self.base = base
        self.dwell = config.scan_dwell
        fov = math.degrees(config.camera_hfov)
        n = max(1, math.ceil(self.SECTOR_DEG / fov - 1e-9))
        self.offsets = [(i - (n - 1) / 2) * fov for i in range(n)]
        self.order = list(range(n)) + list(range(n - 2, 0, -1))  # back and forth

    def aim(self, t: float, drone: int, position: Vector3) -> tuple[float, float]:
        outward = math.degrees(math.atan2(position[1] - self.base[1], position[0] - self.base[0]))
        step = int(max(0.0, t) / self.dwell + 1e-9)
        yaw = outward + self.offsets[self.order[step % len(self.order)]]
        return (0.0, (yaw + 180.0) % 360.0 - 180.0)
