"""Pinhole camera model (Gazebo's cameras are ideal pinholes, no distortion).

Only what the baseline needs so far: focal length, image centre, the horizon row and
range from size. Pixel <-> world direction comes with step 1 of the detection plan.
"""
import math
from dataclasses import dataclass

# gz_x500_gimbal's camera (PX4 model SDF): horizontal_fov 2.0 rad, 1280x720
GIMBAL_HFOV = 2.0
GIMBAL_SIZE = (1280, 720)
TARGET_SIZE_M = 0.5  # x500 frame across, what range_from_width assumes


@dataclass(frozen=True)
class CameraModel:
    hfov: float = GIMBAL_HFOV  # radians
    width: int = GIMBAL_SIZE[0]
    height: int = GIMBAL_SIZE[1]

    @property
    def f(self) -> float:
        """Focal length in pixels (square pixels)."""
        return (self.width / 2) / math.tan(self.hfov / 2)

    @property
    def cx(self) -> float:
        return self.width / 2

    @property
    def cy(self) -> float:
        return self.height / 2

    def horizon_row(self, pitch_deg: float) -> float:
        """Image row of the horizon for a camera pitched pitch_deg up from level (no roll).

        Ignores the dip of the horizon with altitude (0.2 degrees at 50 m)."""
        return self.cy + self.f * math.tan(math.radians(pitch_deg))

    def range_from_width(self, pixels: float, size: float = TARGET_SIZE_M) -> float:
        """Distance at which an object `size` metres across is `pixels` wide: r = f*s/p."""
        return self.f * size / pixels

    def width_at(self, distance: float, size: float = TARGET_SIZE_M) -> float:
        """Pixels across of an object `size` metres across at `distance` metres."""
        return self.f * size / distance
