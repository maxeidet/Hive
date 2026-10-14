"""Pinhole camera model (Gazebo's cameras are ideal pinholes, no distortion).

Focal length, image centre, the horizon row, range from size, and the world direction of
a pixel. The camera's orientation is given as the three unit vectors it looks along
(forward), of image right and of image down, in the world frame (NED), as ground_truth.py
saves them and as the gimbal's attitude will give them on the drone. World point -> pixel
is ground_truth.project for now.
"""
import math
from dataclasses import dataclass

# gz_x500_gimbal's camera (PX4 model SDF): horizontal_fov 2.0 rad, 1280x720
GIMBAL_HFOV = 2.0
GIMBAL_SIZE = (1280, 720)
TARGET_SIZE_M = 0.5  # x500 frame across, what range_from_width assumes

Vector3 = tuple[float, float, float]


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

    def direction(self, u: float, v: float, forward: Vector3, right: Vector3,
                  down: Vector3) -> Vector3:
        """Unit vector in the world frame from the camera toward pixel (u, v)."""
        a = (u - self.cx) / self.f
        b = (v - self.cy) / self.f
        ray = tuple(f + a * r + b * d for f, r, d in zip(forward, right, down))
        norm = math.hypot(*ray)
        return tuple(c / norm for c in ray)


def angle_deg(a: Vector3, b: Vector3) -> float:
    """Angle between two unit vectors."""
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))))
