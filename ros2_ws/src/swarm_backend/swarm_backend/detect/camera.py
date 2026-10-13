"""From a detection in the image to a direction and a rough distance.

An ideal pinhole camera (Gazebo's cameras have no lens distortion). The camera's
orientation is given as the three unit vectors it looks along (forward), of image right
and of image down, in the world frame (NED), as ground_truth.py saves them and as the
gimbal's attitude will give them on the drone. Pure math, no ROS.
"""
import math
from dataclasses import dataclass

Vector3 = tuple[float, float, float]

TARGET_SIZE_M = 0.5  # assumed width of the hostile drone (s in the planning report)


@dataclass(frozen=True)
class CameraModel:
    hfov: float  # horizontal field of view, radians
    width: int  # px
    height: int

    @property
    def focal(self) -> float:
        """Focal length in pixels (square pixels)."""
        return (self.width / 2) / math.tan(self.hfov / 2)

    def direction(self, u: float, v: float, forward: Vector3, right: Vector3,
                  down: Vector3) -> Vector3:
        """Unit vector in the world frame from the camera toward pixel (u, v)."""
        a = (u - self.width / 2) / self.focal
        b = (v - self.height / 2) / self.focal
        ray = tuple(f + a * r + b * d for f, r, d in zip(forward, right, down))
        norm = math.hypot(*ray)
        return tuple(c / norm for c in ray)

    def distance(self, size_px: float, size_m: float = TARGET_SIZE_M) -> float:
        """Distance to an object `size_m` wide that covers `size_px` pixels: r = f s / p.

        Along the optical axis; the few percent more toward the image corners is far less
        than the uncertainty in size_m and size_px."""
        return self.focal * size_m / size_px


def angle_deg(a: Vector3, b: Vector3) -> float:
    """Angle between two unit vectors."""
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))))
