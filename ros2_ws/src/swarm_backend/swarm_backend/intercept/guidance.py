"""Where a gimbal camera should look to see a target. Pure math, world NED.
"""
import math

from swarm_backend.config import Vector3
from swarm_backend.px4.drone import wrap_deg


def bearing(frm: Vector3, to: Vector3) -> float:
    """Radians from north (the yaw that faces `to`)."""
    return math.atan2(to[1] - frm[1], to[0] - frm[0])


def gimbal_angles(me: Vector3, heading: float, target: Vector3) -> tuple[float, float]:
    """(pitch, yaw) degrees that point the gimbal at the target: pitch from the horizon
    (negative = down), yaw relative to the drone's heading (radians from north)."""
    down = target[2] - me[2]
    pitch = -math.degrees(math.atan2(down, horizontal_distance(me, target)))
    yaw = math.degrees(bearing(me, target) - heading)
    return pitch, wrap_deg(yaw)


def horizontal_distance(a: Vector3, b: Vector3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
