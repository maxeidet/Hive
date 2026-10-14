"""Where the interceptor should fly and look, given an estimate of the target.

Pure math, world NED. The interceptor flies at a fixed altitude, so steering is
horizontal (x, y); the target's height only matters for where the gimbal points.
"""
import math
from enum import Enum

from swarm_backend.config import Vector3
from swarm_backend.px4.drone import wrap_deg

MAX_LEAD_S = 15.0  # never aim further ahead than this; a faster target is not catchable anyway
LEAD_ITERATIONS = 5
STILL_SPEED = 0.5  # m/s; slower than this, "behind the target" means on our side of it
MAX_SWING_DEG = 45.0  # how far round the target the shadow point may lead us
# PX4 flies roughly POSITION_GAIN x (distance to the setpoint) m/s on top of the
# feed-forward (MPC_XY_P, default 0.95). Closing speed is kept low enough to stop
# with BRAKE_ACCEL, below PX4's MPC_ACC_HOR (3 m/s^2), so we do not overshoot.
POSITION_GAIN = 0.95
BRAKE_ACCEL = 2.0


class Phase(Enum):
    LOST = 'lost'  # no recent fix: hold position
    INTERCEPT = 'intercept'  # closing in on the predicted meeting point
    SHADOW = 'shadow'  # holding `standoff` metres behind the target


def next_phase(phase: Phase, target_range: float | None, standoff: float,
               capture_margin: float) -> Phase:
    """`target_range` is the horizontal distance to the estimate, None when lost."""
    if target_range is None:
        return Phase.LOST
    if target_range < standoff + capture_margin:
        return Phase.SHADOW
    if phase == Phase.SHADOW and target_range < 2 * standoff:
        return Phase.SHADOW  # hysteresis: a noisy estimate should not flip us back
    return Phase.INTERCEPT


def lead_point(target: Vector3, target_velocity: Vector3, me: Vector3,
               max_speed: float) -> Vector3:
    """Where a target at constant velocity will be when we can get there at max_speed."""
    t_go = 0.0
    point = target
    for _ in range(LEAD_ITERATIONS):
        t_go = min(MAX_LEAD_S, horizontal_distance(point, me) / max_speed)
        point = tuple(p + v * t_go for p, v in zip(target, target_velocity))
    return point


def shadow_point(target: Vector3, target_velocity: Vector3, me: Vector3,
                 standoff: float) -> Vector3:
    """A point `standoff` metres from the target, working round to behind it.

    The point is on the circle around the target, at most MAX_SWING_DEG from where we
    are now, toward the spot behind the target's track. So when the target turns back
    toward us we circle round it at standoff instead of cutting across it. A nearly
    still target is held on our side."""
    here = math.atan2(me[1] - target[1], me[0] - target[0])
    vx, vy = target_velocity[0], target_velocity[1]
    angle = here
    if math.hypot(vx, vy) >= STILL_SPEED:
        behind = math.atan2(-vy, -vx)
        swing = math.radians(wrap_deg(math.degrees(behind - here)))
        limit = math.radians(MAX_SWING_DEG)
        angle = here + max(-limit, min(limit, swing))
    return (target[0] + standoff * math.cos(angle), target[1] + standoff * math.sin(angle),
            target[2])


def approach_point(aim: Vector3, me: Vector3, max_speed: float) -> Vector3:
    """A setpoint toward `aim` close enough that PX4 closes at no more than max_speed,
    slowing down in time to stop there (relative to the target, given its velocity as
    feed-forward)."""
    d = horizontal_distance(me, aim)
    if d == 0:
        return aim
    closing = min(max_speed, math.sqrt(2 * BRAKE_ACCEL * d), POSITION_GAIN * d)
    f = closing / POSITION_GAIN / d
    return (me[0] + f * (aim[0] - me[0]), me[1] + f * (aim[1] - me[1]), aim[2])


def keep_clear(point: Vector3, target: Vector3, min_separation: float) -> Vector3:
    """Move `point` out to `min_separation` from the target (horizontally) if it is closer."""
    dx, dy = point[0] - target[0], point[1] - target[1]
    if math.hypot(dx, dy) >= min_separation:
        return point
    ux, uy = _unit(dx, dy)
    return (target[0] + min_separation * ux, target[1] + min_separation * uy, point[2])


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


def pointing_error_deg(pitch: float, yaw_from_north: float, me: Vector3,
                       target: Vector3) -> float:
    """Angle between where the camera looks (degrees: pitch from the horizon, yaw from
    north) and the true line of sight to the target."""
    p, y = math.radians(pitch), math.radians(yaw_from_north)
    look = (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p))
    los = tuple(t - m for t, m in zip(target, me))
    norm = math.hypot(*los)
    if norm == 0:
        return 0.0
    cos_angle = sum(a * b for a, b in zip(look, los)) / norm
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))


def horizontal_distance(a: Vector3, b: Vector3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _unit(x: float, y: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    return (1.0, 0.0) if norm == 0 else (x / norm, y / norm)
