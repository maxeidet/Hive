"""When has a defender crashed? From Gazebo's true poses, no ROS (scenarios/surveillance.py
records the crashes of a run; gazebo_surveillance.py respawns the drones between runs).

A defender that should be flying has crashed when it is
  - upside down: its up axis leans more than CRASH_TILT_DEG from vertical,
  - or down: its body is lower than CRASH_HEIGHT_M above the ground (z = 0).
Only clear crashes count. The attack banks hard (61 degrees was seen in Gazebo, in an
attack that went on to hit), and PX4 recovers from that, so the tilt limit is well past
it; a crash that ends on the ground the right way up is caught by the height.
The first time is kept, so a drone that tumbles and comes to rest counts once.

A crash of the drone that hit the hostile drone, within INTERCEPT_CRASH_S of the hit, was
caused by the intercept (the two collided or it tumbled in the wake): the run is still a
hit. Any other crash has cause 'other'.
"""
import math
import threading
from dataclasses import dataclass

from swarm_backend.config import Vector3

CRASH_TILT_DEG = 120.0
CRASH_HEIGHT_M = 3.0  # the defenders patrol tens of metres up; the trees reach 20 m
INTERCEPT_CRASH_S = 10.0


def tilt_deg(q) -> float:
    """How far a body leans from upright (degrees), from its (x, y, z, w) quaternion."""
    x, y, z, w = q
    up_z = 1.0 - 2.0 * (x * x + y * y)  # world z of the body's up axis
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def crash_reason(position_ned: Vector3 | None, q) -> str | None:
    """Why a drone that should be flying has crashed, or None if it has not."""
    if q is not None:
        tilt = tilt_deg(q)
        if tilt > CRASH_TILT_DEG:
            return f'tilted {tilt:.0f} deg'
    if position_ned is not None and -position_ned[2] < CRASH_HEIGHT_M:
        return f'down ({-position_ned[2]:.1f} m up)'
    return None


MISPLACED_M = 5.0  # PX4's position this far (horizontally) from Gazebo's: wrong home


def misplaced(px4_positions: dict, true_positions: dict, limit: float = MISPLACED_M) -> dict:
    """{drone id: metres off} for the drones whose PX4 position (world NED, through its
    `home`) is more than `limit` from Gazebo's, horizontally: their PX4 was started
    somewhere other than where this process thinks (a respawn). Drones without both
    positions are left out."""
    out = {}
    for i, p in px4_positions.items():
        t = true_positions.get(i)
        if p is None or t is None:
            continue
        off = math.hypot(p[0] - t[0], p[1] - t[1])
        if off > limit:
            out[i] = off
    return out


@dataclass(frozen=True)
class Crash:
    drone_id: int
    time: float  # on the world's clock
    reason: str


def crash_cause(crash: Crash, killer: int | None, hit_time: float | None) -> str:
    """'intercept' if the drone that hit crashed within INTERCEPT_CRASH_S of the hit
    (or before it: the hit is scored on the segment between two poses), else 'other'."""
    if (crash.drone_id == killer and hit_time is not None
            and crash.time <= hit_time + INTERCEPT_CRASH_S):
        return 'intercept'
    return 'other'


class CrashWatch:
    """Watches the defenders that should be flying. Thread-safe: the poses arrive on
    Gazebo's thread, the scenario reads on its own."""

    def __init__(self):
        self._lock = threading.Lock()
        self._watched: set[int] = set()
        self._crashes: dict[int, Crash] = {}

    def watch(self, drone_ids) -> None:
        """From now on these drones should be flying: their crashes count."""
        with self._lock:
            self._watched = set(drone_ids)

    def stop(self) -> None:
        """Nothing is watched (taking off, landing)."""
        with self._lock:
            self._watched = set()

    def update(self, stamp: float, drone_id: int, position_ned: Vector3 | None, q) -> None:
        with self._lock:
            if drone_id not in self._watched or drone_id in self._crashes:
                return
            reason = crash_reason(position_ned, q)
            if reason is not None:
                self._crashes[drone_id] = Crash(drone_id, stamp, reason)

    def crashes(self) -> dict[int, Crash]:
        with self._lock:
            return dict(self._crashes)

    def clear(self, drone_ids) -> None:
        """These drones were respawned: their next crash counts again."""
        with self._lock:
            for d in drone_ids:
                self._crashes.pop(d, None)
