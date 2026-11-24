"""Scenario 2's pure math: hostile drones flying in a straight line toward the base, and
which of them a detection points at, and birds flying in with them (--birds). World NED,
metres, like the rest of the backend."""
import math
import random
from dataclasses import dataclass

from swarm_backend.config import Vector3


def inbound_line(start: Vector3, base: tuple[float, float], stop_m: float) -> Vector3:
    """Where a hostile drone flying from `start` straight at the base stops: `stop_m` short
    of the base, on its own line in, at the start's height."""
    dx, dy = base[0] - start[0], base[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= stop_m:
        raise ValueError(f'The start point is only {length:.1f} m from the base, '
                         f'not more than stop_m = {stop_m} m')
    f = (length - stop_m) / length
    return (start[0] + f * dx, start[1] + f * dy, start[2])


def straight_at(start: Vector3, end: Vector3, speed: float, t: float
                ) -> tuple[Vector3, Vector3, bool]:
    """(position, velocity, arrived) `t` seconds after leaving `start` toward `end` at
    `speed`; at `end`, with zero velocity, from the moment it gets there."""
    if speed <= 0:
        raise ValueError(f'speed must be positive, got {speed}')
    length = math.dist(start, end)
    if length == 0 or t * speed >= length:
        return tuple(end), (0.0, 0.0, 0.0), True
    f = max(0.0, t) * speed / length
    position = tuple(a + f * (b - a) for a, b in zip(start, end))
    velocity = tuple(speed * (b - a) / length for a, b in zip(start, end))
    return position, velocity, False


def angle_deg(a: Vector3, b: Vector3) -> float:
    na, nb = math.hypot(*a), math.hypot(*b)
    if na == 0 or nb == 0:
        return math.inf
    cos = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def match_hostile(relative: Vector3, observer: Vector3, hostiles: dict[int, Vector3],
                  gate_deg: float = 5.0) -> tuple[int, float] | None:
    """(id, true distance) of the hostile drone whose true direction from `observer` is
    closest to the detection's `relative` vector (target_position), or None when none is
    within `gate_deg`. Matched by direction: the distance from the box size is too rough."""
    best = None
    for drone_id, position in hostiles.items():
        to = tuple(p - o for p, o in zip(position, observer))
        angle = angle_deg(relative, to)
        if angle <= gate_deg and (best is None or angle < best[0]):
            best = (angle, drone_id, math.hypot(*to))
    return None if best is None else (best[1], best[2])


@dataclass(frozen=True)
class BirdPlan:
    name: str  # its Gazebo model name
    kind: str  # a birds.KINDS name
    start: Vector3
    end: Vector3  # past the base, on the bird's own line through it
    speed: float  # m/s


def plan_birds(n: int, hostile_starts: list[Vector3], base: tuple[float, float],
               kinds: list[str], speed: float, rng: random.Random, spread_m: float = 25.0,
               height_m: float = 8.0, speed_spread: float = 0.3, past_m: float = 60.0,
               min_up_m: float = 5.0, prefix: str = 'scn_bird_') -> list[BirdPlan]:
    """`n` birds that fly in with the hostile drones: each starts at a random point within
    `spread_m` (horizontally, at least 5 m off) and `height_m` (vertically) of a randomly
    picked hostile's start, at least `min_up_m` above the ground (z = 0), and flies in a
    straight line over the base and `past_m` beyond it, at `speed` +- `speed_spread`
    (a share). Every kind is used before any is used twice. Named `prefix` + its number."""
    if not hostile_starts:
        raise ValueError('plan_birds needs at least one hostile start')
    plans, pool = [], []
    for i in range(n):
        if not pool:
            pool = list(kinds)
            rng.shuffle(pool)
        kind = pool.pop()
        hn, he, hd = rng.choice(hostile_starts)
        r = rng.uniform(5.0, max(5.0, spread_m))
        a = rng.uniform(0.0, 2 * math.pi)
        z = min(hd + rng.uniform(-height_m, height_m), -min_up_m)
        start = (hn + r * math.cos(a), he + r * math.sin(a), z)
        dx, dy = base[0] - start[0], base[1] - start[1]
        length = math.hypot(dx, dy) or 1.0
        f = (length + past_m) / length
        end = (start[0] + f * dx, start[1] + f * dy, z)
        v = speed * rng.uniform(1.0 - speed_spread, 1.0 + speed_spread)
        plans.append(BirdPlan(f'{prefix}{i}', kind, start, end, max(0.5, v)))
    return plans
