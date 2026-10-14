"""The attack test's scenario and scoring (work plan task 4), as pure math.

The defender hovers at its guard position. The hostile drone flies `height` metres above
it, so the camera sees it against the sky, through the camera's view, at constant speed:

  crossing  a straight line across the view, `ahead` metres in front, from `half_width`
            metres to one side to `half_width` to the other
  headon    a straight line from `ahead` metres in front, over the guard position and
            HEADON_OVERSHOOT_M beyond it
  circle    a circle of radius `half_width` whose centre is `ahead` metres in front
  offset    the simplest run: head-on at the defender's own height, but `half_width`
            metres to its right, so the defender has to turn a little to meet it

"In front" is the attack's search heading. A hit is the two drones within `radius` of
each other, from their true positions; ClosestApproach also catches a pass between two
samples (head-on they close at up to 16 m/s). World NED, metres.
"""
import math
from dataclasses import dataclass

from swarm_backend.config import Vector3
from swarm_backend.intercept.paths import Path, make_path

PATTERNS = ('crossing', 'headon', 'circle', 'offset')
HEADON_OVERSHOOT_M = 20.0
DEFAULT_AHEAD = {'crossing': 35.0, 'headon': 80.0, 'circle': 30.0, 'offset': 60.0}
DEFAULT_HALF_WIDTH = {'crossing': 60.0, 'headon': 0.0, 'circle': 20.0, 'offset': 8.0}
# Metres above the guard position. More and PX4's 3 m/s climb limit makes head-on attacks
# fall short; less and the real detector does not see the target against the sky. The
# offset run is at the same height, for the fake detector only.
DEFAULT_HEIGHT = {'crossing': 10.0, 'headon': 10.0, 'circle': 10.0, 'offset': 0.0}


@dataclass(frozen=True)
class Scenario:
    pattern: str = 'crossing'
    speed: float = 4.0  # m/s
    height: float | None = None  # metres above the guard position; None = the pattern's default
    ahead: float | None = None  # metres in front of the guard; None = the pattern's default
    half_width: float | None = None  # crossing half-length, circle radius or offset to the
    # right; None = the pattern's default

    def __post_init__(self):
        if self.pattern not in PATTERNS:
            raise ValueError(f'Unknown pattern {self.pattern!r}; use one of {", ".join(PATTERNS)}')
        if self.speed <= 0:
            raise ValueError('speed must be positive')

    @property
    def height_m(self) -> float:
        return DEFAULT_HEIGHT[self.pattern] if self.height is None else self.height

    @property
    def ahead_m(self) -> float:
        return DEFAULT_AHEAD[self.pattern] if self.ahead is None else self.ahead

    @property
    def half_width_m(self) -> float:
        return DEFAULT_HALF_WIDTH[self.pattern] if self.half_width is None else self.half_width


def hostile_path(scenario: Scenario, guard: Vector3, heading: float) -> tuple[Path, float | None]:
    """The hostile drone's path and how many seconds it takes to fly it (None for a
    circle, which loops until the run times out). `heading` is the defender's search
    heading, radians from north."""
    f = (math.cos(heading), math.sin(heading))  # in front
    r = (-math.sin(heading), math.cos(heading))  # to the right
    gx, gy, gz = guard
    z = gz - scenario.height_m
    ahead, half = scenario.ahead_m, scenario.half_width_m

    def point(forward: float, right: float) -> Vector3:
        return (gx + forward * f[0] + right * r[0], gy + forward * f[1] + right * r[1], z)

    if scenario.pattern == 'crossing':
        start, end = point(ahead, -half), point(ahead, half)
    elif scenario.pattern == 'headon':
        start, end = point(ahead, 0.0), point(-HEADON_OVERSHOOT_M, 0.0)
    elif scenario.pattern == 'offset':
        start, end = point(ahead, half), point(-HEADON_OVERSHOOT_M, half)
    else:
        # make_path's circle leaves its start point straight ahead and curves right, so
        # starting on the far side, heading right, puts the centre `ahead` in front.
        anchor = point(ahead + half, 0.0)
        return make_path('circle', anchor, heading + math.pi / 2, scenario.speed, 2 * half), None
    path = Path([start, end], scenario.speed)
    return path, math.dist(start, end) / scenario.speed


def segment_distance(a0: Vector3, a1: Vector3, b0: Vector3, b1: Vector3) -> float:
    """Closest distance between two points that move in a straight line, at constant
    speed, from a0 to a1 and from b0 to b1 over the same time."""
    d0 = [p - q for p, q in zip(a0, b0)]
    dv = [(p1 - p0) - (q1 - q0) for p0, p1, q0, q1 in zip(a0, a1, b0, b1)]
    vv = sum(v * v for v in dv)
    s = 0.0 if vv == 0 else max(0.0, min(1.0, -sum(d * v for d, v in zip(d0, dv)) / vv))
    return math.hypot(*[d + s * v for d, v in zip(d0, dv)])


class ClosestApproach:
    """Keeps the closest distance between two drones over a run, between samples too,
    and the time of the first hit (closer than `radius`)."""

    def __init__(self, radius: float = 1.5):
        self.radius = radius
        self.closest = math.inf
        self.closest_time: float | None = None
        self.hit_time: float | None = None
        self._last: tuple[Vector3, Vector3] | None = None

    def update(self, a: Vector3, b: Vector3, time: float) -> None:
        if self._last is None:
            distance = math.dist(a, b)
        else:
            distance = segment_distance(self._last[0], a, self._last[1], b)
        self._last = (a, b)
        if distance < self.closest:
            self.closest, self.closest_time = distance, time
        if self.hit_time is None and distance < self.radius:
            self.hit_time = time

    @property
    def hit(self) -> bool:
        return self.hit_time is not None
