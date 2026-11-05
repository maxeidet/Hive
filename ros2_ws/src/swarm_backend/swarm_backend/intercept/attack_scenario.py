"""The attack test's scenario and scoring, as pure math.

The defenders hold their slots on the surveillance arc (surveillance: in swarm.yaml). The
hostile drone starts `start_m` metres from the base, `bearing_deg` off the threat heading
(positive to the right, like the arc's slot angles: -30, 0, +30 for three defenders on
90 degrees), and flies in a straight line toward the base at constant speed, `height`
metres above the defenders' altitude so the cameras see it against the sky. The run is
over when it is shot down or reaches the base.

A hit is a defender and the hostile drone within `radius` of each other, from their true
positions; ClosestApproach also catches a pass between two samples (head-on they close at
up to 6 m/s). World NED, metres.
"""
import math
from dataclasses import dataclass

from swarm_backend.config import Vector3
from swarm_backend.intercept.paths import Path

DEFAULT_START_M = 150.0  # from the base: well beyond what the defenders on a 60 m arc see
# Metres above the defenders. More and PX4's 3 m/s climb limit makes head-on attacks fall
# short; less and the real detector does not see the target against the sky.
DEFAULT_HEIGHT_M = 10.0


@dataclass(frozen=True)
class Inbound:
    speed: float = 1.0  # m/s
    bearing_deg: float = 0.0  # where it comes from, degrees off the threat heading
    start_m: float = DEFAULT_START_M  # metres from the base
    height_m: float = DEFAULT_HEIGHT_M  # metres above the defenders' altitude

    def __post_init__(self):
        if self.speed <= 0:
            raise ValueError('speed must be positive')
        if self.start_m <= 0:
            raise ValueError('start_m must be positive')


def inbound_path(scenario: Inbound, base: tuple[float, float], defender_z: float,
                 heading: float) -> tuple[Path, float]:
    """The hostile drone's path from its start point to above the base, and how many seconds
    it takes to fly it. `defender_z` is the defenders' patrol height (world NED z), `heading`
    the threat heading, radians from north."""
    z = defender_z - scenario.height_m
    direction = heading + math.radians(scenario.bearing_deg)
    start = (base[0] + scenario.start_m * math.cos(direction),
             base[1] + scenario.start_m * math.sin(direction), z)
    end = (base[0], base[1], z)
    return Path([start, end], scenario.speed), scenario.start_m / scenario.speed


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
