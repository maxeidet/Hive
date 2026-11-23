"""Scoring of intercept runs (planning report, Interception > Evaluation).

Everything is computed from the true positions sampled during a run, never from the
interceptor's estimates. Distances to the base are horizontal (the camp is an area on
the ground); distances between the drones are 3-D. Pure math, no ROS.
"""
import math
import statistics
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from itertools import groupby
from typing import TYPE_CHECKING

from swarm_backend.config import InterceptConfig, Vector3
from swarm_backend.intercept.interceptor import Status

if TYPE_CHECKING:
    from swarm_backend.evaluation.scenario import Scenario


@dataclass(frozen=True)
class Sample:
    """The truth at one step of a run."""
    t: float  # seconds since the run started (first detection)
    interceptor: Vector3
    target: Vector3
    in_view: bool  # target inside the interceptor's camera view
    detected: bool  # detector confirmed it as a drone in this image
    status: Status


class Outcome(Enum):
    HIT = 'hit'  # interceptor within hit_radius of the target
    REACHED_BASE = 'reached base'  # target came within goal_radius of the base
    LOST = 'lost'  # the action lost the target
    TIMEOUT = 'timeout'


@dataclass(frozen=True)
class RunMetrics:
    aspect_deg: float
    speed: float
    seed: int
    outcome: Outcome
    success: bool  # hit while the target was more than goal_radius from the base
    duration: float  # seconds the run lasted
    time_to_hit: float | None  # seconds from first detection; None without a hit
    lock_on: bool  # lock-on held for lock_time before the target came within goal_radius
    time_to_lock: float | None  # seconds from first detection until that lock-on started
    longest_lock: float  # seconds, the longest unbroken lock-on in the run
    base_distance_at_hit: float | None  # metres, target to base when hit
    closest_to_base: float  # metres, the target's closest approach to the base
    closest_miss: float  # metres, the closest the interceptor got to the target


def closest_approach(p0: Vector3, p1: Vector3, q0: Vector3, q1: Vector3) -> tuple[float, float]:
    """Two points moving in straight lines, p0 -> p1 and q0 -> q1, over the same interval:
    (their smallest distance, the fraction 0..1 of the interval where it happens).
    At 29 m/s closing and 10 Hz, the samples alone can step over a 1.5 m hit."""
    r0 = [a - b for a, b in zip(p0, q0)]
    dr = [(a1 - a0) - (b1 - b0) for a0, a1, b0, b1 in zip(p0, p1, q0, q1)]
    dd = sum(d * d for d in dr)
    s = 0.0 if dd == 0 else max(0.0, min(1.0, -sum(a * d for a, d in zip(r0, dr)) / dd))
    return math.hypot(*(a + s * d for a, d in zip(r0, dr))), s


def segment_closest_distance(p0: Vector3, p1: Vector3, q0: Vector3, q1: Vector3) -> float:
    return closest_approach(p0, p1, q0, q1)[0]


def first_within(p0: Vector3, p1: Vector3, q0: Vector3, q1: Vector3,
                 radius: float) -> float | None:
    """The fraction 0..1 of the interval where the two points (moving as in
    closest_approach) first come within `radius`, or None if they do not."""
    r0 = [a - b for a, b in zip(p0, q0)]
    dr = [(a1 - a0) - (b1 - b0) for a0, a1, b0, b1 in zip(p0, p1, q0, q1)]
    a = sum(d * d for d in dr)
    b = 2 * sum(r * d for r, d in zip(r0, dr))
    c = sum(r * r for r in r0) - radius ** 2
    if c <= 0:
        return 0.0
    if a == 0 or b * b < 4 * a * c:
        return None
    s = (-b - math.sqrt(b * b - 4 * a * c)) / (2 * a)
    return s if 0.0 <= s <= 1.0 else None


class LockOnTracker:
    """Lock-on: the target is in view, confirmed in at least k of the last n images and
    within `distance`. It counts once it lasts `duration` seconds without a break."""

    def __init__(self, k: int, n: int, distance: float, duration: float):
        self.k, self.distance, self.duration = k, distance, duration
        self._window: deque[bool] = deque(maxlen=n)
        self._run_start: float | None = None
        self.longest = 0.0  # seconds, the longest unbroken lock-on so far
        self.held_from: float | None = None  # start of the first lock-on that lasted

    @property
    def held(self) -> bool:
        return self.held_from is not None

    def update(self, t: float, in_view: bool, detected: bool, distance: float) -> bool:
        """Feed one image; returns whether we have a lock-on now."""
        self._window.append(detected)
        locked = in_view and sum(self._window) >= self.k and distance <= self.distance
        if not locked:
            self._run_start = None
            return False
        if self._run_start is None:
            self._run_start = t
        length = t - self._run_start
        self.longest = max(self.longest, length)
        if self.held_from is None and length >= self.duration:
            self.held_from = self._run_start
        return True


def score_run(samples: Sequence[Sample], scenario: 'Scenario',
              config: InterceptConfig) -> RunMetrics:
    if not samples:
        raise ValueError('A run needs at least one sample')
    base = scenario.base
    first_seen = next((s.t for s in samples if s.detected), None)

    closest_miss = math.dist(samples[0].interceptor, samples[0].target)
    hit_time = hit_target = None
    flown = list(samples)  # the samples until the hit (the target is stopped there)
    for i, (a, b) in enumerate(zip(samples, samples[1:])):
        closest_miss = min(closest_miss, segment_closest_distance(
            a.interceptor, b.interceptor, a.target, b.target))
        f = first_within(a.interceptor, b.interceptor, a.target, b.target, config.hit_radius)
        if f is not None:
            hit_time = a.t + f * (b.t - a.t)
            hit_target = tuple(p + f * (q - p) for p, q in zip(a.target, b.target))
            flown = list(samples[:i + 1])
            break

    to_base = [_horizontal(s.target, base) for s in flown]
    base_at_hit = None if hit_target is None else _horizontal(hit_target, base)
    closest_to_base = min(to_base + ([base_at_hit] if base_at_hit is not None else []))

    lock = LockOnTracker(config.lock_k, config.lock_n, config.lock_distance, config.lock_time)
    for s, d in zip(flown, to_base):
        if d <= config.goal_radius:
            break  # a lock-on only counts while the target is still outside x
        lock.update(s.t, s.in_view, s.detected, math.dist(s.interceptor, s.target))

    if hit_time is not None:
        outcome = Outcome.HIT
    elif closest_to_base <= config.goal_radius:
        outcome = Outcome.REACHED_BASE
    elif samples[-1].status == Status.LOST:
        outcome = Outcome.LOST
    else:
        outcome = Outcome.TIMEOUT

    since_seen = lambda t: None if t is None or first_seen is None else t - first_seen  # noqa: E731
    return RunMetrics(
        aspect_deg=scenario.aspect_deg, speed=scenario.speed, seed=scenario.seed,
        outcome=outcome,
        success=base_at_hit is not None and base_at_hit > config.goal_radius,
        duration=samples[-1].t - samples[0].t,
        time_to_hit=since_seen(hit_time),
        lock_on=lock.held, time_to_lock=since_seen(lock.held_from),
        longest_lock=lock.longest,
        base_distance_at_hit=base_at_hit, closest_to_base=closest_to_base,
        closest_miss=closest_miss)


def summarize(results: Sequence[RunMetrics], title: str = 'Intercept') -> str:
    """One row per (aspect, speed): rates, and mean +- sd / medians over its runs."""
    header = (f'{"aspect":>6} {"speed":>5} {"runs":>4} {"hit":>5} {"succ":>5} {"lock":>5} '
              f'{"t_hit [s]":>11} {"t_lock [s]":>11} {"base@hit":>8} {"min base":>8} '
              f'{"min miss":>8}  outcomes')
    lines = [f'{title}: {len(results)} runs', header, '-' * len(header)]
    key = lambda r: (r.aspect_deg, r.speed)  # noqa: E731
    for (aspect, speed), group in groupby(sorted(results, key=key), key=key):
        runs = list(group)
        counts = {o: sum(r.outcome == o for r in runs) for o in Outcome}
        lines.append(
            f'{aspect:>6.0f} {speed:>5.1f} {len(runs):>4} '
            f'{_rate(runs, lambda r: r.outcome == Outcome.HIT):>5} '
            f'{_rate(runs, lambda r: r.success):>5} {_rate(runs, lambda r: r.lock_on):>5} '
            f'{_mean_sd([r.time_to_hit for r in runs]):>11} '
            f'{_mean_sd([r.time_to_lock for r in runs]):>11} '
            f'{_median([r.base_distance_at_hit for r in runs]):>8} '
            f'{_median([r.closest_to_base for r in runs]):>8} '
            f'{_median([r.closest_miss for r in runs]):>8}  '
            + ' '.join(f'{o.value}={n}' for o, n in counts.items() if n))
    lines.append('rates in %; t_hit, t_lock from first detection (mean +- sd); '
                 'base@hit, min base, min miss in m (median)')
    return '\n'.join(lines)


def _horizontal(a: Vector3, b: Vector3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _rate(runs, predicate) -> str:
    return f'{100 * sum(map(predicate, runs)) / len(runs):.0f}'


def _mean_sd(values) -> str:
    values = [v for v in values if v is not None]
    if not values:
        return '-'
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return f'{statistics.mean(values):.1f}+-{sd:.1f}'


def _median(values) -> str:
    values = [v for v in values if v is not None]
    return '-' if not values else f'{statistics.median(values):.1f}'
