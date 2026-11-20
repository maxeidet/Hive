"""The intercept evaluation scenarios and the loop that flies one (planning report,
Interception > Evaluation).

A hostile drone flies a straight track at constant speed and altitude toward the base
(the camp, at the origin). The interceptor starts on the defenders' ring and the run
starts at first detection: the target is `detect_range` away, and its line of sight
to the interceptor makes `aspect_deg` with its velocity (0 = head-on, 90 = crossing).

run_scenario only knows the abstractions: a Sim (fast simulation or Gazebo), an
InterceptAction and a Detector, both made by factories. So any guidance or detector
is scored the same way, on the same seeded scenarios. Pure math, world NED, no ROS.
"""
import math
import random
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Protocol

from swarm_backend.config import InterceptConfig, Vector3
from swarm_backend.intercept.interceptor import InterceptAction, Status
from swarm_backend.evaluation.metrics import Sample, segment_closest_distance
from swarm_backend.evaluation.mock import HostileFlight
from swarm_backend.evaluation.paths import Path
from swarm_backend.intercept.target_detector import Detector
from swarm_backend.px4.drone import Drone

STEP_S = 0.1  # 10 Hz, as in the report


@dataclass(frozen=True)
class Scenario:
    aspect_deg: float
    speed: float  # m/s, the target's
    seed: int
    base: Vector3  # on the ground
    interceptor_start: Vector3
    target_start: Vector3
    target_goal: Vector3  # above the base, at the target's altitude

    @property
    def path(self) -> Path:
        return Path([self.target_start, self.target_goal], self.speed)

    @property
    def velocity(self) -> Vector3:
        """The target's velocity along its track."""
        d = [g - s for g, s in zip(self.target_goal, self.target_start)]
        norm = math.hypot(*d)
        return tuple(self.speed * v / norm for v in d)

    @property
    def arrival_time(self) -> float:
        """Seconds until the target is above the base."""
        return math.dist(self.target_start, self.target_goal) / self.speed

    def shifted(self, offset: Vector3) -> 'Scenario':
        """The same scenario moved by `offset` (e.g. to where a Gazebo drone is)."""
        move = lambda p: tuple(a + b for a, b in zip(p, offset))  # noqa: E731
        return replace(self, base=move(self.base), interceptor_start=move(self.interceptor_start),
                       target_start=move(self.target_start), target_goal=move(self.target_goal))


def make_scenario(aspect_deg: float, speed: float, seed: int,
                  config: InterceptConfig) -> Scenario:
    """The seed picks where on the ring the interceptor is, which side the target comes
    from and its altitude; the same seed gives the same draw for every aspect and speed."""
    rng = random.Random(seed)
    ring = rng.uniform(-math.pi, math.pi)  # the interceptor's bearing from the base
    side = rng.choice((-1.0, 1.0))
    target_z = -rng.uniform(config.hostile_altitude_min, config.hostile_altitude_max)

    # Triangle base O, interceptor I (|OI| = R), target T (|TI| = r). The angle at T
    # between T->O (its velocity) and T->I is the aspect; take the solution outside.
    big_r, r = config.perimeter_radius, config.detect_range
    aspect = math.radians(aspect_deg)
    if r * math.sin(aspect) >= big_r:
        raise ValueError(f'No scenario with aspect {aspect_deg} deg at {r} m from the ring')
    at_base = math.asin(r * math.sin(aspect) / big_r)  # angle at O between I and T
    distance = r * math.cos(aspect) + math.sqrt(big_r ** 2 - (r * math.sin(aspect)) ** 2)
    bearing = ring + side * at_base
    return Scenario(
        aspect_deg=aspect_deg, speed=speed, seed=seed, base=(0.0, 0.0, 0.0),
        interceptor_start=(big_r * math.cos(ring), big_r * math.sin(ring), -config.altitude),
        target_start=(distance * math.cos(bearing), distance * math.sin(bearing), target_z),
        target_goal=(0.0, 0.0, target_z))


def scenario_grid(config: InterceptConfig) -> list[Scenario]:
    """Every aspect x speed x seed of the config, seeds 0..eval_runs-1."""
    return [make_scenario(aspect, speed, seed, config)
            for aspect in config.eval_aspects for speed in config.eval_speeds
            for seed in range(config.eval_runs)]


class Sim(Protocol):
    """Two drones and a clock: a fast simulation in tests, or PX4 in Gazebo."""
    hostile: Drone
    interceptor: Drone

    def now(self) -> float | None:
        """Seconds on the drones' clock, None while it is not known."""

    def advance(self, dt: float) -> None:
        """Let `dt` seconds pass (integrate, or wait for the simulation)."""


ActionFactory = Callable[[Drone, Detector, InterceptConfig, Callable[[], float | None]],
                         InterceptAction]
DetectorFactory = Callable[[Sim, Scenario, InterceptConfig], Detector]


def run_scenario(scenario: Scenario, sim: Sim, make_action: ActionFactory,
                 make_detector: DetectorFactory, config: InterceptConfig,
                 dt: float = STEP_S) -> list[Sample]:
    """Fly one scenario from first detection until a hit, the target reaching the base,
    the target being lost (after it was detected) or `eval_timeout`. The drones must
    already be at the scenario's start. Returns the true positions at every step."""
    detector = make_detector(sim, scenario, config)
    action = make_action(sim.interceptor, detector, config, sim.now)
    flight = HostileFlight(sim.hostile, scenario.path, sim.now)
    samples: list[Sample] = []
    start = None
    seen = False
    flight.start()
    action.start()
    try:
        while True:
            now = sim.now()
            if now is None:
                sim.advance(dt)
                continue
            if start is None:
                start = now
            t = now - start
            flight.step()
            detector.update(now)
            action.step()
            seen = seen or detector.detected
            samples.append(Sample(
                t=t, interceptor=sim.interceptor.position, target=sim.hostile.position,
                in_view=detector.in_view, detected=detector.detected, status=action.status))
            if _finished(samples, scenario, config, seen):
                return samples
            sim.advance(dt)
    finally:
        action.stop()
        flight.stop()


def _finished(samples: list[Sample], scenario: Scenario, config: InterceptConfig,
              seen: bool) -> bool:
    last = samples[-1]
    if len(samples) > 1:
        before = samples[-2]
        miss = segment_closest_distance(before.interceptor, last.interceptor,
                                        before.target, last.target)
        if miss <= config.hit_radius:
            return True
    return (last.t >= scenario.arrival_time or last.t >= config.eval_timeout
            or (seen and last.status == Status.LOST))
