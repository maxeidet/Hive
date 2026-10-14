"""The detection evaluation scenarios and the loop that runs one (planning report,
Drone Detection Using Sensors > Evaluation).

The defenders hold a static ring of radius R_p around the base (the origin) and
search with their gimbal cameras. A hostile drone flies straight and level from a
random direction toward the base, and birds fly across from random directions as
distractors. A run ends when the hostile drone reaches the ring (detected or not).

run_scenario only knows the abstractions: a DetectSim (fast simulation or Gazebo), a
GimbalScan and one ImageDetector per defender, all made by factories, so every method
is scored the same way on the same seeded scenarios. Pure math, world NED, no ROS.
"""
import math
import random
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from swarm_backend.config import DetectConfig, Vector3
from swarm_backend.detect.detector import Detection, ImageDetector
from swarm_backend.detect.frame import Body, Frame, Kind
from swarm_backend.detect.scan import GimbalScan

HOSTILE_ID = 'hostile'


@dataclass(frozen=True)
class Mover:
    """A body flying a straight line at constant speed, then holding at its goal."""
    id: str
    kind: Kind
    start: Vector3
    goal: Vector3
    speed: float  # m/s
    size: float  # metres

    def at(self, t: float) -> Body:
        length = math.dist(self.start, self.goal)
        f = 1.0 if length == 0 else min(1.0, self.speed * max(0.0, t) / length)
        position = tuple(s + f * (g - s) for s, g in zip(self.start, self.goal))
        return Body(self.id, self.kind, position, self.size)


@dataclass(frozen=True)
class DetectScenario:
    seed: int
    base: Vector3  # on the ground
    perimeter_radius: float
    defenders: tuple[Vector3, ...]  # where each defender holds, world NED
    hostile: Mover
    birds: tuple[Mover, ...]

    def bodies(self, t: float) -> list[Body]:
        """Where everything that can be seen is at time t."""
        return [self.hostile.at(t)] + [bird.at(t) for bird in self.birds]

    @property
    def ring_time(self) -> float:
        """Seconds until the hostile drone crosses the ring."""
        outside = _horizontal(self.hostile.start, self.base) - self.perimeter_radius
        return max(0.0, outside) / self.hostile.speed


def make_scenario(seed: int, config: DetectConfig) -> DetectScenario:
    """The seed picks the hostile drone's direction and altitude and the birds."""
    rng = random.Random(seed)
    big_r, n = config.perimeter_radius, config.num_defenders
    defenders = tuple((big_r * math.cos(2 * math.pi * i / n), big_r * math.sin(2 * math.pi * i / n),
                       -config.altitude) for i in range(n))

    def altitude() -> float:
        return -rng.uniform(config.hostile_altitude_min, config.hostile_altitude_max)

    def from_outside(distance: float, z: float) -> Vector3:
        bearing = rng.uniform(-math.pi, math.pi)
        return (distance * math.cos(bearing), distance * math.sin(bearing), z)

    z = altitude()
    hostile = Mover(HOSTILE_ID, Kind.DRONE, from_outside(big_r + config.start_margin, z),
                    (0.0, 0.0, z), config.hostile_speed, config.target_size)
    birds = []
    for i in range(config.num_birds):
        z = altitude()
        start = from_outside(rng.uniform(big_r, big_r + config.start_margin), z)
        goal = from_outside(rng.uniform(0.0, big_r / 2), z)  # across the ring, not at the base
        birds.append(Mover(f'bird{i}', Kind.BIRD, start, goal, config.bird_speed,
                           config.bird_size))
    return DetectScenario(seed=seed, base=(0.0, 0.0, 0.0), perimeter_radius=big_r,
                          defenders=defenders, hostile=hostile, birds=tuple(birds))


def scenario_grid(config: DetectConfig) -> list[DetectScenario]:
    """Seeds 0..eval_runs-1."""
    return [make_scenario(seed, config) for seed in range(config.eval_runs)]


class DetectSim(Protocol):
    """The world and the defenders' cameras: a fast simulation, or Gazebo."""

    def now(self) -> float | None:
        """Seconds of simulation time, None while it is not known."""

    def advance(self, dt: float) -> None:
        """Let `dt` seconds pass."""

    def bodies(self) -> list[Body]:
        """The truth now: the hostile drone (id HOSTILE_ID) and the birds."""

    def frame(self, drone: int, pitch_deg: float, yaw_deg: float) -> Frame:
        """Point defender `drone`'s camera and take an image (with its truth)."""


@dataclass(frozen=True)
class FrameRecord:
    """One image of one defender: the truth, and what the detector reported."""
    t: float  # seconds since the run started
    frame: Frame
    detections: tuple[Detection, ...]
    hostile_distance: float  # metres from this camera to the hostile drone
    hostile_outside: float  # metres the hostile drone is outside the ring (horizontal)


DetectorFactory = Callable[[DetectScenario, int, DetectConfig], ImageDetector]
ScanFactory = Callable[[DetectScenario, DetectConfig], GimbalScan]


def run_scenario(scenario: DetectScenario, sim: DetectSim, make_detector: DetectorFactory,
                 make_scan: ScanFactory, config: DetectConfig) -> list[FrameRecord]:
    """Every defender takes an image at `frame_rate_hz` until the hostile drone reaches
    the ring or `eval_timeout`."""
    scan = make_scan(scenario, config)
    detectors = [make_detector(scenario, i, config) for i in range(len(scenario.defenders))]
    dt = 1.0 / config.frame_rate_hz
    records: list[FrameRecord] = []
    start = None
    while True:
        now = sim.now()
        if now is None:
            sim.advance(dt)
            continue
        if start is None:
            start = now
        t = now - start
        hostile = next(b for b in sim.bodies() if b.id == HOSTILE_ID).position
        outside = _horizontal(hostile, scenario.base) - scenario.perimeter_radius
        for i, (position, detector) in enumerate(zip(scenario.defenders, detectors)):
            frame = sim.frame(i, *scan.aim(t, i, position))
            records.append(FrameRecord(
                t=t, frame=frame, detections=tuple(detector.detect(frame)),
                hostile_distance=math.dist(frame.camera, hostile), hostile_outside=outside))
        if outside <= 0 or t >= config.eval_timeout:
            return records
        sim.advance(dt)


def _horizontal(a: Vector3, b: Vector3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
