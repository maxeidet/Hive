"""Stand-ins for the parts of the scenario we do not have yet: a hostile drone flying a
pattern, and what the base station's radar measures. No ROS here.
"""
import math
import random
from collections.abc import Callable

from swarm_backend.config import InterceptConfig, Vector3
from swarm_backend.intercept.paths import Path
from swarm_backend.intercept.target import RadarFix
from swarm_backend.px4.drone import Drone


class HostileFlight:
    """Flies a real (PX4) drone along a Path: call step() at the setpoint rate."""

    def __init__(self, drone: Drone, path: Path, clock: Callable[[], float | None]):
        self.drone = drone
        self.path = path
        self.clock = clock
        self.start_time: float | None = None
        self.running = False

    def start(self) -> None:
        self.start_time = None  # taken from the first step that knows the time
        self.running = True

    def stop(self) -> None:
        if self.running:
            self.running = False
            self.drone.hold()

    def step(self) -> None:
        now = self.clock()
        if not self.running or now is None:
            return
        if self.start_time is None:
            self.start_time = now
        (x, y, z), velocity = self.path.at(now - self.start_time)
        yaw = math.atan2(velocity[1], velocity[0])
        self.drone.set_position(x, y, z, yaw=yaw, velocity=velocity)


def radar_fix(true_position: Vector3, true_velocity: Vector3, config: InterceptConfig,
              rng: random.Random, time: float) -> RadarFix:
    """What the mock radar at config.radar_position measures: the true range, azimuth,
    elevation and radial velocity, each with Gaussian noise of the configured sigma."""
    los = [p - r for p, r in zip(true_position, config.radar_position)]
    rng_true = math.hypot(*los)
    if rng_true == 0:
        raise ValueError('The target is at the radar')
    azimuth = math.atan2(los[1], los[0])
    elevation = math.asin(-los[2] / rng_true)  # NED: up is -z
    radial_velocity = sum(v * d for v, d in zip(true_velocity, los)) / rng_true
    std = radar_std(config)
    return RadarFix(
        time=time, source='radar', radar=config.radar_position,
        range=rng_true + rng.gauss(0, std['range']),
        azimuth=azimuth + rng.gauss(0, std['azimuth']),
        elevation=elevation + rng.gauss(0, std['elevation']),
        radial_velocity=radial_velocity + rng.gauss(0, std['radial_velocity']),
        range_std=std['range'], azimuth_std=std['azimuth'], elevation_std=std['elevation'],
        radial_velocity_std=std['radial_velocity'])


def radar_std(config: InterceptConfig) -> dict:
    """The radar's 1-sigma errors, angles in radians."""
    return {'range': config.radar_range_std,
            'azimuth': math.radians(config.radar_azimuth_std_deg),
            'elevation': math.radians(config.radar_elevation_std_deg),
            'radial_velocity': config.radar_radial_velocity_std}
