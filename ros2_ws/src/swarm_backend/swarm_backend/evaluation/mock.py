"""A stand-in for the hostile drone: a PX4 drone flown along a pattern. No ROS here.
"""
import math
from collections.abc import Callable

from swarm_backend.evaluation.paths import Path
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
