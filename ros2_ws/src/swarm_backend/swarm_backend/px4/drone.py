"""What a drone exposes to the rest of the backend.

Swarm, formation and mission code only use this interface, never ROS topics or PX4
messages. All positions are in the swarm world frame: NED (x north, y east, z down)
with its origin at the Gazebo world origin, so z = -5 is 5 m above the ground.

Commands are requests, not blocking calls: `arm()` returns immediately and the
implementation keeps retrying until the drone reports it is armed. Code that needs
to wait polls the state (see swarm_backend.runner.wait_until).
"""
import math
from abc import ABC, abstractmethod

from swarm_backend.config import Vector3


class Drone(ABC):

    def __init__(self, drone_id: int, home: Vector3 = (0.0, 0.0, 0.0)):
        self.id = drone_id
        self.home = home  # world position where the drone started (on the ground)

        self.position: Vector3 | None = None  # world NED, metres
        self.velocity: Vector3 | None = None  # NED, m/s
        self.attitude: tuple[float, float, float, float] | None = None  # quaternion w, x, y, z

        self.connected = False  # heard from the autopilot at least once
        self.preflight_ok = False  # autopilot would accept an arm command
        self.armed = False
        self.offboard = False  # following our setpoints
        self.landing = False  # executing a land command

        self.target: Vector3 | None = None  # last position requested with set_position

    # --- commands ----------------------------------------------------------

    @abstractmethod
    def arm(self) -> None:
        ...

    @abstractmethod
    def disarm(self) -> None:
        ...

    @abstractmethod
    def start_offboard(self) -> None:
        """Hand control to this backend's setpoints."""

    @abstractmethod
    def land(self) -> None:
        """Land where the drone is. The autopilot disarms once it touches down."""

    @abstractmethod
    def set_position(self, x: float, y: float, z: float, yaw: float | None = None) -> None:
        """Fly to a world NED position. yaw is radians from north; None keeps the last one."""

    @abstractmethod
    def set_velocity(self, vx: float, vy: float, vz: float) -> None:
        """Fly at a NED velocity until told otherwise."""

    def takeoff(self, altitude: float) -> None:
        """Climb to `altitude` metres above the start point, straight up."""
        x, y, _ = self.position or self.home
        self.set_position(x, y, self.home[2] - altitude)

    def hold(self) -> None:
        """Stop and hover where the drone is now."""
        if self.position is not None:
            self.set_position(*self.position)

    # --- state -------------------------------------------------------------

    @property
    def ready(self) -> bool:
        """Connected, position known and pre-flight checks passing: safe to arm."""
        return self.connected and self.position is not None and self.preflight_ok

    @property
    def altitude(self) -> float | None:
        """Metres above the start point."""
        return None if self.position is None else self.home[2] - self.position[2]

    def distance_to(self, point: Vector3) -> float:
        if self.position is None:
            return math.inf
        return math.dist(self.position, point)

    def reached(self, tolerance: float) -> bool:
        """True when the drone is within `tolerance` metres of its position target."""
        return self.target is not None and self.distance_to(self.target) < tolerance

    def __repr__(self) -> str:
        return f'{type(self).__name__}(id={self.id})'
