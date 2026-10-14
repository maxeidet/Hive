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
        # Autopilot clock at the last state update, seconds. In SITL this is simulation
        # time, so code timed by it keeps pace when Gazebo runs slower than real time.
        self.stamp: float | None = None

        self.connected = False  # heard from the autopilot at least once
        self.preflight_ok = False  # autopilot would accept an arm command
        self.armed = False
        self.offboard = False  # following our setpoints
        self.landing = False  # executing a land command

        self.target: Vector3 | None = None  # last position requested with set_position
        # Camera gimbal (pitch, yaw) in degrees: pitch 0 level, -90 down; yaw relative to
        # the nose. None when the drone has no gimbal (or has not reported it yet).
        self.gimbal: tuple[float, float] | None = None

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
    def set_position(self, x: float, y: float, z: float, yaw: float | None = None,
                     velocity: Vector3 | None = None) -> None:
        """Fly to a world NED position. yaw is radians from north; None keeps the last one.

        `velocity` (NED, m/s) is a feed-forward for a moving setpoint: the drone keeps
        this velocity at the point instead of stopping there."""

    @abstractmethod
    def set_velocity(self, vx: float, vy: float, vz: float, yaw: float | None = None) -> None:
        """Fly at a NED velocity until told otherwise. yaw is radians from north; None
        keeps the last one."""

    def point_gimbal(self, pitch: float, yaw: float) -> None:
        """Point the camera gimbal, degrees (see `gimbal`). Only drones with a gimbal."""
        raise NotImplementedError(f'{type(self).__name__} has no gimbal')

    def aim_gimbal(self, pitch: float, yaw: float) -> None:
        """Keep the gimbal pointed at (pitch, yaw) degrees, re-sent continuously so it can
        follow a moving target. Pitch is from the horizon, so it does not tilt with the
        drone. Stays on until release_gimbal()."""
        raise NotImplementedError(f'{type(self).__name__} has no gimbal')

    def release_gimbal(self) -> None:
        """Stop the aim_gimbal stream; the gimbal stays where it was last pointed."""

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


def pitch_yaw_deg(q: tuple[float, float, float, float]) -> tuple[float, float]:
    """Pitch and yaw (degrees, ZYX Euler) of a quaternion w, x, y, z."""
    w, x, y, z = q
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(pitch), math.degrees(yaw)


def wrap_deg(angle: float) -> float:
    """Angle in (-180, 180]."""
    return -((-angle + 180.0) % 360.0 - 180.0)
