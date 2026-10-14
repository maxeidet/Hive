"""The contract between an intercept method and the evaluation harness.

The harness (intercept/scenario.py) builds an InterceptAction, calls step() at the
setpoint rate and reads `status`; it never looks inside. Pure pursuit is the baseline;
proportional navigation, the MPC and anything else plug in by subclassing. No ROS here:
an action only uses the Drone interface and a Detector.
"""
from abc import ABC, abstractmethod
from collections.abc import Callable
from enum import Enum

from swarm_backend.config import InterceptConfig
from swarm_backend.intercept.detection import Detector
from swarm_backend.px4.drone import Drone


class Status(Enum):
    IDLE = 'idle'  # not started, or stopped
    TRACKING = 'tracking'  # has a fresh fix and is steering toward the target
    LOST = 'lost'  # no fresh fix; the harness ends the run once the target was seen


class InterceptAction(ABC):

    def __init__(self, drone: Drone, detector: Detector, config: InterceptConfig,
                 clock: Callable[[], float | None]):
        """`clock` returns seconds on the clock the detector stamps its fixes with
        (None while it is not known yet)."""
        self.drone = drone
        self.detector = detector
        self.config = config
        self.clock = clock

    @abstractmethod
    def start(self) -> None:
        """Begin steering on the next step()."""

    @abstractmethod
    def stop(self) -> None:
        """Hover where we are."""

    @abstractmethod
    def step(self) -> None:
        """Read the detector and send the drone one setpoint (and the gimbal an aim)."""

    @property
    @abstractmethod
    def status(self) -> Status:
        """What the action is doing now."""


class PurePursuitAction(InterceptAction):
    """Baseline (planning report, Interception > Guidance Laws): fly at full speed
    (`interceptor_max_speed`) toward the target's current position with
    guidance.pure_pursuit_velocity, gimbal on the target; LOST when the detector has
    had no fix for `track_timeout` seconds.

    Placeholder: the intercept guidance is being written elsewhere."""

    def __init__(self, drone: Drone, detector: Detector, config: InterceptConfig,
                 clock: Callable[[], float | None]):
        raise NotImplementedError

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def step(self) -> None:
        raise NotImplementedError

    @property
    def status(self) -> Status:
        raise NotImplementedError
