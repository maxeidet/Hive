"""What the interceptor's own camera reports about the target.

A Detector is a TargetSource (fixes for the intercept action) that also says, for the
last image, whether the target was in the camera's view and whether the detector
confirmed it as a drone. The evaluation needs both for lock-on (k of n images). YOLO
on the gimbal camera will be one implementation; until then NoisyTruthDetector stands
in with the true position plus noise. No ROS here.
"""
import random
from abc import abstractmethod
from collections.abc import Callable

from swarm_backend.config import InterceptConfig, Vector3
from swarm_backend.intercept.target import PositionFix, TargetSource
from swarm_backend.px4.drone import Drone


class Detector(TargetSource):

    @abstractmethod
    def update(self, now: float) -> None:
        """Process the image (or truth) for time `now`; call once per step."""

    @abstractmethod
    def latest(self) -> PositionFix | None:
        """The most recent detection as a world position, or None if there has not been one."""

    @property
    @abstractmethod
    def in_view(self) -> bool:
        """The target was inside the camera's field of view in the last image."""

    @property
    @abstractmethod
    def detected(self) -> bool:
        """The detector confirmed the target as a drone in the last image."""


class NoisyTruthDetector(Detector):
    """Stand-in for YOLO: the target's true position with Gaussian noise (bearing
    `detect_bearing_std_deg`, range `detect_range_std_frac` x range), reported only when
    the target is within `camera_fov_deg` / 2 of where `observer`'s gimbal looks and
    within `detect_range`. `truth` returns the target's true position.

    Placeholder: to be replaced by the YOLO detector."""

    def __init__(self, observer: Drone, truth: Callable[[], Vector3], config: InterceptConfig,
                 rng: random.Random):
        raise NotImplementedError

    def update(self, now: float) -> None:
        raise NotImplementedError

    def latest(self) -> PositionFix | None:
        raise NotImplementedError

    @property
    def in_view(self) -> bool:
        raise NotImplementedError

    @property
    def detected(self) -> bool:
        raise NotImplementedError
