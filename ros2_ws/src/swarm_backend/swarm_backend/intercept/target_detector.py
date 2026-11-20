"""What the interceptor's own camera reports about the target, for the evaluation
harness (intercept/scenario.py). The attack's detection topics and fake detector are in
intercept/detection.py.

A Detector is a TargetSource (fixes for the intercept action) that also says, for the
last image, whether the target was in the camera's view and whether the detector
confirmed it as a drone. The evaluation needs both for lock-on (k of n images).
inbox_detector.py implements it from the camera detector's topics. No ROS here.
"""
from abc import abstractmethod

from swarm_backend.intercept.target import PositionFix, TargetSource


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
