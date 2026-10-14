"""The contract between a detection method and the evaluation harness.

A detector runs on one defender's camera: detect() gets every image in order and
returns what it sees, as the detection report of the planning report (Drone Detection
Using Sensors > Interface). It may keep state between images (tracking, the k
consecutive images before a candidate counts as a drone); reset() clears it. The
dark-object baseline and YOLO26 plug in by subclassing. No ROS here.
"""
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from swarm_backend.config import DetectConfig, Vector3
from swarm_backend.detect.frame import Frame, Kind


class Stage(Enum):
    CANDIDATE = 'candidate'  # found by the first step, not classified
    CLASSIFIED = 'classified'  # classified by the model (or confirmed over k images)


@dataclass(frozen=True)
class Detection:
    """One detection report (the report's Table "detection-report")."""
    time: float  # when the image was taken
    drone: int  # which defender saw it
    pixel: tuple[float, float]  # (u, v) of the box centre
    box_px: tuple[float, float]  # (width, height) of the box
    stage: Stage
    kind: Kind | None = None  # None while only a candidate
    confidence: float = 0.0
    track: int | None = None  # identity over time (ByteTrack)
    direction: Vector3 | None = None  # unit vector to the target, world NED
    direction_std_deg: float | None = None
    range: float | None = None  # metres, from the box size
    range_std: float | None = None

    @property
    def is_drone(self) -> bool:
        """Counts as a drone: what starts an intercept."""
        return self.kind == Kind.DRONE


class ImageDetector(ABC):

    @abstractmethod
    def reset(self) -> None:
        """Forget everything seen so far (a new run, or unrelated images)."""

    @abstractmethod
    def detect(self, frame: Frame) -> list[Detection]:
        """The detections in this image."""


class DarkObjectDetector(ImageDetector):
    """Baseline (report, Baseline: Dark Object Against the Sky): groups pixels that
    differ clearly from the surrounding sky into connected regions, reports regions of
    a plausible size as candidates, and reports a candidate seen in `confirm_k`
    consecutive images as a drone. Reads only `frame.image`.

    Placeholder: to be written by the detection team."""

    def __init__(self, config: DetectConfig):
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError

    def detect(self, frame: Frame) -> list[Detection]:
        raise NotImplementedError


class Yolo26Detector(ImageDetector):
    """Improvement (report, Improvement: YOLO26): candidates, then YOLO26n on enlarged
    crops around them, ByteTrack between images; a drone once its confidence is above
    `confidence_threshold` in `confirm_k` consecutive images. Reads only `frame.image`.

    Placeholder: to be written by the detection team."""

    def __init__(self, config: DetectConfig, weights: str):
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError

    def detect(self, frame: Frame) -> list[Detection]:
        raise NotImplementedError


class PixelModelDetector(ImageDetector):
    """Stand-in for the fast simulation, which has no images: reads the truth
    (`frame.objects`) and behaves as the report assumes a camera does. An object at
    least `detect_px` wide is found (with a little pixel noise), one at least
    `classify_px` wide is classified correctly, and a found drone counts as a drone
    after `confirm_k` consecutive images.

    Placeholder: to be replaced by the real detectors when they run in Gazebo."""

    def __init__(self, config: DetectConfig, rng: random.Random):
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError

    def detect(self, frame: Frame) -> list[Detection]:
        raise NotImplementedError
