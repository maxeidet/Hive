"""The contract between a detection method and the evaluation harness.

A detector runs on one defender's camera: detect() gets every image in order and
returns what it sees, as the detection report of the planning report (Drone Detection
Using Sensors > Interface). It may keep state between images (tracking, the k
consecutive images before a candidate counts as a drone); reset() clears it. The
dark-object baseline and YOLO26 plug in by subclassing. No ROS here.
"""
import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from swarm_backend.config import DetectConfig, Vector3
from swarm_backend.detect.camera import CameraModel
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

    Placeholder: wraps the existing baseline, detect.candidates (sky_mask,
    find_candidates; horizon from detect.camera.CameraModel.horizon_row) and
    detect.tracking.Confirmer, as detect.live and detect.predict use them."""

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

    Below `classify_px` nothing can be told apart, so a found bird also counts as a
    drone after `confirm_k` images, as with the dark-object baseline. Until then a found
    object is a CANDIDATE without a kind. The streak is kept per true object (a
    stand-in may track by identity) and broken by any image it is not found in."""

    PIXEL_STD = 0.5  # px, noise of the reported centre

    def __init__(self, config: DetectConfig, rng: random.Random):
        self.config = config
        self.rng = rng
        self.camera = CameraModel(config.camera_hfov, config.image_width, config.image_height)
        self.streaks: dict[str, int] = {}

    def reset(self) -> None:
        self.streaks = {}

    def detect(self, frame: Frame) -> list[Detection]:
        config = self.config
        found = [o for o in frame.objects if o.size_px >= config.detect_px]
        self.streaks = {o.id: self.streaks.get(o.id, 0) + 1 for o in found}
        detections = []
        for o in found:
            u = o.pixel[0] + self.rng.gauss(0.0, self.PIXEL_STD)
            v = o.pixel[1] + self.rng.gauss(0.0, self.PIXEL_STD)
            classified = o.size_px >= config.classify_px
            confirmed = self.streaks[o.id] >= config.confirm_k
            if confirmed:
                stage, kind = Stage.CLASSIFIED, (o.kind if classified else Kind.DRONE)
            else:
                stage, kind = Stage.CANDIDATE, None
            detections.append(Detection(
                time=frame.time, drone=frame.drone, pixel=(u, v), box_px=(o.size_px, o.size_px),
                stage=stage, kind=kind, confidence=1.0 if classified else config.confidence_threshold,
                track=None,
                direction=self.camera.direction(u, v, frame.forward, frame.right, frame.down),
                range=self.camera.range_from_width(o.size_px, config.target_size)))
        return detections
