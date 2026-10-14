"""The detectors, behind one interface, so they can be swapped with one option.

A detector takes an RGB image (and optionally a mask of where to look) and returns a list
of Detection: a box, a score and a class. Everything after it (tracking, the offset angles,
distance and direction, the published topics, `predict`'s comparison with the truth) only
uses those, so it works the same whichever detector runs:

    ./mount-backend.sh detect 2 --detector yolo      live, on drone 2's camera
    ./mount-backend.sh predict yolo                  on saved images

To add a detector: write a function that builds it from the parsed command-line options
and add it to DETECTORS. No ROS here.
"""
import argparse
from collections.abc import Callable
from dataclasses import dataclass

import cv2
import numpy as np

from swarm_backend.detect import candidates


@dataclass(frozen=True)
class Detection:
    x: int  # box, top-left corner, px
    y: int
    width: int
    height: int
    score: float  # how sure the detector is; only compared between detections of one detector
    label: str = 'drone'

    @property
    def u(self) -> float:
        """Centre, px from the left."""
        return self.x + self.width / 2.0

    @property
    def v(self) -> float:
        """Centre, px from the top."""
        return self.y + self.height / 2.0

    @property
    def centre(self) -> tuple[float, float]:
        return (self.u, self.v)

    @property
    def size(self) -> int:
        """The box's longer side, px: what the distance is estimated from."""
        return max(self.width, self.height)


# image (RGB, uint8), mask (True where to look) or None -> detections, highest score first
Detector = Callable[[np.ndarray, np.ndarray | None], list[Detection]]


def baseline(args: argparse.Namespace) -> Detector:
    """Dark object against the sky (candidates.py). Score = contrast in grey levels."""
    settings = candidates.Settings(kernel=args.kernel, contrast=args.contrast,
                                   round_kernel=args.round_kernel)

    def detect(rgb: np.ndarray, mask: np.ndarray | None = None) -> list[Detection]:
        return [Detection(c.x, c.y, c.width, c.height, c.contrast)
                for c in candidates.find_candidates(rgb, mask, settings)]
    return detect


def yolo(args: argparse.Namespace) -> Detector:
    """YOLO26 (not trained yet). Score = confidence; only class `drone` is returned."""
    raise NotImplementedError(
        'The YOLO detector is not here yet: it needs trained weights first (see "Detection '
        'implementation plan" in the vault, steps 6-8). Use --detector baseline for now.')


DETECTORS: dict[str, Callable[[argparse.Namespace], Detector]] = {
    'baseline': baseline,
    'yolo': yolo,
}


def add_options(parser: argparse.ArgumentParser) -> None:
    """The settings of every detector. Which one runs is chosen by the caller (--detector,
    or predict's first argument), with the names in DETECTORS."""
    group = parser.add_argument_group('baseline detector')
    group.add_argument('--kernel', type=int, default=candidates.Settings.kernel,
                       help='px, targets wider than this are not found (default %(default)s)')
    group.add_argument('--round-kernel', action='store_true',
                       help='use the slower round black-hat window of the first version')
    group.add_argument('--contrast', type=float, default=candidates.Settings.contrast,
                       help='grey levels darker than the sky around it (default %(default)s)')


def make(name: str, args: argparse.Namespace) -> Detector:
    """The detector called `name`, built from the parsed options. Raises NotImplementedError
    for one that is not available yet."""
    return DETECTORS[name](args)


def draw(rgb: np.ndarray, detections: list[Detection], color=(255, 0, 0),
         labels: list[str] | None = None, margin: int = 6) -> np.ndarray:
    """Copy of the image with a rectangle (and optional label) around each detection.

    The rectangle is `margin` px bigger than the box on every side, so a 4 px target stays
    visible inside it.
    """
    out = np.ascontiguousarray(rgb.copy())
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2RGB)
    for i, d in enumerate(detections):
        cv2.rectangle(out, (d.x - margin, d.y - margin),
                      (d.x + d.width + margin, d.y + d.height + margin), color, 1)
        if labels is not None:
            cv2.putText(out, labels[i], (d.x - margin, d.y - margin - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    return out
