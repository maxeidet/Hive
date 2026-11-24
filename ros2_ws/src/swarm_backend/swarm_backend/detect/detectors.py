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
from pathlib import Path

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
    """YOLO trained on the Gazebo images (training/), exported to ONNX (yolo_onnx.py).
    Score = confidence. Only class `drone` is returned unless --yolo-birds; the mask drops
    detections whose centre is outside it."""
    from swarm_backend.detect.yolo_onnx import YoloOnnx

    path = Path(args.weights) if args.weights else newest_weights()
    if path is None or not path.exists():
        raise NotImplementedError(
            f'No YOLO weights at {path or MODELS / "*.onnx"}. Train and export one '
            '(training/README.md), put the .onnx in models/ in the repo, or give '
            '--weights. Use --detector baseline meanwhile.')
    try:
        model = YoloOnnx(path, args.imgsz)
    except Exception as e:  # OpenCV's ONNX reader can't read every model
        raise NotImplementedError(
            f'Could not load {path}: {e}\nOpenCV {cv2.__version__} may be too old for this '
            'model: add onnxruntime to the container, or export with opset=12.') from e
    names = {0: 'drone', 1: 'bird'}
    print(f'YOLO: {path} ({model.backend}, {model.imgsz} px, confidence >= {args.conf})')

    def detect(rgb: np.ndarray, mask: np.ndarray | None = None) -> list[Detection]:
        found = []
        for b in model(rgb, args.conf):
            label = names.get(b.cls, str(b.cls))
            if label != 'drone' and not args.yolo_birds:
                continue
            x, y = int(b.x1), int(b.y1)
            d = Detection(x, y, max(1, round(b.x2) - x), max(1, round(b.y2) - y), b.score, label)
            if mask is not None and not mask[min(int(d.v), mask.shape[0] - 1),
                                             min(int(d.u), mask.shape[1] - 1)]:
                continue
            found.append(d)
        return sorted(found, key=lambda d: -d.score)
    return detect


MODELS = Path(__file__).resolve().parents[5] / 'models'  # the repo's models/


def newest_weights() -> Path | None:
    """The most recently changed .onnx anywhere under the repo's models/."""
    found = list(MODELS.rglob('*.onnx')) if MODELS.is_dir() else []
    return max(found, key=lambda p: p.stat().st_mtime, default=None)


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
    group = parser.add_argument_group('yolo detector')
    group.add_argument('--weights', help='the .onnx file (default: the newest .onnx in models/)')
    group.add_argument('--conf', type=float, default=0.25,
                       help='lowest confidence kept (default %(default)s)')
    group.add_argument('--imgsz', type=int,
                       help='network input size, px (default: from the model, else 1280)')
    group.add_argument('--yolo-birds', action='store_true',
                       help='also return birds (label "bird"), not only drones')


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
