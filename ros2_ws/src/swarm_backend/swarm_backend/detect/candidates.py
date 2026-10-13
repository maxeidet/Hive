"""Baseline detector: small dark objects against the sky.

The sky is bright and smooth; a drone or bird far away is a small dark blob. Per image:
1. darkness = min(R, G, B), since the sky is bright in blue and a drone is dark in every
   channel
2. black-hat morphology (closing - image) with a kernel larger than the targets: keeps
   small dark things and drops slow brightness changes across the sky (gradients, clouds)
3. threshold at mean + k*std of the black-hat over the searched area, never below
   `min_contrast` (Gazebo's sky has almost no noise, so std alone would be ~0)
4. connected regions, kept if their size and shape are plausible for a target
5. regions that are part of a bigger dark object are dropped: the black-hat leaves small
   fragments along the edges of anything larger than its kernel (a tree, a near drone)
Only works against the sky: pass a mask that is False below the horizon (above_line).
"""
from dataclasses import dataclass

import cv2
import numpy as np

LARGE_FACTOR = 4  # see large_dark_objects


@dataclass(frozen=True)
class CandidateConfig:
    kernel: int = 15  # px, black-hat kernel; about twice the largest target looked for
    k_sigma: float = 4.0  # threshold = mean + k_sigma * std of the black-hat
    min_contrast: float = 12.0  # grey levels; the threshold never goes below this
    min_area: int = 3  # px^2
    max_aspect: float = 4.0  # long side / short side
    max_candidates: int = 20  # highest contrast first

    @property
    def max_size(self) -> int:
        """Largest box side kept. Bigger objects are hollowed out by the black-hat."""
        return self.kernel


@dataclass(frozen=True)
class Box:
    u: float  # centre, px from the left (centroid of the region)
    v: float  # centre, px from the top
    width: int  # px
    height: int  # px
    area: int  # px^2 above the threshold
    contrast: float  # peak black-hat value, grey levels

    @property
    def corners(self) -> tuple[int, int, int, int]:
        """(x0, y0, x1, y1) of the box, rounded to whole pixels."""
        x0 = round(self.u - self.width / 2)
        y0 = round(self.v - self.height / 2)
        return x0, y0, x0 + self.width, y0 + self.height


def darkness_image(rgb: np.ndarray) -> np.ndarray:
    """Grey image where a target is dark: min over the colour channels."""
    if rgb.ndim == 2:
        return rgb
    return rgb.min(axis=2)


def above_line(shape: tuple[int, ...], v_left: float, v_right: float | None = None) -> np.ndarray:
    """Mask that is True above the line from (0, v_left) to (width - 1, v_right).

    For the horizon: the camera math gives its row at the image edges (they differ when the
    camera is rolled). v_right defaults to v_left, a level horizon.
    """
    height, width = shape[:2]
    if v_right is None:
        v_right = v_left
    line = np.linspace(v_left, v_right, width)
    return np.arange(height)[:, None] < line[None, :]


def black_hat(grey: np.ndarray, kernel: int) -> np.ndarray:
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel, kernel))
    return cv2.morphologyEx(grey, cv2.MORPH_BLACKHAT, element)


def large_dark_objects(grey: np.ndarray, threshold: float, config: CandidateConfig) -> np.ndarray:
    """True on dark objects bigger than config.max_size.

    A black-hat with a kernel LARGE_FACTOR times bigger, run at 1/LARGE_FACTOR of the
    resolution (so it costs the same as the small one), sees these objects whole.
    """
    height, width = grey.shape
    small = cv2.resize(grey, (max(1, width // LARGE_FACTOR), max(1, height // LARGE_FACTOR)),
                       interpolation=cv2.INTER_AREA)
    dark = (black_hat(small, config.kernel) > threshold).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(dark, connectivity=8)
    big = np.zeros(count, bool)
    sides = np.maximum(stats[:, cv2.CC_STAT_WIDTH], stats[:, cv2.CC_STAT_HEIGHT])
    # + LARGE_FACTOR: rounding at the low resolution can make a target one pixel bigger
    big[1:] = sides[1:] * LARGE_FACTOR > config.max_size + LARGE_FACTOR
    return cv2.resize(big[labels].astype(np.uint8), (width, height),
                      interpolation=cv2.INTER_NEAREST).astype(bool)


def find_candidates(rgb: np.ndarray, mask: np.ndarray | None = None,
                    config: CandidateConfig = CandidateConfig()) -> list[Box]:
    """Small dark objects in an RGB (or grey) uint8 image, highest contrast first.

    mask: True where to look (the sky); None looks everywhere.
    """
    grey = darkness_image(rgb)
    hat = black_hat(grey, config.kernel)
    if mask is not None:
        hat = np.where(mask, hat, 0).astype(hat.dtype)
        values = hat[mask]
    else:
        values = hat.ravel()
    if values.size == 0:
        return []
    threshold = max(config.min_contrast, float(values.mean() + config.k_sigma * values.std()))
    binary = (hat > threshold).astype(np.uint8)

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count > 1:
        large = large_dark_objects(grey, threshold, config)
    boxes = []
    for i in range(1, count):  # 0 is the background
        x, y, w, h, area = (int(s) for s in stats[i])
        if area < config.min_area or max(w, h) > config.max_size:
            continue
        if max(w, h) / min(w, h) > config.max_aspect:
            continue
        inside = labels[y:y + h, x:x + w] == i
        if large[y:y + h, x:x + w][inside].any():
            continue
        region = hat[y:y + h, x:x + w][inside]
        u, v = centroids[i]
        boxes.append(Box(float(u), float(v), w, h, area, float(region.max())))
    boxes.sort(key=lambda b: b.contrast, reverse=True)
    return boxes[:config.max_candidates]


def draw(rgb: np.ndarray, boxes: list[Box], color=(255, 0, 0), labels: list[str] | None = None,
         margin: int = 6) -> np.ndarray:
    """Copy of the image with a rectangle (and optional label) around each box.

    The rectangle is `margin` px bigger than the box on every side, so a 4 px target stays
    visible inside it.
    """
    out = np.ascontiguousarray(rgb.copy())
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2RGB)
    for i, box in enumerate(boxes):
        x0, y0, x1, y1 = box.corners
        cv2.rectangle(out, (x0 - margin, y0 - margin), (x1 + margin, y1 + margin), color, 1)
        if labels is not None:
            cv2.putText(out, labels[i], (x0 - margin, y0 - margin - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    return out
