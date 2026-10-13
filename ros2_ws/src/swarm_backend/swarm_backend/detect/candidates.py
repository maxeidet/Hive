"""Baseline detector: a small dark object against the sky.

Pixels that are clearly darker than the sky around them are grouped into connected regions,
and regions of a plausible size that are surrounded by sky are reported as candidates. It
needs no training, but it cannot tell a drone from a bird and it only works against the sky.
Pure image code (NumPy and OpenCV), no ROS.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Settings:
    kernel: int = 41  # px, black-hat window: targets wider than this are not found
    contrast: float = 60.0  # grey levels (0-255) darker than the surrounding sky
    max_darkest: int = 100  # its darkest pixel is at most this grey level: blue gaps
    # between white clouds are also darker than their surroundings, but still bright
    join: int = 5  # px, dark pixels this close together are one object
    min_area: int = 3  # px
    max_size: int = 40  # px, the longer side of the box
    max_aspect: float = 4.0  # longer side / shorter side
    sky_brightness: int = 120  # grey level above which a pixel can be sky
    sky_blue_margin: int = 10  # sky is not greener or redder than blue by more than this
    ring: int = 6  # px around a candidate that must be sky
    ring_sky: float = 0.9  # share of that ring that must be sky


@dataclass(frozen=True)
class Candidate:
    x: int  # box, top-left corner, px
    y: int
    width: int
    height: int
    area: int  # dark pixels in the region
    contrast: float  # mean grey levels darker than the surrounding sky
    darkest: int  # grey level of its darkest pixel

    @property
    def centre(self) -> tuple[float, float]:
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)


def sky_mask(rgb: np.ndarray, settings: Settings = Settings()) -> np.ndarray:
    """True where the pixel looks like sky: bright, and blue or grey-white rather than
    green (grass) or dark (trees, the drone's own arms)."""
    r, g, b = (rgb[..., i].astype(np.int16) for i in range(3))
    grey = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    margin = settings.sky_blue_margin
    return (grey > settings.sky_brightness) & (b + margin >= g) & (b + margin >= r)


def find_candidates(rgb: np.ndarray, settings: Settings = Settings()) -> list[Candidate]:
    """Small dark regions surrounded by sky in an RGB image (height x width x 3, uint8),
    strongest contrast first."""
    grey = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (settings.kernel, settings.kernel))
    # Black-hat: how much darker each pixel is than its surroundings, for dark things
    # narrower than the kernel. Slow brightness changes across the sky give zero.
    darker = cv2.morphologyEx(grey, cv2.MORPH_BLACKHAT, kernel)
    dark = (darker > settings.contrast).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((settings.join, settings.join), np.uint8))
    sky = sky_mask(rgb, settings).astype(np.uint8)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(dark, connectivity=8)
    ring_kernel = np.ones((2 * settings.ring + 1, 2 * settings.ring + 1), np.uint8)
    height, width = grey.shape
    candidates = []
    for label in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[label])
        if area < settings.min_area or max(w, h) > settings.max_size:
            continue
        if max(w, h) / min(w, h) > settings.max_aspect:
            continue
        if x == 0 or y == 0 or x + w == width or y + h == height:
            continue  # cut off by the image edge, e.g. our own propeller: size unknown
        # The ring of pixels around the region must be sky, which drops tree tops, the
        # horizon and the tips of the drone's own propellers.
        x0, y0 = max(0, x - settings.ring), max(0, y - settings.ring)
        x1, y1 = min(width, x + w + settings.ring), min(height, y + h + settings.ring)
        region = (labels[y0:y1, x0:x1] == label).astype(np.uint8)
        ring = cv2.dilate(region, ring_kernel) - region
        ring_pixels = int(ring.sum())
        if ring_pixels == 0:
            continue
        if (ring * sky[y0:y1, x0:x1]).sum() / ring_pixels < settings.ring_sky:
            continue
        contrast = float(darker[y0:y1, x0:x1][region == 1].mean())
        darkest = int(grey[y0:y1, x0:x1][region == 1].min())
        if darkest > settings.max_darkest:
            continue
        candidates.append(Candidate(x, y, w, h, area, contrast, darkest))
    return sorted(candidates, key=lambda c: -c.contrast)

