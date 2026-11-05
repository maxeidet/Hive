"""Fake detections from the truth, with the errors a real detector makes, for building and
testing the tracker (ByteTrack) before a trained detector exists.

The truth is where each target really is in the image: a box per target and image. It
comes from a made-up scenario (`crossing_targets`, no Gazebo needed) or from the truth
saved next to a camera image (`targets_from_truth`, see ground_truth.truth). `detect`
turns one image's truth into what a detector would report, with:

  - noise on the box centre and size,
  - misses (the target is not reported at all),
  - weak detections (reported, but with a low score, as when a target is blurred, small
    or partly hidden; ByteTrack's second matching step is built for exactly these),
  - false detections (clutter: low score, at random places).

Every detection keeps the id of the target it came from (None for false ones), so the
tracker's ids can be compared with the truth. No ROS here.
"""
import math
import random
from dataclasses import dataclass, field

from swarm_backend.detect.detectors import Detection


@dataclass(frozen=True)
class TruthBox:
    """Where one target really is in one image."""
    id: int
    u: float  # centre, px from the left
    v: float  # centre, px from the top
    width: float  # px
    height: float


@dataclass
class Errors:
    """How bad the fake detector is. All zero = a perfect detector."""
    centre_std_px: float = 1.5  # noise on the box centre
    size_std: float = 0.1  # noise on the box size, as a fraction of it
    miss: float = 0.05  # chance a target is not reported in an image
    weak: float = 0.15  # chance a reported target gets a low score
    false_per_image: float = 0.5  # mean number of false detections per image
    score: tuple[float, float] = (0.6, 0.95)  # score range of a normal detection
    weak_score: tuple[float, float] = (0.1, 0.5)  # score range of a weak or false one
    false_size_px: tuple[int, int] = (3, 12)  # size range of a false detection


PERFECT = Errors(centre_std_px=0.0, size_std=0.0, miss=0.0, weak=0.0, false_per_image=0.0)


@dataclass
class FakeImage:
    """One image's fake detections, highest score first, and where each came from."""
    detections: list[Detection]
    sources: list[int | None]  # truth id per detection, None for a false one
    truth: list[TruthBox] = field(default_factory=list)


def detect(truth: list[TruthBox], width: int, height: int, errors: Errors,
           rng: random.Random) -> FakeImage:
    """What a detector with these errors reports for an image whose truth is `truth`."""
    found: list[tuple[Detection, int | None]] = []
    for t in truth:
        if rng.random() < errors.miss:
            continue
        w = max(1, round(t.width * (1 + rng.gauss(0, errors.size_std))))
        h = max(1, round(t.height * (1 + rng.gauss(0, errors.size_std))))
        u = t.u + rng.gauss(0, errors.centre_std_px)
        v = t.v + rng.gauss(0, errors.centre_std_px)
        x, y = round(u - w / 2), round(v - h / 2)
        if x + w <= 0 or y + h <= 0 or x >= width or y >= height:
            continue  # pushed out of the image by the noise
        low, high = errors.weak_score if rng.random() < errors.weak else errors.score
        found.append((Detection(x, y, w, h, round(rng.uniform(low, high), 3)), t.id))

    for _ in range(_poisson(errors.false_per_image, rng)):
        size = rng.randint(*errors.false_size_px)
        x, y = rng.randrange(0, width - size), rng.randrange(0, height - size)
        found.append((Detection(x, y, size, size, round(rng.uniform(*errors.weak_score), 3)),
                      None))

    found.sort(key=lambda pair: -pair[0].score)
    return FakeImage([d for d, _ in found], [s for _, s in found], list(truth))


def run(frames: list[list[TruthBox]], width: int, height: int, errors: Errors = Errors(),
        seed: int = 0) -> list[FakeImage]:
    """Fake detections for a whole sequence of images, repeatable through `seed`."""
    rng = random.Random(seed)
    return [detect(truth, width, height, errors, rng) for truth in frames]


def crossing_targets(images: int = 60, width: int = 1280, height: int = 720,
                     size_px: float = 10.0) -> list[list[TruthBox]]:
    """A made-up scenario: two targets fly towards each other, cross in the middle of the
    image and fly on (the hard case for keeping ids apart), while a third flies slowly
    on its own. Constant velocity, in px per image."""
    starts = [(1, 200.0, 300.0, 15.0, 2.0),  # id, u, v, du, dv
              (2, 1080.0, 360.0, -15.0, -1.0),
              (3, 640.0, 100.0, 1.0, 1.5)]
    frames = []
    for i in range(images):
        boxes = []
        for target_id, u0, v0, du, dv in starts:
            u, v = u0 + du * i, v0 + dv * i
            if 0 <= u < width and 0 <= v < height:
                boxes.append(TruthBox(target_id, u, v, size_px, size_px))
        frames.append(boxes)
    return frames


def targets_from_truth(truth: dict, drone_size_m: float = 0.5) -> list[TruthBox]:
    """The truth boxes in an image from what ground_truth.truth saved with it: every drone
    in view, with a box as big as a drone `drone_size_m` across shows at its distance."""
    camera = truth['camera']
    focal_px = (camera['width'] / 2) / math.tan(camera['hfov_rad'] / 2)
    boxes = []
    for drone in truth['drones']:
        if drone['pixel'] is None:
            continue
        size = max(1.0, focal_px * drone_size_m / drone['distance'])
        boxes.append(TruthBox(drone['id'], drone['pixel'][0], drone['pixel'][1], size, size))
    return boxes


def _poisson(mean: float, rng: random.Random) -> int:
    """A Poisson-distributed count (Knuth's method; the means here are small)."""
    if mean <= 0:
        return 0
    limit, k, p = math.exp(-mean), 0, rng.random()
    while p > limit:
        k += 1
        p *= rng.random()
    return k
