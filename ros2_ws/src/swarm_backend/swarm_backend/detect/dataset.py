"""Evaluation on labelled images, outside any scenario (planning report, Drone
Detection Using Sensors > Training Data and Evaluation): recall versus target size,
background false alarms and inference time per image.

Images are held-out Gazebo images, each a Frame whose `objects` are its labels. They
are split into training, validation and test sets by simulation run, never by image,
since images of one run are nearly identical. No ROS.
"""
import random
import statistics
import time
from abc import ABC, abstractmethod
from bisect import bisect_right
from collections.abc import Callable, Iterable, Iterator, Sequence
from dataclasses import dataclass

from swarm_backend.config import DetectConfig
from swarm_backend.detect.detector import ImageDetector
from swarm_backend.detect.frame import Frame, Kind
from swarm_backend.detect.metrics import match


@dataclass(frozen=True)
class LabelledImage:
    run: str  # the simulation run it came from
    frame: Frame  # image, camera pose and labels (`objects`)


class ImageDataset(ABC):

    @abstractmethod
    def __iter__(self) -> Iterator[LabelledImage]:
        ...

    @abstractmethod
    def __len__(self) -> int:
        ...


class GazeboImageDataset(ImageDataset):
    """The images and labels written by the Gazebo data-collection script (drones and
    birds at random distances, altitudes, backgrounds and lighting; boxes from the true
    positions or the bounding-box camera).

    Placeholder: the format is decided with the data-collection script."""

    def __init__(self, root: str):
        raise NotImplementedError

    def __iter__(self) -> Iterator[LabelledImage]:
        raise NotImplementedError

    def __len__(self) -> int:
        raise NotImplementedError


def split_by_run(images: Iterable[LabelledImage], fractions: Sequence[float] = (0.7, 0.15, 0.15),
                 seed: int = 0) -> list[list[LabelledImage]]:
    """Training, validation and test sets (or as many as `fractions`), with all images
    of a run in the same set. The runs are shuffled with `seed`."""
    images = list(images)
    runs = sorted({image.run for image in images})
    random.Random(seed).shuffle(runs)
    total = sum(fractions)
    cuts, acc = [], 0.0
    for fraction in fractions:
        acc += fraction
        cuts.append(round(len(runs) * acc / total))
    group = {}
    for index, run in enumerate(runs):
        group[run] = next(k for k, cut in enumerate(cuts) if index < cut)
    sets: list[list[LabelledImage]] = [[] for _ in fractions]
    for image in images:
        sets[group[image.run]].append(image)
    return sets


@dataclass(frozen=True)
class DatasetMetrics:
    images: int
    # (lo, hi, drones, found, classified) per size interval [lo, hi) px: found = some
    # detection on it (any stage), classified = detected as a drone.
    size_bins: tuple[tuple[float, float, int, int, int], ...]
    background_images: int  # images with no drone or bird
    background_false_alarms: int  # drone detections in them
    inference_ms: tuple[float, ...]  # wall time of detect() per image


def evaluate_dataset(images: Iterable[LabelledImage], detector: ImageDetector,
                     config: DetectConfig,
                     clock: Callable[[], float] = time.perf_counter) -> DatasetMetrics:
    """Every image on its own: the detector is reset before each, so methods that need
    k consecutive images to call something a drone only report candidates here."""
    edges = list(config.size_bins_px)
    counts = [[0, 0, 0] for _ in edges]
    count = background = alarms = 0
    times = []
    for image in images:
        count += 1
        detector.reset()
        start = clock()
        detections = detector.detect(image.frame)
        times.append(1000 * (clock() - start))
        objects = image.frame.objects
        found = match(detections, objects, config.match_radius_px)
        for obj in objects:
            if obj.kind != Kind.DRONE:
                continue
            k = max(0, bisect_right(edges, obj.size_px) - 1)
            counts[k][0] += 1
            counts[k][1] += any(o is obj for o in found)
            counts[k][2] += any(o is obj and d.is_drone for d, o in zip(detections, found))
        if not objects:
            background += 1
            alarms += sum(d.is_drone for d in detections)
    his = edges[1:] + [float('inf')]
    return DatasetMetrics(
        images=count,
        size_bins=tuple((lo, hi, *c) for lo, hi, c in zip(edges, his, counts)),
        background_images=background, background_false_alarms=alarms,
        inference_ms=tuple(times))


def summarize_dataset(metrics: DatasetMetrics, title: str = 'Detection on images') -> str:
    times = sorted(metrics.inference_ms)
    lines = [f'{title}: {metrics.images} images',
             '  inference time: ' + ('-' if not times else
                                      f'mean {statistics.mean(times):.1f} ms, '
                                      f'p95 {times[min(len(times) - 1, int(0.95 * len(times)))]:.1f} ms'),
             '  background false alarms: ' + ('-' if not metrics.background_images else
                                               f'{metrics.background_false_alarms / metrics.background_images:.4f}'
                                               f' per image ({metrics.background_images} images)'),
             '  recall vs target size:',
             f'    {"size [px]":>10} {"drones":>7} {"found":>6} {"classified":>11}']
    for lo, hi, drones, found, classified in metrics.size_bins:
        label = f'{lo:g}-' + ('' if hi == float('inf') else f'{hi:g}')
        lines.append(f'    {label:>10} {drones:>7} {_pct(found, drones):>6} '
                     f'{_pct(classified, drones):>11}')
    return '\n'.join(lines)


def _pct(part: int, whole: int) -> str:
    return '-' if not whole else f'{100 * part / whole:.0f} %'
