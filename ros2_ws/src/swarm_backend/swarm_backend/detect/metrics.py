"""Scoring of detection runs (planning report, Drone Detection Using Sensors >
Evaluation).

A detection is correct when its centre is within `match_radius_px` of an object's
true centre in the image (box overlap is unstable for targets a few pixels wide).
Everything is scored from the truth in each Frame. Pure math, no ROS.
"""
import statistics
from collections.abc import Sequence
from dataclasses import dataclass

from swarm_backend.config import DetectConfig
from swarm_backend.detect.detector import Detection
from swarm_backend.detect.frame import Kind, TrueObject
from swarm_backend.detect.scenario import HOSTILE_ID, DetectScenario, FrameRecord


def match(detections: Sequence[Detection], objects: Sequence[TrueObject],
          radius_px: float) -> list[TrueObject | None]:
    """The true object each detection found (closest first, each object at most
    once), or None for a false alarm. Same order as `detections`."""
    pairs = sorted(
        (_pixel_distance(d.pixel, o.pixel), i, j)
        for i, d in enumerate(detections) for j, o in enumerate(objects))
    found: list[TrueObject | None] = [None] * len(detections)
    used = set()
    for distance, i, j in pairs:
        if distance > radius_px:
            break
        if found[i] is None and j not in used:
            found[i] = objects[j]
            used.add(j)
    return found


@dataclass(frozen=True)
class DetectRunMetrics:
    seed: int
    missed: bool  # the hostile drone reached the ring without being detected as a drone
    first_detection_time: float | None  # seconds since the run started
    first_detection_range: float | None  # metres from the camera that detected it
    warning_distance: float | None  # metres outside the ring at the first detection
    warning_time: float | None  # seconds until it reaches the ring, at its speed
    hostile_views: tuple[tuple[float, bool], ...]  # (range, detected) per image it was in
    birds_seen: int  # birds that were in some camera's view
    birds_flagged: int  # of those, detected as a drone at least once
    frames: int
    empty_frames: int  # images with no drone or bird in them
    background_false_alarms: int  # drone detections in those images


def score_run(records: Sequence[FrameRecord], scenario: DetectScenario,
              config: DetectConfig) -> DetectRunMetrics:
    first: FrameRecord | None = None
    views = []
    birds_seen, birds_flagged = set(), set()
    empty = background = 0
    for record in records:
        objects = record.frame.objects
        found = match(record.detections, objects, config.match_radius_px)
        flagged = {o.id for d, o in zip(record.detections, found) if o is not None and d.is_drone}
        hostile = next((o for o in objects if o.id == HOSTILE_ID), None)
        if hostile is not None:
            views.append((hostile.distance, HOSTILE_ID in flagged))
        if HOSTILE_ID in flagged and first is None:
            first = record
        birds_seen |= {o.id for o in objects if o.kind == Kind.BIRD}
        birds_flagged |= {o.id for o in objects if o.kind == Kind.BIRD and o.id in flagged}
        if not objects:
            empty += 1
            background += sum(d.is_drone for d in record.detections)
    warning = None if first is None else max(0.0, first.hostile_outside)
    return DetectRunMetrics(
        seed=scenario.seed, missed=first is None,
        first_detection_time=None if first is None else first.t,
        first_detection_range=None if first is None else first.hostile_distance,
        warning_distance=warning,
        warning_time=None if warning is None else warning / scenario.hostile.speed,
        hostile_views=tuple(views), birds_seen=len(birds_seen),
        birds_flagged=len(birds_flagged), frames=len(records), empty_frames=empty,
        background_false_alarms=background)


def detection_by_range(runs: Sequence[DetectRunMetrics],
                       edges: Sequence[float]) -> list[tuple[float, float, int, int]]:
    """Detection probability versus range: per range interval [lo, hi), the number of
    runs whose hostile drone was in view in it, and of those, detected in it."""
    bins = []
    for lo, hi in zip(edges, list(edges[1:]) + [float('inf')]):
        seen = detected = 0
        for run in runs:
            here = [hit for distance, hit in run.hostile_views if lo <= distance < hi]
            seen += bool(here)
            detected += any(here)
        bins.append((lo, hi, seen, detected))
    return bins


def summarize(runs: Sequence[DetectRunMetrics], config: DetectConfig,
              title: str = 'Detection') -> str:
    n = len(runs)
    detected = [r for r in runs if not r.missed]
    birds_seen = sum(r.birds_seen for r in runs)
    empty = sum(r.empty_frames for r in runs)
    lines = [f'{title}: {n} runs, {config.num_defenders} defenders',
             f'  missed drones:           {_pct(n - len(detected), n)} '
             f'({n - len(detected)} of {n} reached the ring undetected)',
             f'  warning when detected:   {_median([r.warning_distance for r in detected])} m '
             f'outside the ring, {_median([r.warning_time for r in detected])} s (median)',
             f'  first detection range:   {_median([r.first_detection_range for r in detected])} m '
             '(median)',
             f'  birds flagged as drones: {_pct(sum(r.birds_flagged for r in runs), birds_seen)} '
             f'of {birds_seen} birds seen',
             f'  background false alarms: '
             + ('-' if not empty else
                f'{sum(r.background_false_alarms for r in runs) / empty:.4f} per image')
             + f' ({empty} empty images)',
             '  detection probability vs range:',
             f'    {"range [m]":>12} {"seen":>5} {"detected":>9} {"share":>6}']
    for lo, hi, seen, hit in detection_by_range(runs, config.range_bins):
        label = f'{lo:.0f}-' + ('' if hi == float('inf') else f'{hi:.0f}')
        lines.append(f'    {label:>12} {seen:>5} {hit:>9} {_pct(hit, seen):>6}')
    return '\n'.join(lines)


def _pixel_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def _pct(part: int, whole: int) -> str:
    return '-' if not whole else f'{100 * part / whole:.0f} %'


def _median(values) -> str:
    values = [v for v in values if v is not None]
    return '-' if not values else f'{statistics.median(values):.0f}'
