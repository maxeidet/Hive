"""Tune the detector's thresholds on recorded Gazebo runs, offline.

A run recorded with `scenarios/yolo.sh --record NAME` (tests/scenario2_test.py) holds,
in data/tune/NAME/ of the repo:

  detect.jsonl  every image the live detector processed: every box YOLO gave (x, y, w, h,
                score; record it with a low --conf, e.g. 0.05, so that every threshold above
                that can be replayed), PX4's camera orientation and the image's time
  truth.jsonl   Gazebo's camera pose and the hostiles' positions, at Gazebo's pose rate
  events.jsonl  when the hostiles left, hits, hostiles taken out

Each box is labelled with the hostile it is on (its centre near where Gazebo's truth
projects that hostile in the image), or none (a false box). Then ByteTrack and the target
choice (--lock or best score) are replayed exactly as detect/live.py runs them, for every
combination of thresholds in a grid, and scored on what the attack would have been given:

  coverage    share of images with a hostile in view where a hostile was published
  false       share of the published images where the target was on no hostile (and on
              no friendly drone)
  friendly    share of the published images where the target was one of our own drones
              (by default drone 0, parked on the base: YOLO rightly sees a drone there)
  switches    the published target jumped to the other hostile while the first still flew
  gaps        silences longer than 0.5 s while a hostile was in view
  first m     distance to the hostile at the first correct publication (mean over runs)

Open loop: the replay changes what is published, not how the defender flew, so it ranks
thresholds on the same pictures; the best few should then be flown in Gazebo.

    python3 -m swarm_backend.detect.replay_eval data/tune/run1 data/tune/run2
    python3 -m swarm_backend.detect.replay_eval data/tune/* --detector-only
    python3 -m swarm_backend.detect.replay_eval data/tune/* --jobs 4 --top 15

No ROS, no Gazebo; needs numpy (and scipy for the Hungarian matching ByteTrack uses).
"""
import argparse
import bisect
import itertools
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

from swarm_backend.detect.bytetrack import ByteTracker, Settings
from swarm_backend.detect.camera import GIMBAL_HFOV, CameraModel
from swarm_backend.detect.detectors import Detection
from swarm_backend.detect.kalman import Noise, rotation_homography
from swarm_backend.detect.live import LOCK_GATE_DEG, TRACK_SCORES, TargetLock, best_target

RATE_HZ = 10.0  # the live detector's --rate in scenario 2 (detect_rate)
MATCH_MIN_PX = 15.0  # a box is on a hostile if its centre is this close to the truth pixel
MATCH_SIZE = 0.75  # ... or within this share of the box's longer side, when that is more
GAP_S = 0.5  # a silence longer than this is counted (the pursuit attack's coast_after)
DISTANCE_BINS = (0, 20, 40, 60, 80, 100, 130, 1e9)
# Our own drones that stand still during a run (world NED, m): drone 0 parked on the base in
# scenario 2 (Gazebo 0, 0, 0). Boxes on them are counted apart from the false ones.
FRIENDLIES = {0: (0.0, 0.0, -0.25)}


# --- loading and labelling --------------------------------------------------------------

@dataclass
class Image:
    stamp: float
    width: int
    height: int
    boxes: list  # Detection, as the detector gave them
    labels: list  # hostile id per box, or None for a false box
    axes: tuple | None  # PX4's camera orientation (what live.py uses), or None
    in_view: dict  # hostile id -> (u, v, distance m) for each hostile in the image
    friends: dict = field(default_factory=dict)  # the same for FRIENDLIES in the image
    friendly: list = field(default_factory=list)  # per box: on a friendly drone (and no hostile)


@dataclass
class Run:
    name: str
    images: list = field(default_factory=list)
    removed_at: dict = field(default_factory=dict)  # hostile id -> time it was taken out


class Truth:
    """Gazebo's samples, interpolated to any time."""

    def __init__(self, samples: list[dict]):
        samples.sort(key=lambda s: s['t'])
        self.samples = samples
        self.times = [s['t'] for s in samples]

    def at(self, t: float) -> dict | None:
        i = bisect.bisect_left(self.times, t)
        if i == 0 or i >= len(self.times):
            return None  # outside what was recorded
        a, b = self.samples[i - 1], self.samples[i]
        if b['t'] - a['t'] > 0.2:
            return None  # a hole in the recording
        f = (t - a['t']) / (b['t'] - a['t']) if b['t'] > a['t'] else 0.0

        def lerp(p, q):
            return [x + f * (y - x) for x, y in zip(p, q)]

        cam = [lerp(p, q) for p, q in zip(a['cam'], b['cam'])]
        for k in (1, 2, 3):  # axes: renormalised after interpolating
            n = math.hypot(*cam[k])
            cam[k] = [c / n for c in cam[k]]
        hostiles = {int(h): lerp(a['hostiles'][h], b['hostiles'][h])
                    for h in a['hostiles'] if h in b['hostiles']}
        return {'position': cam[0], 'forward': cam[1], 'right': cam[2], 'down': cam[3],
                'hostiles': hostiles}


def project(cam: dict, point, camera: CameraModel):
    """(u, v, distance) of a world point in the image, or None when out of view."""
    d = [p - c for p, c in zip(point, cam['position'])]
    depth = sum(a * b for a, b in zip(d, cam['forward']))
    if depth <= 0:
        return None
    u = camera.cx + camera.f * sum(a * b for a, b in zip(d, cam['right'])) / depth
    v = camera.height / 2 + camera.f * sum(a * b for a, b in zip(d, cam['down'])) / depth
    if not (0 <= u < camera.width and 0 <= v < camera.height):
        return None
    return u, v, math.hypot(*d)


def label(box: Detection, in_view: dict):
    best = None
    for hostile, (u, v, _) in in_view.items():
        dist = math.hypot(box.u - u, box.v - v)
        if dist <= max(MATCH_MIN_PX, MATCH_SIZE * max(box.width, box.height)):
            if best is None or dist < best[0]:
                best = (dist, hostile)
    return None if best is None else best[1]


def load_run(folder: Path, friendlies: dict = FRIENDLIES) -> Run:
    def lines(name):
        path = folder / name
        if not path.exists():
            return []
        with open(path) as f:
            return [json.loads(line) for line in f if line.strip()]

    truth = Truth([s for s in lines('truth.jsonl') if s.get('cam')])
    events = lines('events.jsonl')
    start = next((e['t'] for e in events if e['event'] == 'start'), None)
    run = Run(folder.name)
    for e in events:
        if e['event'] == 'removed':
            run.removed_at[e['hostile']] = e['t']
    for rec in lines('detect.jsonl'):
        if 'raw' not in rec:
            raise ValueError(f'{folder}/detect.jsonl has no raw boxes: record it with the '
                             'current detect/live.py')
        stamp = rec['stamp']
        if start is not None and stamp < start:
            continue  # before the hostiles left
        cam = truth.at(stamp)
        if cam is None:
            continue
        camera = CameraModel(GIMBAL_HFOV, rec['width'], rec['height'])
        in_view = {}
        for hostile, position in cam['hostiles'].items():
            if hostile in run.removed_at and stamp >= run.removed_at[hostile]:
                continue
            p = project(cam, position, camera)
            if p is not None:
                in_view[hostile] = p
        friends = {}
        for friend, position in friendlies.items():
            p = project(cam, position, camera)
            if p is not None:
                friends[friend] = p
        boxes = [Detection(int(x), int(y), int(w), int(h), float(s)) for x, y, w, h, s in rec['raw']]
        axes = None if rec.get('axes') is None else tuple(tuple(a) for a in rec['axes'])
        labels = [label(b, in_view) for b in boxes]
        friendly = [lab is None and label(b, friends) is not None for b, lab in zip(boxes, labels)]
        run.images.append(Image(stamp, rec['width'], rec['height'], boxes, labels, axes,
                                in_view, friends, friendly))
    return run


# --- the detector alone -----------------------------------------------------------------

def detector_table(runs: list[Run], confs=(0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6)) -> str:
    """Recall by distance and false boxes per image, for each confidence threshold."""
    bins = list(zip(DISTANCE_BINS, DISTANCE_BINS[1:]))
    seen = {b: 0 for b in bins}  # hostile-images in view, per distance bin
    found = {(c, b): 0 for c in confs for b in bins}
    false = {c: 0 for c in confs}
    images = 0
    true_scores = {b: [] for b in bins}
    for run in runs:
        for im in run.images:
            images += 1
            for hostile, (_, _, distance) in im.in_view.items():
                b = next(b for b in bins if b[0] <= distance < b[1])
                seen[b] += 1
                scores = [box.score for box, lab in zip(im.boxes, im.labels) if lab == hostile]
                if scores:
                    true_scores[b].append(max(scores))
                for c in confs:
                    found[(c, b)] += any(s >= c for s in scores)
            for box, lab, ours in zip(im.boxes, im.labels, im.friendly or [False] * len(im.boxes)):
                if lab is None and not ours:
                    for c in confs:
                        false[c] += box.score >= c
    def cells(values):
        return ''.join(f'{v:>9}' for v in values)

    rows = [f'{"conf":<8}' + cells(_bin_name(b) for b in bins) + '   false/img',
            f'{"in view":<8}' + cells(seen[b] for b in bins)]
    for c in confs:
        rows.append(f'{c:<8g}' + cells(f'{found[(c, b)] / seen[b]:.0%}' if seen[b] else '-'
                                        for b in bins) + f'   {false[c] / max(1, images):9.2f}')
    rows.append(f'{"median":<8}' + cells(f'{_median(true_scores[b]):.2f}' if true_scores[b]
                                         else '-' for b in bins) + '   (true boxes\' score)')
    return '\n'.join(rows)


def _bin_name(b) -> str:
    return f'{b[0]:.0f}+ m' if b[1] > 1e8 else f'{b[0]:.0f}-{b[1]:.0f}'


def _median(values: list) -> float:
    values = sorted(values)
    return values[len(values) // 2]


# --- tracker and lock replay ------------------------------------------------------------

@dataclass(frozen=True)
class TrackerSetting:
    low: float  # = YOLO's --conf (live.py lowers --conf to --low)
    high: float
    new: float
    k: int
    max_lost: float
    gate_sigma: float = 4.0
    min_px: float = Noise.min_px


@dataclass(frozen=True)
class LockSetting:
    on: bool
    timeout: float = 1.0
    gate_deg: float = LOCK_GATE_DEG

    def name(self) -> str:
        return f'lock {self.timeout:g}s' if self.on else 'no lock'


@dataclass
class Snapshot:
    """What the target choice sees of one track in one image."""
    id: int
    confirmed: bool
    detection: Detection | None
    box: tuple  # the filter's (u, v, w, h): for a lost track, its prediction
    kalman: bool = True  # marks it as a ByteTrack track for TargetLock's prediction


def track_run(run: Run, t: TrackerSetting) -> list[list[Snapshot]]:
    """ByteTrack over the run as live.py runs it; the tracks after each image."""
    tracker = ByteTracker(Settings(high=t.high, low=t.low, new=t.new, k=t.k,
                                   max_lost=t.max_lost, rate_hz=RATE_HZ, gate=t.gate_sigma,
                                   gate_low=t.gate_sigma, noise=Noise(min_px=t.min_px)))
    out, last_axes = [], None
    for im in run.images:
        camera = CameraModel(GIMBAL_HFOV, im.width, im.height)
        h = None
        if im.axes is not None and last_axes is not None:
            h = rotation_homography(camera, last_axes, im.axes)
        last_axes = im.axes
        tracker.update([b for b in im.boxes if b.score >= t.low], h, im.stamp)
        out.append([Snapshot(tr.id, tr.confirmed, tr.detection, tuple(tr.box))
                    for tr in tracker.tracks])
    return out


@dataclass
class Score:
    active: int = 0  # images with a hostile in view
    covered: int = 0  # ... where a hostile was published
    published: int = 0
    false: int = 0
    friendly: int = 0  # published targets on our own drones
    switches: int = 0
    gaps: int = 0
    first_m: list = field(default_factory=list)  # per run

    def add(self, o: 'Score') -> None:
        for k in ('active', 'covered', 'published', 'false', 'friendly', 'switches', 'gaps'):
            setattr(self, k, getattr(self, k) + getattr(o, k))
        self.first_m += o.first_m

    @property
    def coverage(self) -> float:
        return self.covered / self.active if self.active else 0.0

    @property
    def false_share(self) -> float:
        return self.false / self.published if self.published else 0.0

    @property
    def friendly_share(self) -> float:
        return self.friendly / self.published if self.published else 0.0


def choose_run(run: Run, tracks: list[list[Snapshot]], lock: LockSetting) -> Score:
    """The target choice over a tracked run, scored."""
    score = Score()
    chooser = TargetLock(lock.gate_deg, timeout=lock.timeout) if lock.on else None
    last_hostile, silent_since, first = None, None, None
    for im, snaps in zip(run.images, tracks):
        if chooser is not None:
            camera = CameraModel(GIMBAL_HFOV, im.width, im.height)

            def direction(tr, predicted, a=im.axes, camera=camera):
                if a is None:
                    return None
                u, v = tr.box[:2] if predicted else (tr.detection.u, tr.detection.v)
                return camera.direction(u, v, *a)
            target = chooser.update(snaps, im.stamp, direction)
        else:
            target = best_target(snaps)
        on = None
        if target is not None:
            score.published += 1
            on = label(target.detection, im.in_view)
            if on is None:
                if label(target.detection, im.friends) is not None:
                    score.friendly += 1
                else:
                    score.false += 1
        if im.in_view:
            score.active += 1
            if on is not None:
                score.covered += 1
                if first is None:
                    first = im.in_view[on][2]
        if on is not None:
            if (last_hostile is not None and on != last_hostile
                    and not (last_hostile in run.removed_at
                             and im.stamp >= run.removed_at[last_hostile])):
                score.switches += 1
            last_hostile = on
        if on is None and im.in_view:
            silent_since = im.stamp if silent_since is None else silent_since
        else:
            if silent_since is not None and im.stamp - silent_since > GAP_S:
                score.gaps += 1
            silent_since = None
    if first is not None:
        score.first_m.append(first)
    return score


def tracker_grid(lows, highs, news, ks, max_losts) -> list[TrackerSetting]:
    return [TrackerSetting(low, high, new, k, ml)
            for low, high, new, k, ml in itertools.product(lows, highs, news, ks, max_losts)
            if low < high <= new]


def evaluate(args) -> list[tuple[TrackerSetting, LockSetting, Score]]:
    runs, setting, locks = args
    tracked = [track_run(run, setting) for run in runs]
    results = []
    for lock in locks:
        total = Score()
        for run, tracks in zip(runs, tracked):
            total.add(choose_run(run, tracks, lock))
        results.append((setting, lock, total))
    return results


MAX_SWITCHES_PER_MIN = 2.0  # more and the attack is useless, whatever the coverage
COVERAGE_STEP = 0.01  # coverage this close counts as equal: then fewer false targets win


def rank_key(item, minutes: float):
    """Settings that keep to one hostile first (at most MAX_SWITCHES_PER_MIN), then most
    coverage (in steps of 1 %), then fewest false and friendly targets, switches, gaps."""
    _, _, s = item
    return (s.switches / max(minutes, 1e-9) > MAX_SWITCHES_PER_MIN,
            -math.floor(s.coverage / COVERAGE_STEP + 1e-9),
            round(s.false_share + s.friendly_share, 3), s.switches, s.gaps)


def table(results, minutes: float, top: int) -> str:
    rows = [f'{"low":>5} {"high":>5} {"new":>5} {"k":>2} {"lost":>5} {"choice":>11} '
            f'{"coverage":>9} {"false":>6} {"ours":>6} {"switch/min":>10} {"gaps/min":>9} '
            f'{"first m":>8}']
    for t, lock, s in results[:top]:
        first = sum(s.first_m) / len(s.first_m) if s.first_m else float('nan')
        rows.append(f'{t.low:5g} {t.high:5g} {t.new:5g} {t.k:2d} {t.max_lost:5g} '
                    f'{lock.name():>11} {s.coverage:9.0%} {s.false_share:6.1%} '
                    f'{s.friendly_share:6.1%} {s.switches / minutes:10.2f} '
                    f'{s.gaps / minutes:9.2f} {first:8.0f}')
    return '\n'.join(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+', type=Path, help='data/tune/NAME folders')
    parser.add_argument('--detector-only', action='store_true',
                        help='only the recall and false-box table per confidence')
    parser.add_argument('--low', type=float, nargs='+', default=[0.05, 0.1, 0.15, 0.2, 0.3])
    parser.add_argument('--high', type=float, nargs='+', default=[0.3, 0.4, 0.5, 0.6])
    parser.add_argument('--new', type=float, nargs='+', default=[0.4, 0.5, 0.6, 0.7])
    parser.add_argument('--k', type=int, nargs='+', default=[2, 3, 4])
    parser.add_argument('--max-lost', type=float, nargs='+', default=[10, 20, 30])
    parser.add_argument('--lock-timeout', type=float, nargs='+', default=[0.5, 1.0, 2.0])
    parser.add_argument('--jobs', type=int, default=1, help='processes (keep it small: shared machine)')
    parser.add_argument('--top', type=int, default=20)
    parser.add_argument('--csv', type=Path, help='write every combination here')
    args = parser.parse_args(argv)

    runs = [load_run(folder) for folder in args.runs]
    images = sum(len(r.images) for r in runs)
    minutes = images / RATE_HZ / 60
    labelled = sum(lab is not None for r in runs for im in r.images for lab in im.labels)
    print(f'{len(runs)} runs, {images} images ({minutes:.1f} min at {RATE_HZ:g} Hz), '
          f'{labelled} boxes on a hostile')
    print('\nDetector: share of hostile-images with a box at or above conf, by distance')
    print(detector_table(runs))
    if args.detector_only:
        return 0

    settings = tracker_grid(args.low, args.high, args.new, args.k, args.max_lost)
    locks = [LockSetting(False)] + [LockSetting(True, t) for t in args.lock_timeout]
    print(f'\nReplaying {len(settings)} tracker settings x {len(locks)} target choices...',
          flush=True)
    jobs = [(runs, s, locks) for s in settings]
    if args.jobs > 1:
        from multiprocessing import Pool
        with Pool(args.jobs) as pool:
            chunks = pool.map(evaluate, jobs, chunksize=4)
    else:
        chunks = [evaluate(j) for j in jobs]
    results = sorted((r for chunk in chunks for r in chunk), key=lambda r: rank_key(r, minutes))
    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('low,high,new,k,max_lost,lock,lock_timeout,coverage,false_share,'
                    'friendly_share,switches_per_min,gaps_per_min,first_m\n')
            for t, lock, s in results:
                first = sum(s.first_m) / len(s.first_m) if s.first_m else ''
                f.write(f'{t.low},{t.high},{t.new},{t.k},{t.max_lost},{int(lock.on)},'
                        f'{lock.timeout if lock.on else ""},{s.coverage:.4f},'
                        f'{s.false_share:.4f},{s.friendly_share:.4f},'
                        f'{s.switches / minutes:.3f},'
                        f'{s.gaps / minutes:.3f},{first}\n')
    print(f'\nBest {args.top} (ranked: at most {MAX_SWITCHES_PER_MIN:g} switches/min, then coverage '
          '(1 % steps), false + our own targets ("ours"), switches, gaps):')
    print(table(results, minutes, args.top))
    high, low, new = TRACK_SCORES['yolo']
    default = TrackerSetting(low, high, new, 3, Settings.max_lost)
    today = [r for r in results if r[0] == default]
    if today:
        print(f'\nToday\'s defaults (low {low:g}, high {high:g}, new {new:g}, k 3, '
              f'max-lost {default.max_lost:g}):')
        print(table(sorted(today, key=lambda r: rank_key(r, minutes)), minutes, len(today)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
