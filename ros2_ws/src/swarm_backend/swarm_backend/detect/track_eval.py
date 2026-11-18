"""How good a tracker is: runs it on fake detections (fake_detections.py) and scores its
confirmed tracks against the truth with the standard multi-object tracking measures, so
trackers and their settings can be compared with numbers instead of by eye. No ROS, no
Gazebo; a run of 50 seeds takes seconds.

    python3 -m swarm_backend.detect.track_eval                         both trackers, defaults
    python3 -m swarm_backend.detect.track_eval --scenario crossing     the camera stays still
    python3 -m swarm_backend.detect.track_eval --miss 0.2 --weak 0.3   a worse detector
    python3 -m swarm_backend.detect.track_eval --sweep new 0.5 0.6 0.7 one ByteTrack setting
    python3 -m swarm_backend.detect.track_eval --drop 0.5              half the images skipped

Scoring, per image: each target in view is paired with at most one confirmed track whose
detection is within --match-px of it (a pair from the last image is kept while it still
fits, else the closest pairs; CLEAR MOT). Only confirmed tracks count: they are what the
attack gets. Then, summed over all images and seeds:

  coverage      share of target-images with a confirmed track on the target
  misses        target-images without one
  false         confirmed track-images on no target (clutter the attack would chase)
  false tracks  confirmed tracks that were never on any target
  id switches   a target's track id changed since it last had one
  fragments     a target's coverage stopped and started again
  MOTA          1 - (misses + false + id switches) / target-images
  IDF1          share of target- and track-images where the target has *its* track: each
                target gets the one track that followed it longest, the rest count as wrong
  to confirm    images from a target's first appearance to its first confirmed image

The scoring (`score`) only needs the truth and each image's confirmed (id, u, v), so the
same can later score a real detector's tracks against Gazebo's saved truth.
"""
import argparse
import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np

from swarm_backend.detect import fake_detections as fake
from swarm_backend.detect.bytetrack import ByteTracker, Settings
from swarm_backend.detect.camera import CameraModel, GIMBAL_SIZE
from swarm_backend.detect.detectors import Detection
from swarm_backend.detect.kalman import Noise, rotation_homography
from swarm_backend.detect.orientation import Axes
from swarm_backend.detect.tracking import Confirmer, angle_between

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:
    linear_sum_assignment = None

Output = tuple[int, float, float]  # a confirmed track in one image: id, u, v (px)


# --- the measures ----------------------------------------------------------------------

@dataclass
class Metrics:
    targets: int = 0  # target-images (truth boxes in view)
    matches: int = 0
    misses: int = 0
    false: int = 0  # confirmed track-images on no target
    outputs: int = 0  # confirmed track-images
    id_switches: int = 0
    fragments: int = 0
    false_tracks: int = 0
    id_matches: int = 0  # IDTP: images where a target has its own (longest) track
    to_confirm: list[int] = field(default_factory=list)  # images, per confirmed target
    never_confirmed: int = 0  # targets that were never confirmed

    def __add__(self, other: 'Metrics') -> 'Metrics':
        return Metrics(*(a + b for a, b in zip(dataclasses.astuple(self),
                                                dataclasses.astuple(other))))

    @property
    def coverage(self) -> float:
        return self.matches / self.targets if self.targets else 0.0

    @property
    def mota(self) -> float:
        errors = self.misses + self.false + self.id_switches
        return 1 - errors / self.targets if self.targets else 0.0

    @property
    def idf1(self) -> float:
        total = self.targets + self.outputs
        return 2 * self.id_matches / total if total else 0.0

    @property
    def mean_to_confirm(self) -> float:
        return sum(self.to_confirm) / len(self.to_confirm) if self.to_confirm else math.nan


def score(truth: list[list[fake.TruthBox]], outputs: list[list[Output]],
          match_px: float = 10.0) -> Metrics:
    """The measures for one run: the truth and the tracker's confirmed tracks, per image."""
    m = Metrics()
    previous: dict[int, int] = {}  # target -> track, in the last image
    last_track: dict[int, int] = {}  # target -> the track it last had
    covered: dict[int, bool] = {}  # target -> had a track in the last image it was in view
    first_seen: dict[int, int] = {}
    first_match: dict[int, int] = {}
    pair_count: dict[tuple[int, int], int] = {}
    on_target: set[int] = set()
    confirmed_ids: set[int] = set()

    for i, (boxes, tracks) in enumerate(zip(truth, outputs)):
        m.targets += len(boxes)
        m.outputs += len(tracks)
        for b in boxes:
            first_seen.setdefault(b.id, i)
        confirmed_ids.update(t[0] for t in tracks)
        pairs = _pair(boxes, tracks, previous, match_px)
        for b in boxes:
            k = pairs.get(b.id)
            if k is None:
                m.misses += 1
                covered[b.id] = False
                continue
            m.matches += 1
            on_target.add(k)
            pair_count[b.id, k] = pair_count.get((b.id, k), 0) + 1
            if b.id in last_track and last_track[b.id] != k:
                m.id_switches += 1
            if b.id in first_match and not covered.get(b.id, False):
                m.fragments += 1
            first_match.setdefault(b.id, i)
            last_track[b.id] = k
            covered[b.id] = True
        m.false += len(tracks) - len(pairs)
        previous = pairs

    for target, start in first_seen.items():
        if target in first_match:
            m.to_confirm.append(first_match[target] - start)
        else:
            m.never_confirmed += 1
    m.false_tracks = len(confirmed_ids - on_target)
    m.id_matches = _best_id_matches(pair_count)
    return m


def _pair(boxes, tracks, previous: dict[int, int], match_px: float) -> dict[int, int]:
    """target id -> track id for one image (CLEAR MOT: keep last image's pairs that fit)."""
    by_id = {t[0]: t for t in tracks}
    pairs: dict[int, int] = {}
    for b in boxes:
        k = previous.get(b.id)
        if k in by_id and math.dist((b.u, b.v), by_id[k][1:]) <= match_px:
            pairs[b.id] = k
    free_boxes = [b for b in boxes if b.id not in pairs]
    used = set(pairs.values())
    free_tracks = [t for t in tracks if t[0] not in used]
    if free_boxes and free_tracks:
        cost = np.array([[math.dist((b.u, b.v), t[1:]) for t in free_tracks]
                         for b in free_boxes])
        for r, c in _assign(cost, match_px):
            pairs[free_boxes[r].id] = free_tracks[c][0]
    return pairs


def _assign(cost: np.ndarray, gate: float) -> list[tuple[int, int]]:
    """Lowest-total-cost pairs with cost <= gate (Hungarian; greedy without scipy)."""
    if linear_sum_assignment is not None:
        rows, cols = linear_sum_assignment(np.where(cost <= gate, cost, gate + 1e6))
        return [(r, c) for r, c in zip(rows, cols) if cost[r, c] <= gate]
    chosen, used_r, used_c = [], set(), set()
    for r, c in sorted(np.ndindex(cost.shape), key=lambda rc: cost[rc]):
        if cost[r, c] <= gate and r not in used_r and c not in used_c:
            chosen.append((r, c))
            used_r.add(r)
            used_c.add(c)
    return chosen


def _best_id_matches(pair_count: dict[tuple[int, int], int]) -> int:
    """IDTP: images matched when each target keeps only one track, and each track one target,
    chosen to keep the most images."""
    if not pair_count:
        return 0
    targets = sorted({t for t, _ in pair_count})
    tracks = sorted({k for _, k in pair_count})
    counts = np.zeros((len(targets), len(tracks)))
    for (t, k), n in pair_count.items():
        counts[targets.index(t), tracks.index(k)] = n
    if linear_sum_assignment is not None:
        rows, cols = linear_sum_assignment(-counts)
        return int(counts[rows, cols].sum())
    return int(sum(n for _, _, n in _greedy_max(counts)))


def _greedy_max(counts: np.ndarray):
    used_r, used_c = set(), set()
    for r, c in sorted(np.ndindex(counts.shape), key=lambda rc: -counts[rc]):
        if r not in used_r and c not in used_c and counts[r, c] > 0:
            used_r.add(r)
            used_c.add(c)
            yield r, c, counts[r, c]


# --- running the trackers ---------------------------------------------------------------

class ByteTrackRunner:
    """ByteTrack as live.py runs it: compensated for the camera when the orientation is known,
    and moved on by the real time between images (use_stamps=False: one step per image)."""

    def __init__(self, settings: Settings, camera: CameraModel, use_stamps: bool = True):
        self.tracker, self.camera = ByteTracker(settings), camera
        self.last_axes: Axes | None = None
        self.use_stamps = use_stamps

    def step(self, detections: list[Detection], axes: Axes | None,
             stamp: float | None = None) -> list[Output]:
        h = None
        if axes is not None and self.last_axes is not None:
            h = rotation_homography(self.camera, self.last_axes, axes)
        self.last_axes = axes
        self.tracker.update(detections, h, stamp if self.use_stamps else None)
        return [(t.id, t.detection.u, t.detection.v) for t in self.tracker.confirmed]


class ConfirmerRunner:
    """The Confirmer as live.py runs it: by world direction when the orientation is known,
    else in pixels."""

    def __init__(self, k: int, gate_px: float, gate_deg: float, camera: CameraModel):
        self.pixels = Confirmer(k=k, gate=gate_px)
        self.world = Confirmer(k=k, gate=math.radians(gate_deg), distance=angle_between)
        self.camera = camera

    def step(self, detections: list[Detection], axes: Axes | None,
             stamp: float | None = None) -> list[Output]:  # the Confirmer has no time
        if axes is None:
            self.pixels.update([(d.u, d.v) for d in detections], detections)
            confirmed = self.pixels.confirmed
        else:
            self.world.update([self.camera.direction(d.u, d.v, *axes) for d in detections],
                              detections)
            confirmed = self.world.confirmed
        return [(t.id, t.detection.u, t.detection.v) for t in confirmed]


def run(make_runner, images: list[fake.FakeImage], axes: list[Axes] | None,
        match_px: float, stamps: list[float] | None = None) -> Metrics:
    runner = make_runner()
    outputs = [runner.step(im.detections, axes[i] if axes else None,
                           stamps[i] if stamps else None)
               for i, im in enumerate(images)]
    return score([im.truth for im in images], outputs, match_px)


def scenario(name: str, camera: CameraModel):
    """(truth per image, camera orientation per image or None)."""
    frames = fake.crossing_targets(width=camera.width, height=camera.height)
    if name == 'crossing':
        return frames, None
    if name == 'panning':
        return fake.panning_camera(frames, camera)
    raise ValueError(f'Unknown scenario {name}')


# --- command line -----------------------------------------------------------------------

SWEEPABLE = {f.name: f.type for f in dataclasses.fields(Settings)
             if f.name not in ('noise', 'cost')}
SWEEPABLE['min_px'] = 'float'


def settings_with(base: Settings, name: str, value: str) -> Settings:
    if name == 'min_px':
        return dataclasses.replace(base, noise=Noise(min_px=float(value)))
    kind = SWEEPABLE[name]
    if kind in (bool, 'bool'):
        parsed = value.lower() in ('1', 'true', 'yes', 'on')
    elif kind in (int, 'int'):
        parsed = int(value)
    else:
        parsed = float(value)
    return dataclasses.replace(base, **{name: parsed})


HEADER = (f'{"tracker":34} {"MOTA":>6} {"IDF1":>6} {"cover":>6} {"miss":>6} {"false":>6} '
          f'{"falseTr":>7} {"idSw":>5} {"frag":>5} {"confirm":>7} {"never":>5}')


def row(name: str, m: Metrics) -> str:
    return (f'{name:34} {m.mota:6.3f} {m.idf1:6.3f} {100 * m.coverage:5.1f}% {m.misses:6} '
            f'{m.false:6} {m.false_tracks:7} {m.id_switches:5} {m.fragments:5} '
            f'{m.mean_to_confirm:7.2f} {m.never_confirmed:5}')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog='\n'.join(__doc__.splitlines()[1:]))
    parser.add_argument('--scenario', choices=('panning', 'crossing'), default='panning',
                        help='crossing targets, with the gimbal swinging (default) or still')
    parser.add_argument('--seeds', type=int, default=50, help='runs per tracker')
    parser.add_argument('--match-px', type=float, default=10.0,
                        help='how near a track must be to a target to count as on it')
    parser.add_argument('--no-axes', action='store_true',
                        help="don't give the trackers the camera orientation")
    errors = parser.add_argument_group('fake detector (fake_detections.Errors)')
    defaults = fake.Errors()
    errors.add_argument('--centre-std', type=float, default=defaults.centre_std_px)
    errors.add_argument('--miss', type=float, default=defaults.miss)
    errors.add_argument('--weak', type=float, default=defaults.weak)
    errors.add_argument('--false', type=float, default=defaults.false_per_image,
                        help='false detections per image (mean)')
    parser.add_argument('--drop', type=float, default=0.0,
                        help='chance each image is skipped (uneven image rate, as in a slow '
                             'simulation); ByteTrack is also run without time stamps to compare')
    parser.add_argument('--k', type=int, default=3, help='images to confirm, both trackers')
    parser.add_argument('--gate-px', type=float, default=20.0, help='Confirmer, pixels')
    parser.add_argument('--gate-deg', type=float, default=10.0, help='Confirmer, world')
    parser.add_argument('--sweep', nargs='+', metavar=('SETTING', 'VALUE'),
                        help=f'run ByteTrack once per value of one setting: '
                             f'{", ".join(SWEEPABLE)}')
    args = parser.parse_args(argv)

    camera = CameraModel(width=GIMBAL_SIZE[0], height=GIMBAL_SIZE[1])
    frames, axes = scenario(args.scenario, camera)
    stamps = None
    if args.drop > 0:
        frames, axes, stamps = fake.uneven_rate(frames, axes, args.drop)
    if args.no_axes:
        axes = None
    errs = fake.Errors(centre_std_px=args.centre_std, miss=args.miss, weak=args.weak,
                       false_per_image=args.false)
    runs = [fake.run(frames, camera.width, camera.height, errs, seed=s)
            for s in range(args.seeds)]

    def total(make_runner) -> Metrics:
        result = Metrics()
        for images in runs:
            result = result + run(make_runner, images, axes, args.match_px, stamps)
        return result

    base = Settings(k=args.k)
    print(f'{args.scenario}, {args.seeds} seeds, camera orientation '
          f'{"not given" if axes is None else "given"}'
          + (f', {100 * args.drop:.0f} % of images skipped' if stamps else '')
          + '; detector: centre std '
          f'{errs.centre_std_px} px, miss {errs.miss}, weak {errs.weak}, '
          f'{errs.false_per_image} false/image\n')
    print(HEADER)
    print(row(f'Confirmer (k {args.k})',
              total(lambda: ConfirmerRunner(args.k, args.gate_px, args.gate_deg, camera))))
    if args.sweep:
        name, values = args.sweep[0], args.sweep[1:]
        if name not in SWEEPABLE or not values:
            parser.error(f'--sweep SETTING VALUE...; SETTING is one of {", ".join(SWEEPABLE)}')
        for value in values:
            s = settings_with(base, name, value)
            print(row(f'ByteTrack {name}={value}', total(lambda: ByteTrackRunner(s, camera))))
    else:
        print(row('ByteTrack (defaults)', total(lambda: ByteTrackRunner(base, camera))))
        if stamps:
            print(row('ByteTrack without time stamps',
                      total(lambda: ByteTrackRunner(base, camera, use_stamps=False))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
