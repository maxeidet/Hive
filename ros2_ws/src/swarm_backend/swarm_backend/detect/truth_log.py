"""What the detector estimates against Gazebo's truth, one JSON line per image (--truth-log).

For finding where the distance and the friendly check go wrong in a run. Each line has,
at the image's time:

  own       the camera's own position: the estimate the friendly check uses
            (LiveDetector.own_position) against Gazebo's, the error in metres, the
            drone's speed, and how far PX4's camera orientation is off (degrees)
  friends   each other friendly drone: where the detector predicts it (FriendlyBook)
            against Gazebo, the error, and how old the broadcast position is
  targets   every drone that is not a friendly (the hostiles): true distance from the
            camera, and whether it is in front of the camera
  tracks    every track with a box: its box-size distance, the drone it really is (the
            one closest in angle, from Gazebo's camera), that drone's true distance, the
            distance error (m and %), the direction error (degrees), and for each
            friendly the friendly check's terms: the angle to it, the gate, its distance,
            whether the box distance fits and whether the whole check fits; `ambiguous`
            when another drone is nearly in line (the box distance picked between them);
            how high it is above the camera (m, elevation in degrees) and how many metres
            of drone the box's width and height cover at its true distance

Summarise a log with: python3 -m swarm_backend.detect.truth_log <file.jsonl>
World NED, metres, seconds (simulation time), angles in degrees. The pure part
(frame_record, summary) needs no ROS or Gazebo.
"""
import json
import math
import sys
from pathlib import Path

from swarm_backend.swarm.friendlies import RANGE_FACTOR, friendly_for, gate_deg

Vector3 = tuple[float, float, float]
MATCH_MAX_DEG = 10.0  # a box further than this from every drone is matched to none
# Two drones nearly in line: the one whose distance fits the box better is the one it is.
# The direction is good to about a degree, the box distance to about 40 % (ln 1.4 = 0.34).
DEG_PER_LOG_RANGE = 3.0
AMBIGUOUS_DEG = 2.0  # another drone this close in angle: the match is marked ambiguous


def _r(v, digits: int = 2):
    if v is None:
        return None
    if isinstance(v, (tuple, list)):
        return [round(c, digits) for c in v]
    return round(v, digits)


def _angle_deg(a: Vector3, b: Vector3) -> float:
    na, nb = math.hypot(*a), math.hypot(*b)
    cos = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def frame_record(stamp: float, tracks, camera, axes, detector_name: str,
                 true_camera: dict, true_bodies: dict[int, Vector3], own_id: int,
                 own_estimate: Vector3 | None, own_velocity: Vector3 | None,
                 friends: dict[int, Vector3], friend_ages: dict[int, float],
                 marked: dict[int, int]) -> dict:
    """One line of the log. `true_camera`: ground_truth.camera_pose (position, forward,
    right, down); `true_bodies`: every drone's true body centre by id; `friends`: the
    detector's predicted positions of the other friendlies (FriendlyBook.at, own left
    out); `marked`: track id -> friendly id (FriendlyMarks)."""
    cam = true_camera['position']
    own_true = true_bodies.get(own_id)
    rec = {'t': round(stamp, 3)}
    own = {'true': _r(own_true), 'estimate': _r(own_estimate),
           'speed': None if own_velocity is None else round(math.hypot(*own_velocity), 2)}
    if own_true is not None and own_estimate is not None:
        own['error_m'] = round(math.dist(own_true, own_estimate), 3)
    if axes is not None:
        own['axes_error_deg'] = round(_angle_deg(axes[0], true_camera['forward']), 3)
    rec['own'] = own
    rec['friends'] = {
        str(fid): {'estimate': _r(p), 'true': _r(true_bodies.get(fid)),
                   'error_m': (None if true_bodies.get(fid) is None
                               else round(math.dist(p, true_bodies[fid]), 3)),
                   'age_s': _r(friend_ages.get(fid), 3)}
        for fid, p in sorted(friends.items())}
    targets = {}
    for did, p in sorted(true_bodies.items()):
        if did == own_id or did in friends:
            continue
        to = tuple(a - b for a, b in zip(p, cam))
        targets[str(did)] = {'distance': round(math.hypot(*to), 2),
                             'above_m': round(-to[2], 2) + 0.0,
                             'in_front': sum(a * b for a, b in zip(to, true_camera['forward'])) > 0}
    rec['targets'] = targets

    rows = []
    for t in tracks:
        d = t.detection
        if d is None:
            continue
        box_range = camera.range_from_detection(detector_name, d.width, d.height)
        row = {'id': t.id, 'confirmed': bool(t.confirmed), 'friendly': marked.get(t.id),
               'box': [d.x, d.y, d.width, d.height], 'box_range': round(box_range, 2)}
        # What the box really is: the drone closest to its direction, from the true camera.
        true_dir = camera.direction(d.u, d.v, true_camera['forward'], true_camera['right'],
                                    true_camera['down'])
        candidates = []
        for did, p in true_bodies.items():
            if did == own_id:
                continue
            to = tuple(a - b for a, b in zip(p, cam))
            angle = _angle_deg(true_dir, to)
            if angle <= MATCH_MAX_DEG:
                distance = math.hypot(*to)
                score = angle + DEG_PER_LOG_RANGE * abs(math.log(box_range / distance))
                candidates.append((score, angle, did, distance, to))
        if candidates:
            _, angle, did, distance, to = min(candidates)
            f = camera.f
            row.update(is_drone=did, is_friend=did in friends, true_distance=round(distance, 2),
                       above_m=round(-to[2], 2) + 0.0,
                       elevation_deg=round(math.degrees(math.asin(-to[2] / distance)), 2),
                       # how much of the drone the box covers, metres: what the range rule needs
                       box_width_m=round(d.width * distance / f, 3),
                       box_height_m=round(d.height * distance / f, 3),
                       range_error_m=round(box_range - distance, 2),
                       range_error_pct=round(100 * (box_range - distance) / distance, 1),
                       direction_error_deg=round(angle, 2),
                       ambiguous=sum(c[1] <= angle + AMBIGUOUS_DEG for c in candidates) > 1)
        if axes is not None and own_estimate is not None and friends:
            direction = camera.direction(d.u, d.v, *axes)
            checks = {}
            for fid, p in sorted(friends.items()):
                to = tuple(a - b for a, b in zip(p, own_estimate))
                dist = math.hypot(*to)
                if dist < 1e-6:
                    continue
                age = friend_ages.get(fid, 0.0)
                checks[str(fid)] = {
                    'angle_deg': round(_angle_deg(direction, to), 2),
                    'gate_deg': round(gate_deg(dist, age), 2),
                    'distance': round(dist, 2),
                    'range_fits': dist / RANGE_FACTOR <= box_range <= dist * RANGE_FACTOR,
                    'fits': friendly_for(direction, box_range, own_estimate, {fid: p},
                                         {fid: age}) == fid}
            row['friend_check'] = checks
        rows.append(row)
    rec['tracks'] = rows
    return rec


def _median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    n = len(xs)
    return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2


def _stats(xs) -> str:
    if not xs:
        return 'none'
    a = [abs(x) for x in xs]
    return (f'median {_median(xs):+.2f}, median |err| {_median(a):.2f}, '
            f'max |err| {max(a):.2f} (n={len(xs)})')


SPEED_BINS = (0.0, 2.0, 5.0, 10.0, 15.0, math.inf)
DIST_BINS = (0.0, 10.0, 20.0, 40.0, 80.0, math.inf)


def _bin(value: float, edges) -> str:
    for lo, hi in zip(edges, edges[1:]):
        if lo <= value < hi:
            return f'{lo:g}-{hi:g}' if hi != math.inf else f'{lo:g}+'
    return '?'


HEIGHT_BINS = ((-math.inf, -5.0, 'below'), (-5.0, 5.0, 'level'), (5.0, math.inf, 'above'))


def _by_height(hostile_rows) -> list[str]:
    """The hostile below, level with and above the camera: box distance error and the
    metres of drone the box covers (median), per distance."""
    lines = ['Hostile by height against the camera (more than 5 m below / within 5 m / above):',
             '  where  distance  n    error %   elev deg  box covers w x h, sqrt (m)']
    for lo, hi, name in HEIGHT_BINS:
        rows = [t for _, t in hostile_rows if 'above_m' in t and lo <= t['above_m'] < hi
                and not t.get('ambiguous')]
        for dlo, dhi in zip(DIST_BINS, DIST_BINS[1:]):
            b = [t for t in rows if dlo <= t['true_distance'] < dhi]
            if not b:
                continue
            w = _median([t['box_width_m'] for t in b])
            h = _median([t['box_height_m'] for t in b])
            span = f'{dlo:g}-{dhi:g}' if dhi != math.inf else f'{dlo:g}+'
            lines.append(f'  {name:6} {span:>8} {len(b):4} {_median([t["range_error_pct"] for t in b]):+8.0f}'
                         f' {_median([t["elevation_deg"] for t in b]):+9.1f}   '
                         f'{w:.2f} x {h:.2f}, {math.sqrt(w * h):.2f}')
    return lines


def summary(records: list[dict]) -> str:
    """The errors of a log: by own speed and by distance, and every hostile marked friendly."""
    lines = [f'{len(records)} images']
    by_speed: dict[str, list] = {}
    axes = []
    friend_err = []
    for r in records:
        own = r.get('own', {})
        if own.get('error_m') is not None and own.get('speed') is not None:
            by_speed.setdefault(_bin(own['speed'], SPEED_BINS), []).append(own['error_m'])
        if own.get('axes_error_deg') is not None:
            axes.append(own['axes_error_deg'])
        friend_err += [f['error_m'] for f in r.get('friends', {}).values()
                       if f.get('error_m') is not None]
    lines.append('Own position error (m) by own speed (m/s):')
    for key in sorted(by_speed, key=lambda k: float(k.split('-')[0].rstrip('+'))):
        lines.append(f'  {key:>6}: {_stats(by_speed[key])}')
    lines.append(f'Other friendlies, position error (m): {_stats(friend_err)}')
    lines.append(f'Camera orientation error (deg): {_stats(axes)}')

    hostile_rows = [(r, t) for r in records for t in r.get('tracks', [])
                    if 'is_drone' in t and not t['is_friend']]
    lines += _by_height(hostile_rows)
    by_dist: dict[str, list] = {}
    by_dist_pct: dict[str, list] = {}
    for _, t in hostile_rows:
        key = _bin(t['true_distance'], DIST_BINS)
        by_dist.setdefault(key, []).append(t['range_error_m'])
        by_dist_pct.setdefault(key, []).append(t['range_error_pct'])
    lines.append('Hostile box distance error by true distance (m): error m | error %:')
    for key in sorted(by_dist, key=lambda k: float(k.split('-')[0].rstrip('+'))):
        lines.append(f'  {key:>6}: {_stats(by_dist[key])} | {_stats(by_dist_pct[key])}')
    dir_err = [t['direction_error_deg'] for _, t in hostile_rows]
    lines.append(f'Hostile direction error (deg): {_stats(dir_err)}')

    wrong = [(r, t) for r, t in hostile_rows if t.get('friendly') is not None]
    fits = [(r, t) for r, t in hostile_rows
            if any(c['fits'] for c in t.get('friend_check', {}).values())]
    lines.append(f'Hostile boxes: {len(hostile_rows)}, fitting a friendly: {len(fits)}, '
                 f'marked friendly: {len(wrong)}')
    for r, t in fits[:20]:
        fid, c = next((f, c) for f, c in t['friend_check'].items() if c['fits'])
        lines.append(f"  t {r['t']:.2f} T{t['id']}: hostile {t['true_distance']:.1f} m, box "
                     f"{t['box_range']:.1f} m; fits friendly {fid} at {c['distance']:.1f} m, "
                     f"{c['angle_deg']:.1f} deg (gate {c['gate_deg']:.1f}); own speed "
                     f"{r['own'].get('speed')}, own error {r['own'].get('error_m')} m"
                     + (' [MARKED]' if t.get('friendly') is not None else ''))
    if len(fits) > 20:
        lines.append(f'  ... {len(fits) - 20} more')
    return '\n'.join(lines)


def load(path: Path) -> list[dict]:
    out = []
    with open(path) as f:
        for line in f:
            if line.strip():
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    pass  # a line cut off when the detector stopped
    return out


class TruthLogger:
    """The ROS/Gazebo side: Gazebo's poses at each image's time, written with frame_record.
    Drone names are looked up again every NAMES_EVERY_S, for drones spawned later, on a
    thread of their own: `gz topic -l` takes about 2 s, and in the image loop it froze the
    detector (and its debug image in RViz) for that long every time."""

    NAMES_EVERY_S = 5.0

    def __init__(self, path: Path, world: str, drone_id: int, camera_link: str):
        import threading
        from swarm_backend.datagen.save_images import PoseHistory
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(path, 'a', buffering=1)
        self._poses = PoseHistory(world)
        self.drone_id, self.link = drone_id, camera_link
        self._names: dict[int, str] = {}
        self._stop = threading.Event()
        self._refresh_names()  # once before the first image, so it has names
        threading.Thread(target=self._keep_names, daemon=True).start()

    def _refresh_names(self) -> None:
        import subprocess
        from swarm_backend.datagen.save_images import drone_names
        from swarm_backend.sim.sensor_view import gz_topics
        try:
            _, names = drone_names(gz_topics())
        except (OSError, subprocess.SubprocessError):
            return  # Gazebo restarting: keep the names we have
        self._names = names  # one assignment: write() on the other thread sees old or new

    def _keep_names(self) -> None:
        while not self._stop.wait(self.NAMES_EVERY_S):
            self._refresh_names()

    def _drone_names(self, now: float) -> dict[int, str]:
        return self._names

    def write(self, stamp: float, tracks, camera, axes, detector_name: str,
              own_estimate, own_velocity, friends, friend_ages, marked) -> None:
        from swarm_backend.sim import ground_truth
        found = self._poses.at(stamp)
        names = self._drone_names(stamp)
        if found is None or self.drone_id not in names:
            return
        _, poses = found
        true_camera = ground_truth.camera_pose(poses, names[self.drone_id], self.link)
        if true_camera is None:
            return
        bodies = {i: p for i, n in names.items()
                  if (p := ground_truth.body_position(poses, n)) is not None}
        rec = frame_record(stamp, tracks, camera, axes, detector_name, true_camera, bodies,
                           self.drone_id, own_estimate, own_velocity, friends, friend_ages,
                           marked)
        self._file.write(json.dumps(rec) + '\n')

    def close(self) -> None:
        self._stop.set()
        self._file.close()


def main(argv=None) -> int:
    paths = [Path(p) for p in (argv if argv is not None else sys.argv[1:])]
    if not paths:
        print(__doc__)
        return 1
    for p in paths:
        print(f'== {p}')
        print(summary(load(p)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
