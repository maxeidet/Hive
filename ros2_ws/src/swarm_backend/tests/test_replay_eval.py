"""The threshold replay (detect/replay_eval.py) on a small made-up recording."""
import json
import math
import random

from swarm_backend.detect import replay_eval as rp
from swarm_backend.detect.camera import CameraModel

CAMERA = CameraModel()
NORTH = [[0.0, 0.0, -30.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]  # pos, f, r, d


def hostile_at(i, t):
    """Hostile 2 (left) and 3 (right) fly in from 150 m at 5 m/s, 10 m above the camera."""
    side = -1 if i == 2 else 1
    r = 150.0 - 5.0 * t
    a = math.radians(20 * side)
    return [r * math.cos(a), r * math.sin(a), -40.0]


def write_run(folder, seconds=10.0, weak_drone=None, seed=0, friend_at=None):
    rng = random.Random(seed)
    folder.mkdir(parents=True)
    with open(folder / 'truth.jsonl', 'w') as f:
        for n in range(int(seconds * 50) + 1):
            t = 100.0 + n / 50
            f.write(json.dumps({'t': t, 'cam': NORTH, 'hostiles': {
                str(i): hostile_at(i, t - 100.0) for i in (2, 3)}}) + '\n')
    with open(folder / 'events.jsonl', 'w') as f:
        f.write(json.dumps({'t': 100.0, 'event': 'start'}) + '\n')
    cam = {'position': NORTH[0], 'forward': NORTH[1], 'right': NORTH[2], 'down': NORTH[3]}
    with open(folder / 'detect.jsonl', 'w') as f:
        for n in range(1, int(seconds * 10)):
            t = 100.0 + n / 10
            raw = []
            for i in (2, 3):
                u, v, d = rp.project(cam, hostile_at(i, t - 100.0), CAMERA)
                size = max(2, round(0.5 * CAMERA.f / d))
                score = 0.25 if i == weak_drone else 0.8
                score += rng.uniform(-0.05, 0.05)
                raw.append([round(u - size / 2), round(v - size / 2), size, size, score])
            if friend_at is not None:  # one of our own drones, always seen
                u, v, d = rp.project(cam, friend_at, CAMERA)
                raw.append([round(u - 3), round(v - 3), 6, 6, 0.95])
            if n % 7 == 0:  # a false box now and then
                raw.append([100, 100, 4, 4, 0.2])
            f.write(json.dumps({'stamp': t, 'width': CAMERA.width, 'height': CAMERA.height,
                                'raw': raw, 'axes': NORTH[1:]}) + '\n')


def test_boxes_are_labelled_with_the_hostile_they_are_on(tmp_path):
    write_run(tmp_path / 'r')
    run = rp.load_run(tmp_path / 'r')
    assert len(run.images) == 99
    im = run.images[10]
    assert set(im.in_view) == {2, 3}
    assert im.labels[:2] == [2, 3] or set(im.labels[:2]) == {2, 3}
    assert all(lab is None for lab in run.images[6].labels[2:])  # the false box


def test_detector_table_shows_recall_by_distance(tmp_path):
    write_run(tmp_path / 'r', weak_drone=3)
    text = rp.detector_table([rp.load_run(tmp_path / 'r')], confs=(0.1, 0.5))
    lines = text.splitlines()
    assert lines[2].startswith('0.1 ') and '100%' in lines[2]
    assert lines[3].startswith('0.5 ') and '50%' in lines[3]  # the weak hostile is lost


def test_replay_scores_coverage_false_and_switches(tmp_path):
    write_run(tmp_path / 'r')
    run = rp.load_run(tmp_path / 'r')
    tracks = rp.track_run(run, rp.TrackerSetting(0.1, 0.5, 0.6, 3, 30))
    locked = rp.choose_run(run, tracks, rp.LockSetting(True, 1.0))
    unlocked = rp.choose_run(run, tracks, rp.LockSetting(False))
    assert locked.coverage > 0.9 and locked.false == 0
    assert locked.switches == 0  # stays on one hostile
    assert unlocked.switches > 5  # similar scores: best-score flips between them
    assert locked.first_m and 140 < locked.first_m[0] < 150


def test_a_weak_drone_is_only_tracked_when_low_reaches_it(tmp_path):
    write_run(tmp_path / 'r', weak_drone=2)
    run = rp.load_run(tmp_path / 'r')
    # high 0.5: drone 2 (score ~0.25) can never start a track, whatever low is
    tracks = rp.track_run(run, rp.TrackerSetting(0.1, 0.5, 0.6, 3, 30))
    on = {rp.label(s.detection, im.in_view) for im, snaps in zip(run.images, tracks)
          for s in snaps if s.confirmed}
    assert on == {3}
    tracks = rp.track_run(run, rp.TrackerSetting(0.1, 0.2, 0.2, 3, 30))
    on = {rp.label(s.detection, im.in_view) for im, snaps in zip(run.images, tracks)
          for s in snaps if s.confirmed}
    assert {2, 3} <= on


def test_the_grid_keeps_low_below_high_below_new():
    grid = rp.tracker_grid([0.1, 0.5], [0.4, 0.5], [0.4, 0.6], [3], [30])
    assert all(t.low < t.high <= t.new for t in grid)
    assert rp.TrackerSetting(0.1, 0.4, 0.4, 3, 30) in grid


def test_main_runs_end_to_end(tmp_path, capsys):
    write_run(tmp_path / 'a')
    write_run(tmp_path / 'b', weak_drone=3, seed=1)
    assert rp.main([str(tmp_path / 'a'), str(tmp_path / 'b'), '--low', '0.1', '--high', '0.3',
                    '--new', '0.4', '--k', '3', '--max-lost', '30', '--lock-timeout', '1',
                    '--top', '3']) == 0
    out = capsys.readouterr().out
    assert 'Best 3' in out and "Today's defaults" in out


def test_boxes_on_our_own_drone_count_apart_from_false_ones(tmp_path):
    friend = (60.0, 0.0, -30.0)  # straight ahead, 60 m
    write_run(tmp_path / 'r', friend_at=friend)
    run = rp.load_run(tmp_path / 'r', friendlies={0: friend})
    assert sum(any(im.friendly) for im in run.images) == len(run.images)
    tracks = rp.track_run(run, rp.TrackerSetting(0.1, 0.5, 0.6, 3, 30))
    s = rp.choose_run(run, tracks, rp.LockSetting(False))  # best score: our drone (0.95)
    assert s.friendly > 0.5 * s.published and s.false == 0
    text = rp.detector_table([run], confs=(0.1,))
    assert text.splitlines()[2].split()[-1] == '0.14'  # only the made-up false box


def test_ranking_puts_target_jumping_settings_last():
    calm = rp.Score(active=100, covered=50, published=50, switches=0)
    jumpy = rp.Score(active=100, covered=90, published=90, switches=50)
    items = [(None, None, jumpy), (None, None, calm)]
    assert sorted(items, key=lambda r: rp.rank_key(r, 10.0))[0][2] is calm
