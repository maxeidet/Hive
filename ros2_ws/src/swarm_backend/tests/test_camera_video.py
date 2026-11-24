import json
import math

import pytest

from swarm_backend.ui import camera_video as cv
from swarm_backend.ui.camera_video import Director, Tracker, look_at, pose_at, project

W, H, HFOV = 1280, 720, 1.2
IDENTITY = (0.0, 0.0, 0.0, 1.0)


def centre_of(cam_p, target):
    u, v, _ = project(cam_p, look_at(cam_p, target), target, HFOV, W, H)
    return u, v


@pytest.mark.parametrize('target', [(10, 0, 0), (0, 10, 0), (-10, -5, 3), (5, 5, -8), (0.1, 0, 30)])
def test_look_at_puts_the_target_in_the_middle(target):
    u, v = centre_of((0.0, 0.0, 0.0), target)
    assert u == pytest.approx(W / 2, abs=1e-6) and v == pytest.approx(H / 2, abs=1e-6)


def test_projection_left_up_and_behind():
    cam = (0.0, 0.0, 0.0)
    q = look_at(cam, (10.0, 0.0, 0.0))                  # looking east
    u, v, depth = project(cam, q, (10.0, 1.0, 1.0), HFOV, W, H)   # a bit north (left) and up
    assert depth == pytest.approx(10.0) and u < W / 2 and v < H / 2
    assert project(cam, q, (-10.0, 0.0, 0.0), HFOV, W, H) is None


def poses(**models):
    return {n: (p, IDENTITY) for n, p in models.items()}


CFG = {'follow': '@attacker', 'offset': [-6.0, 0.0, 2.0], 'look_at_model': 'x500_4'}
DEF = ['x500_gimbal_1', 'x500_gimbal_2', 'x500_gimbal_3']


def test_watch_stays_behind_the_drone_as_seen_from_the_target():
    t = Tracker((450.0, -450.0))
    t.update(poses(x500_gimbal_1=(0.0, 150.0, 40.0), x500_4=(0.0, 250.0, 40.0)))
    d = Director('w', {'follow': 'x500_gimbal_1', 'offset': [-6.0, 0.0, 2.0], 'look_at_model': 'x500_4'},
                 DEF, 'x500_4', smooth_s=0.0)
    cam, q = d.step(t, 0.0, 0.0)
    assert cam == pytest.approx((0.0, 144.0, 42.0))     # 6 m south of it: away from the target
    u, v, _ = project(cam, q, (0.0, 250.0, 40.0), HFOV, W, H)
    assert u == pytest.approx(W / 2) and v == pytest.approx(H / 2)


def test_attacker_is_the_nearest_defender_with_hysteresis():
    t = Tracker((450.0, -450.0))
    d = Director('a', CFG, DEF, 'x500_4', smooth_s=0.0)
    t.update(poses(x500_gimbal_1=(-75.0, 130.0, 40.0), x500_gimbal_2=(0.0, 150.0, 40.0),
                   x500_gimbal_3=(75.0, 130.0, 40.0), x500_4=(60.0, 260.0, 40.0)))
    d.step(t, 10.0, 0.04)
    assert d.attacker == 'x500_gimbal_2'                # nearest at the start (≈124 m vs 131 m)
    t.update(poses(x500_gimbal_3=(75.0, 140.0, 40.0)))  # 3 a little nearer: not enough to switch
    d.step(t, 15.0, 0.04)
    assert d.attacker == 'x500_gimbal_2'
    t.update(poses(x500_gimbal_3=(70.0, 230.0, 40.0)))  # 3 much nearer
    d.step(t, 15.5, 0.04)
    assert d.attacker == 'x500_gimbal_3'
    t.update(poses(x500_gimbal_1=(60.0, 255.0, 40.0)))  # 1 even nearer, but within the hold time
    d.step(t, 16.0, 0.04)
    assert d.attacker == 'x500_gimbal_3'


def test_hostile_at_the_graveyard_is_held_and_let_go_for_the_next_run():
    t = Tracker((450.0, -450.0))
    t.update(poses(x500_4=(10.0, 200.0, 40.0)))
    t.update(poses(x500_4=(450.0, -450.0, 0.5)))        # hit: moved to the graveyard
    assert t.pos['x500_4'] == (10.0, 200.0, 40.0) and 'x500_4' in t.held
    t.update(poses(x500_4=(-30.0, 400.0, 40.0)))        # the next run's start: a jump, but not there
    assert t.pos['x500_4'] == (-30.0, 400.0, 40.0) and not t.held


def test_smoothing_moves_part_of_the_way_per_step():
    t = Tracker((450.0, -450.0))
    spec = {'position': [0.0, 0.0, 10.0], 'look_at_point': [0.0, 100.0, 10.0], 'track_model': 'x500_4'}
    d = Director('tripod', spec, DEF, 'x500_4', smooth_s=0.3)
    t.update(poses(x500_4=(0.0, 100.0, 10.0)))
    d.step(t, 0.0, 0.0)
    t.update(poses(x500_4=(50.0, 100.0, 10.0)))
    d.step(t, 0.3, 0.3)                                  # one time constant: 63 % of the way
    assert d.aim[0] == pytest.approx(50.0 * (1 - math.exp(-1)))


def test_pose_at_interpolates_between_logged_poses(tmp_path):
    lines = [{'sim': 10.0, 'poses': {'c': [0, 0, 0, 0, 0, 0, 1]}},
             {'sim': 10.04, 'poses': {'c': [4, 0, 0, 0, 0, 0, 1]}}]
    path = tmp_path / 'poses.jsonl'
    path.write_text('\n'.join(json.dumps(x) for x in lines) + '\n{"torn')
    sims, rows = cv.load_poses(path)
    p, q = pose_at(sims, rows, 'c', 10.01)
    assert p == pytest.approx((1.0, 0.0, 0.0)) and q == pytest.approx(IDENTITY)
    assert pose_at(sims, rows, 'missing', 10.01) is None


def test_labels():
    lines, kind = cv.label_lines('x500_gimbal_3', {'id': 3, 'status': 'intercept',
                                                   'attack': {'range_m': 41.6}}, 'x500_4')
    assert lines == ['D3 defender', 'intercept  est. 42 m'] and kind == 'engaged'
    assert cv.label_lines('x500_4', None, 'x500_4') == (['D4 hostile'], 'hostile')
    assert cv.ascii_text('Drone 3 searching → intercept') == 'Drone 3 searching -> intercept'


def test_camera_model_has_no_visual_or_collision():
    sdf = cv.camera_model('rec_cam_x', '/hive_rec/x/image', 1280, 720, 1.2, 25)
    assert '<visual' not in sdf and '<collision' not in sdf
    assert '<gravity>false</gravity>' in sdf and '<topic>/hive_rec/x/image</topic>' in sdf


def test_every_configured_live_angle_works(tmp_path):
    from pathlib import Path
    cfg = json.loads((Path(__file__).resolve().parents[4] / 'recording' / 'recording_config.json').read_text())
    t = Tracker(tuple(cfg['graveyard_enu']))
    t.update(poses(x500_gimbal_1=(-75.0, 130.0, 40.0), x500_gimbal_2=(0.0, 150.0, 40.0),
                   x500_gimbal_3=(75.0, 130.0, 40.0), x500_4=(60.0, 260.0, 40.0)))
    for name, spec in cfg['replay_angles'].items():
        d = Director(name, spec, cfg['defenders'], cfg['hostile'], cfg['live_camera']['smooth_s'])
        assert d.step(t, 0.0, 0.0) is not None, name


def test_watch_keeps_its_direction_past_the_target_and_stays_on_the_drone_after_a_hit():
    """The run of 2026-11-18: D3 flew past the place of the hit and the camera swung round."""
    t = Tracker((450.0, -450.0))
    d = Director('w', {'follow': 'x500_gimbal_3', 'offset': [-6.0, 0.0, 2.0], 'look_at_model': 'x500_4'},
                 DEF, 'x500_4', smooth_s=0.0)
    t.update(poses(x500_gimbal_3=(0.0, 100.0, 40.0), x500_4=(0.0, 150.0, 40.0)))
    d.step(t, 0.0, 0.0)
    t.update(poses(x500_gimbal_3=(0.0, 140.0, 40.0), x500_4=(0.0, 145.0, 40.0)))  # 5 m: frozen from here
    t.update(poses(x500_4=(450.0, -450.0, 0.5)))                                  # the hit
    for y in (150.0, 160.0):                                                       # D3 flies on, past it
        t.update(poses(x500_gimbal_3=(0.0, y, 40.0)))
        cam, q = d.step(t, 1.0, 0.04)
        assert cam == pytest.approx((0.0, y - 6.0, 42.0))                         # still behind it
        u, v, _ = project(cam, q, (0.0, y, 40.0), HFOV, W, H)
        assert u == pytest.approx(W / 2) and v == pytest.approx(H / 2)            # looking at D3


def test_orbit_goal_circles():
    import math
    spec = {'orbit': [0.0, 0.0], 'radius': 200.0, 'height': 70.0, 'period_s': 60.0,
            'look_at_point': [0.0, 100.0, 30.0]}
    d = Director('base_orbit', spec, DEF, 'x500_4', smooth_s=0.0)
    tr = Tracker((450.0, -450.0))
    goals = [d.goal(tr, 100.0 + t) for t in (0.0, 15.0, 30.0)]
    for cam, aim in goals:
        assert math.hypot(cam[0], cam[1]) == pytest.approx(200.0)
        assert cam[2] == pytest.approx(70.0) and aim == (0.0, 100.0, 30.0)
    a0, a1, a2 = (math.atan2(c[1], c[0]) for c, _ in goals)
    assert math.remainder(a1 - a0, 2 * math.pi) == pytest.approx(math.pi / 2)
    assert abs(math.remainder(a2 - a0, 2 * math.pi)) == pytest.approx(math.pi)


# --- birds: green in the clips (Demo scenarios plan, Task 3) ---------------------------------

def test_bird_label():
    assert cv.label_lines('bird_3', None, 'x500_4') == (['bird'], 'bird')
    assert cv.COLOURS['bird'] == (80, 200, 80)


def test_birds_are_labelled_models():
    assert cv.labelled_model('bird_1') and cv.labelled_model('x500_gimbal_2')
    assert cv.labelled_model('flock_2', birds_prefix='flock_')
    assert not cv.labelled_model('rec_cam_attacker_watch') and not cv.labelled_model('ground_plane')


def test_overlay_labels_birds_green():
    np = pytest.importorskip('numpy')
    img = np.zeros((H, W, 3), np.uint8)
    cam = (0.0, 0.0, 0.0)
    cv.draw_overlay(img, (cam, look_at(cam, (20.0, 0.0, 0.0))), {'bird_1': (20.0, 0.0, 0.0)}, {},
                    'x500_4', HFOV, 0.0, None)
    centre = img[H // 2 - 120:H // 2 + 60, W // 2 - 120:W // 2 + 120].reshape(-1, 3).astype(int)
    assert (np.abs(centre - np.array((80, 200, 80))).sum(axis=1) < 30).any()


# --- two hostiles (Demo scenarios plan, Task 8) ------------------------------------------------

def test_attacker_nearest_any_hostile():
    t = Tracker((450.0, -450.0))
    d = Director('a', {'follow': '@attacker', 'offset': [-6.0, 0.0, 2.0], 'look_at_model': '@target'},
                 DEF, ['x500_4', 'x500_5'], smooth_s=0.0)
    t.update(poses(x500_gimbal_1=(-75.0, 130.0, 40.0), x500_gimbal_2=(0.0, 150.0, 40.0),
                   x500_gimbal_3=(75.0, 130.0, 40.0), x500_4=(0.0, 400.0, 40.0),
                   x500_5=(-90.0, 160.0, 40.0)))
    d.step(t, 10.0, 0.04)
    assert d.attacker == 'x500_gimbal_1'


def test_target_watch_looks_at_the_nearest_hostile():
    t = Tracker((450.0, -450.0))
    t.update(poses(x500_gimbal_1=(0.0, 150.0, 40.0), x500_4=(300.0, 300.0, 40.0),
                   x500_5=(0.0, 250.0, 40.0)))
    d = Director('w', {'follow': 'x500_gimbal_1', 'offset': [-6.0, 0.0, 2.0], 'look_at_model': '@target'},
                 DEF, ['x500_4', 'x500_5'], smooth_s=0.0)
    cam, q = d.step(t, 0.0, 0.0)
    u, v, _ = project(cam, q, (0.0, 250.0, 40.0), HFOV, W, H)
    assert u == pytest.approx(W / 2) and v == pytest.approx(H / 2)
