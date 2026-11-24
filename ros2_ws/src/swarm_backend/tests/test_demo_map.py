import json
import subprocess

import numpy as np

from swarm_backend.demo.fake_run import GRAVEYARD, make_run
from swarm_backend.demo.map_video import (BG, HOSTILE, CLOCK_BOX, map_state, project, render_frame,
                                          write_map)


def near_colour(img, u, v, colour, r=6):
    patch = img[max(0, v - r):v + r, max(0, u - r):u + r].reshape(-1, 3).astype(int)
    return bool((np.abs(patch - np.array(colour)).sum(axis=1) < 40).any())


def test_frame_shape_and_colours(tmp_path):
    make_run(tmp_path, takes=1, videos=False)
    st = map_state(tmp_path)
    img = render_frame(st, 110.0)
    assert img.shape == (1080, 1920, 3)
    u, v = project(st, 0.0, 0.0)
    assert img[v, u].tolist() != list(BG)  # the base is drawn there
    assert img[5, 960].tolist() == list(BG)


def test_hostile_hidden_in_graveyard(tmp_path):
    truth = make_run(tmp_path, takes=1, videos=False)
    st = map_state(tmp_path)
    hit = truth['hit'][0]
    before = render_frame(st, hit - 0.5)
    after = render_frame(st, hit + 0.5)
    from swarm_backend.ui.camera_video import pose_at
    p = pose_at(st.sims, st.rows, 'x500_4', hit - 0.3)[0]
    u, v = project(st, p[0], p[1])
    assert near_colour(before, u, v, HOSTILE)
    assert not near_colour(after, u, v, HOSTILE)
    gu, gv = project(st, GRAVEYARD[0], GRAVEYARD[1])
    if 0 <= gu < 1920 and 0 <= gv < 1080:
        assert not near_colour(after, gu, gv, HOSTILE, r=20)


def test_map_before_first_pose_holds(tmp_path):
    make_run(tmp_path, takes=1, videos=False)
    st = map_state(tmp_path)
    a, b = render_frame(st, st.sims[0] - 5.0), render_frame(st, st.sims[0])
    x0, y0, x1, y1 = CLOCK_BOX
    a[y0:y1, x0:x1] = 0
    b[y0:y1, x0:x1] = 0
    assert np.array_equal(a, b)


def test_write_map_duration(tmp_path):
    make_run(tmp_path, takes=1, videos=False)
    out = write_map(tmp_path, 110.0, 4.0, tmp_path / 'm.mp4', size=(640, 360))
    info = json.loads(subprocess.run(
        ['ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0', '-show_entries',
         'stream=nb_read_frames,r_frame_rate,width', '-of', 'json', str(out)],
        capture_output=True, text=True, check=True).stdout)['streams'][0]
    assert info['nb_read_frames'] == '100' and info['r_frame_rate'] == '25/1'
    assert info['width'] == 640


# --- step 3: truth vs. belief ------------------------------------------------------------------

def shift_detections(folder, east, keep_after_hit=0.0, hit=None):
    """Move every detection `east` metres, across the hostile's path (and keep it
    `keep_after_hit` s past the hit)."""
    path = folder / 'ui.jsonl'
    lines, last = [], None
    for raw in path.read_text().splitlines():
        d = json.loads(raw)
        for v in d['state']['drones']:
            if v.get('detection'):
                v['detection']['east'] += east
                last = (v['id'], dict(v['detection']))
            elif keep_after_hit and last and hit and hit <= d['sim'] <= hit + keep_after_hit and v['id'] == last[0]:
                v['detection'] = last[1]
        lines.append(json.dumps(d))
    path.write_text('\n'.join(lines) + '\n')


def ring_hit(img, u, v, colour, r=9, tol=3):
    for a in range(0, 360, 15):
        import math
        x, y = int(u + r * math.cos(math.radians(a))), int(v + r * math.sin(math.radians(a)))
        if near_colour(img, x, y, colour, r=tol):
            return True
    return False


def test_estimate_drawn(tmp_path):
    truth = make_run(tmp_path, takes=1, videos=False)
    shift_detections(tmp_path, 40.0)
    st = map_state(tmp_path)
    sim = truth['sighting'][0] + 2.0
    from swarm_backend.ui.camera_video import pose_at
    h = pose_at(st.sims, st.rows, 'x500_4', sim)[0]
    u, v = project(st, h[0] - 1.0 + 40.0, h[1] + 1.5)
    assert ring_hit(render_frame(st, sim), u, v, HOSTILE)
    make_run(tmp_path / 'nodet', takes=1, videos=False, detect=False)
    st2 = map_state(tmp_path / 'nodet')
    assert not ring_hit(render_frame(st2, sim), u, v, HOSTILE)


def test_estimate_off_map(tmp_path):
    truth = make_run(tmp_path, takes=1, videos=False)
    shift_detections(tmp_path, 2000.0)
    st = map_state(tmp_path)
    img = render_frame(st, truth['sighting'][0] + 1.0)
    full_rows = [(np.abs(img[y].astype(int) - np.array(BG)).sum(axis=1) > 40).all() for y in range(img.shape[0])]
    assert not any(full_rows)


def test_no_estimate_after_hit(tmp_path):
    truth = make_run(tmp_path, takes=1, videos=False)
    hit = truth['hit'][0]
    shift_detections(tmp_path, 40.0, keep_after_hit=2.0, hit=hit)
    st = map_state(tmp_path)
    from swarm_backend.ui.camera_video import pose_at
    h = pose_at(st.sims, st.rows, 'x500_4', hit - 0.2)[0]
    u, v = project(st, h[0] + 39.0, h[1] + 1.5)
    assert not near_colour(render_frame(st, hit + 1.0), u, v, HOSTILE, r=30)


def test_velocity_arrow(tmp_path):
    truth = make_run(tmp_path, takes=1, videos=False)
    st = map_state(tmp_path)
    sim = truth['sighting'][0] + 2.0
    line = st.ui.at(sim)['state']
    view = next(v for v in line['drones'] if v['id'] == 3)
    vel = view['attack']['velocity']
    from swarm_backend.ui.camera_video import pose_at
    p = pose_at(st.sims, st.rows, 'x500_gimbal_3', sim)[0]
    from swarm_backend.demo.map_video import FRIEND
    u, v = project(st, p[0] + 1.5 * vel['east'], p[1] + 1.5 * vel['north'])
    assert near_colour(render_frame(st, sim), u, v, FRIEND, r=4)


def test_map_draws_birds_that_appear_later(tmp_path):
    # Birds spawned mid-run (baseline_eval --birds) are in later pose rows only; green dots.
    truth = make_run(tmp_path, takes=1, videos=False, birds_late=True)
    st = map_state(tmp_path)
    t = truth['release'][0] + 2.0
    img = render_frame(st, t)
    from swarm_backend.ui.camera_video import pose_at
    p = pose_at(st.sims, st.rows, 'bird_7', t)[0]
    u, v = project(st, p[0], p[1])
    assert near_colour(img, u, v, (80, 200, 80))
