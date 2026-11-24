"""The truth log (detect/truth_log.py): each image's estimates against Gazebo's truth."""
import json
import math

import pytest

from swarm_backend.detect.camera import BOX_SIZE_M, CameraModel
from swarm_backend.detect.detectors import Detection
from swarm_backend.detect.tracking import Track
from swarm_backend.detect.truth_log import frame_record, load, main, summary

CAMERA = CameraModel()
# The camera of drone 2, 30 m up, looking north and level.
AXES = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
TRUE_CAMERA = {'position': (0.0, 0.0, -30.0), 'forward': AXES[0], 'right': AXES[1],
               'down': AXES[2]}


def box_at(distance: float, u: float = None, v: float = None) -> Detection:
    w = round(CAMERA.f * BOX_SIZE_M / distance)  # a box as big as YOLO's at that distance
    u = CAMERA.cx if u is None else u
    v = CAMERA.cy if v is None else v
    return Detection(round(u - w / 2), round(v - w / 2), w, w, 0.9)


def record(own_estimate=(0.0, 0.0, -30.0), friend_at=(40.0, 0.0, -30.0), marked=None):
    """The hostile (4) 20 m ahead, friendly 1 40 m ahead behind it, both in one line."""
    hostile = Track(7, (CAMERA.cx, CAMERA.cy))
    hostile.detection, hostile.confirmed = box_at(20.0), True
    bodies = {2: (0.0, 0.0, -30.0), 1: (40.0, 0.0, -30.0), 4: (20.0, 0.0, -30.0),
              3: (0.0, 80.0, -30.0)}
    return frame_record(12.0, [hostile], CAMERA, AXES, 'yolo', TRUE_CAMERA, bodies, 2,
                        own_estimate, (10.0, 0.0, 0.0), {1: friend_at, 3: (0.0, 80.0, -30.0)},
                        {1: 0.05, 3: 0.05}, marked or {})


def test_a_record_compares_the_box_with_the_drone_it_really_is():
    rec = record()
    assert rec['own']['error_m'] == 0.0 and rec['own']['speed'] == 10.0
    assert rec['own']['axes_error_deg'] == 0.0
    assert rec['targets'] == {'4': {'distance': 20.0, 'above_m': 0.0, 'in_front': True}}
    (t,) = rec['tracks']
    assert t['is_drone'] == 4 and not t['is_friend'] and t['true_distance'] == 20.0
    assert abs(t['range_error_pct']) < 3 and t['direction_error_deg'] < 0.5
    check = t['friend_check']['1']
    assert check['angle_deg'] == pytest.approx(0.0, abs=0.1)
    assert not check['range_fits'] or check['distance'] / 2 <= t['box_range']
    json.dumps(rec)  # one JSON line


def test_the_friendly_check_terms_show_why_a_hostile_fits():
    # The friendly estimated 25 m ahead (off by 15 m): the hostile's box distance fits it.
    rec = record(friend_at=(25.0, 0.0, -30.0), marked={7: 1})
    t = rec['tracks'][0]
    assert t['friend_check']['1']['fits'] and t['friendly'] == 1
    assert rec['friends']['1']['error_m'] == 15.0
    text = summary([rec])
    assert 'fitting a friendly: 1, marked friendly: 1' in text and '[MARKED]' in text


def test_the_summary_bins_by_speed_and_distance(tmp_path):
    path = tmp_path / 'drone_2.jsonl'
    path.write_text(json.dumps(record()) + '\n' + json.dumps(record((1.0, 0.0, -30.0)))
                    + '\n{"t": 1')  # the last line cut off
    recs = load(path)
    assert len(recs) == 2
    text = summary(recs)
    assert '10-15:' in text and '20-40:' in text
    assert main([str(path)]) == 0


def test_a_hostile_below_is_grouped_below_with_its_box_size():
    hostile = Track(7, (CAMERA.cx, CAMERA.cy))
    bodies = {2: (0.0, 0.0, -30.0), 4: (20.0, 0.0, -20.0)}  # 10 m below, 22 m away
    # level camera: 10 m down over 20 m ahead shows at f * 10 / 20 px below the centre
    hostile.detection, hostile.confirmed = box_at(20.0, v=CAMERA.cy + CAMERA.f * 0.5), True
    rec = frame_record(1.0, [hostile], CAMERA, AXES, 'yolo', TRUE_CAMERA, bodies, 2,
                       (0.0, 0.0, -30.0), (0.0, 0.0, 0.0), {}, {}, {})
    t = rec['tracks'][0]
    assert t['above_m'] == -10.0 and t['elevation_deg'] == pytest.approx(-26.57, abs=0.01)
    assert rec['targets']['4']['above_m'] == -10.0
    assert t['box_width_m'] == pytest.approx(BOX_SIZE_M * math.dist(bodies[2], bodies[4]) / 20.0, rel=0.03)
    text = summary([rec])
    assert '  below ' in text and '  level ' not in text
