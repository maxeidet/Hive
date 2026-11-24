import json
import math

import pytest

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.takes import MOMENTS, find_takes, save_takes, source_ranges
from swarm_backend.ui.camera_video import load_poses, pose_at


def test_two_takes_found(tmp_path):
    truth = make_run(tmp_path, takes=2, videos=False)
    takes = find_takes(tmp_path)
    assert [t.number for t in takes] == [1, 2]
    for t, rel, see, hit in zip(takes, truth['release'], truth['sighting'], truth['hit']):
        assert t.moments['release'] == pytest.approx(rel, abs=0.1)
        assert t.moments['first_sighting'] == pytest.approx(see, abs=0.1)
        assert t.moments['hit'] == pytest.approx(hit, abs=0.1)
        assert set(t.moments) <= set(MOMENTS)


def test_attacker_per_take(tmp_path):
    make_run(tmp_path, takes=2, videos=False)
    assert [t.attacker for t in find_takes(tmp_path)] == [3, 2]


def test_take_without_hit(tmp_path):
    make_run(tmp_path, takes=2, hit=[True, False], videos=False)
    t2 = find_takes(tmp_path)[1]
    assert 'hit' not in t2.moments and 'return' not in t2.moments
    assert t2.facts['hit'] is False
    assert t2.facts['sighting_to_hit_s'] is None


def test_single_take_without_teleport(tmp_path):
    truth = make_run(tmp_path, takes=1, teleport=False, videos=False)
    takes = find_takes(tmp_path)
    assert len(takes) == 1
    sims, _ = load_poses(tmp_path / 'poses.jsonl')
    assert takes[0].moments['start'] == pytest.approx(sims[0])
    assert takes[0].moments['release'] == pytest.approx(truth['release'][0], abs=0.1)


def test_facts_values(tmp_path):
    truth = make_run(tmp_path, takes=1, videos=False)
    t = find_takes(tmp_path)[0]
    assert t.facts['hit'] is True and t.facts['attacker'] == 3
    assert t.facts['closest_pass_m'] < 1.6
    sims, rows = load_poses(tmp_path / 'poses.jsonl')
    s = truth['sighting'][0]
    a, h = pose_at(sims, rows, 'x500_gimbal_3', s)[0], pose_at(sims, rows, 'x500_4', s)[0]
    assert t.facts['sighting_range_m'] == pytest.approx(math.dist(a, h), abs=1.0)
    assert t.facts['sighting_to_hit_s'] == pytest.approx(4.0, abs=0.15)
    assert t.facts['release_to_sighting_s'] == pytest.approx(10.0, abs=0.15)
    assert t.facts['setup'] == 'yolo-bytetrack-mpc'


def test_takes_json_written(tmp_path):
    make_run(tmp_path, takes=2, videos=False)
    takes = find_takes(tmp_path)
    path = save_takes(tmp_path, takes)
    data = json.loads(path.read_text())
    assert [d['moments']['hit'] for d in data] == [t.moments['hit'] for t in takes]


def test_source_ranges(tmp_path):
    make_run(tmp_path, takes=1)
    ranges = source_ranges(tmp_path)
    assert ranges['debug_d3'] == (100.0, 160.0)
    assert 'poses' in ranges and 'ui' in ranges


# --- the real scenario flow (final review): no hit event, a climb, the hostile's own arming,
# --- and the hostile moved back to its spawn point at the end

def test_hit_from_the_graveyard_without_an_event(tmp_path):
    truth = make_run(tmp_path, takes=2, videos=False, hit_event=False)
    takes = find_takes(tmp_path)
    assert [t.facts['hit'] for t in takes] == [True, True]
    for t, hit, att in zip(takes, truth['hit'], truth['attacker']):
        assert t.moments['hit'] == pytest.approx(hit, abs=0.1)
        assert t.attacker == att


def test_release_ignores_the_climb(tmp_path):
    truth = make_run(tmp_path, takes=1, videos=False)
    t = find_takes(tmp_path)[0]
    assert t.moments['release'] == pytest.approx(truth['release'][0], abs=0.1)


def test_takeoff_and_landed_are_the_defenders(tmp_path):
    make_run(tmp_path, takes=2, videos=False)
    for t in find_takes(tmp_path):
        assert t.moments['takeoff'] == pytest.approx(102.0)


def test_no_take_for_the_reset_after_landing(tmp_path):
    make_run(tmp_path, takes=2, videos=False)
    assert len(find_takes(tmp_path)) == 2


def test_error_facts(tmp_path):
    make_run(tmp_path, takes=1, videos=False)
    f = find_takes(tmp_path)[0].facts
    assert f['est_error_median_m'] == pytest.approx(1.80, abs=0.05)
    assert f['est_error_at_sighting_m'] == pytest.approx(1.80, abs=0.05)


def test_error_facts_without_detection(tmp_path):
    make_run(tmp_path, takes=1, videos=False, detect=False)
    f = find_takes(tmp_path)[0].facts
    assert f['est_error_median_m'] is None and f['est_error_at_sighting_m'] is None
    assert f['hit'] is True


def test_idle_pseudo_take(tmp_path):
    import json as _json
    make_run(tmp_path, takes=1, videos=False)
    info = _json.loads((tmp_path / 'run.json').read_text())
    info['command'] = ['idle', '60']
    (tmp_path / 'run.json').write_text(_json.dumps(info))
    takes = find_takes(tmp_path)
    assert len(takes) == 1 and takes[0].name == 'overview'
    assert set(takes[0].moments) == {'start', 'end'}


def test_miss_taken_out_is_not_a_hit(tmp_path):
    # Gazebo moves the hostile to the graveyard after every run, hit or not (surveillance.py fly)
    make_run(tmp_path, takes=2, hit=[True, False], videos=False)
    takes = find_takes(tmp_path)
    assert [t.facts['hit'] for t in takes] == [True, False]
    assert 'hit' not in takes[1].moments


def test_sighting_before_the_release_is_not_the_engagement(tmp_path):
    # A defender sees the hostile parked at its start point and is sent at it (demo-ui3,
    # 24 Nov): the engagement's sighting, intercept and attacker are those after the release.
    truth = make_run(tmp_path, videos=False, early_sighting=True)
    t = find_takes(tmp_path)[0]
    assert t.moments['first_sighting'] == pytest.approx(truth['sighting'][0], abs=0.1)
    assert t.moments['intercept'] > t.moments['release']
    assert t.attacker == truth['attacker'][0]


# --- two hostiles at once (Demo scenarios plan, Task 8) ----------------------------------------

def test_two_hostile_moments(tmp_path):
    truth = make_run(tmp_path, videos=False, hostiles=2)
    takes = find_takes(tmp_path)
    assert len(takes) == 1
    m, f = takes[0].moments, takes[0].facts
    assert m['hit_1'] == pytest.approx(truth['hit'][0], abs=0.1)
    assert m['hit_2'] == pytest.approx(truth['hit2'], abs=0.1)
    assert m['hit'] == max(m['hit_1'], m['hit_2'])
    assert m['first_sighting'] == min(m['first_sighting_1'], m['first_sighting_2'])
    assert f['hits'] == '2/2' and f['attacker_1'] != f['attacker_2']
    assert f['attacker_2'] == truth['attacker2']


def test_takes_one_of_two_hit(tmp_path):
    make_run(tmp_path, videos=False, hostiles=2, hit=[True, False])
    m, f = find_takes(tmp_path)[0].moments, find_takes(tmp_path)[0].facts
    assert 'hit_1' in m and 'hit_2' not in m and 'hit' not in m
    assert f['hits'] == '1/2'


def test_single_hostile_moments_unchanged(tmp_path):
    make_run(tmp_path, takes=2, videos=False)
    for t in find_takes(tmp_path):
        assert not any(k.endswith(('_1', '_2')) for k in t.moments)
        assert 'hits' not in t.facts or t.facts['hits'] is None


def test_run_json_lists_both_hostiles(tmp_path):
    make_run(tmp_path, videos=False, hostiles=2)
    from swarm_backend.demo.runinfo import read_run_info
    assert read_run_info(tmp_path).hostiles == ['x500_4', 'x500_5']
