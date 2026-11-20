"""The baseline evaluation on the surveillance arc (evaluation/arc_eval.py): his seeded
scenarios placed on our arc, and a Gazebo run's recordings turned into his Samples and
FrameRecords."""
import math
import random

import pytest

from swarm_backend.config import DetectConfig, InterceptConfig, SurveillanceConfig
from swarm_backend.detect.metrics import score_run as score_detect
from swarm_backend.evaluation import arc_eval
from swarm_backend.evaluation.arc_eval import (
    MAX_START_M, TruthPoint, first_detection, frame_records, intercept_samples,
    make_detect_scenario, make_intercept_scenario, sector_bearing_deg, slots)
from swarm_backend.intercept.interceptor import Status
from swarm_backend.evaluation.metrics import Outcome, score_run
from swarm_backend.evaluation.scenario import make_scenario

ICONFIG = InterceptConfig()
DCONFIG = DetectConfig()
ARC = SurveillanceConfig(arc_radius=150.0, altitude=30.0)
HFOV = 2.0


def angle_deg(a, b):
    dot = sum(x * y for x, y in zip(a, b))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / (math.hypot(*a) * math.hypot(*b))))))


@pytest.mark.parametrize('aspect', ICONFIG.eval_aspects)
@pytest.mark.parametrize('seed', [0, 1, 7, 29])
def test_the_aspect_holds_against_the_defender_at_detect_range(aspect, seed):
    scenario, index = make_intercept_scenario(aspect, 8.0, seed, ICONFIG, ARC)
    assert scenario.interceptor_start == pytest.approx(slots(ARC)[index])
    # Where the track (horizontally, as in his scenario) first comes detect_range from the
    # defender: solve |start + s*d - me| = r; at 90 deg the root is double (closest point).
    sx, sy = scenario.target_start[:2]
    gx, gy = scenario.target_goal[:2]
    length = math.hypot(gx - sx, gy - sy)
    d = ((gx - sx) / length, (gy - sy) / length)
    mx, my = scenario.interceptor_start[:2]
    px, py = sx - mx, sy - my
    b = px * d[0] + py * d[1]
    c = px * px + py * py - ICONFIG.detect_range ** 2
    s_cross = -b - math.sqrt(max(0.0, b * b - c))
    assert 0.0 < s_cross < length
    target = (sx + s_cross * d[0], sy + s_cross * d[1])
    to_me = (mx - target[0], my - target[1], 0.0)
    velocity = (d[0], d[1], 0.0)
    assert angle_deg(to_me, (velocity[0], velocity[1], 0.0)) == pytest.approx(aspect, abs=1.0)


def test_same_seed_same_draws_as_his_scenario():
    for seed in range(30):
        his = make_scenario(30.0, 8.0, seed, ICONFIG)
        ours, _ = make_intercept_scenario(30.0, 8.0, seed, ICONFIG, ARC)
        assert ours.target_start[2] == his.target_start[2]  # the altitude draw
        assert ours.target_goal[:2] == (0.0, 0.0)
        assert ours.interceptor_start[2] == -30.0


def test_every_defender_gets_scenarios_and_the_start_is_out_of_reach():
    used = set()
    for seed in range(30):
        scenario, index = make_intercept_scenario(0.0, 4.0, seed, ICONFIG, ARC)
        used.add(index)
        start = math.dist(scenario.target_start[:2], scenario.interceptor_start[:2])
        assert start >= ICONFIG.detect_range + 50.0
        assert math.hypot(*scenario.target_start[:2]) <= MAX_START_M
    assert used == {0, 1, 2}


def bearing_deg(point):
    return math.degrees(math.atan2(point[1], point[0]))


def test_a_sector_start_is_seeded_inside_the_middle_drones_30_degrees():
    bearings = []
    for seed in range(50):
        scenario, index = make_intercept_scenario(0.0, 4.0, seed, ICONFIG, ARC, sector=1)
        assert index == 1 and scenario.interceptor_start == pytest.approx(slots(ARC)[1])
        b = bearing_deg(scenario.target_start)
        assert -15.0 <= b <= 15.0
        assert b == pytest.approx(sector_bearing_deg(seed, 1, ARC))
        assert scenario.target_goal[:2] == (0.0, 0.0)
        assert math.hypot(*scenario.target_start[:2]) == pytest.approx(
            ARC.arc_radius + ICONFIG.detect_range + arc_eval.APPROACH_M)
        bearings.append(b)
    assert min(bearings) < -10 and max(bearings) > 10  # spread over the sector
    # the same start every time, and at every altitude and speed
    again, _ = make_intercept_scenario(0.0, 4.0, 7, ICONFIG, ARC, sector=1)
    other, _ = make_intercept_scenario(0.0, 8.0, 7, ICONFIG, ARC, altitude=40.0, sector=1)
    assert again.target_start == make_intercept_scenario(0.0, 4.0, 7, ICONFIG, ARC,
                                                         sector=1)[0].target_start
    assert other.target_start[:2] == again.target_start[:2]
    lo, hi = sector_bearing_deg(3, 0, ARC), sector_bearing_deg(3, 2, ARC)
    assert -45.0 <= lo <= -15.0 and 15.0 <= hi <= 45.0
    with pytest.raises(ValueError):
        sector_bearing_deg(0, 3, ARC)


def test_the_altitude_replaces_the_drawn_one_and_nothing_else():
    for altitude in (20.0, 30.0, 40.0):
        ours, i = make_intercept_scenario(30.0, 8.0, 5, ICONFIG, ARC, altitude=altitude)
        drawn, j = make_intercept_scenario(30.0, 8.0, 5, ICONFIG, ARC)
        assert ours.target_start[2] == ours.target_goal[2] == -altitude
        assert ours.target_start[:2] == drawn.target_start[:2] and i == j
        assert ours.interceptor_start[2] == -30.0


def test_detect_scenario_comes_from_inside_the_arc_and_starts_on_the_ground_plane():
    for seed in range(30):
        s = make_detect_scenario(seed, DCONFIG, ARC)
        bearing = math.degrees(math.atan2(s.hostile.start[1], s.hostile.start[0]))
        assert abs(bearing) <= ARC.arc_deg / 2 + 1e-9
        assert math.hypot(*s.hostile.start[:2]) == pytest.approx(MAX_START_M)
        assert s.birds == () and len(s.defenders) == 3 and s.perimeter_radius == 150.0
        assert DCONFIG.hostile_altitude_min <= -s.hostile.start[2] <= DCONFIG.hostile_altitude_max
    # same altitude draw as his scenario
    rng = random.Random(4)
    assert make_detect_scenario(4, DCONFIG, ARC).hostile.start[2] == -rng.uniform(
        DCONFIG.hostile_altitude_min, DCONFIG.hostile_altitude_max)


# --- recordings -> metrics ----------------------------------------------------------

def camera_north(position):
    """A level camera at `position` looking north."""
    return {'position': position, 'forward': (1.0, 0.0, 0.0), 'right': (0.0, 1.0, 0.0),
            'down': (0.0, 0.0, 1.0)}


def flight(t_end=10.0, hit=True):
    """Defender 1 at (150, 0, -30) looking north; the hostile comes south straight at it
    from 40 m, at 4 m/s, and (if hit) is hit at t=10."""
    truth, records, statuses = [], {1: []}, []
    me = (150.0, 0.0, -30.0)
    for i in range(int(t_end * 50) + 1):
        t = i * 0.02
        hostile = (190.0 - 4.0 * t, 0.0 if hit else 3.0, -30.0)
        truth.append(TruthPoint(t, {1: me}, {1: camera_north(me)}, hostile))
    for i in range(int(t_end * 10) + 1):  # images at 10 Hz, confirmed from t = 1 s
        t = i * 0.1
        confirmed = [[640.0, 360.0 if hit else 360.0, 10, 10, 7]] if t >= 1.0 else []
        records[1].append({'stamp': t, 'width': 1280, 'height': 720, 'boxes': [],
                           'confirmed': confirmed})
        statuses.append((t, {1: Status.TRACKING if t >= 1.0 else Status.IDLE}))
    return truth, records, statuses


def test_intercept_samples_score_with_his_metrics():
    truth, records, statuses = flight()
    samples = intercept_samples(truth, records, statuses, 1, 0.0, 10.0, HFOV, 0.5, 25.0)
    on_grid = [s for s in samples if abs(s.t * 10 - round(s.t * 10)) < 1e-6]
    assert len(on_grid) == 101  # 10 Hz, plus every truth point of the last 0.2 s
    assert len(samples) == 101 + 8
    assert samples[0].in_view and not samples[0].detected
    assert samples[20].detected and samples[20].status is Status.TRACKING
    scenario, _ = make_intercept_scenario(0.0, 4.0, 0, ICONFIG, ARC)
    result = score_run(samples, scenario, ICONFIG)
    assert result.outcome is Outcome.HIT
    assert result.time_to_hit == pytest.approx(9.0 - 0.375, abs=0.15)  # 1.5 m before contact
    assert result.lock_on  # in view, confirmed, within 20 m for 3 s


def test_a_hit_after_the_last_10_hz_sample_is_scored():
    """As on gish (seed 1): the run ends at the hit, between two 10 Hz samples. The last
    one is 1.9 m off; the contact itself, 0.05 s later, must still count."""
    truth, records, statuses = flight(t_end=9.9, hit=False)
    me = (150.0, 0.0, -30.0)

    def closing(t):  # 1.9 m at t = 9.9, closing at 20 m/s
        return (150.0 + 1.9 - 20.0 * (t - 9.9), 0.0, -30.0)

    truth = [TruthPoint(p.stamp, p.bodies, p.cameras, closing(p.stamp)) for p in truth]
    truth += [TruthPoint(t, {1: me}, {1: camera_north(me)}, closing(t)) for t in (9.92, 9.95)]
    samples = intercept_samples(truth, records, statuses, 1, 0.0, 9.95, HFOV, 0.5, 25.0)
    scenario, _ = make_intercept_scenario(0.0, 4.0, 0, ICONFIG, ARC)
    result = score_run(samples, scenario, ICONFIG)
    assert result.outcome is Outcome.HIT
    on_grid = [s for s in samples if abs(s.t * 10 - round(s.t * 10)) < 1e-6]
    assert score_run(on_grid, scenario, ICONFIG).outcome is not Outcome.HIT  # the old bug


def test_the_first_detection_gives_the_3d_distance_to_the_hostile():
    truth, records, _ = flight()
    # 10 m above the defender: the 3-D distance is the space diagonal
    truth = [TruthPoint(p.stamp, p.bodies, p.cameras, (p.hostile[0], p.hostile[1], -40.0))
             for p in truth]
    first = first_detection(truth, records, 0.0, 10.0, HFOV, 0.5, 2000.0)
    assert first.defender == 1 and first.t == pytest.approx(1.0)
    assert first.horizontal == pytest.approx(36.0, abs=0.1)
    assert first.height == pytest.approx(10.0)
    assert first.distance == pytest.approx(math.hypot(36.0, 10.0), abs=0.1)
    # a confirmed track somewhere else in the picture is not the hostile
    assert first_detection(truth, records, 0.0, 10.0, HFOV, 0.5, 3.0) is None
    assert first_detection(truth, {1: []}, 0.0, 10.0, HFOV, 0.5, 25.0) is None


def test_frame_records_give_his_detection_metrics():
    truth, records, _ = flight()
    scenario = make_detect_scenario(0, DCONFIG, ARC)
    records_ = frame_records(truth, records, scenario, 0.0, 10.0, HFOV)
    assert len(records_) == 101
    result = score_detect(records_, scenario, DCONFIG)
    assert not result.missed and result.first_detection_time == pytest.approx(1.0)
    assert result.first_detection_range == pytest.approx(36.0, abs=0.5)


def test_a_confirmed_track_on_empty_sky_is_a_false_alarm_and_a_defender_is_not():
    me, other = (150.0, 0.0, -30.0), (170.0, 0.0, -30.0)
    truth = [TruthPoint(0.0, {1: me, 2: other}, {1: camera_north(me)}, (0.0, 0.0, -30.0))]
    record = {'stamp': 0.0, 'width': 1280, 'height': 720, 'boxes': [],
              'confirmed': [[100.0, 100.0, 4, 4, 1]]}
    scenario = make_detect_scenario(0, DCONFIG, ARC)
    (r,) = frame_records(truth, {1: [record]}, scenario, 0.0, 1.0, HFOV)
    assert [o.id for o in r.frame.objects] == ['defender2']  # the hostile is behind it
    alone = [TruthPoint(0.0, {1: me}, {1: camera_north(me)}, (0.0, 0.0, -30.0))]
    (r,) = frame_records(alone, {1: [record]}, scenario, 0.0, 1.0, HFOV)
    assert r.frame.objects == () and len(r.detections) == 1  # counted as a false alarm
    assert score_detect([r], scenario, DCONFIG).background_false_alarms == 1


def test_missing_truth_is_skipped():
    truth, records, statuses = flight()
    late = [p for p in truth if p.stamp > 5.0]
    assert intercept_samples(late, records, statuses, 1, 0.0, 10.0, HFOV, 0.5,
                             25.0)[0].t == pytest.approx(4.9, abs=0.15)
    assert arc_eval.nearest([], [], 1.0) == (None, math.inf)


def test_saved_runs_load_back_and_summarize(tmp_path):
    from swarm_backend.config import load_config
    from swarm_backend.scenarios.baseline_eval import (
        as_detect, as_intercept, load_runs, save_run, summary_text)

    truth, records, statuses = flight()
    scenario, _ = make_intercept_scenario(0.0, 4.0, 3, ICONFIG, ARC)
    metrics = score_run(intercept_samples(truth, records, statuses, 1, 0.0, 10.0, HFOV, 0.5,
                                          25.0), scenario, ICONFIG)
    save_run(tmp_path / 'intercept.jsonl', metrics, {'killer': 2})
    dscenario = make_detect_scenario(0, DCONFIG, ARC)
    dmetrics = score_detect(frame_records(truth, records, dscenario, 0.0, 10.0, HFOV),
                            dscenario, DCONFIG)
    save_run(tmp_path / 'detect.jsonl', dmetrics, {})
    (row,) = load_runs(tmp_path / 'intercept.jsonl')
    assert as_intercept(row) == metrics and row['killer'] == 2
    assert as_detect(load_runs(tmp_path / 'detect.jsonl')[0]) == dmetrics
    text = summary_text(tmp_path, load_config('scenarios/surveillance.yaml'))
    assert 'hit=1' in text and 'missed drones' in text and '3 defenders' in text
    save_run(tmp_path / 'intercept.jsonl', metrics,
             {'hostile_altitude': 40.0, 'detect_distance': 37.4, 'detect_horizontal': 36.0,
              'detect_height': 10.0})
    text = summary_text(tmp_path, load_config('scenarios/surveillance.yaml'))
    assert 'detect 3-D' in text and '37.4' in text
