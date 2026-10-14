"""The detection evaluation harness: camera truth, scenarios, the run loop and the
metrics of the planning report (Drone Detection Using Sensors > Evaluation). No
detection method is needed: the loop runs with small test doubles."""
import math

import pytest

from swarm_backend.config import DetectConfig, parse_config
from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.dataset import (LabelledImage, evaluate_dataset, split_by_run,
                                          summarize_dataset)
from swarm_backend.detect.detector import Detection, ImageDetector, Stage
from swarm_backend.detect.frame import Body, Frame, Kind, TrueObject, aimed_camera, make_frame
from swarm_backend.detect.metrics import detection_by_range, match, score_run, summarize
from swarm_backend.detect.scan import GimbalScan
from swarm_backend.detect.scenario import (HOSTILE_ID, FrameRecord, make_scenario,
                                           run_scenario, scenario_grid)

from sim import DetectFastSim

CONFIG = DetectConfig()
CAMERA = CameraModel(CONFIG.camera_hfov, CONFIG.image_width, CONFIG.image_height)
ORIGIN = (0.0, 0.0, 0.0)
AXES = dict(forward=(1.0, 0.0, 0.0), right=(0.0, 1.0, 0.0), down=(0.0, 0.0, 1.0))


# --- camera truth ---------------------------------------------------------------

def pixel(pitch, yaw, point):
    """Where `point` shows for a camera at the origin aimed at (pitch, yaw), or None."""
    frame = make_frame(CAMERA, 0.0, 0, aimed_camera(ORIGIN, pitch, yaw),
                       [Body('x', Kind.DRONE, point, CONFIG.target_size)])
    return frame.objects[0].pixel if frame.objects else None


def test_aimed_camera_centre_and_axes():
    assert pixel(0, 0, (10, 0, 0)) == pytest.approx((640, 360))
    u, v = pixel(0, 0, (10, 1, -1))  # east and up
    assert u > 640 and v < 360
    assert pixel(0, 90, (0, 10, 0)) == pytest.approx((640, 360))  # facing east
    down = math.radians(30)  # pitch -30: looking 30 deg below the horizon
    assert pixel(-30, 0, (10 * math.cos(down), 0, 10 * math.sin(down))) == (
        pytest.approx((640, 360)))
    pose = aimed_camera(ORIGIN, -30, 45)
    assert pose['right'] == pytest.approx((-math.sin(math.radians(45)),
                                           math.cos(math.radians(45)), 0))


def test_outside_the_image_is_not_in_the_frame():
    assert pixel(0, 0, (-10, 0, 0)) is None  # behind
    assert pixel(0, 0, (10, 100, 0)) is None  # beyond the side
    assert pixel(0, 0, (10, 0, -10)) is None  # above (720 px is about 70 deg)


def test_size_at_the_detection_range():
    # The report: a 0.5 m drone is p_min = 4 px wide at about 51 m with this camera.
    frame = make_frame(CAMERA, 0.0, 0, aimed_camera(ORIGIN, 0, 0),
                       [Body('x', Kind.DRONE, (51.0, 0, 0), CONFIG.target_size)])
    assert frame.objects[0].size_px == pytest.approx(CONFIG.detect_px, abs=0.05)
    assert frame.objects[0].distance == 51.0


# --- matching -----------------------------------------------------------------------

def obj(id_, kind=Kind.DRONE, pixel=(100.0, 100.0), size=5.0, distance=100.0):
    return TrueObject(id=id_, kind=kind, pixel=pixel, size_px=size, distance=distance)


def det(pixel=(100.0, 100.0), kind=Kind.DRONE, stage=Stage.CLASSIFIED, t=0.0):
    return Detection(time=t, drone=0, pixel=pixel, box_px=(4.0, 4.0), stage=stage, kind=kind)


def test_match_closest_within_radius_each_object_once():
    a, b = obj('a', pixel=(100, 100)), obj('b', pixel=(200, 100))
    found = match([det((101, 100)), det((100, 100)), det((150, 100)), det((202, 100))],
                  [a, b], CONFIG.match_radius_px)
    assert found == [None, a, None, b]


# --- scenarios ----------------------------------------------------------------------

@pytest.mark.parametrize('seed', [0, 1, 2])
def test_scenario_geometry(seed):
    s = make_scenario(seed, CONFIG)
    assert len(s.defenders) == CONFIG.num_defenders
    for d in s.defenders:
        assert math.hypot(d[0], d[1]) == pytest.approx(CONFIG.perimeter_radius)
        assert d[2] == -CONFIG.altitude
    h = s.hostile
    assert math.hypot(h.start[0], h.start[1]) == pytest.approx(
        CONFIG.perimeter_radius + CONFIG.start_margin)
    assert h.goal == (0.0, 0.0, h.start[2])
    assert CONFIG.hostile_altitude_min <= -h.start[2] <= CONFIG.hostile_altitude_max
    assert s.ring_time == pytest.approx(CONFIG.start_margin / CONFIG.hostile_speed)
    assert len(s.birds) == CONFIG.num_birds and all(b.kind == Kind.BIRD for b in s.birds)
    assert make_scenario(seed, CONFIG) == s


def test_movers_fly_at_speed_and_stop_at_the_goal():
    h = make_scenario(0, CONFIG).hostile
    assert math.dist(h.at(0).position, h.start) == 0
    assert math.dist(h.at(10).position, h.start) == pytest.approx(10 * h.speed)
    assert h.at(1e6).position == pytest.approx(h.goal)


def test_grid_and_settings_from_yaml():
    assert len(scenario_grid(CONFIG)) == CONFIG.eval_runs
    config = parse_config({'drones': [{'id': 0}],
                           'detect': {'num_defenders': 16, 'range_bins': [0, 100]}})
    assert config.detect.num_defenders == 16 and config.detect.range_bins == (0.0, 100.0)
    with pytest.raises(ValueError):
        parse_config({'drones': [{'id': 0}], 'detect': {'defenders': 16}})


# --- scoring ------------------------------------------------------------------------

def record(t, objects=(), detections=(), hostile_distance=500.0, outside=300.0):
    frame = Frame(time=t, drone=0, camera=ORIGIN, objects=tuple(objects),
                  **AXES)
    return FrameRecord(t=t, frame=frame, detections=tuple(detections),
                       hostile_distance=hostile_distance, hostile_outside=outside)


def hostile(distance):
    return obj(HOSTILE_ID, distance=distance)


SCENARIO = make_scenario(0, CONFIG)


def test_first_drone_detection_of_the_hostile():
    records = [
        record(0.0),
        record(0.1, [hostile(90)]),  # in view, not detected
        record(0.2, [hostile(80)], [det(stage=Stage.CANDIDATE, kind=None)]),  # not yet a drone
        record(0.3, [hostile(70)], [det((150, 150))]),  # a drone, but somewhere else
        record(0.4, [hostile(60)], [det()], hostile_distance=60, outside=170),
        record(0.5, [hostile(50)], [det()]),
    ]
    run = score_run(records, SCENARIO, CONFIG)
    assert not run.missed and run.first_detection_time == 0.4
    assert run.first_detection_range == 60 and run.warning_distance == 170
    assert run.warning_time == pytest.approx(170 / CONFIG.hostile_speed)
    assert run.hostile_views == ((90, False), (80, False), (70, False), (60, True), (50, True))


def test_missed_drone():
    run = score_run([record(0.0, [hostile(90)], [det((150, 150))]), record(0.1, outside=0.0)],
                    SCENARIO, CONFIG)
    assert run.missed and run.first_detection_time is None and run.warning_time is None


def test_birds_flagged_and_background_false_alarms():
    bird_a = obj('bird0', Kind.BIRD, pixel=(300, 300))
    bird_b = obj('bird1', Kind.BIRD, pixel=(500, 300))
    records = [
        record(0.0, [bird_a, bird_b], [det((300, 300), kind=Kind.BIRD)]),  # correct
        record(0.1, [bird_a], [det((300, 300))]),  # bird0 taken for a drone
        record(0.2, [], [det((10, 10)), det((20, 20))]),  # two background alarms
        record(0.3, [], []),
        record(0.4, [], [det((10, 10), kind=Kind.BIRD)]),  # not a drone: no alarm
    ]
    run = score_run(records, SCENARIO, CONFIG)
    assert run.birds_seen == 2 and run.birds_flagged == 1
    assert run.empty_frames == 3 and run.background_false_alarms == 2 and run.frames == 5


def test_detection_probability_by_range_counts_drones_not_images():
    near = score_run([record(0.0, [hostile(30)], [det()]), record(0.1, [hostile(20)])],
                     SCENARIO, CONFIG)
    far = score_run([record(0.0, [hostile(150)]), record(0.1, [hostile(40)], [det()])],
                    SCENARIO, CONFIG)
    bins = detection_by_range([near, far], (0.0, 25.0, 50.0, 100.0, 200.0))
    assert bins == [(0.0, 25.0, 1, 0), (25.0, 50.0, 2, 2), (50.0, 100.0, 0, 0),
                    (100.0, 200.0, 1, 0), (200.0, float('inf'), 0, 0)]
    text = summarize([near, far], CONFIG)
    assert 'missed drones' in text and '0 %' in text and '25-50' in text


# --- labelled images ------------------------------------------------------------------

def image(run, objects=()):
    frame = Frame(time=0, drone=0, camera=ORIGIN, objects=tuple(objects),
                  **AXES)
    return LabelledImage(run=run, frame=frame)


def test_split_by_run_keeps_runs_together():
    images = [image(f'run{r}') for r in range(20) for _ in range(5)]
    train, val, test = split_by_run(images, (0.7, 0.15, 0.15), seed=1)
    assert (len(train), len(val), len(test)) == (70, 15, 15)
    runs = [{i.run for i in s} for s in (train, val, test)]
    assert not (runs[0] & runs[1] or runs[0] & runs[2] or runs[1] & runs[2])
    assert split_by_run(images, (0.7, 0.15, 0.15), seed=1) == [train, val, test]


class SizeRule(ImageDetector):
    """Finds what is at least 4 px wide, calls it a drone from 20 px, and sees a
    drone in every empty image."""

    def reset(self):
        pass

    def detect(self, frame):
        if not frame.objects:
            return [det((5, 5))]
        return [det(o.pixel, kind=Kind.DRONE if o.size_px >= 20 else None,
                    stage=Stage.CLASSIFIED if o.size_px >= 20 else Stage.CANDIDATE)
                for o in frame.objects if o.size_px >= 4]


def test_recall_by_size_background_alarms_and_time():
    images = [image('r', [obj('d', size=s)]) for s in (1, 3, 5, 10, 25, 30, 150)]
    images += [image('r'), image('r')]
    ticks = iter(range(100))
    metrics = evaluate_dataset(images, SizeRule(), CONFIG, clock=lambda: 0.002 * next(ticks))
    bins = {lo: (n, found, classified) for lo, _, n, found, classified in metrics.size_bins}
    assert bins[0.0] == (1, 0, 0) and bins[2.0] == (1, 0, 0) and bins[4.0] == (1, 1, 0)
    assert bins[8.0] == (1, 1, 0) and bins[20.0] == (2, 2, 2) and bins[100.0] == (1, 1, 1)
    assert metrics.background_images == 2 and metrics.background_false_alarms == 2
    assert metrics.inference_ms == pytest.approx((2.0,) * 9)
    assert 'recall vs target size' in summarize_dataset(metrics)


# --- the run loop, with test doubles ---------------------------------------------------

class SeesTruth(ImageDetector):
    """Not a detector: reports every object in view, correctly classified."""

    def __init__(self, scenario, drone, config):
        pass

    def reset(self):
        pass

    def detect(self, frame):
        return [det(o.pixel, kind=o.kind, t=frame.time) for o in frame.objects]


class LookAtHostile(GimbalScan):
    """Not a scan: every camera points at where the hostile drone is."""

    def __init__(self, scenario, config):
        self.scenario = scenario

    def aim(self, t, drone, position):
        target = self.scenario.hostile.at(t).position
        d = [a - b for a, b in zip(target, position)]
        return (-math.degrees(math.atan2(d[2], math.hypot(d[0], d[1]))),
                math.degrees(math.atan2(d[1], d[0])))


class LookDown(LookAtHostile):
    def aim(self, t, drone, position):
        return -90.0, 0.0


def test_run_detects_at_once_when_every_camera_looks_at_the_target():
    s = make_scenario(0, CONFIG)
    records = run_scenario(s, DetectFastSim(s, CONFIG), SeesTruth, LookAtHostile, CONFIG)
    run = score_run(records, s, CONFIG)
    assert not run.missed and run.first_detection_time == 0.0
    assert run.warning_distance == pytest.approx(CONFIG.start_margin)
    assert len(records) % CONFIG.num_defenders == 0
    assert records[-1].hostile_outside <= 0 < records[-1 - CONFIG.num_defenders].hostile_outside


def test_run_misses_when_no_camera_looks_out():
    s = make_scenario(0, CONFIG)
    records = run_scenario(s, DetectFastSim(s, CONFIG), SeesTruth, LookDown, CONFIG)
    run = score_run(records, s, CONFIG)
    assert run.missed and run.hostile_views == ()
    assert records[-1].t == pytest.approx(s.ring_time, abs=1 / CONFIG.frame_rate_hz)
