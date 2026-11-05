"""SectorScan and PixelModelDetector, the detection baseline's scan and its stand-in
detector for the fast simulation."""
import math
import random

from swarm_backend.config import DetectConfig
from swarm_backend.detect.detector import PixelModelDetector, Stage
from swarm_backend.detect.frame import Frame, Kind, TrueObject
from swarm_backend.detect.scan import SectorScan

CONFIG = DetectConfig()


def test_scan_two_positions_either_side_of_outward():
    scan = SectorScan(CONFIG, (0.0, 0.0, 0.0))
    fov = math.degrees(CONFIG.camera_hfov)
    east = (0.0, 1000.0, -50.0)  # outward = 90 deg
    assert scan.aim(0.0, 0, east) == (0.0, 90.0 - fov / 2)
    assert scan.aim(CONFIG.scan_dwell, 0, east) == (0.0, 90.0 + fov / 2)
    assert scan.aim(2 * CONFIG.scan_dwell, 0, east) == (0.0, 90.0 - fov / 2)
    # together the positions cover the 180 deg outward sector
    assert 2 * fov >= 180.0


def test_scan_narrow_camera_sweeps_back_and_forth():
    config = DetectConfig(camera_hfov=math.radians(45.0))
    scan = SectorScan(config, (0.0, 0.0, 0.0))
    yaws = [scan.aim(i * config.scan_dwell, 0, (1000.0, 0.0, -50.0))[1] for i in range(8)]
    assert yaws == [-67.5, -22.5, 22.5, 67.5, 22.5, -22.5, -67.5, -22.5]


def frame(t, *objects):
    return Frame(time=t, drone=0, camera=(0.0, 0.0, 0.0), forward=(1.0, 0.0, 0.0),
                 right=(0.0, 1.0, 0.0), down=(0.0, 0.0, 1.0), objects=objects)


def obj(id_, kind, size):
    return TrueObject(id=id_, kind=kind, pixel=(640.0, 360.0), size_px=size)


def test_too_small_is_not_found():
    detector = PixelModelDetector(CONFIG, random.Random(0))
    assert detector.detect(frame(0.0, obj('d', Kind.DRONE, CONFIG.detect_px - 0.1))) == []


def test_drone_confirmed_after_k_images_in_a_row():
    detector = PixelModelDetector(CONFIG, random.Random(0))
    drone = obj('d', Kind.DRONE, 6.0)
    stages = [detector.detect(frame(i, drone))[0] for i in range(CONFIG.confirm_k)]
    assert [d.is_drone for d in stages] == [False] * (CONFIG.confirm_k - 1) + [True]
    assert stages[0].stage == Stage.CANDIDATE and stages[-1].stage == Stage.CLASSIFIED
    assert abs(stages[0].pixel[0] - 640.0) < CONFIG.match_radius_px


def test_a_missed_image_breaks_the_streak():
    detector = PixelModelDetector(CONFIG, random.Random(0))
    drone = obj('d', Kind.DRONE, 6.0)
    for _ in range(CONFIG.confirm_k - 1):
        detector.detect(frame(0.0, drone))
    detector.detect(frame(0.0))
    assert not detector.detect(frame(0.0, drone))[0].is_drone


def test_small_bird_flagged_large_bird_classified():
    detector = PixelModelDetector(CONFIG, random.Random(0))
    small, large = obj('b1', Kind.BIRD, 6.0), obj('b2', Kind.BIRD, CONFIG.classify_px)
    for _ in range(CONFIG.confirm_k):
        out = {round(d.box_px[0]): d for d in detector.detect(frame(0.0, small, large))}
    assert out[6].is_drone
    assert out[round(CONFIG.classify_px)].kind == Kind.BIRD


def test_reset_forgets_streaks():
    detector = PixelModelDetector(CONFIG, random.Random(0))
    drone = obj('d', Kind.DRONE, 6.0)
    for _ in range(CONFIG.confirm_k):
        detector.detect(frame(0.0, drone))
    detector.reset()
    assert not detector.detect(frame(0.0, drone))[0].is_drone
