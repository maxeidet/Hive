"""The detection contract and the fake detector (intercept/detection.py), without ROS."""
import math
import random
import statistics
from dataclasses import replace

import pytest

from swarm_backend.config import AttackConfig, parse_config
from swarm_backend.detect.camera import CameraModel
from swarm_backend.intercept.detection import (
    OFFSET_TOPIC, POSITION_TOPIC, FakeDetector, camera_roll, level_offset, measure, topic)

MODEL = CameraModel()
NORTH = {'position': (0.0, 0.0, -20.0), 'forward': (1.0, 0.0, 0.0),
         'right': (0.0, 1.0, 0.0), 'down': (0.0, 0.0, 1.0)}


def rolled(camera, roll_deg):
    """The same camera rolled about its forward axis, right side down for positive roll."""
    r = math.radians(roll_deg)
    right, down = camera['right'], camera['down']
    return {**camera,
            'right': tuple(math.cos(r) * a + math.sin(r) * b for a, b in zip(right, down)),
            'down': tuple(-math.sin(r) * a + math.cos(r) * b for a, b in zip(right, down))}


def at(camera, forward, right, up):
    """World point `forward`, `right`, `up` metres from the camera, in its own axes."""
    return tuple(p + forward * f + right * r - up * d for p, f, r, d in zip(
        camera['position'], camera['forward'], camera['right'], camera['down']))


def test_topic_names_follow_the_px4_namespaces():
    assert topic(0, POSITION_TOPIC) == '/detection/target_position'
    assert topic(2, POSITION_TOPIC) == '/px4_2/detection/target_position'
    assert topic(2, OFFSET_TOPIC) == '/px4_2/detection/target_offset'


def test_target_straight_ahead_is_at_the_centre():
    found = measure(NORTH, at(NORTH, 20, 0, 0), MODEL, max_range=50)
    assert found.position == pytest.approx((20.0, 0.0, 0.0))
    assert found.offset == pytest.approx((0.0, 0.0))


def test_position_is_relative_ned_and_offset_is_right_and_up():
    found = measure(NORTH, at(NORTH, 20, 2, 3), MODEL, max_range=50)
    assert found.position == pytest.approx((20.0, 2.0, -3.0))  # NED: up is -z
    assert found.offset == pytest.approx((math.atan(2 / 20), math.atan(3 / 20)))


@pytest.mark.parametrize('roll', [-35.0, 10.0, 30.0])
def test_offset_is_corrected_for_camera_roll(roll):
    target = at(NORTH, 20, 4, 6)
    level = measure(NORTH, target, MODEL, max_range=50)
    tilted = measure(rolled(NORTH, roll), target, MODEL, max_range=50)
    assert tilted.offset == pytest.approx(level.offset, abs=1e-9)
    assert tilted.position == pytest.approx(level.position)


def test_camera_roll_is_positive_with_the_right_side_down():
    cam = rolled(NORTH, 20.0)
    assert math.degrees(camera_roll(cam['forward'], cam['right'])) == pytest.approx(20.0)
    assert level_offset(0.1, 0.0, 0.0) == pytest.approx((0.1, 0.0))


@pytest.mark.parametrize('where, max_range, blind', [
    ((-20, 0, 0), 50, 0),    # behind the camera
    ((20, 40, 0), 50, 0),    # outside the image sideways (FOV +-57 deg)
    ((20, 0, 20), 50, 0),    # outside the image upward (vertical FOV +-41 deg)
    ((60, 0, 0), 50, 0),     # too far
    ((4, 0, 0), 50, 5),      # in the blind zone
])
def test_nothing_is_seen_outside_view_range_or_blind_zone(where, max_range, blind):
    assert measure(NORTH, at(NORTH, *where), MODEL, max_range, blind) is None


def test_noise_has_the_configured_spread():
    rng = random.Random(1)
    target = at(NORTH, 30, 0, 0)
    found = [measure(NORTH, target, MODEL, 100, angle_noise=math.radians(1.0),
                     range_noise=0.2, rng=rng) for _ in range(4000)]
    distances = [math.hypot(*f.position) for f in found]
    rights = [math.degrees(f.offset[0]) for f in found]
    assert statistics.mean(distances) == pytest.approx(30.0, rel=0.02)
    assert statistics.stdev(distances) == pytest.approx(6.0, rel=0.1)
    assert statistics.stdev(rights) == pytest.approx(1.0, rel=0.1)
    # The direction in target_position has the same error as the offset.
    for f in found[:50]:
        assert math.degrees(math.atan2(f.position[1], f.position[0])) == pytest.approx(
            math.degrees(f.offset[0]), abs=1e-6)


def test_default_range_is_where_the_target_is_min_pixels_wide():
    detector = FakeDetector(AttackConfig())
    assert detector.max_range == pytest.approx(MODEL.f * 0.5 / 4)
    assert 50 < detector.max_range < 52


def test_fake_detector_rate_confirmation_and_delay():
    config = replace(AttackConfig(), fake_rate_hz=4.0, fake_delay=0.25, fake_confirm_frames=3)
    detector = FakeDetector(config)
    target = at(NORTH, 20, 0, 0)
    published = []
    t = 0.0
    while t < 2.0 + 1e-9:
        if detector.wants_image(t):
            detector.observe(t, NORTH, target)
        published += detector.due(t)
        t = round(t + 0.01, 2)
    assert detector.images == 9  # 0, 0.25, ..., 2.0
    stamps = [s for s, _ in published]
    # Confirmed from the third image (t=0.5); each comes out 0.25 s after its image, and
    # the one at 2.0 is not out yet.
    assert stamps == pytest.approx([0.5, 0.75, 1.0, 1.25, 1.5, 1.75])


def test_losing_the_target_resets_the_confirmation():
    detector = FakeDetector(AttackConfig())
    seen, gone = at(NORTH, 20, 0, 0), at(NORTH, -20, 0, 0)
    for i, target in enumerate([seen, seen, gone, seen, seen, seen]):
        detector.observe(i * 0.25, NORTH, target)
    assert [s for s, _ in detector.due(10.0)] == [1.25]  # third in a row after the gap


def test_attack_section_parses_and_rejects_unknown_settings():
    drones = [{'id': 0}, {'id': 2}]
    config = parse_config({'drones': drones, 'attack': {'max_speed': 8, 'fake_delay': 0.25}})
    assert config.attack.max_speed == 8.0 and config.attack.fake_delay == 0.25
    assert parse_config({'drones': drones}).attack == AttackConfig()
    with pytest.raises(ValueError):
        parse_config({'drones': drones, 'attack': {'speed': 8}})
    with pytest.raises(ValueError):
        parse_config({'drones': drones, 'attack': {'attacker_id': 0, 'target_id': 0}})
