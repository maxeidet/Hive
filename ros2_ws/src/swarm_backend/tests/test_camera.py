import math

import pytest

from swarm_backend.detect.camera import (BOX_SIZE_M, CameraModel, angle_deg, camera_roll, level_axes,
                                         true_offset)

CAMERA = CameraModel(hfov=2.0, width=1280, height=720)
NORTH, EAST, DOWN = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def test_focal_length_matches_the_report():
    assert CAMERA.f == pytest.approx(411, abs=1)
    assert CAMERA.range_from_width(4) == pytest.approx(51, abs=1)  # 0.5 m target at 4 px


def test_non_baseline_detectors_use_the_geometric_mean_side():
    assert CAMERA.range_from_detection('yolo', 20, 20) == pytest.approx(CAMERA.f * BOX_SIZE_M / 20)
    assert CAMERA.range_from_detection('yolo', 40, 10) == pytest.approx(
        CAMERA.range_from_detection('yolo', 20, 20))
    assert CAMERA.range_from_detection('yolo', 0, 0) == pytest.approx(CAMERA.f * BOX_SIZE_M)  # no division by zero


def test_baseline_keeps_the_old_rule():
    assert CAMERA.range_from_detection('baseline', 40, 10) == pytest.approx(CAMERA.range_from_width(40))
    assert CAMERA.friendly_range('baseline', 40, 10) == pytest.approx(CAMERA.range_from_width(40))


def test_friendly_range_uses_our_drones_size():
    from swarm_backend.detect.camera import FRIENDLY_BOX_SIZE_M
    assert CAMERA.friendly_range('yolo', 40, 10) == pytest.approx(
        CAMERA.f * FRIENDLY_BOX_SIZE_M / 20)


def test_image_centre_looks_along_forward():
    assert CAMERA.direction(640, 360, NORTH, EAST, DOWN) == pytest.approx(NORTH)


def test_pixel_to_the_right_and_below():
    f = CAMERA.f
    ten = f * math.tan(math.radians(10))
    right = CAMERA.direction(640 + ten, 360, NORTH, EAST, DOWN)
    assert angle_deg(right, NORTH) == pytest.approx(10)
    assert right[1] > 0 and right[2] == pytest.approx(0)
    below = CAMERA.direction(640, 360 + ten, NORTH, EAST, DOWN)
    assert below[2] > 0 and below[1] == pytest.approx(0)


def test_offset_is_zero_at_the_centre():
    assert CAMERA.offset_angles(640, 360) == pytest.approx((0.0, 0.0))


def test_offset_right_and_up_are_positive():
    x, y = CAMERA.offset_angles(740, 360)
    assert x == pytest.approx(math.atan(100 / CAMERA.f)) and x == pytest.approx(0.238, abs=0.001)
    assert y == pytest.approx(0.0)
    x, y = CAMERA.offset_angles(640, 260)  # higher in the image = smaller row
    assert x == pytest.approx(0.0) and y > 0


def test_a_quarter_turn_of_roll_swaps_the_axes():
    # Rolled right side down by 90 degrees, the image's "up" points to the world's right.
    x, y = CAMERA.offset_angles(640, 260, roll=math.pi / 2)
    assert x > 0 and y == pytest.approx(0.0, abs=1e-9)


def rolled(forward, roll):
    """right and down of a camera looking along `forward`, rolled right side down by `roll`."""
    level_right, level_up = level_axes(forward)
    right = tuple(math.cos(roll) * r - math.sin(roll) * u for r, u in zip(level_right, level_up))
    up = tuple(math.sin(roll) * r + math.cos(roll) * u for r, u in zip(level_right, level_up))
    return right, tuple(-c for c in up)


@pytest.mark.parametrize('roll_deg', [0, 30, -30, 90])
def test_roll_correction_matches_the_world_angles(roll_deg):
    forward = (math.cos(0.2), 0.0, -math.sin(0.2))  # north, tilted up a little
    right, down = rolled(forward, math.radians(roll_deg))
    assert math.degrees(camera_roll(forward, right)) == pytest.approx(roll_deg)
    # A target 4 degrees right of and 3 degrees above where the camera looks (world frame).
    level_right, level_up = level_axes(forward)
    target = tuple(f + math.tan(math.radians(4)) * r + math.tan(math.radians(3)) * u
                   for f, r, u in zip(forward, level_right, level_up))
    norm = math.hypot(*target)
    target = tuple(c / norm for c in target)
    # Where it shows in the rolled image.
    depth = sum(a * b for a, b in zip(target, forward))
    u = CAMERA.cx + CAMERA.f * sum(a * b for a, b in zip(target, right)) / depth
    v = CAMERA.cy + CAMERA.f * sum(a * b for a, b in zip(target, down)) / depth
    x, y = CAMERA.offset_angles(u, v, roll=math.radians(roll_deg))
    tx, ty = true_offset(target, forward)
    assert math.degrees(x) == pytest.approx(math.degrees(tx), abs=0.05)
    assert math.degrees(y) == pytest.approx(math.degrees(ty), abs=0.05)
    assert math.degrees(tx) == pytest.approx(4, abs=0.1)


def test_position_is_direction_times_distance():
    # 20 px wide straight ahead of a camera looking north: 0.5 m at f * 0.5 / 20 m.
    distance = CAMERA.f * 0.5 / 20
    assert CAMERA.position(640, 360, 20, NORTH, EAST, DOWN) == pytest.approx((distance, 0, 0))
    # Right of the centre: east of north, same distance.
    n, e, d = CAMERA.position(740, 360, 20, NORTH, EAST, DOWN)
    assert math.hypot(n, e, d) == pytest.approx(distance)
    assert e > 0 and d == pytest.approx(0)


def test_roll_correction_is_exact_far_from_the_centre():
    # 45 degrees right and 30 up in a camera rolled 30 degrees: rotate the point on the
    # image plane (tangents), not the angles.
    roll = math.radians(30)
    u = CAMERA.cx + CAMERA.f * math.tan(math.radians(45))
    v = CAMERA.cy - CAMERA.f * math.tan(math.radians(30))
    x, y = CAMERA.offset_angles(u, v, roll)
    tx, ty = math.tan(math.radians(45)), math.tan(math.radians(30))
    assert math.tan(x) == pytest.approx(tx * math.cos(roll) + ty * math.sin(roll))
    assert math.tan(y) == pytest.approx(-tx * math.sin(roll) + ty * math.cos(roll))
    assert math.degrees(x) == pytest.approx(49.11, abs=0.01)
    assert math.degrees(y) == pytest.approx(0.0, abs=0.01)

