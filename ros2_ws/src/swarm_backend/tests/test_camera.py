import math

import pytest

from swarm_backend.detect.camera import CameraModel, angle_deg

CAMERA = CameraModel(hfov=2.0, width=1280, height=720)
NORTH, EAST, DOWN = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def test_focal_length_matches_the_report():
    assert CAMERA.f == pytest.approx(411, abs=1)
    assert CAMERA.range_from_width(4) == pytest.approx(51, abs=1)  # 0.5 m target at 4 px


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
