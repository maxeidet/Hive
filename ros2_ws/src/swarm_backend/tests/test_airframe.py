"""The drone's own airframe in its gimbal camera's picture (detect/airframe.py): what the
live detector ignores with --body-mask. The pixels below were checked by eye and by colour
in Gazebo pictures from 11 Nov, with the camera axes PX4 reported for them."""
import math

import numpy as np
import pytest

from swarm_backend.detect.airframe import (
    UP_CAP_DEG, airframe_mask, body_axes, camera_position)
from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.orientation import rotate

CAMERA = CameraModel(2.0, 1280, 720)
SHAPE = (720, 1280, 3)


def aimed(pitch_deg, yaw_deg):
    """Camera axes in the body frame (FRD): pitch up from level, yaw right of the nose."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    forward = np.array((math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p)))
    right = np.array((-math.sin(y), math.cos(y), 0.0))
    return tuple(map(tuple, (forward, right, np.cross(forward, right))))


def axes(values):
    return tuple(tuple(values[i:i + 3]) for i in (0, 3, 6))


# Patrol, looking 74 degrees left of the nose: the left leg fills the picture's left side.
LOOKING_LEFT = axes([0.2754, -0.9454, -0.1743, 0.9598, 0.2807, -0.0059, 0.0546, -0.1657,
                     0.9847])
# Attacking, looking 30 degrees down and 34 right: the right skid from its end cap.
LOOKING_DOWN = axes([0.7158, 0.488, 0.4995, -0.566, 0.8244, 0.0058, -0.4089, -0.2869,
                     0.8663])
# Looking 45 degrees up and 75 left: the underside, a leg and two propellers.
LOOKING_UP = axes([0.176, -0.6802, -0.7116, 0.9712, 0.2381, 0.0127, 0.1608, -0.6933,
                   0.7025])


@pytest.mark.parametrize('camera_axes, airframe, open_view', [
    (LOOKING_LEFT, [(280, 320), (280, 120)], [(640, 500), (1100, 450), (700, 360)]),
    (LOOKING_DOWN, [(810, 350), (1000, 450), (1200, 600)], [(300, 500), (640, 200)]),
    (LOOKING_UP, [(1010, 250), (500, 150)], [(900, 560), (1250, 500)]),
])
def test_the_airframe_seen_in_gazebo_is_masked_and_the_view_past_it_is_not(
        camera_axes, airframe, open_view):
    mask = airframe_mask(SHAPE, CAMERA, camera_axes)
    assert all(mask[v, u] for u, v in airframe)
    assert not any(mask[v, u] for u, v in open_view)


def test_looking_ahead_the_middle_of_the_picture_is_free():
    # (Not at 90 degrees to the side: a leg is right in front of the camera there, as in
    # Gazebo.)
    for pitch, yaw in [(10, 0), (10, 45), (10, -60), (0, 180), (-20, 30), (-89, 0)]:
        mask = airframe_mask(SHAPE, CAMERA, aimed(pitch, yaw))
        assert not mask[360, 640], (pitch, yaw)
        assert mask.mean() < 0.5, (pitch, yaw)


def test_straight_to_the_side_the_leg_is_in_the_middle():
    assert airframe_mask(SHAPE, CAMERA, aimed(10, -90))[360, 640]
    assert airframe_mask(SHAPE, CAMERA, aimed(10, 90))[360, 640]


def test_straight_up_is_airframe():
    mask = airframe_mask(SHAPE, CAMERA, aimed(UP_CAP_DEG + 20, 0))
    assert mask[360, 640]
    # just under the cap, between the arms (looking forward), is still free
    mask = airframe_mask(SHAPE, CAMERA, aimed(UP_CAP_DEG - 8, 0))
    assert not mask[360, 640]


def test_body_axes_undo_the_drones_attitude():
    yaw = math.radians(90)  # the drone faces east
    q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    camera_body = aimed(10, -30)
    camera_world = tuple(rotate(q, a) for a in camera_body)
    for got, want in zip(body_axes(camera_world, q), camera_body):
        assert got == pytest.approx(want, abs=1e-9)


def test_the_camera_swings_on_the_gimbal_yaw_arm():
    ahead, behind = camera_position(aimed(0, 0)), camera_position(aimed(0, 180))
    assert ahead == pytest.approx((0.0412, 0.0, 0.142), abs=1e-3)
    assert behind == pytest.approx((0.0098, 0.0, 0.142), abs=1e-3)
    assert camera_position(aimed(-90, 0))[2] == pytest.approx(0.142)


def test_annotate_shades_the_masked_airframe():
    from swarm_backend.detect.live import Result, annotate
    img = np.full((720, 1280, 3), 200, np.uint8)
    own = np.zeros((720, 1280), bool)
    own[600:, :300] = True
    out = annotate(img, Result(0.0, 1280, 720, [], [], None, 1.0, airframe=own))
    assert (out[700, 100] != 200).any() and (out[100, 1000] == 200).all()
    assert (img == 200).all()  # the picture itself is not changed
