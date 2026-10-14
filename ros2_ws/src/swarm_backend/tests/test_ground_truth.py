import math

import pytest

from swarm_backend import ground_truth

IDENTITY = (0.0, 0.0, 0.0, 1.0)
HALF_TURN = (0.0, 0.0, 1.0, 0.0)  # about z; with SENSOR_IN_LINK the camera then looks along +x
NAMES = {1: 'x500_gimbal_1', 2: 'x500_gimbal_2'}


def poses(target_enu):
    """Drone 2 at the origin 20 m up, level, its camera looking east (Gazebo +x)."""
    return [
        ('x500_gimbal_1', 10, target_enu, IDENTITY),
        ('base_link', 11, (0.0, 0.0, 0.0), IDENTITY),
        ('x500_gimbal_2', 20, (0.0, 0.0, 20.0), IDENTITY),
        ('base_link', 21, (0.0, 0.0, 0.0), IDENTITY),
        ('camera_link', 22, (0.0, 0.0, 0.0), HALF_TURN),
    ]


def truth(target_enu):
    return ground_truth.truth(poses(target_enu), NAMES, 2, 'camera_link', 2.0, 1280, 720)


def test_target_straight_ahead_is_at_the_image_centre():
    data = truth((30.0, 0.0, 20.0))  # 30 m east
    target = data['drones'][0]
    assert target['id'] == 1
    assert target['distance'] == pytest.approx(30.0, abs=0.01)
    assert target['direction'] == pytest.approx([0.0, 1.0, 0.0], abs=0.01)  # NED: east is +y
    assert target['pixel'] == pytest.approx([640.0, 360.0], abs=2.0)
    assert data['camera']['forward'] == pytest.approx([0.0, 1.0, 0.0], abs=0.01)


def test_target_to_the_south_and_below_is_right_and_down_in_the_image():
    data = truth((30.0, -5.0, 15.0))  # looking east: south is to the right
    u, v = data['drones'][0]['pixel']
    f = 640 / math.tan(1.0)
    assert u == pytest.approx(640 + f * 5 / 30, abs=2.0)
    assert v == pytest.approx(360 + f * 5 / 30, abs=2.0)


def test_target_behind_the_camera_has_no_pixel():
    assert truth((-30.0, 0.0, 20.0))['drones'][0]['pixel'] is None


def test_positions_are_ned():
    data = truth((30.0, 4.0, 25.0))
    assert data['drones'][0]['position'] == pytest.approx([4.0, 30.0, -25.0])
    assert data['camera_drone']['position'] == pytest.approx([0.0, 0.0, -20.0])
