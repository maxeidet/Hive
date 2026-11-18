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


def test_drone_straight_ahead_has_a_box_around_its_pixel_of_the_right_size():
    data = truth((30.0, 0.0, 20.0))
    box = data['drones'][0]['box']
    f = 640 / math.tan(1.0)
    assert (box['x0'] + box['x1']) / 2 == pytest.approx(640, abs=1)
    assert (box['y0'] + box['y1']) / 2 == pytest.approx(360, abs=1)
    width = 2 * ground_truth.HALF_SIZE['drone'][0]  # seen side on: its y extent, here x
    assert box['x1'] - box['x0'] == pytest.approx(f * width / 30, rel=0.1)
    assert box['in_image'] == 1.0


def test_box_at_the_image_edge_is_cut_and_says_how_much_is_inside():
    f = 640 / math.tan(1.0)
    east_of_edge = 30 * 640 / f  # the drone's centre exactly on the right edge
    box = truth((30.0, -east_of_edge, 20.0))['drones'][0]['box']
    assert box['x1'] == 1280.0
    assert 0.3 < box['in_image'] < 0.8  # about half: seen at an angle, so not exactly


def test_drone_behind_the_camera_has_no_box():
    assert truth((-30.0, 0.0, 20.0))['drones'][0]['box'] is None


def test_birds_are_objects_with_their_class():
    p = poses((30.0, 0.0, 20.0)) + [('bird_1', 30, (20.0, 2.0, 20.0), IDENTITY),
                                    ('body', 31, (0.0, 0.0, 0.0), IDENTITY)]
    data = ground_truth.truth(p, NAMES, 2, 'camera_link', 2.0, 1280, 720)
    by_name = {o['name']: o for o in data['objects']}
    assert by_name['x500_gimbal_1']['class'] == 'drone'
    bird = by_name['bird_1']
    assert bird['class'] == 'bird'
    assert bird['distance'] == pytest.approx(math.hypot(20, 2), abs=0.01)
    assert bird['box'] is not None and bird['box']['x0'] < 640  # 2 m north = left, looking east
    assert len(data['drones']) == 1  # birds are not drones
