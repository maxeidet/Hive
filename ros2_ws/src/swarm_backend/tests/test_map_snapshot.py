import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from swarm_backend.map_snapshot import camera_height, camera_sdf, map_extent


def test_camera_height_sees_the_half_size_each_way():
    h = camera_height(150.0, 0.6)
    assert h * math.tan(0.3) == pytest.approx(150.0)


def test_extent_is_a_square_round_the_centre_in_ned():
    assert map_extent(10.0, -5.0, 100.0) == {'north_min': -90.0, 'north_max': 110.0,
                                             'east_min': -105.0, 'east_max': 95.0}


def rotation(roll, pitch, yaw):
    """SDF pose rotation: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def test_camera_looks_down_with_north_up_and_west_left():
    sdf = camera_sdf(30.0, 40.0, 500.0, 0.6, 1024)
    root = ET.fromstring(sdf)
    x, y, z, roll, pitch, yaw = map(float, root.find('model/pose').text.split())
    assert (x, y, z) == (40.0, 30.0, 500.0)  # Gazebo ENU: x east, y north
    r = rotation(roll, pitch, yaw)
    # Camera axes: x forward (where it looks), y left, z up (the top of the image).
    assert r @ [1, 0, 0] == pytest.approx([0, 0, -1], abs=1e-9)  # straight down
    assert r @ [0, 0, 1] == pytest.approx([0, 1, 0], abs=1e-9)  # image top = north
    assert r @ [0, 1, 0] == pytest.approx([-1, 0, 0], abs=1e-9)  # image left = west
    assert root.find('model/link/sensor/camera/image/width').text == '1024'
