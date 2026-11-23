import math
from pathlib import Path

import numpy as np
import yaml

import pytest

from swarm_backend.mapping import (
    MapStats, is_depth_camera, map_stats, mapping_rviz_config, ned_to_enu, octomap_args,
    pixel_scale, point_clouds, reach, saver_args, wait_for_clouds, with_free_rays)
from swarm_backend.sim.sensor_view import find_sensors

TEMPLATE = yaml.safe_load((Path(__file__).parent.parent / 'config' / 'sensors.rviz').read_text())

W = '/world/hive_base/model'
TOPICS = [
    '/clock',
    f'{W}/x500_depth_0/link/camera_link/sensor/IMX214/image',
    '/depth_camera',
    '/depth_camera/points',
    f'{W}/x500_lidar_3d_1/link/lidar_link/sensor/lidar_3d/scan/points',
    f'{W}/x500_mono_cam_2/link/camera_link/sensor/camera/image',
]


def test_depth_drone_maps_with_its_point_cloud_only():
    [cloud] = point_clouds(find_sensors(TOPICS, 0))
    assert cloud.ros_topic == '/depth_camera/points'
    assert cloud.frame == 'camera_link'


def test_lidar_drone_maps_with_its_lidar():
    [cloud] = point_clouds(find_sensors(TOPICS, 1))
    assert cloud.ros_topic == '/px4_1/lidar_3d/points'


def test_camera_only_drone_cannot_map():
    assert point_clouds(find_sensors(TOPICS, 2)) == []


def test_octomap_args_use_private_tf_and_clock():
    args = octomap_args('/depth_camera/points', '/mapping/', '/mapping_0/tf', '/mapping_0/clock',
                        resolution=0.2, max_range=6.0)
    assert args[0] == '--ros-args'
    pairs = dict(zip(args[1::2], args[2::2]))  # after --ros-args: flag, value, flag, value...
    remaps = [v for k, v in zip(args[1::2], args[2::2]) if k == '-r']
    params = [v for k, v in zip(args[1::2], args[2::2]) if k == '-p']
    assert '__ns:=/mapping' in remaps
    assert 'cloud_in:=/depth_camera/points' in remaps
    assert '/tf:=/mapping_0/tf' in remaps
    assert '/tf_static:=/mapping_0/tf_static' in remaps
    assert '/clock:=/mapping_0/clock' in remaps
    assert 'frame_id:=world' in params
    assert 'resolution:=0.2' in params
    assert 'sensor_model.max_range:=6.0' in params
    assert 'use_sim_time:=true' in params
    assert set(pairs) <= {'-r', '-p'}


def test_rviz_shows_the_map():
    [cloud] = point_clouds(find_sensors(TOPICS, 0))
    config = mapping_rviz_config(TEMPLATE, [cloud], 'mapping')
    displays = config['Visualization Manager']['Displays']
    [voxels] = [d for d in displays if d['Class'] == 'rviz_default_plugins/MarkerArray']
    assert voxels['Topic']['Value'] == '/mapping/occupied_cells_vis_array'
    assert any(d.get('Topic', {}).get('Value') == '/depth_camera/points' for d in displays)


def test_map_stats():
    assert map_stats([]) == MapStats(0, None, None)
    stats = map_stats([(1.0, -2.0, 0.0), (3.0, 4.0, 0.5), (-1.0, 0.0, 2.0)])
    assert stats.occupied == 3
    assert stats.min == (-1.0, -2.0, 0.0)
    assert stats.max == (3.0, 4.0, 2.0)


def test_ned_to_enu():
    # 10 m north, 2 m east, 3 m up in the swarm frame is x=2 (east), y=10 (north), z=3 in Gazebo.
    assert ned_to_enu((10.0, 2.0, -3.0)) == (2.0, 10.0, 3.0)


def test_reach_along_a_bearing():
    points = [(0.0, 8.0, 0.0), (5.0, 1.0, 0.0), (-3.0, -4.0, 1.0)]  # ENU
    assert reach([], (0.0, 0.0, 0.0), 0.0) is None
    assert reach(points, (0.0, 0.0, 0.0), 0.0) == pytest.approx(8.0)    # north
    assert reach(points, (0.0, 0.0, 0.0), 90.0) == pytest.approx(5.0)   # east
    assert reach(points, (0.0, 0.0, 0.0), 180.0) == pytest.approx(4.0)  # south
    assert reach(points, (0.0, 6.0, 0.0), 0.0) == pytest.approx(2.0)    # from another origin


def test_saver_runs_in_the_servers_namespace():
    assert saver_args('/mapping_test_0/', '/tmp/map.bt') == [
        '--ros-args', '-r', '__ns:=/mapping_test_0', '-p', 'octomap_path:=/tmp/map.bt']


def depth_cloud(depth: np.ndarray, scale: float) -> np.ndarray:
    """The cloud Gazebo's depth camera gives for a depth image (x forward, y left, z up)."""
    h, w = depth.shape
    du = (w - 1) / 2.0 - np.arange(w)[None, :]
    dv = (h - 1) / 2.0 - np.arange(h)[:, None]
    with np.errstate(invalid='ignore'):  # inf * 0 on the centre lines
        return np.stack([depth, depth * scale * du, depth * scale * dv], axis=-1).astype(np.float32)


SCALE = 2.0 * math.tan(1.274 / 2.0) / 64  # the OakD-Lite's 73 degrees over 64 pixels


def test_depth_camera_is_recognised_and_lidar_is_not():
    depth, = point_clouds(find_sensors(TOPICS, 0))
    lidar, = point_clouds(find_sensors(TOPICS, 1))
    assert is_depth_camera(depth) and not is_depth_camera(lidar)


def test_pixel_scale_is_fitted_from_the_returns():
    depth = np.full((48, 64), np.inf)
    depth[24:, :] = np.linspace(3.0, 12.0, 24)[:, None]  # ground in the lower half
    assert pixel_scale(depth_cloud(depth, SCALE)) == pytest.approx(SCALE, rel=1e-4)


def test_pixel_scale_needs_returns_that_fit_a_camera():
    assert pixel_scale(depth_cloud(np.full((48, 64), np.inf), SCALE)) is None  # only sky
    rng = np.random.default_rng(0)
    assert pixel_scale(rng.uniform(1.0, 5.0, (48, 64, 3)).astype(np.float32)) is None


def test_pixels_without_a_return_become_far_points_along_their_ray():
    depth = np.full((48, 64), np.inf)
    depth[24:, :] = 5.0
    depth[47, 0] = -np.inf  # closer than the camera can see
    out = with_free_rays(depth_cloud(depth, SCALE), SCALE, free_range=9.0)
    assert out.shape == (48 * 64 - 1, 3) and out.dtype == np.float32
    assert np.isfinite(out).all()
    # The returns are unchanged, the others are 9 m out.
    far = np.linalg.norm(out, axis=1) > 8.0
    assert far.sum() == 24 * 64
    assert np.linalg.norm(out[far], axis=1) == pytest.approx(9.0, rel=1e-5)
    # The top left pixel looks up and to the left (y left, z up).
    x, y, z = out[0]
    assert y / x == pytest.approx(SCALE * 31.5, rel=1e-5)
    assert z / x == pytest.approx(SCALE * 23.5, rel=1e-5)


def test_cloud_step_keeps_every_nth_pixel():
    depth = np.full((48, 64), 5.0)
    out = with_free_rays(depth_cloud(depth, SCALE), SCALE, free_range=9.0, step=4)
    assert out.shape == (12 * 16, 3)
    assert out[1] == pytest.approx(depth_cloud(depth, SCALE)[0, 4])


class FakeGazebo:
    """Topic lists that change over time: the drone first, its sensors `delay` s later."""

    def __init__(self, delay: float):
        self.t, self.delay = 0.0, delay

    def topics(self):
        drone_only = [t for t in TOPICS if 'x500_depth_0' in t and 'camera_link' not in t] or \
            ['/world/hive_base/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu']
        return TOPICS if self.t >= self.delay else ['/clock', *drone_only]

    def sleep(self, s):
        self.t += s


def test_waits_for_the_sensor_to_appear():
    gz = FakeGazebo(delay=5.0)
    drone, clouds = wait_for_clouds(0, 60.0, gz.topics, gz.sleep, lambda: gz.t)
    assert drone.name == 'x500_depth_0'
    assert [c.ros_topic for c in clouds] == ['/depth_camera/points']
    assert gz.t == 5.0


def test_gives_up_after_the_timeout():
    gz = FakeGazebo(delay=100.0)
    drone, clouds = wait_for_clouds(0, 10.0, gz.topics, gz.sleep, lambda: gz.t)
    assert drone is not None and clouds == []
    assert gz.t == 10.0
    gz = FakeGazebo(delay=0.0)
    assert wait_for_clouds(7, 3.0, gz.topics, gz.sleep, lambda: gz.t) == (None, [])

