from pathlib import Path

import yaml

import pytest

from swarm_backend.mapping import (
    MapStats, map_stats, mapping_rviz_config, ned_to_enu, octomap_args, point_clouds, reach,
    saver_args)
from swarm_backend.sensor_view import find_sensors

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
