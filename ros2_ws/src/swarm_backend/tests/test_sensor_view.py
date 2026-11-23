from pathlib import Path

import yaml

from swarm_backend.sim.sensor_view import (
    Drone, bridge_config, drone_transforms, find_drone, find_sensors, rviz_config)

TEMPLATE = yaml.safe_load((Path(__file__).parent.parent / 'config' / 'sensors.rviz').read_text())

W = '/world/hive_base/model'
TOPICS = [
    '/clock',
    f'{W}/x500_0/link/base_link/sensor/imu_sensor/imu',
    f'{W}/x500_lidar_3d_1/link/lidar_link/sensor/lidar_3d/scan',
    f'{W}/x500_lidar_3d_1/link/lidar_link/sensor/lidar_3d/scan/points',
    f'{W}/x500_mono_cam_2/link/camera_link/sensor/camera/image',
    f'{W}/x500_mono_cam_2/link/camera_link/sensor/camera/camera_info',
    f'{W}/x500_mono_cam_12/link/camera_link/sensor/camera/image',
    f'{W}/x500_depth_3/link/camera_link/sensor/IMX214/image',
    '/depth_camera',
    '/depth_camera/points',
]


def test_find_drone():
    assert find_drone(TOPICS, 0) == Drone('hive_base', 'x500', 0)
    assert find_drone(TOPICS, 1).name == 'x500_lidar_3d_1'
    assert find_drone(TOPICS, 12).model == 'x500_mono_cam'
    assert find_drone(TOPICS, 4) is None


def test_plain_drone_has_no_sensors():
    assert find_sensors(TOPICS, 0) == []


def test_lidar():
    [lidar] = find_sensors(TOPICS, 1)
    assert lidar.kind == 'points'
    assert lidar.ros_topic == '/px4_1/lidar_3d/points'
    assert lidar.frame == 'lidar_link'


def test_camera_does_not_match_other_instance():
    [camera] = find_sensors(TOPICS, 2)
    assert camera.kind == 'image'
    assert camera.gz_topic == f'{W}/x500_mono_cam_2/link/camera_link/sensor/camera/image'
    assert camera.ros_topic == '/px4_2/camera/image'


def test_depth_camera_adds_global_topics():
    sensors = find_sensors(TOPICS, 3)
    assert [(s.name, s.kind) for s in sensors] == [
        ('IMX214', 'image'), ('depth_camera', 'image'), ('depth_camera', 'points')]
    assert sensors[2].ros_topic == '/px4_3/depth_camera/points'


def test_bridge_config():
    [entry, clock] = bridge_config(find_sensors(TOPICS, 1), clock_topic='/sensor_view_1/clock')
    assert entry['ros_type_name'] == 'sensor_msgs/msg/PointCloud2'
    assert entry['gz_type_name'] == 'gz.msgs.PointCloudPacked'
    assert entry['direction'] == 'GZ_TO_ROS'
    assert (clock['gz_topic_name'], clock['ros_topic_name']) == ('/clock', '/sensor_view_1/clock')


Q = (0.0, 0.0, 0.0, 1.0)
POSES = [
    ('x500_0', 10, (1.0, 2.0, 3.0), Q),
    ('x500_gimbal_2', 50, (0.0, 5.0, 0.0), Q),
    ('base_link', 11, (0.0, 0.0, 0.24), Q),
    ('base_link', 51, (0.0, 0.0, 0.24), Q),
    ('camera_link', 60, (0.1, 0.0, 0.1), Q),
    ('camera_link', 90, (9.0, 9.0, 9.0), Q),  # a later model's link
]


def test_drone_transforms_pick_the_drones_own_links():
    transforms = drone_transforms(POSES, 'x500_gimbal_2', ['base_link', 'camera_link'])
    assert transforms == [
        ('world', 'x500_gimbal_2', (0.0, 5.0, 0.0), Q),
        ('x500_gimbal_2', 'base_link', (0.0, 0.0, 0.24), Q),
        ('x500_gimbal_2', 'camera_link', (0.1, 0.0, 0.1), Q),
    ]


def test_drone_transforms_skip_missing():
    assert drone_transforms(POSES, 'x500_7', ['base_link']) == []
    assert drone_transforms(POSES, 'x500_0', ['base_link', 'lidar_link']) == [
        ('world', 'x500_0', (1.0, 2.0, 3.0), Q), ('x500_0', 'base_link', (0.0, 0.0, 0.24), Q)]


def test_rviz_config_one_display_per_sensor():
    config = rviz_config(TEMPLATE, find_sensors(TOPICS, 3))
    manager = config['Visualization Manager']
    classes = [d['Class'] for d in manager['Displays']]
    assert classes.count('rviz_default_plugins/Image') == 2
    assert classes.count('rviz_default_plugins/PointCloud2') == 1
    assert manager['Global Options']['Fixed Frame'] == 'world'
    # The template itself is untouched.
    assert 'rviz_default_plugins/Image' not in [
        d['Class'] for d in TEMPLATE['Visualization Manager']['Displays']]


def test_rviz_config_without_sensors():
    classes = [d['Class'] for d in rviz_config(TEMPLATE, [])['Visualization Manager']['Displays']]
    assert 'rviz_default_plugins/PointCloud2' not in classes
    assert 'rviz_default_plugins/TF' in classes


def test_rviz_config_shows_the_detection_image_first_for_a_camera():
    from swarm_backend.sim.sensor_view import detection_debug_topic
    sensors = find_sensors(TOPICS, 3)
    displays = rviz_config(TEMPLATE, sensors, detection_debug_topic(3))['Visualization Manager']['Displays']
    images = [d for d in displays if d['Class'] == 'rviz_default_plugins/Image']
    assert images[0]['Name'] == 'detection (boxes)'
    assert images[0]['Topic']['Value'] == '/px4_3/detection/debug'
    assert len(images) == 1 + sum(s.kind == 'image' for s in sensors)
    assert images[0]['Enabled']
    assert not any(d['Enabled'] for d in images[1:])  # the camera images are there, but off


def test_camera_image_on_when_there_is_no_detection_display():
    displays = rviz_config(TEMPLATE, find_sensors(TOPICS, 3))['Visualization Manager']['Displays']
    images = [d for d in displays if d['Class'] == 'rviz_default_plugins/Image']
    assert images and all(d['Enabled'] for d in images)


def test_no_detection_image_without_a_camera():
    from swarm_backend.sim.sensor_view import detection_debug_topic
    lidar_only = [s for s in find_sensors(TOPICS, 3) if s.kind == 'points']
    displays = rviz_config(TEMPLATE, lidar_only, detection_debug_topic(3))['Visualization Manager']['Displays']
    assert 'detection (boxes)' not in [d.get('Name') for d in displays]


def test_detection_topic_is_where_the_detector_publishes():
    from swarm_backend.detect.live import offset_topic
    from swarm_backend.sim.sensor_view import detection_debug_topic
    for drone_id in (0, 1, 3):
        assert detection_debug_topic(drone_id) == offset_topic(drone_id, 'debug')
