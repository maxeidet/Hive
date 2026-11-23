"""3D mapping: one PX4 drone explores an area with the DAEP planner and maps it.

Starts the 3D map (swarm_backend/mapping.py: octomap_server on the drone's depth camera or
lidar), every DAEP node, the bridge (explore/daep_bridge.py), which takes off, flies DAEP's
goals and lands when the exploration is completed, and explore/save_map.py, which then saves
the map as data/map.bt. `mount-backend.sh explore` runs it.

DAEP is LiU's private code and not part of Hive: clone the forks daep and daep_msgs into
ros2_ws/src and build them first (colcon build --packages-up-to daep). PX4 SITL, Gazebo and
the DDS agent must be running with a depth or lidar drone, e.g.
`mount-backend.sh restart -m gz_x500_depth`.

    ros2 launch swarm_backend daep_exploration.launch.py
    ros2 launch swarm_backend daep_exploration.launch.py center:=13.5,-14 radius:=7 alt:=4 z_max:=5

Without `center` the box to explore is the one in px4_exploration.yaml and DAEP
starts where the drone took off. With `center` (x,y in the Gazebo world frame, ENU) the
box is centre +- radius, from z_min to z_max metres up, and the drone first flies to the
centre at `alt` metres; keep that above the obstacles on the way. The drone's size and
camera, and when the exploration stops, are in px4_exploration.yaml in the DAEP fork.

Run it from ros2_ws: rpl_exploration writes its logs (coverage.csv, ...) and save_map the
map to ./data, overwriting the last run.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def box_around(center: str, radius: float, z_min: float, z_max: float) -> dict:
    """DAEP's boundary parameters for a square box around `center` ("x,y")."""
    x, y = (float(v) for v in center.split(','))
    if radius <= 0 or z_max <= z_min:
        raise ValueError(f'Bad box: radius {radius}, z {z_min}..{z_max}')
    return {'boundary/min': [x - radius, y - radius, z_min],
            'boundary/max': [x + radius, y + radius, z_max]}


def nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    ns, drone_id, map_ns = arg('ns'), arg('drone_id'), arg('map_ns')
    config = PathJoinSubstitution([FindPackageShare(arg('config_package')), 'config'])
    param_file = [config, '/', arg('param_file')]
    octomap = [('octomap', f'/{map_ns}/octomap_full')]
    center = arg('center').strip()
    box = box_around(center, float(arg('radius')), float(arg('z_min')),
                     float(arg('z_max'))) if center else {}
    # DAEP counts what a viewpoint would see out to aep/gain/r_max; give it the range the map
    # is built with, so it neither over- nor underrates far viewpoints.
    overrides = {**box, 'aep/gain/r_max': float(arg('map_range'))}
    bridge_args = ['--id', drone_id, '--alt', arg('alt')]
    if center:
        bridge_args += ['--start', *center.split(',')]

    def daep_node(executable, extra_params=(), remappings=()):
        # The overrides come last, so they replace the yaml file's box and gain range.
        return Node(package='daep', executable=executable, namespace=ns, output='screen',
                    emulate_tty=True, parameters=[param_file, *extra_params, overrides],
                    remappings=list(remappings))

    return [
        ExecuteProcess(
            cmd=['python3', '-m', 'swarm_backend.mapping', '--id', drone_id, '--ns', map_ns,
                 '--max-range', arg('map_range')],
            output='screen', condition=IfCondition(arg('mapping'))),
        daep_node('pig.py', extra_params=[[config, '/pig_config.yaml']]),
        daep_node('dfm.py', extra_params=[[config, '/dfm_config.yaml']]),
        daep_node('aeplanner_node', remappings=octomap),
        daep_node('rrt', remappings=octomap),
        daep_node('coverage_evaluation', remappings=octomap),
        daep_node('rpl_exploration'),
        # On Ctrl-C the bridge lands the drone, so it gets time for that before launch kills it.
        Node(package='swarm_backend', executable='daep_bridge', namespace=ns, output='screen',
             emulate_tty=True, arguments=bridge_args,
             sigterm_timeout='200', sigkill_timeout='30'),
        Node(package='swarm_backend', executable='save_map', namespace=ns, output='screen',
             emulate_tty=True, parameters=[{'map_ns': map_ns, 'path': 'data/map.bt'}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='daep', description='namespace of the DAEP nodes'),
        DeclareLaunchArgument('drone_id', default_value='0', description='PX4 instance number'),
        DeclareLaunchArgument('alt', default_value='2.0',
                              description='take-off altitude in metres, where DAEP starts'),
        DeclareLaunchArgument('center', default_value='',
                              description='x,y (ENU) of the area to explore; empty = the yaml box'),
        DeclareLaunchArgument('radius', default_value='6.0', description='half the box side, metres'),
        DeclareLaunchArgument('z_min', default_value='0.5', description='box bottom, metres up'),
        DeclareLaunchArgument('z_max', default_value='3.0', description='box top, metres up'),
        DeclareLaunchArgument('param_file', default_value='px4_exploration.yaml'),
        DeclareLaunchArgument('config_package', default_value='daep'),
        DeclareLaunchArgument('map_ns', default_value='mapping',
                              description='namespace of octomap_server (the map is <map_ns>/octomap_full)'),
        DeclareLaunchArgument('mapping', default_value='true',
                              description='start the map here; false when `mount-backend.sh map` runs'),
        DeclareLaunchArgument('map_range', default_value='8.0',
                              description='how far from the sensor points go into the map, metres '
                                          '(the depth camera sees 19.1 m); DAEP counts what it '
                                          'would see out to the same range'),
        OpaqueFunction(function=nodes),
    ])
