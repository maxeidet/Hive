"""Start the swarm node.

    ros2 launch swarm_backend swarm.launch.py
    ros2 launch swarm_backend swarm.launch.py num_drones:=2 config_file:=/path/to/swarm.yaml

The PX4 instances themselves are started separately (see the package README).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value='',
            description='swarm.yaml to use; empty means the one installed with the package'),
        DeclareLaunchArgument(
            'num_drones', default_value='0',
            description='fly only the first N drones in the config; 0 means all'),
        Node(
            package='swarm_backend',
            executable='swarm_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'num_drones': ParameterValue(LaunchConfiguration('num_drones'), value_type=int),
            }],
        ),
    ])
