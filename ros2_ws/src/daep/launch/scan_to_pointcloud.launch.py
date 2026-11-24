import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    ns_arg = DeclareLaunchArgument("ns", default_value=TextSubstitution(text="/storm"))
    topic_arg = DeclareLaunchArgument("topic", default_value=TextSubstitution(text="/storm/scan_filtered"))
    
    return LaunchDescription([
        ns_arg,
        topic_arg,
        Node(
            package='pointcloud_to_laserscan',
            output='screen',
            emulate_tty=True,            
            namespace=LaunchConfiguration('ns'),
            executable='laserscan_to_pointcloud_node',
            parameters=[
                {"angle_min": -1.717,
                 "angle_max": 1.717,
                 "range_min": 0.2,
                }
            ],
            remappings=[
                ("scan_in", LaunchConfiguration('topic')),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        )
        ]
    )
