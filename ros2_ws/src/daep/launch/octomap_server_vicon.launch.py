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
    cloud_in_arg = DeclareLaunchArgument("cloud_in", default_value=TextSubstitution(text="/storm/cloud"))
    
    return LaunchDescription([
        ns_arg,
        cloud_in_arg,
        Node(
            package='octomap_server',
            output='screen',
            emulate_tty=True,            
            namespace=LaunchConfiguration('ns'),
            executable='octomap_server_node',
            parameters=[
                {
                    "colored_map": False,
                    "filter_ground_plane": False,
                    "publish_free_space": True,
                    "resolution": 0.1,
                    "use_sim_time": False,
                    "frame_id": "world",
                    "base_frame_id": "base_link",
                    "sensor_model": {
                        "min_range": 0.1,
                        "max_range": 5.0,
                        "min": 0.12,
                        "max": 0.97,
                        "miss": 0.4,
                        "hit": 0.7,
                    },
                }
            ],
            remappings=[
                ("cloud_in", LaunchConfiguration('cloud_in')),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        )
        ]
    )
