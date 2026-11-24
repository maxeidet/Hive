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
    ns_arg = DeclareLaunchArgument("ns", default_value=TextSubstitution(text="/turtle0"))
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="false"))
    
    return LaunchDescription([
        ns_arg,
        use_sim_time_arg,
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
                    "publish_free_space": False,
                    "resolution": 0.1,
                    "publish_free_space": True,
                    "_use_sim_time" : LaunchConfiguration('use_sim_time'),
                    "frame_id": "world",
                    "base_frame_id": "base_link",
                    "sensor_model": { "max_range": 5.0},
                }],
            remappings=[
                ("cloud_in", "/oakd/rgb/preview/depth/points")
            ]
        )
        ]
    )
