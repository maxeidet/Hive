import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    ns_arg = DeclareLaunchArgument("ns", default_value=TextSubstitution(text="/dji1"))
    cloud_in_arg = DeclareLaunchArgument("cloud_in", default_value=TextSubstitution(text="/dji1/laser0/scan/points"))
    param_file_arg = DeclareLaunchArgument("param_file", default_value="")
    config_package_arg = DeclareLaunchArgument("config_package", default_value="")

    # Build parameters list
    parameters_list = []


    # Base parameters (these will be used as defaults if not overridden by param file)
    base_parameters = {
        "colored_map": False,
        "filter_ground_plane": False,
        "publish_free_space": True,
        "resolution": 0.1,  # Default resolution - will be overridden by param file
        "use_sim_time": True,
        "frame_id": "world",
        "base_frame_id": "base_link",
        "sensor_model": {
            "min_range": 0.0,
            "max_range": 5.0,
            "min": 0.12,
            "max": 0.97,
            "miss": 0.4,
            "hit": 0.7,
        },
    }
    parameters_list.append(base_parameters)
    
    # Construct the full path to the parameter file using the config package
    param_file_path = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration('config_package')),
        LaunchConfiguration('param_file')
    ])
    
    # Add param file if provided (will override defaults)

    parameters_list.append(param_file_path)
    

    return LaunchDescription([
        ns_arg,
        cloud_in_arg,
        param_file_arg,
        config_package_arg,
        Node(
            package='octomap_server',
            output='screen',
            emulate_tty=True,            
            namespace=LaunchConfiguration('ns'),
            executable='octomap_server_node',
            parameters=parameters_list,            
            remappings=[
                ("cloud_in", LaunchConfiguration('cloud_in')),
            ]
        )
        ]
    )
