import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

from launch_ros.actions import Node

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ns_arg = DeclareLaunchArgument("ns", default_value=TextSubstitution(text="/turtle0"))
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="false"))    
    location_arg = DeclareLaunchArgument("location", default_value=TextSubstitution(text="granso"))
    param_file_arg = DeclareLaunchArgument("param_file", default_value=TextSubstitution(text="maze_exploration.yaml"))
    config_package_arg = DeclareLaunchArgument("config_package", default_value=TextSubstitution(text="daep"))    
    dfm_param_file_arg = DeclareLaunchArgument("dfm_param_file", default_value=TextSubstitution(text="dfm_config.yaml"))

    config_package = LaunchConfiguration('config_package')

    config_path = PathJoinSubstitution([
        FindPackageShare(config_package),
        'config'])
    
    return LaunchDescription([
        ns_arg,
        config_package_arg,
        param_file_arg,
        dfm_param_file_arg,
        use_sim_time_arg,
        Node(
            package='daep',
            output='screen',
            emulate_tty=True,            
            namespace=LaunchConfiguration('ns'),
            executable='dfm.py',
            parameters=[(config_path, "/", LaunchConfiguration('param_file')),
                        (config_path, "/", LaunchConfiguration('dfm_param_file')),
                        { "_use_sim_time" : LaunchConfiguration('use_sim_time') }                        
                        ],
        )
        ]
    )
