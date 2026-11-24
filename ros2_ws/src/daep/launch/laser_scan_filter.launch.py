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
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="false"))    
    location_arg = DeclareLaunchArgument("location", default_value=TextSubstitution(text="granso"))
    param_file_arg = DeclareLaunchArgument("param_file", default_value=TextSubstitution(text="angular_filter.yaml"))
    
    config_path = os.path.join(
      get_package_share_directory('daep'),
      'config'
      )
    
    return LaunchDescription([
        ns_arg,
        param_file_arg,
        use_sim_time_arg,
        Node(
            package='laser_filters',
            output='screen',
            emulate_tty=True,            
            namespace=LaunchConfiguration('ns'),
            executable='scan_to_scan_filter_chain',
            parameters=[(config_path, "/", LaunchConfiguration('param_file'))],
        )
        ]
    )
