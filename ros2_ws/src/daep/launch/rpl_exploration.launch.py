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
    location_arg = DeclareLaunchArgument("location", default_value=TextSubstitution(text="granso"))
    param_file_arg = DeclareLaunchArgument("param_file", default_value=TextSubstitution(text="maze_exploration.yaml"))

    config_path = os.path.join(
      get_package_share_directory('daep'),
      'config'
    )
    
    return LaunchDescription([
        ns_arg,
        param_file_arg,
        Node(
            package='daep',
            output='screen',
            emulate_tty=True,            
            namespace=LaunchConfiguration('ns'),
            executable='rpl_exploration',
            parameters=[(config_path, "/", LaunchConfiguration('param_file'))],
        )
        ]

  )
