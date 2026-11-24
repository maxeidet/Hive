#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
import os
from pathlib import Path

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Launch Configurations
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    arg_rviz_config = DeclareLaunchArgument(
        name='config',
        default_value='a201.rviz',
        description='RViz config file name'
    )

    # Use source directory directly: /root/husky_ws/src/daep/rviz2/a201.rviz
    # Get the path to this launch file's directory
    launch_file_dir = Path(__file__).parent
    # Go up to src/daep and then to rviz2
    daep_source_dir = launch_file_dir.parent
    # Use a201.rviz from source directory (hardcoded as requested)
    rviz_config_path = daep_source_dir / 'daep' / 'rviz2' / 'a201.rviz'

    print(rviz_config_path)
    
    # Convert to absolute path string
    config_rviz = str(rviz_config_path.resolve())

    group_view_model = GroupAction([
        PushRosNamespace(namespace),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', config_rviz],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
             ],
             output='screen')
    ])

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_namespace)
    ld.add_action(arg_rviz_config)
    ld.add_action(arg_use_sim_time)
    # Nodes - OpaqueFunction will be expanded automatically
    ld.add_action(group_view_model)
    return ld
