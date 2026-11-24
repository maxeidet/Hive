#!/usr/bin/env python3

# Copyright (c) 2020, Michael Ferguson
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    ns_arg = DeclareLaunchArgument("ns", default_value=TextSubstitution(text="/turtle0"))    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="false"))
    
    namespace = '/camera'

    # ros2("static_transform_pc", f"ros2 run tf2_ros static_transform_publisher ")
    static_transform = Node(
        package='tf2_ros',
        output='screen',
        emulate_tty=True,            
        namespace=LaunchConfiguration('ns'),
        executable='static_transform_publisher',
        arguments=["--frame-id", "base_link",  "--child-frame-id",  "openni_rgb_optical_frame", "--x", "-0.1", "--z", "0.1", "--roll", "-1.57",
                   "--pitch", "0.0", "--yaw", "-1.57"],
        parameters=[{ "_use_sim_time" : LaunchConfiguration('use_sim_time') }
                    ],
    )
    

    container = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container_isolated',
            composable_node_descriptions=[
                # Driver
                launch_ros.descriptions.ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driverXXXX',
                    namespace=namespace,
                    parameters=[{'depth_registration': True},
                                {'use_device_time': False}],
                    remappings=[('depth/image', 'depth_registered/image_raw')],
                    #extra_arguments=[{'use_intra_process_comms': True}],                    
                ),
                # Create XYZRGB point cloud
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='points_xyz',
                    namespace=namespace,
                    parameters=[{'queue_size': 10}],
                    remappings=[('camera_info', 'depth/camera_info'),
                                ('image_rect', 'depth_registered/image_raw'),
                                ('points', 'depth_registered/points'), ],
                    #extra_arguments=[{'use_intra_process_comms': True}],                    
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([ns_arg, use_sim_time_arg, static_transform, container])
