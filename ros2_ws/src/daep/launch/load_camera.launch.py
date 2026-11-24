import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
import launch_ros.actions
import launch_ros.descriptions
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

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
    
    driver = Node(
        package='openni2_camera',
        output='screen',        
        name='driverXXXX',
        namespace=namespace,
        executable="openni2_camera_driver",
        parameters=[{'depth_registration': True},
                    {'use_device_time': False}],
        remappings=[('depth/image', 'depth_registered/image_raw')],
    )

    pointcloud = Node(
        package='depth_image_proc',
        output='screen',        
        name='points_xyz',
        executable='point_cloud_xyz_node',
        namespace=namespace,
        parameters=[{'queue_size': 10}],
        remappings=[('camera_info', 'depth/camera_info'),
                    ('image_rect', 'depth_registered/image_raw'),
                    ('points', 'depth_registered/points'), ],
    )



    return LaunchDescription([ns_arg, use_sim_time_arg, static_transform, driver, pointcloud])
