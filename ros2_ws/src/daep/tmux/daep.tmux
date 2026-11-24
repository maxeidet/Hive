#!/usr/bin/env python3

from os import system, environ, getlogin, getenv
import time
import sys
import os

from tmux import *

from optparse import OptionParser
parser = OptionParser()
parser.add_option ("-a", "--attach", action="store_true", dest="attach", help="Attach")
parser.add_option ("", "--ns", action="store", type="string", dest="ns", default="/storm", help="Namespace to use (default /husky0 namespace)")
parser.add_option ("", "--world", action="store", type="string", dest="world", default="maze", help="World to use (default maze)")
parser.add_option ("", "--stop", action="store_true", dest="stop", help="Stop execution.")
parser.add_option ("", "--sim", action="store_true", dest="sim", help="Use Gazebo Sim.")
parser.add_option ("", "--slam", action="store_true", dest="slam", help="Use Slam.")
parser.add_option ("", "--vicon", action="store_true", dest="vicon", help="Use Vicon and real Turtlebot 4.")
parser.add_option ("", "--no-vicon", action="store_true", dest="novicon", help="Use Vicon and real Turtlebot 4.")
parser.add_option ("", "--laser", action="store_true", dest="laser", help="Use Laser for Pointcloud.")
parser.add_option ("", "--depth", action="store_true", dest="depth", help="Use ASUS Xtion for Pointcloud.")
parser.add_option ("", "--t4depth", action="store_true", dest="t4depth", help="Use ASUS Xtion for Pointcloud.")
parser.add_option ("", "--dynamic-objects", action="store_true", dest="dynamic_objects", help="Add simulated dynamic obstacles to simulation.")

parser.add_option ("", "--camera", action="store_true", dest="camera", help="Use Laser for Pointcloud.")
parser.add_option ("", "--husky", action="store_true", dest="husky", help="Use Husky Laser for Pointcloud (Velodyne).")
parser.add_option ("", "--turtle4", action="store_true", dest="turtle4", help="Use Turtlebot 4 for Pointcloud.")
parser.add_option ("", "--config", action="store", type="string", dest="config", default='maze_exploration.yaml', help="Default: maze_exploration.yaml")
parser.add_option ("", "--package", action="store", type="string", dest="package", default='daep', help="Default: daep")

parser.add_option ("", "--wara", action="store_true", dest="wara", help="Connect to the specified waraps broker. Start the json api bridge.)")
parser.add_option ("", "--main-wara", action="store_true", dest="main_wara", help="Connect to main waraps broker. Start the json api bridge.)")
parser.add_option ("", "--location", action="store", type="string", dest="location", default=f'{os.getenv("LRS_LOCATION")}', help=f'Location to use (default: {os.getenv("LRS_LOCATION")})')
parser.add_option ("", "--scale", action="store", type="int", dest="scale", default=1, help="Location to use")
parser.add_option ("", "--tststream", action="store_true", dest="tststream", help="Stream tst")
parser.add_option ("", "--wara-broker-host", action="store", type="string", dest="wara_broker_host", default="broker.waraps.org", help="Wara broker (default broker.waraps.org)")
parser.add_option ("", "--wara-broker-port", action="store", type="string", dest="wara_broker_port", default="8883", help="Wara broker port (default 8883)")
parser.add_option ("", "--wara-broker-user", action="store", type="string", dest="wara_broker_user", default="", help="Wara broker user")
parser.add_option ("", "--wara-broker-password", action="store", type="string", dest="wara_broker_password", default="", help="Wara broker passwd")
parser.add_option ("", "--octree-filename", action="store", type="string", dest="octree_filename", default=f'{os.getenv("HOME")}/lrs2_ws/src/lrs_granso/octomaps/granso_l0_center_12.5cm.bt', help="")
parser.add_option ("", "--omehost", action="store", type="string", dest="omehost", default="ome.waraps.org", help="Ome host (default ome.waraps.org)")
parser.add_option ("", "--color", action="store_true", dest="color", help="Stream color camera")
parser.add_option ("", "--bitrate", action="store", type="string", dest="bitrate", default="1000k", help="Bitrate ffmpeg (default 1000k)")
parser.add_option ("", "--omews", action="store", type="string", dest="omews", default="wss://ome.waraps.org:3334/", help="Ome ws (default wss://ome.waraps.org:3334/)")
(options, args) = parser.parse_args()

def get_list_param(name, items):
    res = f"{name}:='["
    first_item = True
    for item in items:
        if first_item:
            res += f'\\\"{item}\\\"'
            first_item = False
        else:
            res += f', \\\"{item}\\\"'
    res += "]'"
    return res

def ffmpeg_command(name, fifo, resolution="1024x600", pixfmt="bgr24", omehost="ome.waraps.org"):
    command = f'while [ true ] ; do ffmpeg -f rawvideo -r 15 -vcodec rawvideo -pix_fmt {pixfmt} -s {resolution} -i {fifo} -r 15 -c:v libx264 -b:v {options.bitrate} -bufsize {options.bitrate} -pix_fmt yuv420p -tune zerolatency -preset ultrafast -f flv rtmp://{omehost}/app/{name} ; sleep 2; done'
    return command

def main_unit(ns):
    if ns:
        command = f"new-session -n MISC -d -s {ns} 'exec bash'"
    else:
        command = f"new-session -n MISC -d -s NOAMESPACE 'exec bash'"        
    tmux(command)


    # Turtlebot 4 simulation
    if not options.slam and options.sim and options.turtle4:
        use_sim_time="true"
        ros2("static_transform_pc", f"ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id turtlebot4/oakd_rgb_camera_frame/rgbd_camera --z -0.15 --ros-args -p _use_sim_time:={use_sim_time}")
        ros2("turtle_connect", f"ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/turtle0 -p _use_sim_time:={use_sim_time} -p vicon:=false -p husky:=false")
        ros2("turtle_manager", f"ros2 run lrs_turtle4 turtle_manager --ros-args -r __ns:=/turtle0 -p vicon:=false -p _use_sim_time:={use_sim_time} -p husky:=false")
        ros2("octomap_server", f"ros2 launch daep octomap_server_turtle4.launch.py cloud_in:=/oakd/rgb/preview/depth/points ns:=/turtle0 param_file:=config/{options.config} config_package:={options.package}")
        ros2("reeval_frontier", f"ros2 launch daep reeval_frontier.launch.py ns:=/turtle0 use_sim_time:={use_sim_time} param_file:={options.config} config_package:={options.package}")
        ros2("pig", f"ros2 launch daep pig.launch.py ns:=/turtle0 use_sim_time:={use_sim_time} param_file:={options.config} config_package:={options.package}")
        ros2("dfm", f"ros2 launch daep dfm.launch.py ns:=/turtle0 use_sim_time:={use_sim_time} param_file:={options.config} config_package:={options.package}")
        if options.dynamic_objects:
            ros2("dynamic_objects", f"ros2 launch daep animate.launch.py ns:=/turtle0 use_sim_time:={use_sim_time} param_file:={options.config} config_package:={options.package}")
        ros2("coverage_evaluation", f"ros2 launch daep coverage_evaluation.launch.py ns:=/turtle0 param_file:={options.config} config_package:={options.package}")
        ros2("rrt_planner", f"ros2 launch daep rrtplanner.launch.py ns:=/turtle0 param_file:={options.config} config_package:={options.package}")
        ros2("frustum", f"ros2 launch daep frustum.launch.py ns:=/turtle0 param_file:={options.config} config_package:={options.package} use_sim_time:={use_sim_time}")        
        ros2("aeplanner", f"ros2 launch daep aeplanner.launch.py ns:=/turtle0 param_file:={options.config} config_package:={options.package}")
        ros2("rpl_exploration", f'ros2 run daep rpl_exploration --ros-args -r __ns:=/turtle0 -p use_sim_time:={use_sim_time} --params-file {os.environ["HOME"]}/lrs2_ws/install/{options.package}/share/{options.package}/config/{options.config}', run=False)

    # Husky simulation
    if options.husky and options.sim:
        use_sim_time="true"
        ros2("static_transform_world_to_odom", f"ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id odom --x 0.0 --y 0.0 --z 0.0 --yaw 0.0 --pitch 0.0 --roll 0.0 --ros-args -p _use_sim_time:={use_sim_time}")
        ros2("static_transform_pc", f"ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id lidar3d_0_laser --z 0.5187 --ros-args -p _use_sim_time:={use_sim_time}")
        if options.slam:
            ros2("turtle_connect", f"ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/husky0 -p _use_sim_time:={use_sim_time} -p vicon:=false -p husky:=true -p slam:=true -p world_frame:=odom")
        else:
            ros2("turtle_connect", f"ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/husky0 -p _use_sim_time:={use_sim_time} -p vicon:=false -p husky:=true -p slam:=false -p world_frame:=odom")
        ros2("turtle_manager", f"ros2 run lrs_turtle4 turtle_manager --ros-args -r __ns:=/husky0 -p vicon:=false -p _use_sim_time:={use_sim_time} -p husky:=true -p world_frame:=odom")
        if options.slam:
            ros2("octomap_server", f"ros2 launch daep octomap_server_turtle4.launch.py cloud_in:=/kiss/frame ns:=/husky0 param_file:=config/{options.config} config_package:={options.package}")
            ros2("localization", f"ros2 launch kiss_icp odometry.launch.py topic:=/a201_0000/sensors/lidar3d_0/points lidar_odom_frame:=odom")
        else:
            ros2("octomap_server", f"ros2 launch daep octomap_server_turtle4.launch.py cloud_in:=/a201_0000/sensors/lidar3d_0/points ns:=/husky0 param_file:=config/{options.config} config_package:={options.package}")
        ros2("reeval_frontier", f"ros2 launch daep reeval_frontier.launch.py ns:=/husky0 use_sim_time:={use_sim_time} param_file:={options.config} config_package:={options.package}")
        ros2("pig", f"ros2 launch daep pig.launch.py ns:=/husky0 use_sim_time:={use_sim_time} param_file:={options.config} config_package:={options.package}")
        ros2("dfm", f"ros2 launch daep dfm.launch.py ns:=/husky0 use_sim_time:={use_sim_time} param_file:={options.config} config_package:={options.package}")
        if options.dynamic_objects:
            ros2("dynamic_objects", f"ros2 launch daep animate.launch.py ns:=/husky0 use_sim_time:={use_sim_time} param_file:={options.config} config_package:={options.package}")
        ros2("coverage_evaluation", f"ros2 launch daep coverage_evaluation.launch.py ns:=/husky0 param_file:={options.config} config_package:={options.package}")
        ros2("rrt_planner", f"ros2 launch daep rrtplanner.launch.py ns:=/husky0 param_file:={options.config} config_package:={options.package}")
        ros2("frustum", f"ros2 launch daep frustum.launch.py ns:=/husky0 param_file:={options.config} config_package:={options.package} use_sim_time:={use_sim_time}")        
        ros2("aeplanner", f"ros2 launch daep aeplanner.launch.py ns:=/husky0 param_file:={options.config} config_package:={options.package}")
        ros2("rviz2", f"LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 launch daep view_robot.launch.py namespace:=a201_0000", run=False)
        ros2("rosbag", f"ros2 bag record {ns}/simple_pose /tf /a201_0000/sensors/lidar3d_0/points {ns}/octomap_full", run=False)
        ros2("rpl_exploration", f'ros2 run daep rpl_exploration --ros-args -r __ns:=/husky0 -p use_sim_time:={use_sim_time} --params-file {os.environ["HOME"]}/husky_ws/install/{options.package}/share/{options.package}/config/{options.config}', run=False)


    # Husky with Velodyne
    if options.husky and not options.sim:
        # husky("static_transform_map_to_world", "ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id map --yaw 0.0 --x -5.0 --y -2.0 --z 0.0 --ros-args -r /tf:=/husky0/tf -r /tf_static:=/husky0/tf_static")
        # husky("static_transform_velodyne", "ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id lidar3d_0_laser --yaw 0.0 --x 0.15 --y 0.15 --z 0.25 --ros-args -r /tf:=/husky0/tf -r /tf_static:=/husky0/tf_static")
        husky("octomap_server", "ros2 launch daep octomap_server_husky.launch.py cloud_in:=/husky0/sensors/lidar3d_0/points ns:=/husky0")
        husky("octomap_saver", f"ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=map.bt -r octomap_binary:={ns}/octomap_binary", run=False)
        husky("bag_record", f'ros2 bag record {ns}/pose {ns}/geopose /tf {ns}/laser0/scan/points {ns}/octomap_full', run=False)
        husky("pig", "ros2 launch daep pig.launch.py ns:=/husky0 use_sim_time:=false param_file:=terra_exploration.yaml param_file:=terra_exploration.yaml")
        husky("dfm", "ros2 launch daep dfm.launch.py ns:=/husky0 use_sim_time:=false")
        if options.dynamic_objects:
            husky("dynamic_objects", "ros2 launch daep animate.launch.py ns:=/husky0 use_sim_time:=false param_file:=terra_exploration.yaml")
        husky("coverage_evaluation", "ros2 launch daep coverage_evaluation.launch.py ns:=/husky0 param_file:=terra_exploration.yaml")
        husky("rrt_planner", "ros2 launch daep rrtplanner.launch.py ns:=/husky0 param_file:=terra_exploration.yaml")
        husky("aeplanner", "ros2 launch daep aeplanner.launch.py ns:=/husky0 param_file:=terra_exploration.yaml")
        husky("rpl_exploration", f'ros2 run daep rpl_exploration --ros-args -r __ns:=/husky0 -p use_sim_time:=false --params-file {os.environ["HOME"]}/lrs2_ws/install/daep/share/daep/config/husky_terra_exploration.yaml', run=False)
        husky("position", "ros2 topic echo /husky0/pose")
        husky("test_driver", "ros2 run  daep daeptest.py --goal -x -1.0 -y 0.0 -z 0.0 --ros-args -r __ns:=/husky0", run=False)

    # Turtlebot 4 with SLAM
    if options.slam and options.turtle4:
        if options.sim:
            use_sim_time="true"
            ros2("static_transform_map_to_world", "ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id map")
            # ros2("ignition", f'ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite world:={options.world} rviz:=true')
            ros2("SLAM", f'ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:={use_sim_time}')
            ros2("turtle_connect", f"ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/turtle0 -p _use_sim_time:={use_sim_time} -p vicon:=false -p slam:=true")
            ros2("turtle_manager", f"ros2 run lrs_turtle4 turtle_manager --ros-args -r __ns:=/turtle0 -p vicon:=false -p _use_sim_time:={use_sim_time} -p slam:=true")
            ros2("laser_scan_filter", "ros2 run daep laser_filter")
            ros2("scan_to_pointcloud", "ros2 launch daep scan_to_pointcloud.launch.py topic:=/scan_filtered ns:='/'")
            ros2("octomap_server", "ros2 launch daep octomap_server_vicon.launch.py cloud_in:=/cloud ns:=/turtle0")
            ros2("pig", f"ros2 launch daep pig.launch.py ns:=/turtle0 use_sim_time:={use_sim_time}")
            ros2("dfm", f"ros2 launch daep dfm.launch.py ns:=/turtle0 use_sim_time:={use_sim_time}")
            if options.dynamic_objects:
                ros2("dynamic_objects", f"ros2 launch daep animate.launch.py ns:=/turtle0 use_sim_time:={use_sim_time}")
            ros2("coverage_evaluation", f"ros2 launch daep coverage_evaluation.launch.py ns:=/turtle0 use_sim_time:={use_sim_time}")
            ros2("rrt_planner", f"ros2 launch daep rrtplanner.launch.py ns:=/turtle0 use_sim_time:={use_sim_time}")
            ros2("aeplanner", "ros2 launch daep aeplanner.launch.py ns:=/turtle0 use_sim_time:=true", run=True)
            run_flag = True
            if options.stop:
                run_flag = False
            ros2("rpl_exploration", "ros2 run daep rpl_exploration --ros-args -r __ns:=/turtle0 -p _use_sim_time:=true -p iteration_limit:=1000000 -p dock_station:=true", run=run_flag)
            ros2("position", "ros2 topic echo /turtle0/pose")
            ros2("test_driver", "ros2 run  daep daeptest.py --goal -x -1.0 -y 0.0 -z 0.0 --ros-args -r __ns:=/turtle0", run=False)
        else:
            turtle4("static_transform_map_to_world", "ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id map --ros-args -r /tf_static:=/storm/tf_static")
            turtle4("SLAM", f'ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=false namespace:=/storm')
            turtle4("turtle_connect", f"ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/storm -p _use_sim_time:=false -p vicon:=false -p slam:=true -r /pose:=/storm/pose")
            turtle4("turtle_manager", "ros2 run lrs_turtle4 turtle_manager --ros-args -r __ns:=/storm -p vicon:=false -p slam:=true")
            turtle4("static_transform_pc", "ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id rplidar_link --yaw 1.55 --z 0.15 --ros-args -r /tf_static:=/storm/tf_static")
            turtle4("laser_scan_filter", "ros2 run daep laser_filter --ros-args -r __ns:=/storm")
            turtle4("scan_to_pointcloud", "ros2 launch daep scan_to_pointcloud.launch.py")
            turtle4("octomap_server", "ros2 launch daep octomap_server_vicon.launch.py")
            turtle4("pig", "ros2 launch daep pig.launch.py ns:=/storm use_sim_time:=false param_file:=terra_exploration.yaml param_file:=terra_exploration.yaml")
            turtle4("dfm", "ros2 launch daep dfm.launch.py ns:=/storm use_sim_time:=false")
            if options.dynamic_objects:
                turtle4("dynamic_objects", "ros2 launch daep animate.launch.py ns:=/storm use_sim_time:=false param_file:=terra_exploration.yaml")
            turtle4("coverage_evaluation", "ros2 launch daep coverage_evaluation.launch.py ns:=/storm param_file:=terra_exploration.yaml")
            turtle4("rrt_planner", "ros2 launch daep rrtplanner.launch.py ns:=/storm param_file:=terra_exploration.yaml")
            turtle4("aeplanner", "ros2 launch daep aeplanner.launch.py ns:=/storm param_file:=terra_exploration.yaml")
            turtle4("SLAM_position", "ros2 topic echo /storm/simple_pose")
            turtle4("rpl_exploration", f'ros2 run daep rpl_exploration --ros-args -r __ns:=/storm -p use_sim_time:=false --params-file {os.environ["HOME"]}/lrs2_ws/install/daep/share/daep/config/terra_exploration.yaml', run=False)
            turtle4("joystick", "ros2 launch lrs_turtle4 joy_teleop.launch.py namespace:=/storm", run=False)
            turtle4("rviz", "ros2 run rviz2 rviz2 -d ~/lrs2_ws/src/daep/rviz2/daep_slam_terra.rviz --ros-args -r /tf:=/storm/tf -r /tf_static:=/storm/tf_static", run=False)

    # Turtlebot 4 with Vicon
    if options.vicon and options.turtle4:
        # turtle4("oak_driver", "ros2 launch depthai_ros_driver pointcloud.launch.py")
        if options.camera:
            turtle4("static_transform_pc", "ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id oak_right_camera_optical_frame --roll -1.717 --yaw -1.717")
        if options.laser:
            turtle4("static_transform_pc", "ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id rplidar_link --yaw 1.55 --z 0.15")
        if options.depth:
            ros2("depth_camera", "ros2 launch daep camera_with_cloud.launch.py")
            ros2("pc_to_mqtt", "ros2 run lrs_mqtt pc_to_mqtt --ros-args -r points:=/camera/depth_registered/points -r octomap:=/storm/octomap_binary -p octomap:=true")
            turtle4("mqtt_to_pc", "ros2 run lrs_mqtt mqtt_to_pc --ros-args -r points:=/camera/depth_registered/points -r octomap:=/storm/octomap_binary -p octomap:=true")
        if options.t4depth:
            turtle4("depth_camera", "ros2 launch daep camera_with_cloud.launch.py")
        if not options.novicon:
            turtle4("vicon", "ros2 launch lrs_vicon vicon.launch.py rate:=10")
            if options.depth:
                ros2("ros2run_vicon", "ros2 launch lrs_vicon vicon.launch.py rate:=10")
        if options.laser:
            turtle4("laser_scan_filter", "ros2 run daep laser_filter --ros-args -r __ns:=/storm")        
            turtle4("scan_to_pointcloud", "ros2 launch daep scan_to_pointcloud.launch.py")
        turtle4("turtle_connect", f"ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/storm -p _use_sim_time:=false -p vicon:=true")
        if options.depth:
            ros2("ros2run_turtle_connect", "ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/storm -p _use_sim_time:=false -p vicon:=true")            

        turtle4("turtle_manager", "ros2 run lrs_turtle4 turtle_manager --ros-args -r __ns:=/storm -p vicon:=true")
        if options.camera:
            turtle4("octomap_server", "ros2 launch daep octomap_server_vicon.launch.py cloud_in:=/oak/points")
        elif options.depth:
            ros2("octomap_server", "ros2 launch daep octomap_server_vicon.launch.py cloud_in:=/camera/depth_registered/points")
        elif options.husky:
            pass
        else:
            turtle4("octomap_server", "ros2 launch daep octomap_server_vicon.launch.py")                
                
        turtle4("pig", "ros2 launch daep pig.launch.py ns:=/storm use_sim_time:=false param_file:=terra_exploration.yaml param_file:=terra_exploration.yaml")
        turtle4("dfm", "ros2 launch daep dfm.launch.py ns:=/storm use_sim_time:=false")
        if options.dynamic_objects:
            turtle4("dynamic_objects", "ros2 launch daep animate.launch.py ns:=/storm use_sim_time:=false param_file:=terra_exploration.yaml")
        turtle4("coverage_evaluation", "ros2 launch daep coverage_evaluation.launch.py ns:=/storm param_file:=terra_exploration.yaml")
        turtle4("rrt_planner", "ros2 launch daep rrtplanner.launch.py ns:=/storm param_file:=terra_exploration.yaml")
        turtle4("aeplanner", "ros2 launch daep aeplanner.launch.py ns:=/storm param_file:=terra_exploration.yaml")
        turtle4("rpl_exploration", f'ros2 run daep rpl_exploration --ros-args -r __ns:=/storm -p use_sim_time:=false --params-file {os.environ["HOME"]}/lrs2_ws/install/daep/share/daep/config/terra_exploration.yaml', run=False)
        turtle4("position", "ros2 topic echo /storm/pose")
        turtle4("test_driver", "ros2 run  daep daeptest.py --goal -x -1.0 -y 0.0 -z 0.0 --ros-args -r __ns:=/storm", run=False)


    tmux(f'attach-session -t {ns}')
    
if __name__ == "__main__":
    ns = options.ns

    main_unit(ns)    
