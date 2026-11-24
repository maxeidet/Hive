# daep

# NOTE: remember to use --turtle4 in tmux

## SLAM for Real

- Remeber to set route to create 3d on PC computer
- The clock in create 3D must be set correctly (notice set it in UTC)
- Hard time yawing because update rate of simple_pose is so low
- Long startup time
- turtle42 (for terminal use)
- odom for control in manager
- maybe use sync:=flse when starting slam

SYNC CLOCKS:

```bash
ros2 topic echo /storm/scan --field header
```

```bash
ros2 topic echo /storm/imu --field header
```

SLAM plus Exploration
```bash
~/lrs2_ws/src/daep/tmux/daep.tmux --slam
```

Check TF delays:
```bash
ros2 run tf2_ros tf2_monitor --ros-args -r /tf:=/storm/tf -r /tf_static:=/storm/tf_static
```

view frames:

ros2 run tf2_tools view_frames /tf:=/storm/tf /tf_static:=/storm/tf_static

RVIZ:
```bash
ros2 run rviz2 rviz2 -d ~/lrs2_ws/src/daep/rviz2/daep_slam_terra.rviz --ros-args -r /tf:=/storm/tf -r /tf_static:=/storm/tf_static
```

## SLAM Simulation

### Start Ignition Simulation with Maze World

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite world:=maze rviz:=true
```

Start simulation and move Turtlebot away from dock.

### Start wih TMUX

```bash
~/lrs2_ws/src/daep/tmux/daep.tmux --sim --slam
```


## Simulation

### Start Ignition Simulation with Maze World

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite world:=maze
```

### Start wih TMUX

```bash
~/lrs2_ws/src/daep/tmux/daep.tmux --sim
```
For now aeplanner and rpl_exploarion is started manually according to below.

### Start Manually

##### Static Transform

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id turtlebot4/oakd_rgb_camera_frame/rgbd_camera
```

#### turtle_connect

```bash
ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/turtle0 -p use_sim_time:=true -p vicon:=false
```

#### turtle manager

```bash
ros2 run lrs_turtle4 turtle_manager --ros-args -r __ns:=/turtle0 -p vicon:=false
```

#### octomap server

```bash
ros2 launch daep octomap_server.launch.py
```

#### Pig

```bash
ros2 launch daep pig.launch.py ns:=/turtle0 use_sim_time:=true
```


#### DFM

```bash
ros2 launch daep dfm.launch.py ns:=/turtle0 use_sim_time:=true
```

#### Coverage Evaluation

```bash
ros2 launch daep coverage_evaluation.launch.py ns:=/turtle0
```

#### RRT Planner

```bash
ros2 launch daep rrtplanner.launch.py ns:=/turtle0
```

#### AE Planner

```bash
ros2 launch daep aeplanner.launch.py ns:=/turtle0
```

#### RPL Exploraion

```bash
ros2 run daep rpl_exploration --ros-args -r __ns:=/turtle0 -p use_sim_time:=true
```


## VICON

### Start wih TMUX

```bash
~/lrs2_ws/src/daep/tmux/daep.tmux --vicon --ns /storm --laser
```
For now aeplanner and rpl_exploarion is started manually according to below.

### Start Manually

##### Static Transform

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id rplidar_link --yaw 1.55 --z 0.15
```

#### turtle_connect

```bash
ros2 run lrs_turtle4 turtle_connect --ros-args -r __ns:=/storm -p _use_sim_time:=false -p vicon:=true
```

#### turtle manager

```bash
ros2 run lrs_turtle4 turtle_manager --ros-args -r __ns:=/storm -p vicon:=true
```

#### octomap server

```bash
ros2 launch daep octomap_server_vicon.launch.py
```

#### Pig

```bash
ros2 launch daep pig.launch.py ns:=/storm use_sim_time:=false param_file:=terra_exploration.yaml param_file:=terra_exploration.yaml
```


#### DFM

```bash
ros2 launch daep dfm.launch.py ns:=/storm use_sim_time:=false
```

#### Coverage Evaluation

```bash
ros2 launch daep coverage_evaluation.launch.py ns:=/storm param_file:=terra_exploration.yaml
```

#### RRT Planner

```bash
ros2 launch daep rrtplanner.launch.py ns:=/storm param_file:=terra_exploration.yaml
```

#### AE Planner

```bash
ros2 launch daep aeplanner.launch.py ns:=/storm param_file:=terra_exploration.yaml
```

#### RPL Exploraion

```bash
ros2 run daep rpl_exploration --ros-args -r __ns:=/storm -p use_sim_time:=false --params-file /home/tompe/lrs2_ws/install/daep/share/daep/config/terra_exploration.yaml
```



## Asus Xtion

```bash
git clone --recurse-submodules https://github.com/mgonzs13/ros2_asus_xtion
# remove gazebo subdir
# in description cmake file comment away gazebo
sudo apt install ros-humble-depth-image-proc
#start with:
#ros2 launch asus_xtion asus_xtion.launch.py
ros2 launch daep camera_with_cloud.launch.py
```


Frame for points: openni_rgb_optical_frame



### MISC: test driving

```bash
ros2 run  daep daeptest.py --goal -x 1.0 -y 0.5 -z 0.0 --ros-args -r __ns:=/turtle0
```



