#!/usr/bin/env bash
docker exec -it px4-gazebo-vnc bash -c "
  source /opt/ros/jazzy/setup.bash
  source /opt/px4_ros2/install/local_setup.bash
  cd /workspace/ros2_ws
  colcon build --symlink-install
  source install/setup.bash
  ros2 run swarm_backend backend_node &
  ros2 run swarm_backend keyboard
"
