# swarm_backend

Python backend for the drone swarm, running as a ROS 2 node (`rclpy`) inside the same container as PX4/Gazebo — not on the host.

## Bind mounting

`docker/start-dev.sh` mounts the whole repo into the container:
```bash
-v "$(pwd)/..":/workspace
```
So this package's source ends up at `/workspace/ros2_ws/src/swarm_backend` inside the container. Because it's a bind mount (not a copy baked into the image), editing files locally shows up inside the container immediately — no image rebuild needed after a code change, just rebuild the ROS 2 workspace and rerun the node:
```bash
docker exec -it px4-gazebo-vnc bash -c "
  source /opt/ros/jazzy/setup.bash
  source /opt/px4_ros2/install/local_setup.bash
  cd /workspace/ros2_ws
  colcon build --symlink-install
  source install/setup.bash
  ros2 run swarm_backend backend_node
"
```
