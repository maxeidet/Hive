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

## What the node does

`backend_node.py` flies the drone to 5 m above its start point and hovers:

- It streams an offboard heartbeat and a hover setpoint to PX4 at 10 Hz (PX4 needs this stream before it accepts offboard mode).
- Once PX4 reports that its pre-flight checks pass, it sends the offboard-mode and arm commands, and repeats them once per second until PX4 says it is armed and in offboard mode. It logs PX4's answers, so a refusal is visible instead of silent.
- It logs the position once per second and `Reached 5.0 m, hovering` when it gets there.
- Stopping the node makes PX4 land the drone (offboard-loss failsafe).

Setpoint fields the node does not control (velocity, acceleration) are set to NaN, as the PX4 message definition requires. Leaving them at the ROS default of 0 would command zero velocity while climbing.
