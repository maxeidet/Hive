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

## Swarm backend (`swarm_node`)

Built bottom-up following `hive-vault/px4_ros2_drone_swarm_development_roadmap.md`. `backend_node` and `keyboard` above are the original single-drone scripts and are unchanged.

| Module | Role |
|---|---|
| `config/swarm.yaml`, `config.py` | Drones (PX4 instance ids, spawn poses) and swarm settings. Namespace and MAVLink system id default to what PX4 uses per instance: `""`/`1` for instance 0, `px4_N`/`N+1` otherwise. |
| `px4/topics.py` | The only place PX4 topic names are built. |
| `px4/messages.py` | Builders for the PX4 messages we send. |
| `px4/drone.py` | `Drone` interface used by everything above it. No ROS or PX4 types. |
| `px4/px4_drone.py` | `PX4Drone`: streams the offboard heartbeat and setpoint, repeats arm/offboard/land until PX4 confirms. |
| `swarm/swarm.py`, `swarm/manager.py` | `Swarm` = which drones exist; `SwarmManager` = takeoff, move, formation, land. |
| `swarm/formation.py` | Grid, line and circle target positions plus slot assignment. Pure math. |
| `runner.py` | Spins ROS in a background thread so flight sequences read top to bottom. |
| `nodes/swarm_node.py` | Wires it together and flies a demo: takeoff, grid, line, circle, land. |

Conventions: drone commands are non-blocking requests (poll `drone.armed`, `drone.reached(tol)` and similar). Positions are in one shared world frame, NED with its origin at the Gazebo origin (`z = -5` means 5 m up). `spawn_pose` in `swarm.yaml` is in Gazebo ENU, the same numbers you pass as `PX4_GZ_MODEL_POSE`, and must match where each drone was actually spawned, because it is what converts between each PX4's local frame and the world frame.

### Running

With the container and drone 0 running (`docker/start-dev.sh`), from this folder on the host:

```bash
./mount-backend.sh spawn        # start drones 1..N from swarm.yaml in the same Gazebo world
./mount-backend.sh              # interactive shell: takeoff(), goto(0, 5, 0), form('circle'), land(), status()
./mount-backend.sh single       # Milestone 1 test on drone 0
./mount-backend.sh multi        # Milestone 2 test on every drone
./mount-backend.sh demo         # formation demo (swarm_node)
./mount-backend.sh legacy       # the old backend_node + keyboard teleop
```

Each command rebuilds the package first and reads `config/swarm.yaml` from this folder.
Ctrl-C or a timeout in any of these lands the drones.

Unit tests for the parts that do not need ROS (config, topics, formations, swarm logic): `python3 -m pytest tests/test_*.py`.
