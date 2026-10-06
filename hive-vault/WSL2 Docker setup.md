# PX4 + Gazebo + ROS 2 in Docker on Windows/WSL2

This guide sets up a PX4 SITL development environment with Gazebo and ROS 2 Jazzy using Docker Desktop and WSL2.

> [!note] The repo now has start scripts
> This is the manual setup with the stock `px4io/px4-sitl-gazebo-ros2` image and the default world. The project now uses its own image (`docker/Dockerfile`: VNC GUI, RViz, `radar_msgs`, arming without QGroundControl), the container name `px4-gazebo-vnc`, the `hive_base` world and the backend's `mount-backend.sh`. On Linux use `docker/start-dev-linux.sh`; on Stellar use `docker/start-stellar.sh`. Not yet checked: whether `start-dev-linux.sh` works unchanged under WSL2. See [[Codebase map]] and [[StackV2]].

The setup uses **one Docker container** containing:

- PX4 SITL
    
- Gazebo
    
- MicroXRCE-DDS Agent
    
- ROS 2 Jazzy
    
- `px4_msgs`
    

Multiple WSL terminals attach to the same container, with each terminal running a different process.

---

## 1. Architecture

The final setup looks like this:

```text
Windows
└── WSL2
    └── Ubuntu 24.04
        └── Docker Desktop
            └── px4-gazebo container
                ├── PX4 SITL
                ├── Gazebo
                │   └── X500
                ├── MicroXRCE-DDS Agent
                └── ROS 2 Jazzy
```

Communication:

```text
                    ┌──────────────┐
                    │    Gazebo    │
                    │     X500     │
                    └──────┬───────┘
                           │
                           ▼
                    ┌──────────────┐
                    │  PX4 SITL    │
                    └──────┬───────┘
                           │
                           ▼
                 MicroXRCE-DDS Agent
                           │
                           ▼
                    ┌──────────────┐
                    │    ROS 2     │
                    │   /fmu/*     │
                    └──────────────┘
```

The important principle is:

> **One container, multiple terminals.**

Do not create a separate container for PX4, Gazebo, ROS 2, and MicroXRCE-DDS.

---

# 2. Prerequisites

You need:

- Windows 10/11
    
- WSL2
    
- Ubuntu 24.04 in WSL2
    
- Docker Desktop
    
- Docker Desktop WSL2 integration enabled for Ubuntu 24.04
    

This guide assumes an x86-64/AMD64 computer.

You do **not** need to install PX4, Gazebo, or ROS 2 directly inside Ubuntu. They are provided by the Docker image.

---

# 3. Verify WSL2

Open Ubuntu 24.04 and check:

```bash
uname -m
```

You should get:

```text
x86_64
```

Check Docker:

```bash
docker --version
```

Then:

```bash
docker info
```

The Docker server should be running and the architecture should be:

```text
Architecture: x86_64
```

If Docker works from Ubuntu, continue.

---

# 4. Pull the PX4 Docker image

Pull the pre-built PX4 image:

```bash
docker pull px4io/px4-sitl-gazebo-ros2:latest
```

Verify:

```bash
docker images
```

You should see:

```text
px4io/px4-sitl-gazebo-ros2
```

The image contains the required PX4, Gazebo, ROS 2, and PX4 ROS 2 packages.

---

# 5. Start the container

Start one container:

```bash
docker run --rm -it \
    --name px4-gazebo \
    --network=host \
    px4io/px4-sitl-gazebo-ros2:latest \
    bash
```

You should get a prompt similar to:

```text
root@docker-desktop:/workspace#
```

This is now your main PX4 development container.

---

# 6. Verify the software inside the container

Inside the container:

```bash
which px4
```

Expected:

```text
/opt/px4-gazebo/bin/px4
```

Check Gazebo:

```bash
which gz
```

Expected:

```text
/usr/bin/gz
```

Check MicroXRCE-DDS:

```bash
which MicroXRCEAgent
```

Expected:

```text
/usr/local/bin/MicroXRCEAgent
```

ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

---

# 7. Terminal 1 — MicroXRCE-DDS Agent

Keep the original container terminal open.

Start the DDS agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

Leave this terminal running.

The agent listens on UDP port `8888`.

Do **not** start another MicroXRCEAgent on the same port.

---

# 8. Terminal 2 — PX4 SITL

Open another Ubuntu terminal.

Attach to the existing container:

```bash
docker exec -it px4-gazebo bash
```

Start PX4:

```bash
cd /opt/px4-gazebo
./bin/px4
```

Leave this terminal running.

There is still only **one container**.

`docker exec` simply opens another shell inside the existing container.

---

# 9. Terminal 3 — Gazebo

Open another Ubuntu terminal:

```bash
docker exec -it px4-gazebo bash
```

Start the PX4 Gazebo simulation:

```bash
gz sim --verbose=1 -r -s /opt/px4-gazebo/share/gz/worlds/default.sdf
```

Important:

```text
-s
```

runs Gazebo in server/headless mode.

This is useful when working inside Docker because GUI forwarding from Docker Desktop/WSL2 can require additional configuration.

Leave Gazebo running.

The default world contains the PX4 X500:

```text
/model/x500_0
```

---

# 10. Terminal 4 — ROS 2

Open another Ubuntu terminal:

```bash
docker exec -it px4-gazebo bash
```

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Then source the PX4 ROS 2 workspace:

```bash
source /opt/px4_ros2/install/setup.bash
```

The second workspace contains `px4_msgs`.

You need both:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
```

for a fresh ROS 2 terminal.

---

# 11. Verify ROS 2 ↔ PX4 communication

Run:

```bash
ros2 topic list
```

You should see PX4 topics such as:

```text
/fmu/in/...
/fmu/out/...
```

For example:

```text
/fmu/out/vehicle_attitude
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status_v4
```

This confirms that ROS 2 can see the PX4 DDS topics.

---

# 12. Verify PX4 message definitions

Check that `px4_msgs` is available:

```bash
ros2 pkg prefix px4_msgs
```

Expected:

```text
/opt/px4_ros2/install/px4_msgs
```

You can also inspect a message:

```bash
ros2 interface show px4_msgs/msg/VehicleStatus
```

If this returns the message definition, the PX4 ROS 2 interfaces are correctly sourced.

---

# 13. Read PX4 vehicle state

Now test an actual PX4 topic:

```bash
ros2 topic echo /fmu/out/vehicle_status_v4 --once
```

You should receive something similar to:

```text
timestamp: ...
armed_time: 0
takeoff_time: 0
arming_state: 1
nav_state: 4
failsafe: false
vehicle_type: 1
system_id: 1
component_id: 1
...
```

This confirms that data is travelling through the complete system:

```text
Gazebo
   ↓
PX4 SITL
   ↓
MicroXRCE-DDS Agent
   ↓
ROS 2
```

At this point the basic environment is operational.

---

# 14. Terminal overview

When everything is running, the terminals should look like this:

### Terminal 1 — MicroXRCE-DDS

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2 — PX4

```bash
cd /opt/px4-gazebo
./bin/px4
```

### Terminal 3 — Gazebo

```bash
gz sim --verbose=1 -r -s /opt/px4-gazebo/share/gz/worlds/default.sdf
```

### Terminal 4 — ROS 2

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash

ros2 topic list
```

All four terminals are connected to:

```text
px4-gazebo
```

the same Docker container.

---

# 15. Checking the running processes

From any additional shell:

```bash
docker exec -it px4-gazebo bash
```

Then:

```bash
ps aux | grep '[p]x4'
```

You should see PX4 and Gazebo.

Check MicroXRCE:

```bash
ps aux | grep '[M]icroXRCE'
```

You should see:

```text
MicroXRCEAgent udp4 -p 8888
```

Check Gazebo:

```bash
ps aux | grep '[g]z sim'
```

You should see:

```text
gz sim --verbose=1 -r -s /opt/px4-gazebo/share/gz/worlds/default.sdf
```

---

# 16. Starting from scratch next time

Once the image has been downloaded, starting the environment is simple.

Start the container:

```bash
docker run --rm -it \
    --name px4-gazebo \
    --network=host \
    px4io/px4-sitl-gazebo-ros2:latest \
    bash
```

Then use four terminals:

```text
Terminal 1 → MicroXRCEAgent
Terminal 2 → PX4
Terminal 3 → Gazebo
Terminal 4 → ROS 2
```

All additional terminals attach with:

```bash
docker exec -it px4-gazebo bash
```

---

# 17. Stopping everything

Because the container was started with:

```text
--rm
```

the container is automatically removed when the main container process exits.

Stop the individual processes with:

```text
Ctrl+C
```

Then exit the main container:

```bash
exit
```

If necessary, verify:

```bash
docker ps
```

There should be no `px4-gazebo` container.

---

# 18. Important notes

### Don't start multiple containers

You only need:

```text
px4-gazebo
```

Multiple terminals are just multiple shells into that container.

### Don't start multiple MicroXRCE agents

Only one process should use:

```text
UDP 8888
```

Starting another one will result in a port/bind error.

### Don't start multiple Gazebo servers

Only one Gazebo server should load:

```text
default.sdf
```

### Source `px4_msgs` in every ROS 2 terminal

Use:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
```

Otherwise ROS 2 may see the `/fmu/*` topics but fail to resolve PX4 message types.

---

# 19. Current limitations

This guide intentionally uses:

```bash
-s
```

for Gazebo.

That means Gazebo runs without its graphical interface.

The simulation itself works normally, and ROS 2/PX4 communication can be developed and tested without the GUI.

GUI forwarding can be configured separately if visualization is required.

---

# 20. Final verification

The setup is considered working when all of the following are true:

```text
[✓] Docker works from WSL2
[✓] PX4 Docker image is available
[✓] PX4 SITL is running
[✓] Gazebo is running
[✓] X500 exists in Gazebo
[✓] MicroXRCE-DDS Agent is running
[✓] ROS 2 sees /fmu/in/*
[✓] ROS 2 sees /fmu/out/*
[✓] px4_msgs is available
[✓] ROS 2 can echo PX4 messages
```

A successful test is:

```bash
ros2 topic echo /fmu/out/vehicle_status_v4 --once
```

If this produces a `VehicleStatus` message, the PX4 + Gazebo + ROS 2 Docker environment is ready for development.