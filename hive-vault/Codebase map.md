# Codebase map

What is in the repo, where it lives, and how to run it. Last checked against the `feat-python-swarm` branch on 1 Oct 2026. The package README (`ros2_ws/src/swarm_backend/README.md`) has the details. This note is the overview.

Back to [[Home]].

---

## Repo layout

```text
Hive/
├── docker/                      dev container: PX4 SITL + Gazebo + ROS 2 Jazzy + VNC
│   ├── Dockerfile               on top of px4io/px4-sitl-gazebo-ros2; adds VNC, ros_gz_bridge, rviz2,
│   │                            radar_msgs; sets NAV_DLL_ACT=0 (arm without QGroundControl)
│   ├── start-dev.sh             macOS: build, start container, open a 2nd terminal with PX4 (-m MODEL, -w WORLD)
│   ├── start-dev-linux.sh       Linux version of start-dev.sh (default world hive_base)
│   ├── start-stellar.sh         Stellar lab computers: one terminal, --native-gui (default) / --vnc / --headless, --gpu
│   ├── start-vnc.sh             VNC/noVNC startup inside the container (macOS image)
│   ├── start-vnc-linux.sh       same for Linux
│   ├── README.md                container, worlds, drone sensor variants
│   └── README-stellar.md        Stellar rules, start, cleanup, troubleshooting
├── ros2_ws/src/swarm_backend/   the Python ROS 2 package (see below)
├── worlds/
│   ├── hive_base.sdf            military tent camp in a dense Swedish forest (default world on Linux/Stellar)
│   ├── generate_hive_base.py    generates hive_base.sdf (python3 worlds/generate_hive_base.py)
│   ├── trees/                   CC0 pine and oak meshes, loaded from /workspace so no internet is needed
│   └── models/x500_lidar_3d/    custom drone: 360° 3D lidar, 16 beams, 100 m, 10 Hz
├── recording/                   demo videos (recording/README.md): Gazebo log record + replay angles,
│                                the UI rendered on sim time, compose, compress (video.sh)
├── hive-vault/                  this Obsidian vault
├── .gitattributes               *.sh text eol=lf (scripts must keep LF line endings in the container)
└── README.md                    (empty)
```

The start scripts mount the repo at `/workspace` inside the container, so code edits show up there without rebuilding the image. The container is named `px4-gazebo-vnc`.

---

## `swarm_backend` package

`ros2_ws/src/swarm_backend/`, built with `colcon`, run inside the container.

```text
swarm_backend/
├── config/
│   ├── swarm.yaml               drones (id, spawn_pose, model), swarm settings, intercept settings
│   └── sensors.rviz             RViz layout for `view`
├── launch/swarm.launch.py       starts swarm_node (num_drones, config_file)
├── mount-backend.sh             host-side entry point: builds the package in the container, runs a command
├── swarm_backend/
│   ├── config.py                loads swarm.yaml → SwarmConfig / DroneConfig / InterceptConfig
│   ├── runner.py                spins ROS in a background thread; wait_until()
│   ├── shell.py                 interactive shell (Pilot): takeoff, goto, move, form, land, gimbal, hostile, chase
│   ├── sensor_view.py           bridges a drone's Gazebo sensors to ROS 2 and opens RViz
│   ├── backend_node.py          original single-drone script: arm, offboard, hover at 5 m
│   ├── keyboard.py              original keyboard teleop
│   ├── px4/
│   │   ├── topics.py            the only place PX4 topic names are built
│   │   ├── messages.py          builders for the PX4 messages we send
│   │   ├── drone.py             Drone interface (no ROS/PX4 types)
│   │   ├── px4_drone.py         PX4Drone: heartbeat + setpoint stream, retries arm/offboard/land
│   │   └── observer.py          read-only view of a drone (position, clock), for the mock radar
│   ├── swarm/
│   │   ├── swarm.py             Swarm: which drones exist
│   │   ├── manager.py           SwarmManager: takeoff, hold, land for all/selected drones
│   │   └── formation.py         grid / line / circle targets + slot assignment (pure math)
│   ├── intercept/               one drone chases a hostile drone using radar fixes (see [[1octstella]])
│   │   ├── paths.py             hostile patterns: line, circle, figure8, random
│   │   ├── mock.py              HostileFlight flies the hostile drone; radar_fix
│   │   ├── radar.py             MockRadar (publishes radar_msgs/RadarScan) and RadarTarget
│   │   ├── target.py            TargetSource, RadarFix, PositionFix
│   │   ├── tracker.py           Kalman filter (EKF on range, azimuth, radial velocity)
│   │   ├── guidance.py          lead point, shadow point, braking, phases, gimbal angles (pure math)
│   │   └── chase.py             ChaseAction: one step per setpoint tick
│   └── nodes/
│       ├── swarm_node.py        wires everything together; formation demo
│       └── mock_radar.py        standalone mock radar node
└── tests/
    ├── single_drone_test.py     Milestone 1 flight test (Gazebo)
    ├── multi_drone_swarm_test.py  Milestone 2 flight test (Gazebo)
    ├── intercept_test.py        chase flight test with a scored summary and optional CSV (Gazebo)
    └── test_*.py                unit tests without ROS: config, topics, formation, swarm, intercept, sensor_view
```

Dependencies only point downwards: `shell` / `nodes` / `tests` → `swarm` / `intercept` → `Drone` interface → `PX4Drone` → `topics` / `messages`. `formation.py` and `guidance.py` don't use ROS or PX4.

### ROS executables (`setup.py`)

| Executable | Module |
|---|---|
| `swarm_node` | `nodes/swarm_node.py` |
| `swarm_shell` | `shell.py` |
| `mock_radar` | `nodes/mock_radar.py` |
| `backend_node` | `backend_node.py` (legacy) |
| `keyboard` | `keyboard.py` (legacy) |

### Conventions

- **Frames:** all positions in code are the swarm world frame: **NED** (x north, y east, z down), with its origin at the Gazebo origin. So `z = -5` is 5 m up. `spawn_pose` in `swarm.yaml` is Gazebo **ENU** (the same numbers as `PX4_GZ_MODEL_POSE`) and must match where the drone actually spawned.
- **Namespaces:** PX4 instance 0 → no prefix (`/fmu/...`), system id 1. Instance N → `/px4_N/fmu/...`, system id N+1.
- **Commands don't block:** `arm()`, `set_position()` and the rest return at once. Wait by polling state (`drone.armed`, `drone.reached(tol)`, `runner.wait_until`).
- **Timing:** uses each PX4's own clock (simulation time), so speeds stay right when Gazebo runs slower than real time.
- **Safety:** Ctrl-C or a timeout lands the drones. Stopping the setpoint stream makes PX4 land (offboard-loss failsafe).

---

## How to run

1. Start the container and drone 0: `docker/start-dev.sh` (macOS), `docker/start-dev-linux.sh` (Linux), or `docker/start-stellar.sh` (Stellar).
2. From `ros2_ws/src/swarm_backend/` on the host:

| Command | What it does |
|---|---|
| `./mount-backend.sh spawn` | Start drones 1..N from `swarm.yaml` in the same world |
| `./mount-backend.sh spawn TYPE` | One extra drone: `plain`, `lidar`, `lidar2d`, `lidar_front`, `lidar_down`, `camera`, `camera_down`, `depth`, `flow`, `gimbal`, or any `PX4_SIM_MODEL` |
| `./mount-backend.sh` | Interactive shell: `takeoff()`, `goto(0, 5, 0)`, `form('circle')`, `gimbal(2, -30, 0)`, `hostile()`, `chase()`, `status()`, `land()` |
| `./mount-backend.sh single` / `multi` | Milestone 1 / 2 flight tests |
| `./mount-backend.sh intercept [--pattern figure8 --speed 5 --csv /tmp/run.csv]` | Chase flight test |
| `./mount-backend.sh demo` | Formation demo (`swarm_node`) |
| `./mount-backend.sh view ID` | Drone ID in RViz with its cameras and lidars |
| `./mount-backend.sh legacy` | `backend_node` + keyboard teleop |

Unit tests: `python3 -m pytest tests/test_*.py`.

Default drones in `swarm.yaml`: 0 = `gz_x500` (hostile in the intercept test), 1 = `gz_x500_lidar_3d`, 2 = `gz_x500_gimbal` (the interceptor).

---

## Status against the plans

| Plan item | Status |
|---|---|
| Roadmap 1–9: config, topics, messages, Drone, PX4Drone, single test, Swarm, SwarmManager, Formation | Done |
| Roadmap 10: `mission/mission.py` state machine | Not started |
| Roadmap 11: `nodes/swarm_node.py` | Done (formation demo) |
| Roadmap 12: launch files | Only `swarm.launch.py`. PX4 instances are started by `mount-backend.sh spawn` |
| [[ai_components]] 1a: simulated radar | Mock radar exists (`intercept/radar.py`). It publishes `radar_msgs/RadarScan`, not the planned `Detection` message |
| [[ai_components]] `hive_interfaces` package | Not started |
| [[ai_components]] 1b fusion, 1c decisions | Single-target Kalman tracker only (`intercept/tracker.py`). No multi-track fusion or assignment yet |
| [[ai_components]] 2: lidar detection | 3D lidar drone model and RViz view exist. No detector yet |
| [[ai_components]] 3: path planning | Chase guidance (lead pursuit + shadow) exists. No obstacle-aware planner or search yet |
| Report: intercept baseline evaluation | Harness built (`intercept/scenario.py`, `intercept/metrics.py`, `tests/test_intercept_*.py`, `mount-backend.sh baseline-intercept`). Pure pursuit (`intercept/interceptor.py`) and the stand-in detector (`intercept/target_detector.py`) are interfaces only; they are meant to wrap the attack team's `intercept/attack.py` and `intercept/detection.py` |
| Report: detection evaluation | Harness built (`detect/scenario.py`, `detect/metrics.py`, `detect/dataset.py`, `detect/camera.py`, `tests/test_detect_*.py`). The detectors (`detect/detector.py`), the sector scan (`detect/scan.py`) and the image dataset loader are interfaces only. See [[Testing]] |
| Safety layer (minimum distance, geofence) | Only the 3 m abort in `intercept_test.py` |

Related: [[px4_ros2_drone_swarm_development_roadmap]], [[ai_components]], [[1octstella]], [[StackV2]]
