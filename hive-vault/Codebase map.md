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
├── hive-vault/                  this Obsidian vault
├── .gitattributes               *.sh text eol=lf (scripts must keep LF line endings in the container)
└── README.md                    (empty)
```

The start scripts mount the repo at `/workspace` inside the container, so code edits show up there without rebuilding the image. The container is named `px4-gazebo-vnc`.

---

## `swarm_backend` package

`ros2_ws/src/swarm_backend/`, built with `colcon`, run inside the container.

```text
swarm_backend/                   (ros2_ws/src/swarm_backend/)
├── config/                      swarm.yaml (drones, swarm, attack, intercept, surveillance settings), sensors.rviz
├── scenarios/                   the scenario shell scripts and configs (compare_paths.sh, surveillance.yaml, ...)
├── mount-backend.sh             host-side entry point: builds the package in the container, runs a command
├── tests/                       test_*.py: unit tests without ROS (pytest); *_test.py: flight tests in Gazebo
└── swarm_backend/               the Python package, below
```

```text
swarm_backend/
├── config.py                  Swarm configuration, loaded from config/swarm.yaml
├── runner.py                  Helpers for writing flight sequences as plain top-to-bottom code
├── shell.py                   Interactive Python shell for flying the swarm by hand while watching Gazebo
├── px4/
│   ├── drone.py               What a drone exposes to the rest of the backend
│   ├── messages.py            Builders for the PX4 messages the backend sends, so no other module fills in f…
│   ├── observer.py            Read-only view of a PX4 drone: position and clock, no commands
│   ├── px4_drone.py           A Drone backed by one PX4 SITL instance over ROS 2 (uXRCE-DDS)
│   └── topics.py              The one place PX4 topic names are built
├── swarm/                     The swarm: how the defenders work together, and the base station that decides
│   ├── arc.py                 The surveillance arc: where the drones hold their points and which slice each …
│   ├── association.py         Which sightings are the same target. The drones' cameras do not name what they…
│   ├── base_station.py        The base station's decisions: who attacks which target, who holds back, and ho…
│   ├── dispatcher.py          The base station's message handling, without ROS: what the defenders report go…
│   ├── formation.py           Formation geometry: target positions for N drones
│   ├── friendlies.py          Friendly drones: the hivemind's broadcast of where they all are, and the check…
│   ├── manager.py             Swarm-level commands: what the swarm should do
│   ├── messages.py            The messages between the defenders and the base station, as JSON in std_msgs/S…
│   ├── patrol.py              One defender in the surveillance plan: hold a point on the arc, sweep the came…
│   └── swarm.py               The collection of drones: which drones exist, and broadcasting to them
├── intercept/
│   ├── attack.py              The baseline attack: fly into the hostile drone with pure pursuit, on what the…
│   ├── detection.py           The two detection topics the attack flies on, and a fake detector that makes t…
│   ├── guidance.py            Where a gimbal camera should look to see a target. Pure math, world NED
│   ├── inbox_detector.py      The camera detector's topics as a Detector, so MpcIntercept (or any InterceptA…
│   ├── interceptor.py         The contract between an intercept method and the evaluation harness
│   ├── mpc.py                 Model predictive control for the intercept (planning report, Interception >
│   ├── mpc_action.py          The MPC as an intercept method for the evaluation harness (evaluation/scenario…
│   ├── mpc_attack.py          The MPC intercept (mpc_action.MpcIntercept) behind the baseline attack's face,…
│   ├── target.py              Where target measurements come from: a TargetSource gives the latest Fix of th…
│   └── target_detector.py     What the interceptor's own camera reports about the target, for the evaluation
├── evaluation/                Scoring and scenario maths for the evaluations, without ROS: the hostile drone…
│   ├── arc_eval.py            The baseline evaluation's scenarios on the surveillance arc (`mount-backend.sh
│   ├── attack_scenario.py     The attack test's scenario and scoring, as pure math
│   ├── inbound.py             Scenario 2's pure math: hostile drones flying in a straight line toward the ba…
│   ├── metrics.py             Scoring of intercept runs (planning report, Interception > Evaluation)
│   ├── mock.py                A stand-in for the hostile drone: a PX4 drone flown along a pattern. No ROS he…
│   ├── paths.py               Flight patterns for the hostile drone: where it should be at time t, at consta…
│   └── scenario.py            The intercept evaluation scenarios and the loop that flies one (planning repor…
├── detect/                    Camera detection of hostile drones (see hive-vault/detection/Detection impleme…
│   ├── airframe.py            Where a drone's own airframe shows in its gimbal camera's picture, so the dete…
│   ├── bytetrack.py           ByteTrack (Zhang et al. 2022): keeps an id on each target from image to image,…
│   ├── camera.py              Pinhole camera model (Gazebo's cameras are ideal pinholes, no distortion)
│   ├── candidates.py          Baseline detector: a small dark object against the sky
│   ├── dataset.py             Evaluation on labelled images, outside any scenario (planning report, Drone
│   ├── detector.py            The contract between a detection method and the evaluation harness
│   ├── detectors.py           The detectors, behind one interface, so they can be swapped with one option
│   ├── fake_detections.py     Fake detections from the truth, with the errors a real detector makes, for bui…
│   ├── frame.py               One camera image as the evaluation sees it: the image (if there is one), the
│   ├── kalman.py              Where a tracked box will be in the next image: a constant-velocity Kalman filt…
│   ├── live.py                Run a detector on a drone's camera and publish the target (`mount-backend.sh d…
│   ├── metrics.py             Scoring of detection runs (planning report, Drone Detection Using Sensors >
│   ├── orientation.py         Which way the camera points, from what PX4 reports (task 2 of the baseline att…
│   ├── predict.py             Run a detector on saved images and save what it finds (`mount-backend.sh predi…
│   ├── px4_camera.py          Reads which way a drone's camera points from PX4, for the moment an image was …
│   ├── replay_eval.py         Tune the detector's thresholds on recorded Gazebo runs, offline
│   ├── scan.py                Where a defender's gimbal camera looks while it searches (planning report, Bas…
│   ├── scenario.py            The detection evaluation scenarios and the loop that runs one (planning report…
│   ├── track_eval.py          How good a tracker is: runs it on fake detections (fake_detections.py) and sco…
│   ├── tracking.py            Associate detections between images and confirm targets seen k images in a row
│   └── yolo_onnx.py           Run a YOLO model exported to ONNX (training/README.md) on an image. No PyTorch
├── scenarios/                 Scenarios: a scripted situation flown from start to end and scored from the tr…
│   ├── attack.py              Surveillance attack scenario: the whole chain, scored from the true positions
│   ├── baseline_eval.py       Baseline evaluation scenario: the report's 30-seed scenarios and metrics on th…
│   ├── common.py              What every scenario shares: where the repo and configs are, the --sim flag, an…
│   ├── crash.py               When has a defender crashed? From Gazebo's true poses, no ROS (scenarios/surve…
│   ├── gazebo_surveillance.py The surveillance world in Gazebo (see surveillance.py), in one ROS process:
│   ├── sim_surveillance.py    The surveillance world without Gazebo or ROS (see surveillance.py): the same
│   └── surveillance.py        The world the surveillance scenarios fly in (`attack`, `baseline_eval`): the d…
├── nodes/
│   ├── attack_node.py         Baseline attack node (`mount-backend.sh attack`): the defender flies into the …
│   ├── base_station_node.py   The base station (`mount-backend.sh basestation`): reads what the defenders re…
│   ├── fake_detection.py      Fake camera detector (`mount-backend.sh fakedetect`): the two detection topics…
│   ├── hivemind_node.py       The hivemind: broadcasts where every friendly drone is, so that no drone's det…
│   ├── operator_ui.py         Operator UI (`mount-backend.sh ui`): a web page that shows where the drones ar…
│   └── patrol_node.py         One defender on the arc (`mount-backend.sh patrol ID`): takes off, flies to it…
├── ui/
│   └── model.py               What the operator UI shows and sets, as plain data. No ROS, so it can be teste…
├── sim/                       The simulated world: the fast stand-in for PX4, Gazebo's true poses, birds, th…
│   ├── birds.py               Birds in Gazebo: distractors for the detector, and for training images (`mount…
│   ├── fast_sim.py            A fast stand-in for PX4 in Gazebo: for closed-loop unit tests and for running …
│   ├── ground_truth.py        Where the drones, birds and a camera really are, from Gazebo's poses (cheating…
│   ├── map_snapshot.py        Take one picture of the running Gazebo world from straight above, as the opera…
│   └── sensor_view.py         Show one drone in RViz with its cameras and lidars (`mount-backend.sh view ID`…
└── datagen/                   Training data for YOLO from Gazebo and from flying drones: images, labels, cro…
    ├── box_test.py            Try Gazebo's bounding-box camera for training labels (`mount-backend.sh boxtes…
    ├── gen_data.py            Labelled training images for YOLO from Gazebo (`mount-backend.sh gendata`)
    ├── gen_live.py            Labelled training images taken by a real flying gimbal drone (`mount-backend.s…
    ├── make_crops.py          Cut-outs around the baseline's candidates, for YOLO as a second stage (`mount-…
    ├── save_images.py         Save a drone's camera images from Gazebo as PNG files (`mount-backend.sh snap …
    ├── target_models.py       Target drones of several kinds and colours for the training-data generators
    └── yolo_labels.py         YOLO label files from the truth saved with each image (`mount-backend.sh yolol…
```

Where to look:

- **Swarm behaviour and the base station:** `swarm/`. The rules are in `base_station.py`, the protocol in `messages.py`, each defender's side in `patrol.py`.
- **One drone hitting one target** (pure pursuit, MPC): `intercept/`.
- **Scoring and scenario maths:** `evaluation/`. `scenarios/` flies them, in Gazebo or `--sim`.
- **ROS:** only `nodes/`, `px4/`, `runner.py`, `shell.py`, `scenarios/gazebo_surveillance.py`, `detect/live.py`, `detect/px4_camera.py` and some of the `sim/` and `datagen/` tools touch ROS or Gazebo. Everything else is pure Python and runs in the unit tests.

Dependencies only point downwards: `nodes` / `scenarios` / `shell` / `tests` → `swarm` / `intercept` / `evaluation` / `detect` → `Drone` interface (`px4/drone.py`) → `PX4Drone` → `topics` / `messages`.

### ROS executables (`setup.py`)

| Executable | Module |
|---|---|
| `swarm_shell` | `shell.py` |
| `fake_detection` | `nodes/fake_detection.py` |
| `attack_node` | `nodes/attack_node.py` |
| `base_station_node` | `nodes/base_station_node.py` |
| `patrol_node` | `nodes/patrol_node.py` |
| `operator_ui` | `nodes/operator_ui.py` |
| `hivemind_node` | `nodes/hivemind_node.py` |

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
| Report: intercept baseline evaluation | Harness built (`evaluation/scenario.py`, `evaluation/metrics.py`, `tests/test_intercept_*.py`, `mount-backend.sh baseline-intercept`). Pure pursuit (`intercept/interceptor.py`) and the stand-in detector (`intercept/target_detector.py`) are interfaces only; they are meant to wrap the attack team's `intercept/attack.py` and `intercept/detection.py` |
| Report: detection evaluation | Harness built (`detect/scenario.py`, `detect/metrics.py`, `detect/dataset.py`, `detect/camera.py`, `tests/test_detect_*.py`). The detectors (`detect/detector.py`), the sector scan (`detect/scan.py`) and the image dataset loader are interfaces only. See [[Testing]] |
| Safety layer (minimum distance, geofence) | Only the 3 m abort in `intercept_test.py` |

Related: [[px4_ros2_drone_swarm_development_roadmap]], [[ai_components]], [[1octstella]], [[StackV2]]
