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
  ros2 run swarm_backend swarm_shell
"
```

## Swarm backend

Built bottom-up following `hive-vault/px4_ros2_drone_swarm_development_roadmap.md`.

| Module | Role |
|---|---|
| `config/swarm.yaml`, `config.py` | Drones (PX4 instance ids, spawn poses) and swarm settings. Namespace and MAVLink system id default to what PX4 uses per instance: `""`/`1` for instance 0, `px4_N`/`N+1` otherwise. |
| `px4/topics.py` | The only place PX4 topic names are built. |
| `px4/messages.py` | Builders for the PX4 messages we send. |
| `px4/drone.py` | `Drone` interface used by everything above it. No ROS or PX4 types. |
| `px4/px4_drone.py` | `PX4Drone`: streams the offboard heartbeat and setpoint, repeats arm/offboard/land until PX4 confirms. |
| `swarm/swarm.py`, `swarm/manager.py` | `Swarm` = which drones exist; `SwarmManager` = takeoff, move, formation, land. |
| `swarm/formation.py` | Grid, line and circle target positions plus slot assignment. Pure math. |
| `swarm/arc.py` | The surveillance arc: each defender's slot and the sector it sweeps. |
| `swarm/patrol.py` | `PatrolAction`: one defender, PATROL / ENGAGE / RETURN, on the base station's orders. |
| `swarm/base_station.py`, `swarm/dispatcher.py` | The base station: the rules (who attacks, the reserve, sectors, call-off) and the message handling around them. |
| `swarm/messages.py`, `swarm/association.py` | The `/surveillance/*` topics between defenders and base station; which sightings are the same target. |
| `swarm/friendlies.py` | The hivemind's broadcast of where every friendly drone is (`/hive/friendlies`). |
| `runner.py` | Spins ROS in a background thread so flight sequences read top to bottom. |

Conventions: drone commands are non-blocking requests (poll `drone.armed`, `drone.reached(tol)` and similar). Positions are in one shared world frame, NED with its origin at the Gazebo origin (`z = -5` means 5 m up). `spawn_pose` in `swarm.yaml` is in Gazebo ENU, the same numbers you pass as `PX4_GZ_MODEL_POSE`, and must match where each drone was actually spawned, because it is what converts between each PX4's local frame and the world frame.

### Running

With the container and drone 0 running (`docker/start-dev.sh`), from this folder on the host:

```bash
./mount-backend.sh spawn        # start drones 1..N from swarm.yaml in the same Gazebo world
./mount-backend.sh              # interactive shell: takeoff(), goto(0, 5, 0), form('circle'), land(), status()
./mount-backend.sh single       # Milestone 1 test on drone 0
./mount-backend.sh multi        # Milestone 2 test on every drone
./mount-backend.sh spawn camera # ONE more drone with a camera, at the next free id
./mount-backend.sh view 1       # drone 1 in RViz with its cameras and lidars (`lidar 1` / `camera 1` too)
./mount-backend.sh restart [N]  # fresh Gazebo + drone 0 (and 1..N-1) without restarting the container
```

Each command rebuilds the package first and reads `config/swarm.yaml` from this folder.

Drone models come from the `model:` field in `swarm.yaml` (drone 1 is `gz_x500_lidar_3d`: 16 beams, 360°, 100 m, 10 Hz, defined in `worlds/models/`).

`spawn TYPE` starts one extra drone instead: `plain`, `lidar` (the 3D lidar above), `lidar2d`, `lidar_front` / `lidar_down` (1-beam rangefinders), `camera`, `camera_down`, `depth` (RGB + depth camera), `flow`, `gimbal`, or any `PX4_SIM_MODEL` such as `gz_x500_depth`. It takes the lowest instance id that is not running and that id's `spawn_pose` from `swarm.yaml`, so the shell can fly it; ids beyond `swarm.yaml` get `(4*id, -5, 0)` and are not flown by the shell.

`view ID` (`swarm_backend/sim/sensor_view.py`) works for every drone type. It finds the drone's sensors in Gazebo, bridges them to ROS 2 as `/px4_ID/<sensor>/image` (`sensor_msgs/Image`) and `/px4_ID/<sensor>/points` (`sensor_msgs/PointCloud2`, frame = the sensor's link), e.g. `/px4_1/lidar_3d/points`, and opens RViz from `config/sensors.rviz` with one display per sensor. It also publishes TF from Gazebo's poses, `world` (Gazebo ENU) -> drone model (e.g. `x500_gimbal_2`) -> `base_link` and the sensor links, so the view follows the drone, point clouds sit where they are in the world, and a drone without sensors still shows where it is. TF and the sim clock go on `/sensor_view_ID/tf` and `/sensor_view_ID/clock`, which only that RViz is remapped to, so several views can run at once even though every drone has a `base_link`. The bridge only runs while RViz is open. RViz needs a GUI: the screen with `start-stellar.sh` (default), the VNC desktop with `start-dev.sh` or `--vnc`, none with `--headless`. All `depth` drones share one depth topic in Gazebo (`/depth_camera`), so view only one of them at a time.
In the shell, `gimbal(id, pitch, yaw)` points a gimbal drone's camera (`spawn gimbal`, or `model: gz_x500_gimbal` in `swarm.yaml`): degrees, pitch 0 level and -90 straight down, yaw relative to the nose; `gimbal(id)` prints where it points. It sends PX4 `DO_GIMBAL_MANAGER_CONFIGURE` (take control) and `DO_GIMBAL_MANAGER_PITCHYAW`, and reads `gimbal_device_attitude_status` back. The read-back yaw is compared with PX4's heading estimate, which can be a few degrees off the simulated truth. Other camera drones have fixed cameras that turn with the drone.

Ctrl-C or a timeout in any of these lands the drones.

### Intercept: pure pursuit and MPC

A defender flies into the hostile drone on what its own camera reports: the two detection topics `/<ns>/detection/target_position` and `target_offset` (`intercept/detection.py`), from the real detector (`detect ID`) or the fake one (`fakedetect`). There are two guidance laws behind one interface, chosen with `--guidance pursuit|mpc` (the attack node, the patrols, the scenarios; `--setup` in the scenarios picks one too):

| Module (`swarm_backend/intercept/`) | Role |
|---|---|
| `attack.py` | `AttackAction`, the baseline: pure pursuit at `attack.max_speed` toward the target, SEARCH / ATTACK / COAST / LOST. `limit_velocity` (speed, climb, minimum height) is shared with the MPC. |
| `mpc.py` | `Mpc`: the receding-horizon calculator (closed-form least squares, one acceleration per step). |
| `mpc_action.py` | `MpcIntercept`: Kalman filter on the target (`TargetTracker3D`), full speed to the predicted meeting point, the MPC in the last `terminal_s` seconds. An `InterceptAction`, so the evaluation harness can score it. |
| `mpc_attack.py` | `MpcAttack`: the MPC behind `AttackAction`'s face, and `make_attack(guidance, ...)`, the one place that picks pursuit or MPC. |
| `inbox_detector.py` | The detection topics as a `Detector`, for the MPC. |
| `target.py` | `TargetSource` and `PositionFix`. |
| `guidance.py` | Gimbal angles toward a target (`tests/detection_test.py`). |

The hostile drone's flight patterns and `HostileFlight`, which flies a PX4 drone along one (the scenarios' hostile), are in `evaluation/paths.py` and `evaluation/mock.py`.

```bash
./mount-backend.sh attack [--guidance mpc]              # one defender, drone 2
./mount-backend.sh scenario attack --sim                # the whole chain, YOLO + ByteTrack + MPC
./mount-backend.sh scenario attack --sim --guidance pursuit
```

Timing uses each PX4's own clock (`Drone.stamp`, simulation time in SITL), so speeds and rates stay right when Gazebo runs slower than real time.

### Intercept baseline evaluation

The scenarios and metrics of the planning report (Interception > Evaluation). A hostile drone flies straight and level toward the base at the origin. The run starts at first detection: the target is `detect_range` from an interceptor on the `perimeter_radius` ring, and its line of sight makes the *aspect* with its track (0° head-on, 30/60/90° crossing). Each aspect × speed (4–17 m/s) is run with `eval_runs` seeds. A run ends with a **hit** (within 1.5 m, found along the segment between samples, not only at the samples), the **target reaching the base**, the **target lost**, or a **timeout**. Each run records hit and success (a hit more than `goal_radius` = x from the base), time to hit, lock-on (k of n images, within d_lock, held t_lock), distance to base at hit, the target's closest distance to base, and the closest miss. All of it is computed from true positions. The settings are the `hit_radius` … `eval_timeout` keys in `intercept:`.

The harness never sees how a method works. A method plugs in as two classes (`intercept/`), and the harness is in `evaluation/`:

| Module (`swarm_backend/`) | Role |
|---|---|
| `intercept/interceptor.py` | `InterceptAction`: `start()`, `stop()`, `step()`, `status` (`Status.TRACKING`/`LOST`/`IDLE`), built from `(drone, detector, config, clock)`. The MPC (`mpc_action.MpcIntercept`) implements it. |
| `intercept/target_detector.py` | `Detector`: a `TargetSource` with `update(now)`, `in_view` and `detected`, the last two for lock-on. `inbox_detector.InboxDetector` implements it from the detection topics. |
| `evaluation/scenario.py` | `make_scenario`, `scenario_grid`, and `run_scenario(scenario, sim, make_action, make_detector, config)`. `sim` is anything with `hostile`, `interceptor`, `now()`, `advance(dt)`. |
| `evaluation/metrics.py` | `score_run(samples, scenario, config) -> RunMetrics`, `summarize(results)` (the table), `LockOnTracker`, `closest_approach` / `first_within`. |

```bash
python3 -m pytest tests/test_intercept_metrics.py           # the harness itself (test doubles, no guidance)
```

On the surveillance arc the same seeds and metrics are flown by `scenario baseline_eval` (`evaluation/arc_eval.py`).

### Detection evaluation

The same pattern for the report's detection section, in `swarm_backend/detect/`. Defenders hold a static ring (`num_defenders` at `perimeter_radius`) and search with their gimbal cameras. A hostile drone and birds come from random directions, and a run ends when the hostile drone reaches the ring. Every image is a `Frame` (pixels, camera pose, and the truth: what projects into the image, where and how many pixels wide). A detection is correct when its centre is within `match_radius_px` of the true centre. Settings are in `detect:` in `swarm.yaml`.

| Module (`swarm_backend/detect/`) | Role |
|---|---|
| `frame.py` | The truth per image: `Frame` (image, camera pose as forward/right/down like `ground_truth.camera_pose`, true objects), `TrueObject`, `aimed_camera`, `make_frame` (projects with `ground_truth.project` and `camera.CameraModel`). |
| `detector.py` | `ImageDetector` (`reset()`, `detect(frame) -> [Detection]`, the report's detection report). `DarkObjectDetector`, `Yolo26Detector` and the fast-sim stand-in `PixelModelDetector` are **placeholders**. |
| `scan.py` | `GimbalScan.aim(t, drone, position)`. `SectorScan` (the static ring's outward sweep) is a **placeholder**. |
| `scenario.py` | `make_scenario`, `scenario_grid`, `run_scenario(scenario, sim, make_detector, make_scan, config)`. `sim` has `now()`, `advance(dt)`, `bodies()`, `frame(drone, pitch, yaw)`. |
| `metrics.py` | `match`, `score_run` (missed, warning, range views, birds flagged, background alarms), `detection_by_range`, `summarize`. |
| `dataset.py` | Labelled images: `split_by_run`, `evaluate_dataset` (recall vs size, background alarms, inference time), `summarize_dataset`. `GazeboImageDataset` is a **placeholder**. |

```bash
python3 -m pytest tests/test_detect_metrics.py         # the harness itself (test doubles)
python3 -m pytest tests/test_detect_baseline.py -s     # sweep + tables; skipped until the baseline exists
HIVE_DETECT_DATASET=/path/to/images python3 -m pytest tests/test_detect_baseline.py -s -k recall
```

The default sweep (30 runs, 12 defenders) takes under a minute.

How tests and evaluations are written, and what a run should save: `hive-vault/Testing.md`.

### Operator UI

A web page for the operator: a full-window top-down map of the base, the search area, every drone in swarm.yaml with its gimbal camera's view and the last 10 s of where the gimbal pointed (so a search sweep is visible), and detections, with glass panels over it. On the left: counts (airborne, hostile, engaged, no link), the search area, and a card per drone (armed, mode, altitude, speed, gimbal heading, time since PX4 last spoke, a gimbal compass and a 24 s sparkline; a drone in an attack moves to the top and shows range, closing speed, time to impact). On the right: the **event log**, which the UI node builds by comparing each drone with how it was 0.2 s earlier, so no other node sends anything: link up / lost / back, armed, taking off, landing, landed, target seen / lost from sight, attack state changes with range, hits with the time since first sighting, and search area changes. It keeps the last 500; the page gets the new ones with `/api/state?events_after=<id>`. A drone running the attack node, or a defender running the patrol (`patrol`, and the `attacktest`/`baseline-eval` scenarios in Gazebo), also gets its attack state (IDLE, SEARCH, ATTACK, COAST, LOST, HIT) on its card and next to it on the map, from `/<ns>/attack/state` (`std_msgs/String`). Both publish it (`nodes/attack_node.py` `AttackReporter`) on every change and once a second; the UI hides it after 3 s without a message.

```bash
./mount-backend.sh ui        # then open http://localhost:8090 on the host
```

The drones are autonomous, so the UI never commands them. It only listens to their PX4 and detection topics. The one thing the operator sets is the **search area**, an arc around the base (heading, width, radius; default the quarter circle toward `attack.search_heading_deg`, 150 m). Drag it on the map or use the fields, then press Apply. It is published on `/base/search_area` (`std_msgs/String`, JSON: `heading_deg`, `width_deg`, `radius_m`, `centre` [north, east], `revision`, latched), for the surveillance planner or base station to follow. Nothing listens to it yet.

Without the simulation (on a laptop, no ROS): `uv run --with pyyaml python tests/ui_preview.py` (or `pip install pyyaml` and `python3 tests/ui_preview.py`) puts the defenders on the surveillance arc of `scenarios/surveillance.yaml` (as in the eval scenarios), each flown by the real patrol (sweeping its sector; the first to see the hostile turns the real attack on it), flies hostiles at the base, and serves the same page on http://localhost:8090. The page is read on every request, so a reload shows edits to `index.html`.

The container publishes port 8090 to `127.0.0.1` (`HOST_UI_PORT` changes the host side). A container started before this change has no published port and must be restarted.

| Module | Role |
|---|---|
| `ui/model.py` | `SearchArea` and its checks, what the page shows per drone, and the `EventLog`. No ROS. |
| `nodes/operator_ui.py` | Subscribes to every drone, publishes the search area, serves the page and `/api/state`. |
| `ui/static/index.html` | The page: one file, no build step, polls `/api/state` 5 times a second. |

Unit tests for the parts that do not need ROS (config, topics, formations, swarm logic, pursuit, MPC, scenarios): `python3 -m pytest tests/test_*.py`.
