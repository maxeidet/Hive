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
./mount-backend.sh spawn camera # ONE more drone with a camera, at the next free id
./mount-backend.sh view 1       # drone 1 in RViz with its cameras and lidars (`lidar 1` / `camera 1` too)
./mount-backend.sh restart [N]  # fresh Gazebo + drone 0 (and 1..N-1) without restarting the container
```

Each command rebuilds the package first and reads `config/swarm.yaml` from this folder.

Drone models come from the `model:` field in `swarm.yaml` (drone 1 is `gz_x500_lidar_3d`: 16 beams, 360°, 100 m, 10 Hz, defined in `worlds/models/`).

`spawn TYPE` starts one extra drone instead: `plain`, `lidar` (the 3D lidar above), `lidar2d`, `lidar_front` / `lidar_down` (1-beam rangefinders), `camera`, `camera_down`, `depth` (RGB + depth camera), `flow`, `gimbal`, or any `PX4_SIM_MODEL` such as `gz_x500_depth`. It takes the lowest instance id that is not running and that id's `spawn_pose` from `swarm.yaml`, so the shell can fly it; ids beyond `swarm.yaml` get `(4*id, -5, 0)` and are not flown by the shell.

`view ID` (`swarm_backend/sensor_view.py`) works for every drone type. It finds the drone's sensors in Gazebo, bridges them to ROS 2 as `/px4_ID/<sensor>/image` (`sensor_msgs/Image`) and `/px4_ID/<sensor>/points` (`sensor_msgs/PointCloud2`, frame = the sensor's link), e.g. `/px4_1/lidar_3d/points`, and opens RViz from `config/sensors.rviz` with one display per sensor. It also publishes TF from Gazebo's poses, `world` (Gazebo ENU) -> drone model (e.g. `x500_gimbal_2`) -> `base_link` and the sensor links, so the view follows the drone, point clouds sit where they are in the world, and a drone without sensors still shows where it is. TF and the sim clock go on `/sensor_view_ID/tf` and `/sensor_view_ID/clock`, which only that RViz is remapped to, so several views can run at once even though every drone has a `base_link`. The bridge only runs while RViz is open. RViz needs a GUI: the screen with `start-stellar.sh` (default), the VNC desktop with `start-dev.sh` or `--vnc`, none with `--headless`. All `depth` drones share one depth topic in Gazebo (`/depth_camera`), so view only one of them at a time.
In the shell, `gimbal(id, pitch, yaw)` points a gimbal drone's camera (`spawn gimbal`, or `model: gz_x500_gimbal` in `swarm.yaml`): degrees, pitch 0 level and -90 straight down, yaw relative to the nose; `gimbal(id)` prints where it points. It sends PX4 `DO_GIMBAL_MANAGER_CONFIGURE` (take control) and `DO_GIMBAL_MANAGER_PITCHYAW`, and reads `gimbal_device_attitude_status` back. The read-back yaw is compared with PX4's heading estimate, which can be a few degrees off the simulated truth. Other camera drones have fixed cameras that turn with the drone.

Ctrl-C or a timeout in any of these lands the drones.

### Intercept (cat and mouse)

One drone chases a hostile drone using only what the base station reports. For now that report comes from a **mock radar** that measures the hostile drone's real position the way a radar does, at 2 Hz: range (σ 3.5 m), azimuth (σ 1°, so ±35 m across the line of sight at 2 km), elevation (σ 2°, unused while the altitude is fixed) and radial velocity (σ 0.5 m/s). `radar_position` places the radar; the default puts it virtually 2 km south, so the errors are realistic while the drones fly near the origin. The hostile drone is a real PX4 drone in the same Gazebo world, flying a pattern, so every run is scored against where it really was. Both fly at a fixed 50 m. Settings are in the `intercept:` section of `swarm.yaml`. By default drone 2 (`gz_x500_gimbal`) chases drone 0.

```bash
./mount-backend.sh spawn                         # drones 1 and 2 (2 is the gimbal interceptor)
./mount-backend.sh intercept                     # chase test: circle, 120 s, prints a summary
./mount-backend.sh intercept --pattern figure8 --speed 5 --csv /tmp/run.csv
./mount-backend.sh view 2                        # the interceptor's gimbal camera in RViz
```

In the shell: `takeoff(50)`, `hostile()` (or `hostile(pattern='random')`), `chase()`, `status()`, `stop_chase()`, `land()`.

| Module (`swarm_backend/intercept/`) | Role |
|---|---|
| `paths.py` | Hostile patterns, flown at constant speed: `line` (a racetrack), `circle`, `figure8`, `random` (seeded waypoints, turns no tighter than 8 m). |
| `mock.py` | `HostileFlight` flies a drone along a pattern (position setpoint + velocity feed-forward); `radar_fix` is what the radar measures. |
| `radar.py` | `MockRadar` publishes `radar_msgs/RadarScan` (range, azimuth, elevation, doppler) on `/base/radar/hostile`, frame `base_radar_ned` (azimuth clockwise from north); `RadarTarget` receives it. Standalone: `ros2 run swarm_backend mock_radar`. |
| `target.py` | `TargetSource` and the fix types: `RadarFix` (polar, with its sigmas) and `PositionFix` (for a drone's own depth camera / lidar later). The chase uses the first source in its list with a fresh fix. |
| `tracker.py` | Kalman filter (x, y, vx, vy). Radar returns go in as measured (an EKF on range, azimuth, radial velocity), so it knows the error is a thin ellipse across the radar's line of sight. It reports its own uncertainty. |
| `guidance.py` | Pure math: lead point, the shadow point, braking approach, phases, gimbal angles. |
| `chase.py` | `ChaseAction`: one `step()` per setpoint tick. |

How the chase flies:
- **INTERCEPT**: far out, it heads for the spot behind where the target will be when it gets there (the lead point). Within 3 × standoff it works round the target itself. It closes no faster than it can brake (2 m/s²), on top of the target's velocity.
- **Standoff** is `standoff` (10 m), or `standoff_sigmas` (3) × the tracker's uncertainty if that is more. With the radar alone at 2 km that is about 45 m. Close following needs a sensor on the drone; the radar cannot keep a same-altitude chase safe at 10 m.
- **SHADOW**: once within standoff + capture_margin, it holds the standoff behind the target. The setpoint swings at most 45° round the target per step, so when the target turns back it circles at standoff instead of cutting across. It never aims closer than `min_separation` to the estimate.
- **LOST**: no fix for `track_timeout` s, so it hovers.
- The nose turns toward the target. With a gimbal, the camera is kept on the target (`Drone.aim_gimbal`, streamed at 5 Hz). Pitch is commanded relative to the horizon, so PX4's gimbal manager removes the drone's own tilt while it accelerates. A fixed camera would point at the ground then.

Timing uses each PX4's own clock (`Drone.stamp`, simulation time in SITL), so speeds and rates stay right when Gazebo runs slower than real time. The flight test aborts and flies both drones home (at different heights) if they get within 3 m of each other.

### Intercept baseline evaluation

The scenarios and metrics of the planning report (Interception > Evaluation). A hostile drone flies straight and level toward the base at the origin. The run starts at first detection: the target is `detect_range` from an interceptor on the `perimeter_radius` ring, and its line of sight makes the *aspect* with its track (0° head-on, 30/60/90° crossing). Each aspect × speed (4–17 m/s) is run with `eval_runs` seeds. A run ends with a **hit** (within 1.5 m, found along the segment between samples, not only at the samples), the **target reaching the base**, the **target lost**, or a **timeout**. Each run records hit and success (a hit more than `goal_radius` = x from the base), time to hit, lock-on (k of n images, within d_lock, held t_lock), distance to base at hit, the target's closest distance to base, and the closest miss. All of it is computed from true positions. The settings are the `hit_radius` … `eval_timeout` keys in `intercept:`.

The harness never sees how a method works. A method plugs in as two classes:

| Module (`swarm_backend/intercept/`) | Role |
|---|---|
| `interceptor.py` | `InterceptAction`: `start()`, `stop()`, `step()`, `status` (`Status.TRACKING`/`LOST`/`IDLE`), built from `(drone, detector, config, clock)`. `PurePursuitAction` is the baseline, **a placeholder for now**, meant to wrap the attack's `attack.AttackAction`. |
| `target_detector.py` | `Detector`: a `TargetSource` with `update(now)`, `in_view` and `detected`, the last two for lock-on. `NoisyTruthDetector` (true position + noise, inside the camera's view and range) stands in for YOLO, **a placeholder for now**, meant to wrap the attack's fake detector (`detection.measure`). |
| `scenario.py` | `make_scenario`, `scenario_grid`, and `run_scenario(scenario, sim, make_action, make_detector, config)`. `sim` is anything with `hostile`, `interceptor`, `now()`, `advance(dt)`. |
| `metrics.py` | `score_run(samples, scenario, config) -> RunMetrics`, `summarize(results)` (the table), `LockOnTracker`, `closest_approach` / `first_within`. |

```bash
python3 -m pytest tests/test_intercept_metrics.py           # the harness itself (test doubles, no guidance)
python3 -m pytest tests/test_intercept_baseline.py -s       # full sweep + table; skipped until the baseline exists
./mount-backend.sh baseline-intercept --aspect 90 --speed 8 --seed 3   # one scenario in Gazebo
```

To evaluate another method or detector, pass its classes to `run_scenario` in place of `PurePursuitAction` / `noisy_truth` (see `tests/test_intercept_baseline.py`). The fast sim (`swarm_backend/fast_sim.py`) runs the full 600-run grid in about 20 s.

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

Unit tests for the parts that do not need ROS (config, topics, formations, swarm logic, intercept incl. a closed-loop chase sim): `python3 -m pytest tests/test_*.py`.
