# PX4 / Gazebo / ROS 2 dev container

## Starting local dev on macOS (`start-dev.sh`)

`start-dev.sh` only works on macOS (it opens Terminal.app with AppleScript). On the Stellar (Ubuntu) computers use `start-stellar.sh`, see below.

```bash
./start-dev.sh                        # default drone (gz_x500) in baylands
./start-dev.sh -m gz_x500_depth        # pick a different sensor loadout
./start-dev.sh --model gz_x500_lidar_front
./start-dev.sh -w forest               # pick a different world
./start-dev.sh -m gz_x500_depth -w ridge
./start-dev.sh -h                      # show usage
```

The `-m`/`--model` flag sets which drone variant gets spawned — any value from the `PX4_SIM_MODEL` table below. Defaults to `gz_x500` (no extra sensors) if omitted.

The `-w`/`--world` flag sets which Gazebo world gets loaded — any value from the world table below. **Defaults to `baylands`** if omitted.

**Available worlds** (all bundled in the image already, no download needed):

| `-w` value                      | Description                                                                                                                                                                             |
| ------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `baylands` (default)            | Terrain + water, more realistic environment. Takes noticeably longer to load than the others — the "Waiting for Gazebo world..." line can repeat for 30–60s, that's normal, not a hang. |
| `default`                       | Flat empty ground plane                                                                                                                                                                 |
| `forest`                        | Trees / vegetation                                                                                                                                                                      |
| `ridge`                         | Hilly / elevated terrain                                                                                                                                                                |
| `walls`                         | Enclosed walled area                                                                                                                                                                    |
| `lawn`                          | Grass field                                                                                                                                                                             |
| `windy`                         | Wind-affected environment                                                                                                                                                               |
| `aruco`                         | Contains ArUco markers (for vision/tag detection)                                                                                                                                       |
| `frictionless`                  | Frictionless ground (rover/testing edge cases)                                                                                                                                          |
| `underwater`                    | Underwater environment                                                                                                                                                                  |
| `rover`                         | Rover-oriented terrain                                                                                                                                                                  |
| `kthspacelab`, `kth_marinarium` | KTH-specific scenario worlds                                                                                                                                                            |
| `moving_platform`               | Includes a moving landing platform                                                                                                                                                      |
| `hive_base`                     | Ours (`worlds/`): the field camp in a 240 m forest. Slow to load (thousands of separate trees and parts); raise `WORLD_TIMEOUT`.                                                        |
| `hive_valley`                   | Ours (`worlds/`): 4 km valley with hills, a river, a bridge and the camp at the origin; trees thin out from the centre. Baked into a few meshes, so it loads fast. Run `python3 worlds/generate_hive_valley.py` once first (~75 s). |

**What the script actually starts:**

1. Builds the `hive-px4-vnc` image from `docker/Dockerfile` (the official `px4io/px4-sitl-gazebo-ros2` base image, plus `Xvfb` + `fluxbox` + `x11vnc` + `noVNC`/`websockify` for the GUI layer). The Dockerfile also sets PX4's `NAV_DLL_ACT` to 0 in the x500 airframe (simulation only), so the drone can be armed by the backend without QGroundControl connected.
2. Runs that image as a container named `px4-gazebo-vnc`, publishing:
   - port `6080` → noVNC in the browser (`http://localhost:6080/vnc.html`)
   - port `5900` → native VNC client (`vnc://127.0.0.1:5900`)
   - both bound to `127.0.0.1` only, and protected by a **random password generated on every run** (printed by the script, nothing hardcoded in the repo)
   - the repo root mounted at `/workspace` inside the container
   - the container's default command starts the DDS agent (`MicroXRCEAgent udp4 -p 8888`), bridging PX4 ↔ ROS 2
3. Opens a **second terminal window** automatically, which waits for the container to be running, then starts PX4 + Gazebo (drone instance `0`, GUI mode — not headless) with the chosen `PX4_SIM_MODEL`.
4. Connect QGroundControl separately (it auto-detects the running PX4 instance on macOS via Docker Desktop's gateway IP) — see `hive-vault/StackV2.md`, section 5.2.1.

Stop everything with `Ctrl+C` in the first terminal (the container runs with `--rm`, so it's removed on exit).

## Dev GUI (`dev_gui.py`): everything from one browser page

Instead of several terminals, start the container, run backend commands and scenarios, and read their logs from one page:

```bash
python3 docker/dev_gui.py            # prints http://127.0.0.1:8095/?token=...
python3 docker/dev_gui.py --port 8096 # if another user already holds 8095
```

Open the printed URL **on this computer**. The page:

- **Friendly and hostile drones pass through each other** in Gazebo, and both still hit the ground, trees and camp. A hit is scored from true positions at 1.5 m, so physical contact only made the defender crash. `docker/drone-models.sh` runs before every spawn: it puts the stock x500 and gimbal collisions in one collision group, and generates `x500_hostile` (the same x500, in another group) for the hostile, `gz_x500_hostile` in `scenarios/surveillance.yaml`. Defenders still collide with each other. Details in `docker/drone_collisions.py`.
- **Crashes:** a defender that crashes is no longer teleported and respawned. Its run is saved with the crash, then Gazebo is restarted with every drone fresh, and the batch goes on with the next unsaved run. A crash on the last run also restarts Gazebo, so no broken drone is left for whatever runs next. This applies to the quick starts and single scenarios, and to `scenarios/baseline_eval.sh` and `compare_paths.sh` from a terminal. It needs a session started with no drone 0, which is the GUI's default. With **Start drone 0**, drone 0's PX4 is the container itself, so crashed defenders are respawned as before.
- **Quick start:** one click for the 36-case comparison (4 paths x 3 altitudes x 3 seeds, about 3 hours in Gazebo):
  - **36 runs: baseline** runs `scenarios/compare_paths.sh 3 --setup baseline`.
  - **36 runs: YOLO + ByteTrack + MPC** runs `scenarios/compare_paths.sh 3 --setup yolo-bytetrack-mpc`. It needs the YOLO weights, so it waits for the download.

  Tick **--sim** for the dry run first: the fast simulation takes minutes and needs no Gazebo or weights. The page asks before starting a 3-hour run, and again if the machine is booked by someone else. It allows only one scenario at a time. Results go to `data/path_compare/<date-time>_<setup>/`, as in `ros2_ws/src/swarm_backend/scenarios/README.md`. More quick starts can be added to `QUICK_STARTS` in `dev_gui.py`.
- **Single scenario:** fly one hostile path at a time: straight, weave ±5 m, weave ±15 m or zigzag ±15 m. Choose:
  - the hostile's altitude: level, below, above, or all three
  - the number of runs (seeds 0..N-1)
  - the setup: baseline, or YOLO + ByteTrack + MPC

  **RViz opens for drone 2**, the middle defender the hostile flies at, once the run has spawned it. It shows its camera and the detector's boxes, and stays open through the restarts. It is reopened if it crashes, but not if you close it.

  **Gazebo is restarted before every run**, with every drone fresh. The operator UI and RViz views keep running through it. A restart takes seconds for the `default` world and longer for `hive_base`, plus about a minute for the drones to connect.

  By default the defenders **start on the ground right below their slots** on the arc, so they only climb straight up instead of flying 150 m out from the camp first. It runs `scenarios/compare_paths.sh N --paths PATH --setup SETUP [--altitude-offsets X] --spawn-at-slots`. Results go to `data/path_compare/<date-time>_<setup>_<path>/`, laid out like a 36-run comparison, so the two can be compared.
- **Container:** pick the world, GUI mode (native / VNC / none) and GPU, then press Start. **Gazebo runs headless by default** (`--no-gazebo-gui`): no Gazebo window, which is faster, and the cameras still render. RViz and the operator UI still use the display. Tick **Gazebo window** to get it back. It runs `start-stellar.sh --no-drone` with those flags: Gazebo starts with **no drones**. Spawn them with the backend's `spawn`, or let a scenario (like the quick starts) start the ones it needs. Tick **Start drone 0** for the old behaviour, where drone 0 starts with the world and its `pxh>` prompt shows up in the page. The swarm shell and the `single`/`multi` tests use drone 0 from `config/swarm.yaml`. In a no-drone session, `mount-backend.sh restart` keeps it that way. Before starting, it shows the Stellar booking, who else is on the GPU, and free disk. If the machine is booked by someone else, or another user is on the GPU, it asks you to confirm first. Status lights show the container, Gazebo and the running drone ids, plus a noVNC link (with the VNC password).
- **Views:**
  - **Operator UI** starts `mount-backend.sh ui` if it isn't running, and opens `http://localhost:8090` once it answers. The first start builds the backend, so it can take a while.
  - **RViz N:** one button per drone that has a camera in Gazebo. It runs `mount-backend.sh view N`, and RViz opens on the GUI display (this screen with native GUI, the VNC desktop with VNC). The camera drones are found from Gazebo's topics every few seconds, so newly spawned ones show up by themselves.
- **Backend:** runs `mount-backend.sh CMD ARGS` for a fixed list of commands (`spawn`, `restart`, `ui`, `shell`, `view`, `explorenode`, …). `explorenode` maps the area sent from the operator UI's Map area tab with DAEP.
- **Scenarios:** runs `mount-backend.sh scenario NAME [--sim]` or one of the `scenarios/*.sh` launchers.
- **YOLO weights:** at startup it downloads `models/yolo26n-gazebo-only/yolo26n-gazebo-only-1280.onnx` from the repo's GitHub Releases if it is missing, in the background. A light shows when it's there; `--no-weights` skips this. Only that model is fetched, because the detector uses the newest `.onnx` in `models/` and a second download would change which one runs. The list is `WEIGHTS` in `dev_gui.py`.
- **Jobs:** each run gets a tab with its live log and an input line, so you can type at `pxh>`, in the swarm shell or at a prompt. **Stop** sends Ctrl-C first (so drones land) and only kills the job if it doesn't end.

It needs only Python 3 (standard library) on the host. It binds to `127.0.0.1` only, and every API call needs the random token in the URL, because other users on a shared computer can reach `127.0.0.1` too. Jobs run as children of the GUI, so **Ctrl-C on `dev_gui.py` stops every job, including the container.** On Stellar, run it inside tmux: `tmux new -s devgui 'python3 docker/dev_gui.py'`. On macOS it uses `start-stellar.sh --vnc` or `--headless`, without the GPU.

Tests: `python3 -m pytest docker/test_dev_gui.py`

## Running on Stellar (LiU lab computers)

Use `start-stellar.sh` there (one terminal, no AppleScript). The default is just `./start-stellar.sh`, which means `--native-gui -w hive_base`. The full steps, the rules that affect us, and how to clean up after a session are in [README-stellar.md](README-stellar.md).

---

# Drone sensors (PX4 SITL / Gazebo)

Verified by inspecting the model files directly inside the `px4io/px4-sitl-gazebo-ros2` image (`/opt/px4-gazebo/share/gz/models/`).

## Default: every drone always has this

No matter which `PX4_SIM_MODEL` variant we pick, they all build on the base model `x500_base`, which always includes:

1. Every drone (base `x500`) already has, regardless of variant — confirmed from `x500_base/model.sdf`:
   - IMU (`imu_sensor`)
   - Magnetometer (`magnetometer_sensor`)
   - Barometer (`air_pressure_sensor`)
   - GPS (`navsat_sensor`)

No camera, LiDAR, or radar is included in the base model.

## Extra sensors: pick via `PX4_SIM_MODEL`

We switch the sensor loadout by setting this environment variable before starting `px4-gazebo`, e.g:

```bash
export PX4_SIM_MODEL=gz_x500_depth
```

| `PX4_SIM_MODEL` value | Extra sensor(s) on top of the default                             |
| --------------------- | ----------------------------------------------------------------- |
| `gz_x500`             | None — just IMU/magnetometer/barometer/GPS                        |
| `gz_x500_mono_cam`    | Regular camera (`mono_cam`)                                       |
| `gz_x500_depth`       | RGB camera + depth camera (OAK-D Lite: `camera` + `depth_camera`) |
| `gz_x500_lidar_2d`    | 2D LiDAR                                                          |
| `gz_x500_lidar_front` | 3D LiDAR, forward-facing (`gpu_lidar`)                            |
| `gz_x500_lidar_down`  | 3D LiDAR, downward-facing (`gpu_lidar`)                           |
| `gz_x500_flow`        | Optical flow sensor + LiDAR (altitude hold without GPS)           |
| `gz_x500_vision`      | No extra sensor — for external vision/mocap input instead         |
| `gz_x500_gimbal`      | Gimbal mount (pair with a camera model)                           |

**Radar doesn't exist** as a built-in sensor in PX4/Gazebo. For Counter-UAS detection, we'll either need to fake it with a scripted "detection radius" node, or build a custom Gazebo sensor plugin.

## Example: starting a drone with a depth camera

If the container is running via `./start-dev.sh` (container name `px4-gazebo-vnc`), the simplest way to get a depth-camera drone as drone 0 is to just pass the model to the script itself:

```bash
./start-dev.sh -m gz_x500_depth
```

## Example: adding a second drone with a depth camera

To add an _extra_ drone with a depth camera alongside whatever `start-dev.sh` already started (drone 0), run this in a new terminal — note the different container name, instance number, and spawn pose so it doesn't collide with drone 0:

```bash
docker exec -it px4-gazebo-vnc bash -c "
  export PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE='5,0,0,0,0,0'
  /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 1
"
```

It'll spawn into the same running Gazebo world and show up namespaced as `/px4_1/fmu/...` (see `StackV2.md`, section 5.2.2 for how multi-drone namespacing works). Bump `-i` and the pose further for a third, fourth, etc. drone.

Sensor data from camera/LiDAR variants gets bridged to ROS 2 via `ros_gz` (see StackV2.md, section 3) — check `ros2 topic list` after startup to see which topics are actually published for the chosen model.
