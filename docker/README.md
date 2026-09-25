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

**What the script actually starts:**

1. Builds the `hive-px4-vnc` image from `docker/Dockerfile` (the official `px4io/px4-sitl-gazebo-ros2` base image, plus `Xvfb` + `fluxbox` + `x11vnc` + `noVNC`/`websockify` for the GUI layer).
2. Runs that image as a container named `px4-gazebo-vnc`, publishing:
   - port `6080` → noVNC in the browser (`http://localhost:6080/vnc.html`)
   - port `5900` → native VNC client (`vnc://127.0.0.1:5900`)
   - both bound to `127.0.0.1` only, and protected by a **random password generated on every run** (printed by the script, nothing hardcoded in the repo)
   - the repo root mounted at `/workspace` inside the container
   - the container's default command starts the DDS agent (`MicroXRCEAgent udp4 -p 8888`), bridging PX4 ↔ ROS 2
3. Opens a **second terminal window** automatically, which waits for the container to be running, then starts PX4 + Gazebo (drone instance `0`, GUI mode — not headless) with the chosen `PX4_SIM_MODEL`.
4. Connect QGroundControl separately (it auto-detects the running PX4 instance on macOS via Docker Desktop's gateway IP) — see `hive-vault/StackV2.md`, section 5.2.1.

Stop everything with `Ctrl+C` in the first terminal (the container runs with `--rm`, so it's removed on exit).

## Running on Stellar (LiU lab computers)

### Starting on Stellar: `start-stellar.sh`

Runs in one terminal, no second window:

```bash
cd docker
./start-stellar.sh                        # default drone, baylands world, GUI via VNC
./start-stellar.sh -m gz_x500_depth -w hive_base
./start-stellar.sh --headless             # no GUI, no ports published
./start-stellar.sh --native-gui           # Gazebo window on this computer's own screen (no VNC)
./start-stellar.sh --gpu                  # also give the container the GPU(s)
./start-stellar.sh --native-gui --gpu     # GUI rendered on the GPU
./start-stellar.sh -h
```

It checks that we're on Linux and that the Docker daemon is reachable, builds the image, starts the container in the background (DDS agent, repo mounted at `/workspace`), then runs PX4 + Gazebo in *this* terminal (the `pxh>` shell). Run the backend in a second terminal with `./ros2_ws/src/swarm_backend/mount-backend.sh`. Stop with Ctrl+C or `shutdown` at `pxh>`; the container is removed automatically. Agent logs: `docker logs -f px4-gazebo-vnc`.

**Using the GPU** is two separate things:

- `--gpu` passes `--device nvidia.com/gpu=all` (all GPUs in that computer) and prints `nvidia-smi` first so we can check that nobody else is using it. It then runs `nvidia-smi -L` inside the container to show the GPU is visible.
- Passing the GPU in does not by itself make Gazebo use it. The VNC display (Xvfb) is software-rendered. Gazebo only renders on the GPU with `--native-gui`, which uses the computer's own X display (we are always at the machine, since we have no remote access). To check, run `nvidia-smi` on the host while the GUI is up: `gz` should be listed as a graphics (G) process. If it isn't, rendering is still software.
- Camera/lidar sensors also render inside the simulation, using whatever display the container has, so they only benefit from the GPU in `--native-gui` mode. For ML training the image has no PyTorch/CUDA yet; that is a later step (Tier 1).

All of the GPU and `--native-gui` parts are untested on Stellar (only the flag handling and the container entrypoint were tested, on a Mac).

### The rules, and what they mean for us

Based on the [Stellar rules](https://gitlab.liu.se/stellar-support/rules) (last read 2026-09-25 — they change, and keeping up is our responsibility, so re-read them). What they mean for us:

- **No root, no sudo, nothing to install.** Every Stellar desktop gives each user their own rootless Docker daemon (Stellar guide: rootless Docker). First time on a computer: log in and run `docker run --rm hello-world`. **Do not run `dockerd-rootless-setuptool.sh`** (that instruction is out of date, it's done for us). If the daemon isn't running: `systemctl --user start docker`, or log out and in.
- **Each computer has its own daemon and its own images**, so we build on the computer we use (`./start-stellar.sh` does that). Images live in `~/.local/share/docker` on the disk shared with everyone on that computer: check with `docker system df`, clean with `docker system prune`.
- **Nothing reachable from outside the university network.** No reverse SSH tunnels, ngrok, Tailscale. Our scripts bind ports to `127.0.0.1` only. Never change that to `0.0.0.0`.
- **Shared, multi-user machines.** That's why the VNC password is random per run (a fixed one in the repo would let other users take over our session) and why ports can be overridden if someone else already holds them:
  ```bash
  HOST_NOVNC_PORT=6081 HOST_VNC_PORT=5901 ./start-stellar.sh
  ```
- **Course students have no remote access.** We sit at the machine in the room and open `http://localhost:6080/vnc.html` in a browser on that same computer. SSH (`ssh -J`, port forwarding) is only for people stellar-support has granted remote access.
- **Course students should use the rooms only when they need a GPU**, when resources are free, and with respect for research and AI Academy projects. Normally one computer per group; leave at once when someone needs it; no reservation notes. So we go to Stellar when we actually need a GPU (heavy rendering, RL training) and do everything else on our own computers.
- **Use the computer our work needs, not the best free one.** A PX4 sim is CPU-heavy but does not need a top GPU. Check `computers.md` in the rules repo before picking. Don't hog.
- **Clean up.** Images are several GB and disks are shared and get cleared when full: `docker system prune`, `ncdu`, and remove what we don't use when a phase ends.
- **When done: `Ctrl+C` the container, then log out.** Never leave containers running.
- **Storage is not backed up and stays on that machine.** Everything that matters goes to Git (push before leaving). Don't keep the only copy of anything on Stellar.

**Not tested on Stellar yet** (only verified on macOS + Docker Desktop):

- Whether the machine can pull from Docker Hub and clone from GitHub (`docker run --rm hello-world` tests the first, `git clone` the second) and so whether `docker build` of our image works there.
- **QGroundControl:** the image sends MAVLink to `192.168.65.254`, which is Docker Desktop for macOS's host gateway. That address does not exist on Linux, so QGC on the Stellar machine will not auto-connect. It also matters beyond visualization: PX4 denied arming ("No connection to the GCS") until QGC was connected when we tried it manually, and we have not verified that the backend's offboard arm works without a GCS link. Needs its own solution (e.g. a configurable GCS target IP, or disabling that preflight check in SITL).
- **GPU and `--native-gui`:** see above. Not tried on a real Stellar computer yet.

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
