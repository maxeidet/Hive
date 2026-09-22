# PX4 / Gazebo / ROS 2 dev container

## Starting local dev

```bash
./start-dev.sh                        # default drone (gz_x500)
./start-dev.sh -m gz_x500_depth        # pick a different sensor loadout
./start-dev.sh --model gz_x500_lidar_front
./start-dev.sh -h                      # show usage
```

The `-m`/`--model` flag sets which drone variant gets spawned — any value from the `PX4_SIM_MODEL` table below. Defaults to `gz_x500` (no extra sensors) if omitted.

**What the script actually starts:**

1. Builds the `hive-px4-vnc` image from `docker/Dockerfile` (the official `px4io/px4-sitl-gazebo-ros2` base image, plus `Xvfb` + `fluxbox` + `x11vnc` + `noVNC`/`websockify` for the GUI layer).
2. Runs that image as a container named `px4-gazebo-vnc`, publishing:
   - port `6080` → noVNC in the browser (`http://localhost:6080/vnc.html`)
   - port `5900` → native VNC client (`vnc://127.0.0.1:5900`, password `hive1234`)
   - the repo root mounted at `/workspace` inside the container
   - the container's default command starts the DDS agent (`MicroXRCEAgent udp4 -p 8888`), bridging PX4 ↔ ROS 2
3. Opens a **second terminal window** automatically, which waits for the container to be running, then starts PX4 + Gazebo (drone instance `0`, GUI mode — not headless) with the chosen `PX4_SIM_MODEL`.
4. Connect QGroundControl separately (it auto-detects the running PX4 instance on macOS via Docker Desktop's gateway IP) — see `hive-vault/StackV2.md`, section 5.2.1.

Stop everything with `Ctrl+C` in the first terminal (the container runs with `--rm`, so it's removed on exit).

---

# Drone sensors (PX4 SITL / Gazebo)

Verified by inspecting the model files directly inside the `px4io/px4-sitl-gazebo-ros2` image (`/opt/px4-gazebo/share/gz/models/`).

## Default: every drone always has this

No matter which `PX4_SIM_MODEL` variant we pick, they all build on the base model `x500_base`, which always includes:

1. Every drone (base `x500`) already has, regardless of variant — confirmed from `x500_base/model.sdf`:
   * IMU (`imu_sensor`)
   * Magnetometer (`magnetometer_sensor`)
   * Barometer (`air_pressure_sensor`)
   * GPS (`navsat_sensor`)

No camera, LiDAR, or radar is included in the base model.

## Extra sensors: pick via `PX4_SIM_MODEL`

We switch the sensor loadout by setting this environment variable before starting `px4-gazebo`, e.g:

```bash
export PX4_SIM_MODEL=gz_x500_depth
```

| `PX4_SIM_MODEL` value | Extra sensor(s) on top of the default |
|---|---|
| `gz_x500` | None — just IMU/magnetometer/barometer/GPS |
| `gz_x500_mono_cam` | Regular camera (`mono_cam`) |
| `gz_x500_depth` | RGB camera + depth camera (OAK-D Lite: `camera` + `depth_camera`) |
| `gz_x500_lidar_2d` | 2D LiDAR |
| `gz_x500_lidar_front` | 3D LiDAR, forward-facing (`gpu_lidar`) |
| `gz_x500_lidar_down` | 3D LiDAR, downward-facing (`gpu_lidar`) |
| `gz_x500_flow` | Optical flow sensor + LiDAR (altitude hold without GPS) |
| `gz_x500_vision` | No extra sensor — for external vision/mocap input instead |
| `gz_x500_gimbal` | Gimbal mount (pair with a camera model) |

**Radar doesn't exist** as a built-in sensor in PX4/Gazebo. For Counter-UAS detection, we'll either need to fake it with a scripted "detection radius" node, or build a custom Gazebo sensor plugin.

## Example: starting a drone with a depth camera

If the container is running via `./start-dev.sh` (container name `px4-gazebo-vnc`), the simplest way to get a depth-camera drone as drone 0 is to just pass the model to the script itself:

```bash
./start-dev.sh -m gz_x500_depth
```

## Example: adding a second drone with a depth camera

To add an *extra* drone with a depth camera alongside whatever `start-dev.sh` already started (drone 0), run this in a new terminal — note the different container name, instance number, and spawn pose so it doesn't collide with drone 0:

```bash
docker exec -it px4-gazebo-vnc bash -c "
  export PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE='5,0,0,0,0,0'
  /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 1
"
```

It'll spawn into the same running Gazebo world and show up namespaced as `/px4_1/fmu/...` (see `StackV2.md`, section 5.2.2 for how multi-drone namespacing works). Bump `-i` and the pose further for a third, fourth, etc. drone.

Sensor data from camera/LiDAR variants gets bridged to ROS 2 via `ros_gz` (see StackV2.md, section 3) — check `ros2 topic list` after startup to see which topics are actually published for the chosen model.
