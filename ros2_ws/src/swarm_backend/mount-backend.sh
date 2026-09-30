#!/usr/bin/env bash
# Build the swarm backend inside the running px4-gazebo-vnc container and run it.
# Start the container and drone 0 first (docker/start-dev.sh or start-stellar.sh).
#
# Usage:
#   ./mount-backend.sh                 interactive swarm shell (same as `shell`)
#   ./mount-backend.sh shell [N]       shell using the first N drones of config/swarm.yaml
#   ./mount-backend.sh spawn [N]       start PX4 drones 1..N-1 at their swarm.yaml poses
#   ./mount-backend.sh spawn TYPE      start ONE more drone of TYPE (see below) at the next free id
#   ./mount-backend.sh single [args]   Milestone 1 test (tests/single_drone_test.py --help)
#   ./mount-backend.sh multi [args]    Milestone 2 test (tests/multi_drone_swarm_test.py --help)
#   ./mount-backend.sh demo [N]        formation demo (swarm_node)
#   ./mount-backend.sh legacy          old backend_node + keyboard teleop
#   ./mount-backend.sh view [ID]       show drone ID in RViz with its cameras and lidars, default 1.
#                                      Any drone type; one without sensors shows where it is.
#                                      (`lidar [ID]` and `camera [ID]` do the same)
#
# spawn TYPE: plain, lidar (360° 3D, our model), lidar2d, lidar_front, lidar_down (1-beam
#   rangefinders), camera, camera_down, depth (RGB + depth), flow, gimbal, or any PX4_SIM_MODEL
#   such as gz_x500_depth. It takes the lowest instance id that is not running and that id's
#   swarm.yaml spawn_pose (so the shell can fly it), else a free spot at (4*id, -5, 0).
#   The shell only flies drones listed in swarm.yaml.
#   Example: ./mount-backend.sh spawn camera && ./mount-backend.sh view 1
#   A gimbal drone's camera is pointed from the shell: gimbal(id, pitch, yaw)
# spawn [N]: drones use the `model:` from swarm.yaml, else $MODEL (default gz_x500).
# Custom models in worlds/models/ (e.g. gz_x500_lidar_3d) are copied into the container first.
# RViz opens on the GUI display: the screen with start-stellar.sh --native-gui, the VNC
# desktop with start-dev.sh or --vnc. It cannot open with --headless.
# Every command reads the swarm.yaml in this folder, so edits apply without rebuilding.
set -e

CONTAINER=px4-gazebo-vnc
PKG=/workspace/ros2_ws/src/swarm_backend
CONFIG=$PKG/config/swarm.yaml
MODEL="${MODEL:-gz_x500}"

SETUP="
  source /opt/ros/jazzy/setup.bash
  source /opt/px4_ros2/install/local_setup.bash
  cd /workspace/ros2_ws
  colcon build --symlink-install --packages-select swarm_backend >/dev/null || colcon build --symlink-install --packages-select swarm_backend
  source install/setup.bash
"

case "${1:-}" in
  -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
esac

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "Container $CONTAINER is not running. Start it with docker/start-dev.sh (or start-stellar.sh) first."
  exit 1
fi

run() {
  docker exec -it "$CONTAINER" bash -c "$SETUP
  $1"
}

num_arg() {
  if [ -n "$1" ]; then echo "--num-drones $1"; fi
}

copy_models() {
  docker exec "$CONTAINER" bash -c "cp -r /workspace/worlds/models/. /opt/px4-gazebo/share/gz/models/"
}

start_drone() {
  local id="$1" x="$2" y="$3" z="$4" model="$5"
  echo "Starting drone $id ($model) at Gazebo ($x, $y, $z), log: /tmp/px4_$id.log in the container"
  # -dit: detached, but with a terminal so the PX4 shell stays alive.
  # Models without their own PX4 airframe (our custom ones) fly with the plain x500 airframe.
  docker exec -dit "$CONTAINER" bash -c "
    export PX4_SIM_MODEL=$model PX4_GZ_MODEL_POSE='$x,$y,$z,0,0,0'
    ls /opt/px4-gazebo/etc/init.d-posix/airframes/*_$model >/dev/null 2>&1 || export PX4_SYS_AUTOSTART=4001
    /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i $id > /tmp/px4_$id.log 2>&1"
}

model_for() {
  # Short names for `spawn TYPE`.
  case "$1" in
    plain)       echo gz_x500 ;;
    lidar)       echo gz_x500_lidar_3d ;;
    lidar2d)     echo gz_x500_lidar_2d ;;
    lidar_front) echo gz_x500_lidar_front ;;
    lidar_down)  echo gz_x500_lidar_down ;;
    camera)      echo gz_x500_mono_cam ;;
    camera_down) echo gz_x500_mono_cam_down ;;
    depth)       echo gz_x500_depth ;;
    flow)        echo gz_x500_flow ;;
    gimbal)      echo gz_x500_gimbal ;;
    gz_*)        echo "$1" ;;
    *)           return 1 ;;
  esac
}

spawn_one() {
  local model id pose
  if ! model=$(model_for "$1"); then
    echo "Unknown drone type: $1 (see --help)"
    exit 1
  fi
  # Lowest instance id >= 1 that no PX4 in the container is using.
  local running
  running=$(docker exec "$CONTAINER" ps -eo args | grep -oE 'px4(-gazebo)? -i [0-9]+' | awk '{print $NF}' | sort -un)
  id=1
  while grep -qx "$id" <<< "$running"; do id=$((id + 1)); done
  pose=$(docker exec "$CONTAINER" python3 -c "
import yaml
poses = {d['id']: d.get('spawn_pose', [0, 0, 0]) for d in yaml.safe_load(open('$CONFIG'))['drones']}
print(*poses.get($id, [4 * $id, -5, 0]))
")
  copy_models
  start_drone "$id" $pose "$model"
  echo "Give it ~15 s to appear in Gazebo. Then: ./mount-backend.sh view $id"
}

spawn() {
  # "id x y z model" for every drone but 0 (which the start script already runs).
  local drones
  drones=$(docker exec "$CONTAINER" python3 -c "
import yaml
drones = yaml.safe_load(open('$CONFIG'))['drones'][:${1:-1000}]
for d in drones[1:]:
    print(d['id'], *d.get('spawn_pose', [0, 0, 0]), d.get('model', '$MODEL'))
")
  if [ -z "$drones" ]; then
    echo "Nothing to spawn: swarm.yaml (or N) has only drone 0."
    return
  fi
  copy_models
  while read -r id x y z model; do
    if docker exec "$CONTAINER" pgrep -f "px4(-gazebo)? -i $id( |\$)" >/dev/null; then
      echo "Drone $id is already running."
      continue
    fi
    start_drone "$id" "$x" "$y" "$z" "$model"
  done <<< "$drones"
  echo "Give them ~15 s to appear in Gazebo and connect; status() in the shell shows when they are ready."
}

view() {
  # Bridge the drone's Gazebo camera/lidar topics and poses to ROS 2 and open RViz (sensor_view.py).
  run "python3 -m swarm_backend.sensor_view --id ${1:-1} --rviz $PKG/config/sensors.rviz"
}

cmd="${1:-shell}"
shift || true
case "$cmd" in
  shell)  run "ros2 run swarm_backend swarm_shell --config $CONFIG $(num_arg "$1")" ;;
  spawn)
    if [ -z "$1" ] || [[ "$1" =~ ^[0-9]+$ ]]; then spawn "$1"; else spawn_one "$1"; fi ;;
  single) run "python3 $PKG/tests/single_drone_test.py --config $CONFIG $*" ;;
  multi)  run "python3 $PKG/tests/multi_drone_swarm_test.py --config $CONFIG $*" ;;
  demo)   run "ros2 run swarm_backend swarm_node --ros-args -p config_file:=$CONFIG -p num_drones:=${1:-0}" ;;
  legacy) run "ros2 run swarm_backend backend_node & ros2 run swarm_backend keyboard" ;;
  view|lidar|camera) view "$1" ;;
  *) echo "Unknown command: $cmd (see --help)"; exit 1 ;;
esac
