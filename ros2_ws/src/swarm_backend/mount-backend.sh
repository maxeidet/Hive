#!/usr/bin/env bash
# Build the swarm backend inside the running px4-gazebo-vnc container and run it.
# Start the container and drone 0 first (docker/start-dev.sh or start-stellar.sh).
#
# Usage:
#   ./mount-backend.sh                 interactive swarm shell (same as `shell`)
#   ./mount-backend.sh shell [N]       shell using the first N drones of config/swarm.yaml
#   ./mount-backend.sh spawn [N]       start PX4 drones 1..N-1 at their swarm.yaml poses
#   ./mount-backend.sh single [args]   Milestone 1 test (tests/single_drone_test.py --help)
#   ./mount-backend.sh multi [args]    Milestone 2 test (tests/multi_drone_swarm_test.py --help)
#   ./mount-backend.sh demo [N]        formation demo (swarm_node)
#   ./mount-backend.sh legacy          old backend_node + keyboard teleop
#
# Extra drones use the gz_x500 model; override with MODEL=gz_x500_depth ./mount-backend.sh spawn
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

spawn() {
  # "id x y z" for every drone but 0 (which the start script already runs).
  local drones
  drones=$(docker exec "$CONTAINER" python3 -c "
import yaml
drones = yaml.safe_load(open('$CONFIG'))['drones'][:${1:-1000}]
for d in drones[1:]:
    print(d['id'], *d.get('spawn_pose', [0, 0, 0]))
")
  if [ -z "$drones" ]; then
    echo "Nothing to spawn: swarm.yaml (or N) has only drone 0."
    return
  fi
  while read -r id x y z; do
    if docker exec "$CONTAINER" pgrep -f "px4(-gazebo)? -i $id( |\$)" >/dev/null; then
      echo "Drone $id is already running."
      continue
    fi
    echo "Starting drone $id at Gazebo ($x, $y, $z), log: /tmp/px4_$id.log in the container"
    # -dit: detached, but with a terminal so the PX4 shell stays alive.
    docker exec -dit "$CONTAINER" bash -c "
      export PX4_SIM_MODEL=$MODEL PX4_GZ_MODEL_POSE='$x,$y,$z,0,0,0'
      /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i $id > /tmp/px4_$id.log 2>&1"
  done <<< "$drones"
  echo "Give them ~15 s to appear in Gazebo and connect; status() in the shell shows when they are ready."
}

cmd="${1:-shell}"
shift || true
case "$cmd" in
  shell)  run "ros2 run swarm_backend swarm_shell --config $CONFIG $(num_arg "$1")" ;;
  spawn)  spawn "$1" ;;
  single) run "python3 $PKG/tests/single_drone_test.py --config $CONFIG $*" ;;
  multi)  run "python3 $PKG/tests/multi_drone_swarm_test.py --config $CONFIG $*" ;;
  demo)   run "ros2 run swarm_backend swarm_node --ros-args -p config_file:=$CONFIG -p num_drones:=${1:-0}" ;;
  legacy) run "ros2 run swarm_backend backend_node & ros2 run swarm_backend keyboard" ;;
  *) echo "Unknown command: $cmd (see --help)"; exit 1 ;;
esac
