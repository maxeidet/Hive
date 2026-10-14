#!/usr/bin/env bash
# Scenario 1: two drones with gimbal cameras. Drone 1 flies in circles at 20 m and drone 2
# chases it, keeping its camera on it. Opens an RViz window per drone with its camera image.
# Settings: scenario1.yaml in this folder.
#
# Start the container and drone 0 first (docker/start-dev.sh or start-stellar.sh), then:
#   scenarios/scenario1.sh                 fly until Ctrl-C, then both drones land
#   scenarios/scenario1.sh --duration 120  stop after 120 simulation seconds
#   scenarios/scenario1.sh --speed 6       any other tests/intercept_test.py argument works too
#   scenarios/scenario1.sh --no-view       without the RViz windows
#
# While it runs, save camera images from another terminal:
#   ./mount-backend.sh snap 2 --count 10 --label chase
# Do not run the swarm shell at the same time: two programs would fight over the drones.
set -e

CONTAINER=px4-gazebo-vnc
PKG=/workspace/ros2_ws/src/swarm_backend
export SWARM_CONFIG=$PKG/scenarios/scenario1.yaml
DRONES="1 2"
BACKEND="$(cd "$(dirname "$0")/.." && pwd)/mount-backend.sh"

VIEW=1
ARGS=()
for arg in "$@"; do
  case "$arg" in
    -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
    --no-view) VIEW=0 ;;
    *) ARGS+=("$arg") ;;
  esac
done
# Until Ctrl-C unless a duration is given.
if [[ ! " ${ARGS[*]} " =~ " --duration" ]]; then ARGS+=(--duration 100000); fi
# Two cameras and two RViz windows make Gazebo slow, so allow the climb plenty of time.
if [[ ! " ${ARGS[*]} " =~ " --climb-timeout" ]]; then ARGS+=(--climb-timeout 1200); fi

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "Container $CONTAINER is not running. Start it with docker/start-dev.sh (or start-stellar.sh) first."
  exit 1
fi
if docker exec "$CONTAINER" pgrep -f 'lib/swarm_backend/swarm_shell|tests/intercept_test.py' >/dev/null; then
  echo "A swarm shell or intercept test is already running and would fight over the drones. Close it first."
  exit 1
fi

"$BACKEND" spawn

ROS="source /opt/ros/jazzy/setup.bash; source /opt/px4_ros2/install/local_setup.bash"

drones_up() {
  # In Gazebo with a gimbal camera, and PX4 talking to ROS (that comes some seconds later).
  local gz ros
  gz=$(docker exec "$CONTAINER" gz topic -l)
  ros=$(docker exec "$CONTAINER" bash -c "$ROS; ros2 topic list")
  for id in $DRONES; do
    grep -q "/model/x500_gimbal_$id/link/camera_link/sensor/camera/image\$" <<< "$gz" || return 1
    grep -q "^/px4_$id/fmu/out/vehicle_odometry\$" <<< "$ros" || return 1
  done
}
echo "Waiting for drones $DRONES and their gimbal cameras..."
for _ in $(seq 60); do
  if drones_up; then break; fi
  sleep 2
done
if ! drones_up; then
  echo "Drones $DRONES are not both gimbal drones talking to ROS. If they were already running"
  echo "as another type, restart the container (docker/start-dev.sh) and run this again."
  echo "PX4 logs: /tmp/px4_<id>.log in the container."
  exit 1
fi

SOURCE="$ROS; cd /workspace/ros2_ws"
docker exec "$CONTAINER" bash -c "$SOURCE; colcon build --symlink-install --packages-select swarm_backend >/dev/null"
SOURCE="$SOURCE; source install/setup.bash"

stop_views() {
  # Closing RViz makes each sensor_view stop its bridge and exit.
  docker exec "$CONTAINER" pkill -f 'rviz2 -d /tmp/sensors_' >/dev/null 2>&1 || true
  sleep 1
  docker exec "$CONTAINER" pkill -f 'parameter_bridge.*sensors_bridge_' >/dev/null 2>&1 || true
}
if [ "$VIEW" = 1 ]; then
  stop_views
  trap stop_views EXIT
  for id in $DRONES; do
    echo "Opening RViz with drone $id's camera (on the GUI display)"
    docker exec -d "$CONTAINER" bash -c "$SOURCE; python3 -m swarm_backend.sensor_view --id $id --rviz $PKG/config/sensors.rviz > /tmp/sensor_view_$id.log 2>&1"
  done
fi

# With a terminal, Ctrl-C reaches the flight script, which then lands both drones.
TTY=-i
if [ -t 0 ]; then TTY=-it; fi
docker exec $TTY "$CONTAINER" bash -c "$SOURCE; python3 $PKG/tests/intercept_test.py --config $SWARM_CONFIG ${ARGS[*]}"
