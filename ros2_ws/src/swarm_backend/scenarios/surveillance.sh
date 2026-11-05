#!/usr/bin/env bash
# Surveillance scenario in one command: spawns the three gimbal defenders, then starts a fake
# detector and a patrol node for each, and the base station. The hostile drone (drone 0) you
# fly yourself, from the swarm shell. Settings: surveillance.yaml in this folder.
#
# Start the container and drone 0 first (docker/start-dev.sh or start-stellar.sh), then:
#   scenarios/surveillance.sh              run until Ctrl-C, then the defenders land
#   scenarios/surveillance.sh --no-realistic   a perfect fake detector (no delay, noise)
#   IDS="1 2" scenarios/surveillance.sh    only two defenders (also set drone_ids in the yaml)
#
# Everything runs in the container; the output of each program is in /tmp/surv_*.log there,
# and the lines of all of them are shown here. In another terminal:
#   docker exec px4-gazebo-vnc tail -F /tmp/surv_basestation.log
# Do not run the swarm shell on the defenders (drones 1-3) at the same time.
set -e

CONTAINER=px4-gazebo-vnc
PKG=/workspace/ros2_ws/src/swarm_backend
export SWARM_CONFIG=$PKG/scenarios/surveillance.yaml
IDS="${IDS:-1 2 3}"
BACKEND="$(cd "$(dirname "$0")/.." && pwd)/mount-backend.sh"

REALISTIC=--realistic
for arg in "$@"; do
  case "$arg" in
    -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
    --no-realistic) REALISTIC= ;;
    *) echo "Unknown argument: $arg (see --help)"; exit 1 ;;
  esac
done

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "Container $CONTAINER is not running. Start it with docker/start-dev.sh (or start-stellar.sh) first."
  exit 1
fi
if docker exec "$CONTAINER" pgrep -f 'patrol_node|base_station_node|fake_detection' >/dev/null; then
  echo "Surveillance programs are already running in the container. Stop them first:"
  echo "  docker exec $CONTAINER pkill -INT -f patrol_node   (the defenders land, then:)"
  echo "  docker exec $CONTAINER pkill -f 'base_station_node|fake_detection'"
  exit 1
fi

"$BACKEND" spawn

ROS="source /opt/ros/jazzy/setup.bash; source /opt/px4_ros2/install/local_setup.bash"

drones_up() {
  # In Gazebo with a gimbal camera, and PX4 talking to ROS (that comes some seconds later).
  local gz ros
  gz=$(docker exec "$CONTAINER" gz topic -l)
  ros=$(docker exec "$CONTAINER" bash -c "$ROS; ros2 topic list")
  for id in $IDS; do
    grep -q "/model/x500_gimbal_$id/link/camera_link/sensor/camera/image\$" <<< "$gz" || return 1
    grep -q "^/px4_$id/fmu/out/vehicle_odometry\$" <<< "$ros" || return 1
  done
}
echo "Waiting for drones $IDS and their gimbal cameras..."
for _ in $(seq 60); do
  if drones_up; then break; fi
  sleep 2
done
if ! drones_up; then
  echo "Drones $IDS are not all gimbal drones talking to ROS. If they were already running"
  echo "as another type, restart the container and run this again."
  echo "PX4 logs: /tmp/px4_<id>.log in the container."
  exit 1
fi

SOURCE="$ROS; cd /workspace/ros2_ws"
docker exec "$CONTAINER" bash -c "$SOURCE; colcon build --symlink-install --packages-select swarm_backend >/dev/null"
SOURCE="$SOURCE; source install/setup.bash"

start() {  # start NAME COMMAND...: in the background in the container, output in /tmp/surv_NAME.log
  local name=$1; shift
  docker exec -d "$CONTAINER" bash -c "$SOURCE; $* > /tmp/surv_$name.log 2>&1"
}

stop_all() {
  trap - EXIT INT TERM
  echo
  echo "Stopping: the defenders land first (this can take a few minutes)..."
  docker exec "$CONTAINER" pkill -INT -f 'nodes.patrol_node' >/dev/null 2>&1 || true
  for _ in $(seq 300); do
    docker exec "$CONTAINER" pgrep -f 'nodes.patrol_node' >/dev/null 2>&1 || break
    sleep 3
  done
  docker exec "$CONTAINER" pkill -f 'nodes.base_station_node|nodes.fake_detection' >/dev/null 2>&1 || true
  docker exec "$CONTAINER" pkill -f 'tail -F /tmp/surv_' >/dev/null 2>&1 || true
}
trap stop_all EXIT INT TERM

docker exec "$CONTAINER" bash -c "rm -f /tmp/surv_*.log"
for id in $IDS; do
  start "detect_$id" python3 -m swarm_backend.nodes.fake_detection --config "$SWARM_CONFIG" --id "$id" $REALISTIC
done
start basestation python3 -m swarm_backend.nodes.base_station_node --config "$SWARM_CONFIG"
sleep 5
for id in $IDS; do
  start "patrol_$id" python3 -m swarm_backend.nodes.patrol_node --config "$SWARM_CONFIG" --id "$id"
done

echo
echo "Running. The defenders take off and fly to their points on the arc ('On patrol')."
echo "Fly the hostile drone (drone 0) toward the base from the north; Ctrl-C here stops everything."
echo
# Follow all the logs; the container's own tail ends when it is killed above.
docker exec "$CONTAINER" bash -c "sleep 3; tail -F /tmp/surv_basestation.log $(for id in $IDS; do printf '/tmp/surv_patrol_%s.log ' "$id"; done)" || true
