#!/usr/bin/env bash
# Scenario "yolo" (scenario 2, YOLO only, formerly scenario2.sh): drone 1 (gimbal camera)
# guards right above the base at 30 m with the YOLO detector running on its camera, and two hostile drones (2 and 3) come from 150 m out,
# 20 deg left and right of north, straight toward the base, 10 and 14 m above the
# defender (they stop 20 m short of the base and hover). The defender attacks what the
# detector reports with the baseline attack (pure pursuit on target_position, as the
# `attack` node and the surveillance patrols). A hit (1.5 m, from Gazebo's true positions)
# takes the hostile out and the defender goes back to guarding for the next one. Opens an
# RViz window with drone 1's camera. Settings: scenario2.yaml (attack: and scenario2:).
#
# Start the container and drone 0 in the hive_base world first (docker/start-stellar.sh,
# or docker/start-dev.sh -w hive_base), then:
#   scenarios/yolo.sh                     fly until Ctrl-C, then all drones land
#   scenarios/yolo.sh --speed 8           hostile speed in m/s
#   scenarios/yolo.sh --stagger 20        the second hostile leaves 20 s after the first
#   scenarios/yolo.sh --attack-speed 8    the defender's pursuit speed in m/s
#   scenarios/yolo.sh --no-attack         the defender only watches
#   scenarios/yolo.sh --detector baseline the baseline detector instead of YOLO
#   scenarios/yolo.sh --duration 120      stop 120 simulation seconds after they leave
#   scenarios/yolo.sh --no-view           without the RViz window
#   scenarios/yolo.sh --detect-args --tracker bytetrack   the rest goes to the detector
#   scenarios/yolo.sh --birds 10               ten birds fly in with the hostiles
# Any other tests/scenario2_test.py argument works too.
#
# YOLO uses the newest .onnx under models/ in the repo; the detector's log is
# /tmp/scenario2_detect.log in the container. Its debug image (boxes drawn) is on
# /px4_1/detection/debug. Do not run the swarm shell at the same time.
set -e

CONTAINER=px4-gazebo-vnc
PKG=/workspace/ros2_ws/src/swarm_backend
export SWARM_CONFIG=$PKG/scenarios/scenario2.yaml
DEFENDER=1
HOSTILES="2 3"
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

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "Container $CONTAINER is not running. Start it with docker/start-stellar.sh (or start-dev.sh -w hive_base) first."
  exit 1
fi
if docker exec "$CONTAINER" pgrep -f 'lib/swarm_backend/swarm_shell|tests/scenario2_test.py|swarm_backend\.scenarios' >/dev/null; then
  echo "A swarm shell or another scenario is already running and would fight over the drones. Close it first."
  exit 1
fi
if ! docker exec "$CONTAINER" gz topic -l | grep -q '^/world/hive_base/'; then
  echo "Warning: the world is not hive_base; the base, trees and start points of scenario2.yaml are made for it."
fi

"$BACKEND" spawn

ROS="source /opt/ros/jazzy/setup.bash; source /opt/px4_ros2/install/local_setup.bash"

drones_up() {
  # In Gazebo (the defender with a gimbal camera), and PX4 talking to ROS.
  local gz ros
  gz=$(docker exec "$CONTAINER" gz topic -l)
  ros=$(docker exec "$CONTAINER" bash -c "$ROS; ros2 topic list")
  grep -q "/model/x500_gimbal_$DEFENDER/link/camera_link/sensor/camera/image\$" <<< "$gz" || return 1
  for id in $DEFENDER $HOSTILES; do
    grep -q "^/px4_$id/fmu/out/vehicle_odometry\$" <<< "$ros" || return 1
  done
}
echo "Waiting for drones $DEFENDER $HOSTILES (and drone $DEFENDER's gimbal camera)..."
for _ in $(seq 60); do
  if drones_up; then break; fi
  sleep 2
done
if ! drones_up; then
  echo "Drones $DEFENDER $HOSTILES are not all up and talking to ROS, or drone $DEFENDER is not a"
  echo "gimbal drone. If they were already running as another type, restart the simulation"
  echo "(./mount-backend.sh restart) and run this again. PX4 logs: /tmp/px4_<id>.log in the container."
  exit 1
fi

SOURCE="$ROS; cd /workspace/ros2_ws"
docker exec "$CONTAINER" bash -c "$SOURCE; colcon build --symlink-install --packages-select swarm_backend >/dev/null"
SOURCE="$SOURCE; source install/setup.bash"

stop_views() {
  # Closing RViz makes the sensor_view stop its bridge and exit.
  docker exec "$CONTAINER" pkill -f 'rviz2 -d /tmp/sensors_' >/dev/null 2>&1 || true
  sleep 1
  docker exec "$CONTAINER" pkill -f 'parameter_bridge.*sensors_bridge_' >/dev/null 2>&1 || true
}
if [ "$VIEW" = 1 ]; then
  stop_views
  trap stop_views EXIT
  echo "Opening RViz with drone $DEFENDER's camera (on the GUI display)"
  docker exec -d "$CONTAINER" bash -c "$SOURCE; python3 -m swarm_backend.sim.sensor_view --id $DEFENDER --rviz $PKG/config/sensors.rviz > /tmp/sensor_view_$DEFENDER.log 2>&1"
fi

# With a terminal, Ctrl-C reaches the flight script, which then lands the drones.
TTY=-i
if [ -t 0 ]; then TTY=-it; fi
docker exec $TTY "$CONTAINER" bash -c "$SOURCE; python3 $PKG/tests/scenario2_test.py --config $SWARM_CONFIG ${ARGS[*]}"
