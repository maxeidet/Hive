# What the scenario launchers share. Source it from a script in this folder:
#   source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
#
# The launcher says what it needs and calls these in order; set SWARM_CONFIG first:
#   require_simulation         the container and Gazebo are up
#   refuse_if_busy             nothing else is flying the drones
#   "$BACKEND" spawn           start the drones of SWARM_CONFIG that are not running
#   wait_for_drones IDS [CAMERA_IDS]   in Gazebo, with cameras, and talking to ROS
#   run_in_container CMD...    build the package and run CMD, arguments kept as they are
#
# Starting drones stays in mount-backend.sh, so there is one place that does it.

CONTAINER=px4-gazebo-vnc
PKG=/workspace/ros2_ws/src/swarm_backend
BACKEND="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/mount-backend.sh"
DRONE_WAIT_S="${DRONE_WAIT_S:-300}"  # freshly spawned drones can take minutes to reach ROS

require_container() {
  if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
    echo "Container $CONTAINER is not running. Start it with docker/start-dev.sh (or start-stellar.sh) first."
    exit 1
  fi
}

require_simulation() {
  require_container
  if ! docker exec "$CONTAINER" pgrep -f 'gz sim' >/dev/null; then
    echo "Gazebo is not running in $CONTAINER. Start it with docker/start-dev.sh (or start-stellar.sh) and wait for pxh>."
    exit 1
  fi
}

refuse_if_busy() {
  # Two programs would fight over the drones. A detector left running also stops the
  # surveillance scenarios, which start their own.
  local busy='swarm_shell|swarm_backend\.scenarios|swarm_backend\.detect\.live|nodes\.(patrol_node|base_station_node|fake_detection|attack_node)|tests/[a-z_]*\.py'
  if docker exec "$CONTAINER" pgrep -f "$busy" >/dev/null; then
    echo "Something else is using the drones in $CONTAINER:"
    docker exec "$CONTAINER" pgrep -af "$busy" || true
    echo "Stop it first (Ctrl-C in its terminal, so its drones land)."
    exit 1
  fi
}

surveillance_defenders() {
  docker exec "$CONTAINER" python3 -c "
import yaml
print(*yaml.safe_load(open('$SWARM_CONFIG'))['surveillance']['drone_ids'])"
}

surveillance_hostile() {
  docker exec "$CONTAINER" python3 -c "
import yaml
print(yaml.safe_load(open('$SWARM_CONFIG'))['attack']['target_id'])"
}

_not_ready() {
  # Sets NOT_READY to what is still missing; returns 0 when nothing is.
  local ids="$1" cameras="$2" topics id
  NOT_READY=""
  topics=$(docker exec "$CONTAINER" gz topic -l 2>/dev/null || true)
  for id in $ids; do
    grep -qE "^/world/[^/]+/model/[^/]+_$id/" <<< "$topics" || NOT_READY+=" drone $id is not in Gazebo;"
  done
  for id in $cameras; do
    grep -qE "/model/[^/]+_$id/link/camera_link/sensor/camera/image\$" <<< "$topics" \
      || NOT_READY+=" drone $id has no gimbal camera;"
  done
  # `ros2 topic list` still shows drones that are gone, so ask each PX4 if it is connected.
  local offline
  offline=$(docker exec "$CONTAINER" bash -c "for i in $(echo $ids); do /opt/px4-gazebo/bin/px4-uxrce_dds_client --instance \$i status 2>/dev/null | head -1 | grep -q ', connected' || echo \$i; done")
  for id in $offline; do NOT_READY+=" drone $id is not connected to ROS;"; done
  [ -z "$NOT_READY" ]
}

wait_for_drones() {
  # wait_for_drones "1 2 3 4" "1 2 3": all of them up, and the second list with cameras.
  local ids="$1" cameras="${2:-}" deadline=$((SECONDS + DRONE_WAIT_S))
  echo "Waiting for drones $ids (up to $DRONE_WAIT_S s)..."
  until _not_ready "$ids" "$cameras"; do
    if [ "$SECONDS" -ge "$deadline" ]; then
      echo "Not ready:$NOT_READY"
      echo "If they were already running as another type, restart with ./mount-backend.sh restart."
      echo "If Gazebo is paused, press play. PX4 logs: /tmp/px4_<id>.log in the container."
      exit 1
    fi
    sleep 2
  done
  echo "Drones $ids are up."
}

run_in_container() {
  # The arguments go through as they are (empty ones and spaces too). A terminal only when
  # there is one, so this also runs from nohup; with one, Ctrl-C reaches the program.
  local tty=-i
  if [ -t 0 ]; then tty=-it; fi
  docker exec $tty "$CONTAINER" bash -c '
    source /opt/ros/jazzy/setup.bash
    source /opt/px4_ros2/install/local_setup.bash
    cd /workspace/ros2_ws
    colcon build --symlink-install --packages-select swarm_backend >/dev/null || colcon build --symlink-install --packages-select swarm_backend
    source install/setup.bash
    exec "$@"' _ "$@"
}
