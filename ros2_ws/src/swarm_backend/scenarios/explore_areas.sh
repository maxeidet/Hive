#!/usr/bin/env bash
# Explore several areas with DAEP, one after the other, into ONE map: for each line of an area
# file it runs `explore` (launch/daep_exploration.launch.py) with mapping:=false, waits until
# DAEP is done and the drone has landed, stops the launch and its DAEP nodes, and starts the
# next area from where the drone stands. The map comes from one `map` that runs the whole
# time, so every area adds to it; after each area it is saved as ros2_ws/data/map.bt (the
# whole map so far) and copied to data/explore_area_N.bt.
#
# Start the container (drone with a depth camera or lidar), then the map at DAEP's resolution
# (0.1 m, raycast/dr in DAEP's px4_exploration.yaml) and the range used here:
#   ./mount-backend.sh map 0 --max-range 18
#   scenarios/explore_areas.sh                       all areas of hive_base_forest.txt
#   scenarios/explore_areas.sh FILE 5                FILE, starting at its 5th area
# Live map in the operator UI meanwhile: ./mount-backend.sh scanmap --watch 30 --world hive_base
#
# Ctrl-C lands the drone and stops. Each area's launch output: /tmp/explore_area_N.log in the
# container. AREA_TIMEOUT_S (default 2400) ends an area that takes longer.
set -u

CONTAINER=px4-gazebo-vnc
DIR="$(cd "$(dirname "$0")" && pwd)"
AREAS="${1:-$DIR/hive_base_forest.txt}"
FIRST="${2:-1}"
AREA_TIMEOUT_S="${AREA_TIMEOUT_S:-2400}"
MAP_RANGE="${MAP_RANGE:-18}"
SETUP='source /opt/ros/jazzy/setup.bash; source /opt/px4_ros2/install/local_setup.bash;
  cd /workspace/ros2_ws; source install/setup.bash'
# [x] in each pattern: pgrep/pkill must not match the bash -c that runs them.
LAUNCH_PAT='[d]aep_exploration.launch'
BRIDGE_PAT='[l]ib/swarm_backend/daep_bridge'

inside() { docker exec "$CONTAINER" bash -c "$SETUP
$1"; }
alive() { inside "pgrep -f '$1' >/dev/null"; }

stop_daep() {
  # The launch first, with SIGINT: the bridge lands the drone if it is still flying.
  inside "pkill -INT -f '$LAUNCH_PAT'" || true
  for _ in $(seq 60); do alive "$BRIDGE_PAT" || break; sleep 5; done
  # Leftovers keep publishing /exploration_completed and would end the next area at once.
  inside "pkill -f '$LAUNCH_PAT'; pkill -f '[i]nstall/daep/lib'; pkill -f '[l]ib/swarm_backend/save_map'; true"
  sleep 3
}

[ -f "$AREAS" ] || { echo "No area file $AREAS"; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || {
  echo "Container $CONTAINER is not running. Start it with docker/start-stellar.sh -m gz_x500_depth"; exit 1; }
inside "ros2 pkg prefix daep >/dev/null 2>&1" || {
  echo 'DAEP is not built. Clone the daep and daep_msgs forks into ros2_ws/src, then: colcon build --packages-up-to daep'; exit 1; }
inside "pgrep -af '[o]ctomap_server_node' | grep -q 'resolution:=0.1 '" || {
  echo "Start the map first, at DAEP's 0.1 m resolution: ./mount-backend.sh map 0 --max-range $MAP_RANGE"; exit 1; }
inside "colcon build --symlink-install --packages-select swarm_backend >/dev/null" || exit 1

stopping=0
trap 'stopping=1' INT TERM

n=0
while read -r center radius alt zmin zmax _; do
  case "$center" in ''|\#*) continue ;; esac
  n=$((n + 1))
  [ "$n" -lt "$FIRST" ] && continue
  [ "$stopping" = 1 ] && break
  log=/tmp/explore_area_$n.log
  echo "=== $(date +%H:%M) area $n: centre $center, +-$radius m, z $zmin..$zmax m (log $log)"
  stop_daep
  inside "setsid nohup ros2 launch swarm_backend daep_exploration.launch.py center:=$center \
    radius:=$radius alt:=$alt z_min:=$zmin z_max:=$zmax map_range:=$MAP_RANGE mapping:=false \
    > $log 2>&1 < /dev/null &"

  start=$(date +%s)
  result=timeout
  while [ $(( $(date +%s) - start )) -lt "$AREA_TIMEOUT_S" ]; do
    sleep 10
    if [ "$stopping" = 1 ]; then result=stopped; break; fi
    if inside "grep -q -e 'Map saved:' -e 'Could not save the map' $log"; then result=done; break; fi
    if inside "grep -q -e 'Flight termination' -e 'Stopping (' $log"; then result=failed; break; fi
    if [ $(( $(date +%s) - start )) -gt 60 ] && ! alive "$BRIDGE_PAT"; then result=failed; break; fi
  done
  echo "    $(date +%H:%M) area $n: $result after $(( ($(date +%s) - start) / 60 )) min"
  stop_daep
  inside "cp data/map.bt data/explore_area_$n.bt 2>/dev/null" && echo "    map so far: data/explore_area_$n.bt"
  if [ "$result" = failed ] || [ "$result" = stopped ]; then
    inside "grep -e 'Stopping (' -e 'Flight termination' -e ERROR $log | tail -5" || true
    echo "Stopped at area $n. Continue later with: $0 $AREAS $n"
    exit 1
  fi
done < "$AREAS"
echo "=== $(date +%H:%M) all areas done. Map: ros2_ws/data/map.bt (scanmap draws it)"
