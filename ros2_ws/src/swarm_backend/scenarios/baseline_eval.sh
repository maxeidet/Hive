#!/usr/bin/env bash
# Baseline evaluation in one command: starts the drones of surveillance.yaml that are not
# running, waits for them, then flies the scenario (swarm_backend/scenarios/baseline_eval.py).
#
# Start the container and drone 0 first (docker/start-dev.sh or start-stellar.sh), then:
#   scenarios/baseline_eval.sh intercept --aspects 0 --speeds 4 --seeds 0-2   a first check
#   scenarios/baseline_eval.sh intercept       the whole grid, many hours: use tmux or nohup
#   scenarios/baseline_eval.sh detect          the 30 detection seeds
#   scenarios/baseline_eval.sh summary         the report's tables from the saved runs
#   scenarios/baseline_eval.sh intercept --sim the fast simulation: no Gazebo needed
#   scenarios/baseline_eval.sh intercept --help   the scenario's own arguments
#   scenarios/baseline_eval.sh intercept --spawn-at-slots ...   the defenders start on the
#                                ground right below their slots on the arc and only climb
#                                (a copy of surveillance.yaml, swarm_backend/scenarios/at_slots.py,
#                                in data/configs/). Drones already started elsewhere are
#                                restarted at those points by the scenario.
#   scenarios/baseline_eval.sh intercept --restart-each-run ...   restart Gazebo (and every
#                                drone) between runs, so each run starts from a fresh world
#
# A defender that crashes is not respawned: after the run is saved, Gazebo is restarted with
# every drone fresh and the batch goes on (the scenario's --restart-on-crash, exit code 75).
# That needs a session with no drone 0 (start-stellar.sh --no-drone, the dev GUI): drone 0's
# PX4 is the container itself otherwise, so there crashed defenders are respawned as before.
# The operator UI and RViz views keep running through the restarts.
#
# Saved runs are skipped, so a batch can be stopped and started again. Ctrl-C lands the
# defenders. Without a terminal (nohup), stop it with:
#   docker exec px4-gazebo-vnc pkill -INT -f swarm_backend.scenarios
set -e

export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

case "${1:-}" in
  ''|-h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
esac

# --spawn-at-slots and --restart-each-run are this launcher's own: take them out of what
# the scenario gets.
AT_SLOTS=0
EACH_RUN=0
args=()
for arg in "$@"; do
  case "$arg" in
    --spawn-at-slots) AT_SLOTS=1 ;;
    --restart-each-run) EACH_RUN=1 ;;
    *) args+=("$arg") ;;
  esac
done
set -- "${args[@]}"

# Only flying in Gazebo needs the drones.
flies_in_gazebo=1
if [ "$1" = summary ]; then flies_in_gazebo=0; fi
for arg in "$@"; do
  case "$arg" in
    --sim|-h|--help) flies_in_gazebo=0 ;;
  esac
done

require_container
if [ "$AT_SLOTS" = 1 ]; then
  at_slots_config=/workspace/data/configs/surveillance_at_slots.yaml
  docker exec "$CONTAINER" bash -c "cd $PKG && python3 -m swarm_backend.scenarios.at_slots --config $SWARM_CONFIG --out $at_slots_config"
  export SWARM_CONFIG=$at_slots_config
fi

prepare_drones() {
  "$BACKEND" spawn
  defenders=$(surveillance_defenders)
  wait_for_drones "$defenders $(surveillance_hostile)" "$defenders"
}

if [ "$flies_in_gazebo" = 0 ]; then
  run_in_container python3 -m swarm_backend.scenarios baseline_eval --config "$SWARM_CONFIG" "$@"
  exit $?
fi

require_simulation
refuse_if_busy
restart_args=()
if docker exec "$CONTAINER" grep -qx NO_DRONE=1 /tmp/hive_sim.env 2>/dev/null; then
  restart_args=(--restart-on-crash)
  if [ "$EACH_RUN" = 1 ]; then
    restart_args+=(--one-run)
    # The first run from a fresh world too, unless it is one already (no PX4 running).
    if docker exec "$CONTAINER" pgrep -f '[b]in/px4 -i' >/dev/null; then
      echo "=== Restarting Gazebo for a fresh world (the operator UI and RViz views keep running) ==="
      "$BACKEND" restart --keep-tools || exit 1
    fi
  fi
else
  echo "Drone 0's PX4 runs this container (not started with --no-drone), so Gazebo cannot be"
  echo "restarted from here: crashed defenders are respawned instead$([ "$EACH_RUN" = 1 ] && echo ', and --restart-each-run is ignored')."
fi
prepare_drones

while true; do
  code=0
  run_in_container python3 -m swarm_backend.scenarios baseline_eval --config "$SWARM_CONFIG" \
    "${restart_args[@]}" "$@" || code=$?
  [ "$code" = 75 ] || exit "$code"
  echo
  echo "=== Restarting Gazebo (the operator UI and RViz views keep running) ==="
  "$BACKEND" restart --keep-tools || exit 1
  prepare_drones
done
