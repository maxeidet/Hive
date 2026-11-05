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

# Only flying in Gazebo needs the drones.
flies_in_gazebo=1
if [ "$1" = summary ]; then flies_in_gazebo=0; fi
for arg in "$@"; do
  case "$arg" in
    --sim|-h|--help) flies_in_gazebo=0 ;;
  esac
done

if [ "$flies_in_gazebo" = 1 ]; then
  require_simulation
  refuse_if_busy
  "$BACKEND" spawn
  defenders=$(surveillance_defenders)
  wait_for_drones "$defenders $(surveillance_hostile)" "$defenders"
else
  require_container
fi

run_in_container python3 -m swarm_backend.scenarios baseline_eval --config "$SWARM_CONFIG" "$@"
