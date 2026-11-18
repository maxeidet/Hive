#!/usr/bin/env bash
# Record scenario 2 runs for the threshold replay, then run the replay. Unattended: start
# it in tmux and walk away. Each run starts its own simulation with
# docker/start-stellar.sh --headless (hive_base, no Gazebo GUI, to save CPU) in the tmux
# session "tune-sim", flies scenario 2 with YOLO + ByteTrack + --lock and every YOLO box down
# to --conf 0.05 recorded, saves it in data/tune/<TAG>-<run>/ of the repo, and stops the
# simulation again (start-stellar.sh then removes the container). At the end it runs
# detect/replay_eval.py over all of them (on the host, ~/Hive/.venv). Nothing is left
# running: start the simulation again yourself (docker/start-stellar.sh).
#
# It refuses to start while the px4-gazebo-vnc container runs: it would be replaced.
# Not `mount-backend.sh restart`: that stops drone 0's PX4, and start-stellar.sh then
# removes the container.
#
#   tmux new -s tune 'scenarios/record_tuning.sh'
#   scenarios/record_tuning.sh watch1 pursuit1        only these runs
#   TAG=nov11 scenarios/record_tuning.sh
#
# Logs and the report: runs/tune/<TAG>/ in the repo (report.txt, one log per run).
cd "$(dirname "$0")/.."
REPO="$(cd ../../.. && pwd)"
TAG="${TAG:-$(date +%Y%m%d-%H%M)}"
OUT="$REPO/runs/tune/$TAG"
mkdir -p "$OUT"
DETECT="--detect-args --tracker bytetrack --lock --conf 0.05"

declare -A RUNS=(
  [watch1]="--no-attack --duration 40"
  [watch2]="--no-attack --speed 7 --stagger 12 --duration 45"
  [pursuit1]="--duration 90"
  [pursuit2]="--speed 8 --duration 90"
  [mpc1]="--guidance mpc --duration 90"
  [mpc2]="--guidance mpc --stagger 12 --duration 90"
)
ORDER="watch1 watch2 pursuit1 pursuit2 mpc1 mpc2"
[ $# -gt 0 ] && ORDER="$*"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/progress.txt"; }
CONTAINER=px4-gazebo-vnc
running() { docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; }
if running; then
  echo "The simulation ($CONTAINER) is running. Stop it first (Ctrl-C in its terminal)."
  exit 1
fi

stop_sim() {
  tmux kill-session -t tune-sim 2>/dev/null || true  # start-stellar.sh removes the container
  for _ in $(seq 60); do running || break; sleep 2; done
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
}

start_sim() {  # start_sim LOG: true when Gazebo and drone 0 are up and talking to ROS
  stop_sim
  tmux new -d -s tune-sim "$REPO/docker/start-stellar.sh --headless -w hive_base > $1 2>&1"
  for _ in $(seq 300); do running && break; sleep 2; done
  running || return 1
  for _ in $(seq 300); do
    docker exec "$CONTAINER" /opt/px4-gazebo/bin/px4-uxrce_dds_client --instance 0 status \
      2>/dev/null | head -1 | grep -q ', connected' && return 0
    sleep 2
  done
  return 1
}
trap 'stop_sim; exit 130' INT TERM HUP

log "Recording runs: $ORDER (tag $TAG)"
for run in $ORDER; do
  args="${RUNS[$run]}"
  if [ -z "$args" ]; then log "Unknown run $run, skipped"; continue; fi
  log "$run: starting the simulation (headless)"
  if ! start_sim "$OUT/$run.sim.log"; then
    log "$run: the simulation did not come up (see $OUT/$run.sim.log), skipped"; stop_sim; continue
  fi
  log "$run: flying ($args)"
  # A run that hangs is stopped after 25 minutes; the next restart cleans up after it.
  timeout 1500 scenarios/yolo.sh --no-view --record "$TAG-$run" $args $DETECT \
    > "$OUT/$run.log" 2>&1
  log "$run: done (exit $?): $(grep -m1 -A3 '^Scenario 2:' "$OUT/$run.log" | tr '\n' ' ' | cut -c1-200)"
  stop_sim
done

log "Replaying the thresholds"
folders=$(ls -d "$REPO/data/tune/$TAG"-* 2>/dev/null)
if [ -n "$folders" ]; then
  nice -n 10 "$REPO/.venv/bin/python" -m swarm_backend.detect.replay_eval $folders \
    --jobs 2 --top 30 --csv "$OUT/all.csv" > "$OUT/report.txt" 2>&1
  log "Replay exit $?: $OUT/report.txt"
else
  log "No recordings to replay"
fi
log "All done; the simulation is stopped (start it again with docker/start-stellar.sh)"
