#!/usr/bin/env bash
# Records a whole shoot: the runs of a shoot file one after the other, unattended.
# Run on the HOST (Stellar) in tmux and walk away; recording/README.md, "Many runs".
#
#   recording/shoot.sh recording/shoots/demo.yaml
#   recording/shoot.sh recording/shoots/demo.yaml --prescreen 30   pick random seeds in the fast sim
#   recording/shoot.sh recording/shoots/demo.yaml --name demo-1     the folder (default yaml + date)
#   recording/shoot.sh recording/shoots/demo.yaml --dry-run         print what it would do
#   recording/shoot.sh recording/shoots/demo.yaml --fresh-sim       a sim of its own for every
#                                                                   run (a crash spoils one run)
#   recording/shoot.sh SHOOT.yaml -- --screen                       more record.sh options, every run
#
# Each run is one recording/record.sh into data/recordings/<shoot>/<run>/. Runs in a row with
# the same config share the sim (--keep-sim / --reuse-sim); a different config restarts it.
# A run that fails is noted and the shoot goes on. shoot.json in the shoot folder keeps each
# run's status: run the same command again to carry on after a stop (done runs are skipped).
# At the end: the runs' status and the takes ranked (video.sh moments <shoot>).
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
PKG="/workspace/ros2_ws/src/swarm_backend"
IMAGE="${VIDEO_IMAGE:-hive-px4-vnc}"
CONTAINER="px4-gazebo-vnc"

[ $# -ge 1 ] || { sed -n '2,/^set -u/p' "$0" | grep '^#' | sed 's/^# \{0,1\}//'; exit 1; }
YAML="$1"; shift
STEM="$(basename "${YAML%.yaml}")"
NAME=""
PRESCREEN=""; DRY=0; FRESH=0; RECORD_OPTS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --name) NAME="$2"; shift 2 ;;
    --prescreen) PRESCREEN="$2"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    --fresh-sim) FRESH=1; shift ;;
    --) shift; RECORD_OPTS=("$@"); break ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done
case "$YAML" in /*) ;; *) YAML="$(pwd)/$YAML" ;; esac
[ -f "$YAML" ] || { echo "No shoot file $YAML"; exit 1; }
REL_YAML="${YAML#"$REPO"/}"
[ "$REL_YAML" != "$YAML" ] || { echo "The shoot file must be inside the repo ($REPO): the container sees only that."; exit 1; }
# No --name: carry on with the newest unfinished shoot of this file, else start a new one.
if [ -z "$NAME" ]; then
  for d in $(ls -d "$REPO/data/recordings/$STEM"-*/ 2>/dev/null | sort -r); do
    if [ -f "$d/shoot.json" ] && grep -q '"status": "\(todo\|failed\)"' "$d/shoot.json"; then
      NAME="$(basename "$d")"
      echo "RESUME $NAME (unfinished; --name NEW for a new shoot)"
      break
    fi
  done
  [ -n "$NAME" ] || NAME="$STEM-$(date +%Y%m%d-%H%M)"
fi
FOLDER="data/recordings/$NAME"

say() { echo "[$(date +%H:%M:%S)] $*"; }

# Python in the image: directly when already in it (the tests), else a temporary container.
py() {
  if ! command -v docker >/dev/null 2>&1; then (cd "$PKG" && python3 "$@"); return; fi
  docker run --rm -e NO_VNC=1 -v "$REPO":/workspace -w "$PKG" "$IMAGE" python3 "$@"
}

PLAN_ARGS=(-m swarm_backend.demo.shoot plan "/workspace/$REL_YAML")
[ "$FRESH" = 1 ] && PLAN_ARGS+=(--fresh-sim)
if [ -n "$PRESCREEN" ]; then
  if [ "$DRY" = 1 ]; then echo "PRESCREEN: $PRESCREEN candidates per random entry in the fast sim"
  else
    say "Pre-screening in the fast sim ($PRESCREEN candidates per random entry)"
    py -m swarm_backend.demo.prescreen "/workspace/$REL_YAML" --candidates "$PRESCREEN" \
      --out "/workspace/$FOLDER/prescreen" || { echo "Pre-screening failed"; exit 1; }
    PLAN_ARGS+=(--prescreened "/workspace/$FOLDER/prescreen/prescreened.json")
  fi
fi
# The plan and shoot.json (a dry run only reads an existing shoot.json, never writes one).
if [ "$DRY" = 0 ] || [ -f "$REPO/$FOLDER/shoot.json" ]; then PLAN_ARGS+=(--folder "/workspace/$FOLDER"); fi
PLAN="$(py "${PLAN_ARGS[@]}")" || { echo "Could not plan the shoot: see above"; exit 1; }

[ "$DRY" = 1 ] || say "Shoot $NAME: $(echo "$PLAN" | grep -c .) run(s) into $FOLDER"
KEPT=0           # the previous run left the sim running
FAILED=()
INTERRUPTED=0
trap 'INTERRUPTED=1' INT TERM

# The sim a group of same-config runs shared: shut it down cleanly (pxh> shutdown).
stop_kept_sim() {
  if [ "$DRY" = 1 ]; then echo "STOP sim: the reused sim of this config group (shutdown, then remove)"; return; fi
  say "Stopping the shared sim"
  tmux send-keys -t hive-rec:sim "shutdown" Enter 2>/dev/null
  for _ in $(seq 60); do docker inspect "$CONTAINER" >/dev/null 2>&1 || break; sleep 2; done
  docker rm -f "$CONTAINER" >/dev/null 2>&1
  tmux kill-session -t hive-rec 2>/dev/null
}
echo "Resume after a stop with: recording/shoot.sh $REL_YAML --name $NAME"
while IFS=$'\t' read -r state id keep config pair opts cmd; do
  [ -n "$id" ] || continue
  if [ "$state" = done ]; then
    echo "SKIP $id (done)"
    continue
  fi
  args=(--name "$NAME/$id")
  [ "$keep" = 1 ] && args+=(--keep-sim)
  [ "$KEPT" = 1 ] && args+=(--reuse-sim)
  [ "$pair" != "-" ] && args+=(--pair "$pair")
  [ "$opts" != "-" ] && args+=($opts)
  line="SWARM_CONFIG=$config recording/record.sh ${args[*]} ${RECORD_OPTS[*]:-} $cmd"
  if [ "$DRY" = 1 ]; then
    echo "RECORD $id: $line"
    [ "$KEPT" = 1 ] && [ "$keep" = 0 ] && stop_kept_sim
    KEPT=$keep
    continue
  fi
  say "Run $id"
  # A run that failed before starts again from an empty folder (the recorders append).
  [ "$state" = failed ] && rm -rf "${REPO:?}/$FOLDER/$id"
  (cd "$REPO" && env SWARM_CONFIG="$config" bash recording/record.sh "${args[@]}" "${RECORD_OPTS[@]}" $cmd) </dev/null
  code=$?
  if [ "$code" = 130 ] || [ "$INTERRUPTED" = 1 ]; then
    # Ctrl-C: record.sh stopped its parts in order; this run is not done, the shoot stops.
    say "Interrupted: $id stays to do; resume with the same command and --name $NAME"
    stop_kept_sim
    exit 130
  fi
  if [ "$code" = 0 ]; then
    py -m swarm_backend.demo.shoot mark "/workspace/$FOLDER" "$id" done >/dev/null
    # A reused sim with no same-config run after it: record.sh kept it, stop it here.
    [ "$KEPT" = 1 ] && [ "$keep" = 0 ] && stop_kept_sim
    KEPT=$keep
  else
    say "Run $id FAILED: see $FOLDER/$id/logs/"
    py -m swarm_backend.demo.shoot mark "/workspace/$FOLDER" "$id" failed >/dev/null
    FAILED+=("$id")
    # Start the next run clean: no half-stopped sim or session left behind.
    tmux kill-session -t hive-rec 2>/dev/null
    docker rm -f "$CONTAINER" >/dev/null 2>&1
    KEPT=0
  fi
done <<< "$PLAN"

[ "$DRY" = 1 ] && exit 0
if [ "$KEPT" = 1 ]; then  # the last run kept it (it should not, but be safe)
  tmux kill-session -t hive-rec 2>/dev/null; docker rm -f "$CONTAINER" >/dev/null 2>&1
fi
py -m swarm_backend.demo.shoot status "/workspace/$FOLDER"
[ ${#FAILED[@]} -gt 0 ] && say "Failed: ${FAILED[*]} (run the same command again to retry them)"
"$REPO/recording/video.sh" moments "$NAME" || true
