#!/usr/bin/env bash
# Records one run of a scenario for the demo videos, in one command, in one terminal.
# Run on the HOST (Stellar) from anywhere; recording/README.md has the whole workflow.
#
#   recording/record.sh scenarios/trial.sh 1
#   recording/record.sh --screen --watch scenarios/trial.sh 1     also film the UI, show Gazebo
#   recording/record.sh --name head-on --cameras attacker_watch ./mount-backend.sh attacktest
#   recording/record.sh --dry-run scenarios/trial.sh 1             print what it would do
#
# SCENARIO runs from ros2_ws/src/swarm_backend/ (scenarios/..., ./mount-backend.sh ...).
# `idle 60` runs no scenario: only the cameras record, 60 s (e.g. --cameras base_orbit).
# Options:
#   --name NAME          run folder data/recordings/NAME (default: the date and time)
#   --screen             also film the operator UI: opens this computer's browser in kiosk
#                        mode on http://localhost:8090 on the leftmost monitor, films that one
#   --watch              show Gazebo's window (default headless). Slow: the cameras then
#                        render without the GPU (0.17x against 0.67x on 24 Nov)
#   --cameras A,B        live camera angles (default attacker_watch: the chase camera; add
#                        arc_side for the wide shot), --no-cameras
#   --no-yolo            do not record the defenders' YOLO views
#   --export DIR         copy the run (without gz/ and cache/) to DIR/NAME/ at the end
#   --keep-sim           leave the sim and the tmux session running at the end (shoot.sh)
#   --reuse-sim          use the sim a --keep-sim run left running (shoot.sh); its Gazebo
#                        log stays in that first run's gz/
#   --pair ID            this run is one side of a baseline/ours pair (shoot.sh compare:)
#   --dry-run            print every step (start, wait, stop, check), run nothing
#
# Everything runs in the tmux session hive-rec, one window per part, each logging to
# data/recordings/NAME/logs/<part>.log; look in with `tmux attach -t hive-rec`. Each part is
# started only when the one before is ready. If a part never gets ready or stops during the
# run, everything is stopped and the part named. At the end, or on Ctrl-C, the parts are
# stopped in order (the scenario first, so the defenders land), then the run folder is
# checked. SWARM_CONFIG (a path in the container) picks the config, default
# scenarios/surveillance.yaml.
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SB="ros2_ws/src/swarm_backend"
PKG="/workspace/$SB"
SESSION="hive-rec"
CONTAINER="px4-gazebo-vnc"
CONFIG="${SWARM_CONFIG:-$PKG/scenarios/surveillance.yaml}"

NAME="$(date +%Y%m%d-%H%M%S)"
SCREEN=0; WATCH=0; YOLO=1; CAMERAS="attacker_watch"; EXPORT=""; KEEP_SIM=0; KEEP_NEXT=0; DRY=0; REUSE=0; PAIR=""
while [ $# -gt 0 ]; do
  case "$1" in
    --name) NAME="$2"; shift 2 ;;
    --screen) SCREEN=1; shift ;;
    --watch) WATCH=1; shift ;;
    --cameras) CAMERAS="$2"; shift 2 ;;
    --no-cameras) CAMERAS=""; shift ;;
    --no-yolo) YOLO=0; shift ;;
    --export) EXPORT="$2"; shift 2 ;;
    --keep-sim) KEEP_SIM=1; KEEP_NEXT=1; shift ;;
    --reuse-sim) REUSE=1; shift ;;
    --pair) PAIR="$2"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    -h|--help) sed -n '2,/^set -u/p' "$0" | grep '^#' | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "Unknown option: $1 (see --help)"; exit 1 ;;
    *) break ;;
  esac
done
if [ $# -eq 0 ]; then
  echo "Which scenario? recording/record.sh [OPTIONS] SCENARIO [ARGS] (see --help)"
  exit 1
fi
SCENARIO="$*"
if [ "$1" = idle ]; then SCENARIO_CMD="sleep ${2:-60}"; else SCENARIO_CMD="$SCENARIO"; fi
RUN="data/recordings/$NAME"
LOGS="$REPO/$RUN/logs"
ANGLES="${CAMERAS//,/ }"
STARTED=()      # parts that run, in start order
DEFENDER_IDS="" # from run.json, once written
STOPPING=0

say() { echo "[$(date +%H:%M:%S)] $*"; }
log_of() { echo "$LOGS/$1.log"; }

# start_part PART COMMAND: in its own tmux window, output to its log; "[exit N]" when it ends.
start_part() {
  local part="$1" cmd="$2"
  if [ "$DRY" = 1 ]; then echo "START $part: $cmd"; STARTED+=("$part"); return; fi
  say "Starting $part"
  printf 'cd %q\n{ %s ; } 2>&1 | tee -i %q\necho "[exit ${PIPESTATUS[0]}]" >> %q\nsleep infinity\n' \
    "$REPO" "$cmd" "$(log_of "$part")" "$(log_of "$part")" > "$LOGS/$part.sh"
  : > "$(log_of "$part")"
  tmux new-window -d -t "$SESSION" -n "$part" "bash $(printf %q "$LOGS/$part.sh")"
  STARTED+=("$part")
}

ended() { grep -q '^\[exit ' "$(log_of "$1")" 2>/dev/null; }

fail() { say "FAILED: $*"; stop_all; exit 1; }

# wait_log PART REGEX TIMEOUT [COUNT]: until the log has COUNT (1) lines matching REGEX.
wait_log() {
  local part="$1" re="$2" timeout="$3" count="${4:-1}"
  if [ "$DRY" = 1 ]; then echo "WAIT $part: $count line(s) matching '$re' in its log (${timeout}s)"; return; fi
  for _ in $(seq "$timeout"); do
    [ "$(grep -c -E "$re" "$(log_of "$part")" 2>/dev/null)" -ge "$count" ] && { say "$part ready"; return; }
    ended "$part" && fail "$part stopped before it was ready (log: $(log_of "$part"))"
    sleep 1
  done
  fail "$part not ready after ${timeout}s (log: $(log_of "$part"))"
}

# wait_cmd PART DESCRIPTION TIMEOUT COMMAND...: until COMMAND succeeds.
wait_cmd() {
  local part="$1" desc="$2" timeout="$3"; shift 3
  if [ "$DRY" = 1 ]; then echo "WAIT $part: $desc (${timeout}s)"; return; fi
  for _ in $(seq $((timeout / 2))); do
    "$@" >/dev/null 2>&1 && { say "$part: $desc"; return; }
    ended "$part" && fail "$part stopped (log: $(log_of "$part"))"
    sleep 2
  done
  fail "$part: not '$desc' after ${timeout}s (log: $(log_of "$part"))"
}

dds_connected() {
  docker exec "$CONTAINER" /opt/px4-gazebo/bin/px4-uxrce_dds_client --instance 0 status 2>/dev/null \
    | head -1 | grep -q ', connected'
}
file_grows() {
  local f="$REPO/$RUN/$1" a b
  [ -f "$f" ] || return 1
  a=$(wc -c < "$f"); sleep 2; b=$(wc -c < "$f"); [ "$b" -gt "$a" ]
}
container_gone() { ! docker inspect "$CONTAINER" >/dev/null 2>&1; }

# stop_part PART: the gentle way first, then kill its window.
stop_part() {
  local part="$1" how wait=30
  case "$part" in
    # The defenders land only when the sim is kept for the next run (--keep-sim). Otherwise
    # the sim is shut down right after, and landing at the slow sim's pace took minutes.
    scenario)
      if [ "$KEEP_NEXT" = 1 ]; then how="Ctrl-C, wait up to 300 s for the defenders to land"; wait=300
      else how="Ctrl-C, wait up to 20 s (no landing: the sim is shut down next)"; wait=20; fi ;;
    sim) how="type shutdown at pxh>, wait up to 120 s for the container to go, then docker rm -f" ;;
    session) how="tmux kill-session -t $SESSION" ;;
    browser) how="close the browser window" ;;
    screen) how="Ctrl-C, wait up to 30 s, then kill, then docker rm -f px4-gazebo-vnc-screen" ;;
    *) how="Ctrl-C, wait up to 30 s, then kill" ;;
  esac
  if [ "$KEEP_SIM" = 1 ] && { [ "$part" = sim ] || [ "$part" = session ]; }; then
    [ "$DRY" = 1 ] && echo "KEEP $part: left running for the next run (--keep-sim)"
    [ "$DRY" = 1 ] || say "Leaving $part running (--keep-sim)"
    return
  fi
  if [ "$DRY" = 1 ]; then echo "STOP $part: $how"; return; fi
  case "$part" in
    session)
      if [ "$KEEP_SIM" = 1 ]; then say "Leaving tmux session $SESSION for the sim (--keep-sim)"
      else tmux kill-session -t "$SESSION" 2>/dev/null; fi
      return ;;
    sim)
      if [ "$KEEP_SIM" = 1 ]; then say "Leaving the sim running (--keep-sim)"; return; fi
      say "Stopping the sim"
      tmux send-keys -t "$SESSION:sim" "shutdown" Enter 2>/dev/null
      for _ in $(seq 60); do container_gone && break; sleep 2; done
      docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
      tmux kill-window -t "$SESSION:sim" 2>/dev/null; return ;;
  esac
  ended "$part" || { say "Stopping $part"; tmux send-keys -t "$SESSION:$part" C-c 2>/dev/null; }
  for _ in $(seq "$wait"); do ended "$part" && break; sleep 1; done
  ended "$part" || say "$part did not stop in ${wait}s: killed (log: $(log_of "$part"))"
  tmux kill-window -t "$SESSION:$part" 2>/dev/null
  # `video.sh screen` is a `docker run -it`: killing its client leaves the container running.
  [ "$part" = screen ] && docker rm -f px4-gazebo-vnc-screen >/dev/null 2>&1
  return 0
}

STOP_ORDER=(scenario cams yolo screen browser poses ui sim)
stop_all() {
  [ "$STOPPING" = 1 ] && return
  STOPPING=1
  local part s
  for part in "${STOP_ORDER[@]}"; do
    for s in "${STARTED[@]}"; do [ "$s" = "$part" ] && stop_part "$part"; done
    [ "$part" = scenario ] && copy_detect_records
  done
  stop_part session
}

# The detectors' own box records (detect/live.py --record, started by the scenarios): for
# the zoom on the YOLO box. In the container's /tmp; the next run in a kept sim overwrites them.
copy_detect_records() {
  if [ "$DRY" = 1 ]; then echo "STOP detect-records: copy /tmp/detect_<id>.jsonl into the run folder"; return; fi
  local id
  for id in $DEFENDER_IDS; do
    docker cp "$CONTAINER:/tmp/detect_$id.jsonl" "$REPO/$RUN/detect_d$id.jsonl" >/dev/null 2>&1 || true
  done
}

on_int() {
  if [ "$STOPPING" = 1 ]; then
    # A second Ctrl-C while stopping: keep going, or the sim and the session stay behind.
    if [ "$DRY" = 1 ]; then echo "ON SECOND INT: still stopping, wait (nothing is abandoned)"
    else say "Still stopping in order, please wait (the sim must shut down cleanly)"; fi
    return
  fi
  say "Interrupted: stopping everything in order"; stop_all; health; exit 130
}

# ---- the folder at the end ----------------------------------------------------------------------
check() {  # check FILE MIN_BYTES
  if [ "$DRY" = 1 ]; then echo "CHECK $1 (at least $2 bytes)"; return; fi
  local size
  size=$(du -sk "$REPO/$RUN/$1" 2>/dev/null | cut -f1)
  if [ -n "$size" ] && [ $((size * 1024)) -ge "$2" ]; then echo "  ✓ $1"; else echo "  ✗ $1"; BAD=1; fi
}
BAD=0
health() {
  [ "$DRY" = 1 ] || say "Checking $RUN"
  [ "$REUSE" = 1 ] || check gz 20000   # a reused sim logs into the first run's gz/
  check ui.jsonl 1
  check poses.jsonl 1
  check run.json 1
  if [ "$YOLO" = 1 ]; then
    if [ "$DRY" = 1 ]; then echo "CHECK debug_d<ID>.mkv for each defender"
    else for id in $DEFENDER_IDS; do check "debug_d$id.mkv" 1; done; fi
  fi
  for a in $ANGLES; do check "cam_$a.mkv" 1; done
  [ "$SCREEN" = 1 ] && check screen.mkv 1
  return 0
}

# ---- go -----------------------------------------------------------------------------------------
if [ "$DRY" = 0 ]; then
  command -v tmux >/dev/null || { echo "tmux is needed (it is on the Stellar computers)."; exit 1; }
  RUNNING=0
  [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ] && RUNNING=1
  if [ "$REUSE" = 1 ]; then
    if [ "$RUNNING" = 0 ] || ! tmux has-session -t "$SESSION" 2>/dev/null; then
      echo "--reuse-sim: no sim left running by a --keep-sim run (container and tmux $SESSION)."
      exit 1
    fi
  else
    if [ "$RUNNING" = 1 ]; then
      echo "The sim ($CONTAINER) is already running. Stop it first: record.sh starts its own."
      exit 1
    fi
    if tmux has-session -t "$SESSION" 2>/dev/null; then
      echo "A tmux session $SESSION exists (another record.sh?). tmux kill-session -t $SESSION"
      exit 1
    fi
  fi
  mkdir -p "$LOGS"
  say "Recording $SCENARIO into $RUN (watch with: tmux attach -t $SESSION)"
fi
trap on_int INT TERM HUP

if [ "$REUSE" = 1 ]; then
  KEEP_SIM=1  # it belongs to the shoot, not to this run (the shoot stops it; KEEP_NEXT
              # says whether a next run needs it, with the defenders landed)
  wait_cmd sim "drone 0 (kept sim) connected to ROS" 120 dds_connected
else
  if [ "$DRY" = 1 ]; then echo "START session: tmux new-session -d -s $SESSION"
  else tmux new-session -d -s "$SESSION" -n main "sleep infinity"; fi
  STARTED+=(session)
  SIM="RECORD_RUN=$NAME docker/start-stellar.sh --record"
  [ "$WATCH" = 0 ] && SIM="$SIM --headless"
  start_part sim "$SIM"
  wait_log sim 'Recording this run to' 600
  wait_cmd sim "drone 0 connected to ROS" 600 dds_connected
fi

OPTIONS="{\"screen\": $SCREEN, \"watch\": $WATCH, \"yolo\": $YOLO, \"cameras\": \"$CAMERAS\", \"pair\": \"$PAIR\"}"
RUNINFO="docker exec -w $PKG $CONTAINER python3 -m swarm_backend.demo.runinfo write /workspace/$RUN --config $CONFIG --options '$OPTIONS' -- $SCENARIO"
if [ "$DRY" = 1 ]; then echo "START runinfo: $RUNINFO"
else
  say "Writing run.json"
  eval "$RUNINFO" || fail "run.json could not be written (is SWARM_CONFIG=$CONFIG right?)"
fi
DEFENDER_IDS="<the defender ids in run.json>"
if [ "$DRY" = 0 ]; then
  DEFENDER_IDS=$(docker exec "$CONTAINER" python3 -c "import json
d = json.load(open('/workspace/$RUN/run.json'))
print(' '.join(n.rsplit('_', 1)[1] for n in d['defenders']))")
fi

start_part ui "SWARM_CONFIG=$CONFIG $SB/mount-backend.sh ui --record /workspace/$RUN"
wait_log ui 'Recording the UI on' 120
start_part poses "SWARM_CONFIG=$CONFIG $SB/mount-backend.sh posesrec --out /workspace/$RUN"
wait_log poses 'Logging poses of' 120

if [ "$SCREEN" = 1 ]; then
  BROWSER=""
  for b in firefox chromium-browser chromium google-chrome; do command -v "$b" >/dev/null && { BROWSER="$b"; break; }; done
  [ "$DRY" = 1 ] && BROWSER="${BROWSER:-firefox}"
  [ -n "$BROWSER" ] || fail "--screen: no firefox, chromium or google-chrome on this computer"
  # A browser of its own (new instance, fresh profile): an open Firefox would otherwise take
  # the URL in a normal tab and the command would end at once.
  PROFILE="$LOGS/browser-profile"
  # The leftmost monitor (xrandr: "0: +*DP-0 2560/597x1440/336+0+0 DP-0"): the kiosk opens
  # there and only that monitor is filmed, so a terminal on another monitor stays free.
  MONITOR=$(xrandr --listmonitors 2>/dev/null | awk 'NR > 1 { split($3, a, /[\/x+]/);
    print a[5], a[1] "x" a[3] "+" a[5] "+" a[6] }' | sort -n | head -1 | cut -d' ' -f2)
  [ "$DRY" = 1 ] && MONITOR="${MONITOR:-2560x1440+0+0}"
  [ -n "$MONITOR" ] || fail "--screen: xrandr lists no monitor"
  MON_X=${MONITOR#*+}; MON_Y=${MON_X#*+}; MON_X=${MON_X%%+*}
  case "$BROWSER" in
    # Firefox has no window-position flag: its profile's xulstore.json places the window,
    # and --kiosk then fills the monitor the window is on.
    firefox) BROWSER_CMD="firefox --new-instance --profile $PROFILE --kiosk http://localhost:8090" ;;
    *) BROWSER_CMD="$BROWSER --user-data-dir=$PROFILE --window-position=$MON_X,$MON_Y --kiosk --no-first-run http://localhost:8090" ;;
  esac
  if [ "$DRY" = 0 ]; then
    mkdir -p "$PROFILE"
    printf '{"chrome://browser/content/browser.xhtml": {"main-window": {"screenX": "%s", "screenY": "%s", "width": "1280", "height": "800", "sizemode": "normal"}}}\n' \
      $((MON_X + 100)) $((MON_Y + 100)) > "$PROFILE/xulstore.json"
  fi
  start_part browser "$BROWSER_CMD"
  if [ "$DRY" = 1 ]; then echo "WAIT browser: 5 s, still running"
  else sleep 5; ended browser && fail "the browser closed (log: $(log_of browser))"; fi
  start_part screen "recording/video.sh screen --region $MONITOR $NAME"
  wait_cmd screen "screen.mkv growing" 30 file_grows screen.mkv
fi

if [ "$YOLO" = 1 ]; then
  start_part yolo "SWARM_CONFIG=$CONFIG $SB/mount-backend.sh debugrec $DEFENDER_IDS --out /workspace/$RUN"
  wait_log yolo 'Recording the debug images' 120
fi
if [ -n "$ANGLES" ]; then
  start_part cams "SWARM_CONFIG=$CONFIG $SB/mount-backend.sh camrec $ANGLES --no-pose-log --out /workspace/$RUN"
  wait_log cams 'first image at sim time' 180 "$(echo $ANGLES | wc -w)"
fi

# Quick end: the surveillance scenarios log LAST_LINE once their last engagement is over
# (stop_and_land), then fly the defenders home and land them: minutes at the slow sim's pace,
# for nothing when the sim is shut down next. So after AFTER_S more sim seconds (the story's
# aftermath) the run stops there. Not with --keep-sim: the next run needs them landed (a
# reused sim without --keep-sim is the shoot's last run in it: the shoot stops it next).
LAST_LINE='Defenders back to their slots'
AFTER_S=15
sim_now() { tail -n 1 "$REPO/$RUN/ui.jsonl" 2>/dev/null | grep -o '"sim": *[0-9.]*' | grep -o '[0-9.]*$'; }

start_part scenario "cd $SB && SWARM_CONFIG=$CONFIG $SCENARIO_CMD"
if [ "$DRY" = 1 ]; then
  echo "WAIT scenario: until it ends; every 5 s check that every part still runs"
  [ "$KEEP_NEXT" = 1 ] || echo "WAIT scenario: or ${AFTER_S} sim s after '$LAST_LINE' (its last engagement), then stop: no landing"
else
  say "Flying: $SCENARIO (log: $(log_of scenario))"
  LAST_AT=""
  while ! ended scenario; do
    for p in "${STARTED[@]}"; do
      [ "$p" = scenario ] || [ "$p" = session ] && continue
      ended "$p" && fail "$p stopped during the run (log: $(log_of "$p"))"
    done
    if [ "$KEEP_NEXT" = 0 ]; then
      if [ -z "$LAST_AT" ] && grep -aq "$LAST_LINE" "$(log_of scenario)"; then
        LAST_AT=$(sim_now); LAST_AT=${LAST_AT:-0}
        say "Last engagement over (sim ${LAST_AT} s): ${AFTER_S} sim s more for the aftermath, then stop"
      fi
      if [ -n "$LAST_AT" ] && awk -v n="$(sim_now)" -v t="$LAST_AT" -v a="$AFTER_S" 'BEGIN { exit !(n >= t + a) }'; then
        say "Done: stopping without landing (the sim is shut down next)"
        break
      fi
    fi
    sleep 5
  done
  ended scenario && say "The scenario ended: $(tail -n 1 "$(log_of scenario)")"
fi

stop_all
if [ "$DRY" = 1 ]; then
  echo "ON INT:"
  STOPPING=0
  stop_all
  on_int
fi
health
if [ "$DRY" = 1 ]; then
  echo "START moments: recording/video.sh moments $NAME"
  [ -n "$EXPORT" ] && echo "START export: copy $RUN to $EXPORT/$NAME (without gz/, cache/)"
  exit 0
fi
"$REPO/recording/video.sh" moments "$NAME" || true
if [ -n "$EXPORT" ]; then
  mkdir -p "$EXPORT/$NAME"
  if command -v rsync >/dev/null; then rsync -a --exclude gz --exclude cache "$REPO/$RUN/" "$EXPORT/$NAME/"
  else (cd "$REPO/$RUN" && tar cf - --exclude ./gz --exclude ./cache . | tar xf - -C "$EXPORT/$NAME"); fi
  say "Copied to $EXPORT/$NAME"
fi
if [ "$BAD" = 1 ]; then say "Some recordings are missing (✗ above): see $LOGS"; exit 1; fi
say "Ready to cut: recording/video.sh final --preview $NAME"
