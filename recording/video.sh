#!/usr/bin/env bash
# Demo videos of a Hive scenario: Gazebo filmed from a replay of the run, and the operator
# UI rendered from its recording, both on simulation time so they line up frame by frame.
# Full guide: recording/README.md. Run on the HOST from the repo root (Stellar or Mac).
#
#   1. Record (live run):  docker/start-stellar.sh --record        (Mac: mount-backend.sh restart --record)
#                          mount-backend.sh ui --record auto        then the scenario (attacktest)
#   2. Pick a start:       recording/video.sh events                 sim times of what happened
#   3. Film Gazebo:        recording/video.sh replay -a hostile_chase --start 312
#                          (record button, mp4, play, record button; save in the run folder)
#   4. Render the UI:      recording/video.sh ui --start 312 --duration 60 [--collapse-events]
#   5. Put together:       recording/video.sh compose --ui ui_312.mp4 --gazebo hostile_chase.mp4 --layout pip
#   6. Compress:           recording/video.sh compress demo.mp4 demo_10mb.mp4 10
#
# Subcommands (RUN = a folder name in data/recordings/, default the newest):
#   events [RUN]                         the UI's events with their sim times
#   angles                               the replay camera angles (recording_config.json)
#   replay -a ANGLE [--start S] [-p GAIN] [-v] [RUN]   open the Gazebo replay at one angle
#   ui --start S [--duration D] [--fps 25] [--size 1920x1080] [--collapse-events] [RUN]
#                                        render the UI to RUN/ui_<S>.mp4 (headless, no sim needed)
#   debug --id ID --start S [--duration D] [RUN]
#                                        drone ID's debug images (mount-backend.sh debugrec) from
#                                        sim time S: RUN/debug_d<ID>_<S>.mp4, composes like a replay clip
#   cam --angle A --start S [--duration D] [--plain] [RUN]
#                                        a live camera (mount-backend.sh camrec) from sim time S with
#                                        each drone labelled: RUN/cam_<A>_<S>.mp4 (--plain: no labels)
#   screen [--region WxH+X+Y] [RUN]       during the run: film the UI's browser window (click it)
#                                        into RUN/screen.mkv until Ctrl-C (recording/screen.py)
#   screen-cut --start S --duration D [RUN]
#                                        that film on sim time: RUN/screen_<S>.mp4, composes
#                                        like `ui` (use it instead: video.sh ui hangs on Stellar)
#   compose --ui F --gazebo F [--layout pip|side|stack] [--gz-offset S] [--out F] [RUN]
#                                        file names are relative to the run folder
#   compress IN OUT [MB] [WIDTH]         two-pass H.264 to a target size; paths relative to the
#                                        newest run folder (RUN_NAME=<run> for another)
#
# The sim container (px4-gazebo-vnc) is used when it runs. Stellar's start script removes
# it when PX4 stops, so otherwise a temporary one is started from the same image
# (hive-px4-vnc) and removed again; `replay` gives it this computer's screen and GPU.
set -e
IMAGE="${VIDEO_IMAGE:-hive-px4-vnc}"
LIVE="px4-gazebo-vnc"
TEMP="px4-gazebo-vnc-video"
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(dirname "$0")/.."
REPO="$(pwd)"
RUNS="data/recordings"
PKG="/workspace/ros2_ws/src/swarm_backend"

usage() { sed -n '2,/^set -e/p' "$SELF" | grep '^#' | sed 's/^# \{0,1\}//'; }

run_dir() {  # $1: a run name or empty (newest)
  local name="$1"
  [ -n "$name" ] || name=$(ls -d "$RUNS"/*/ 2>/dev/null | sort | tail -n 1 | xargs -n1 basename 2>/dev/null || true)
  if [ -z "$name" ] || [ ! -d "$RUNS/$name" ]; then
    echo "No run folder ${name:+$RUNS/$name }found. Record a run first (docker/start-stellar.sh --record)." >&2
    exit 1
  fi
  echo "$name"
}

# Echoes the container to use. $1 = gui: the temporary one gets the screen and the GPU.
container() {
  if [ "$(docker inspect -f '{{.State.Running}}' "$LIVE" 2>/dev/null)" = "true" ]; then
    echo "$LIVE"; return
  fi
  if [ "$(docker inspect -f '{{.State.Running}}' "$TEMP" 2>/dev/null)" = "true" ]; then
    echo "$TEMP"; return
  fi
  local args=(-e NO_VNC=1)
  if [ "$1" = "gui" ]; then
    if [ "$(uname -s)" != "Linux" ] || [ -z "${DISPLAY:-}" ]; then
      echo "The sim container is not running. On the Mac start it with docker/start-dev.sh." >&2
      exit 1
    fi
    args+=(-e "DISPLAY=${DISPLAY}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix)
    local xauth="${XAUTHORITY:-$HOME/.Xauthority}"
    [ -f "$xauth" ] && args+=(-e XAUTHORITY=/tmp/.Xauthority -v "$xauth":/tmp/.Xauthority:ro)
    command -v nvidia-smi >/dev/null 2>&1 && args+=(--device nvidia.com/gpu=all)
  fi
  docker run -d --rm --name "$TEMP" "${args[@]}" -v "$REPO":/workspace "$IMAGE" sleep infinity >/dev/null
  echo "$TEMP"
}

stop_temp() { docker rm -f "$TEMP" >/dev/null 2>&1 || true; }

# Arguments that are not flags: the last one may be a run name.
split_run() {  # sets RUN and ARGS from "$@"
  RUN=""; ARGS=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --collapse-events|-v|--verbose) ARGS+=("$1"); shift ;;
      -*) ARGS+=("$1" "$2"); shift 2 ;;
      *) RUN="$1"; shift ;;
    esac
  done
}

cmd="${1:-help}"; shift || true
case "$cmd" in
  events)
    RUN=$(run_dir "$1")
    C=$(container); [ "$C" = "$TEMP" ] && trap stop_temp EXIT
    docker exec -w "$PKG" "$C" python3 -m swarm_backend.ui.render "/workspace/$RUNS/$RUN/ui.jsonl" --list
    ;;

  angles)
    python3 -c "import json; a = json.load(open('recording/recording_config.json'))['replay_angles']
for k, v in a.items(): print(f'{k:16s} {v}')"
    ;;

  replay)
    ANGLE=""; START=""; PGAIN="0.1"; VERBOSE="1"; RUN=""
    while [ $# -gt 0 ]; do
      case "$1" in
        -a|--angle) ANGLE="$2"; shift 2 ;;
        -s|--start) START="$2"; shift 2 ;;
        -p|--pgain) PGAIN="$2"; shift 2 ;;
        -v|--verbose) VERBOSE="4"; shift ;;
        -*) echo "Unknown flag: $1"; exit 1 ;;
        *) RUN="$1"; shift ;;
      esac
    done
    [ -n "$ANGLE" ] || { echo "Which angle? -a NAME (recording/video.sh angles)"; exit 1; }
    RUN=$(run_dir "$RUN")
    [ -d "$RUNS/$RUN/gz" ] || { echo "$RUNS/$RUN has no Gazebo log (gz/): was the sim started with --record?"; exit 1; }
    C=$(container gui)
    [ "$C" = "$TEMP" ] && trap stop_temp EXIT
    if docker exec "$C" pgrep -f "[g]z sim" >/dev/null; then
      echo "Gazebo is still running in $C. Stop the live sim first (Ctrl+C in the start-stellar.sh terminal;"
      echo "on the Mac: docker exec $LIVE pkill -f 'gz sim')."
      exit 1
    fi
    echo "Replaying $RUNS/$RUN from the $ANGLE angle${START:+, starting at sim time $START s}."
    echo "Save the video as /workspace/$RUNS/$RUN/$ANGLE.mp4"
    RES_PATHS=$(python3 recording/kitconf.py extra_resource_paths | tr ' ' ':')
    docker exec -it \
      -e LOG_DIR="/workspace/$RUNS/$RUN/gz" -e ANGLE="$ANGLE" -e PGAIN="$PGAIN" -e START="$START" \
      -e VERBOSE="$VERBOSE" -e KIT=/workspace/recording -e RES_PATHS="$RES_PATHS" \
      "$C" bash -c '
      # Same model paths as the live sim, so meshes resolve (the log has no copied meshes).
      export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${RES_PATHS}"
      # Our playback layout with a sim-time video recorder, where Gazebo looks for an override
      # (passing --gui-config to --playback shows the start menu instead).
      PB_GUI=$(ls /usr/share/gz/gz-sim*/gui/playback_gui.config 2>/dev/null | tail -n 1)
      if [ -n "$PB_GUI" ]; then
        MAJOR=$(basename "$(dirname "$(dirname "$PB_GUI")")" | tr -dc "0-9")
        mkdir -p "$HOME/.gz/sim/$MAJOR"
        python3 "$KIT/replay_gui_config.py" "$PB_GUI" "$HOME/.gz/sim/$MAJOR/playback_gui.config" \
          || echo "Could not add the sim-time video recorder to the replay window."
      fi
      # Only these two server systems: PX4 points GZ_SIM_SERVER_CONFIG_PATH at its own list,
      # whose Contact system segfaults on replayed contact sensors.
      cat > /tmp/replay_server.config <<CFG
<server_config><plugins>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
</plugins></server_config>
CFG
      export GZ_SIM_SERVER_CONFIG_PATH=/tmp/replay_server.config
      cd "$KIT" && python3 -u replay_camera.py "$ANGLE" "$PGAIN" "$START" &
      gz sim -v "$VERBOSE" --playback "$LOG_DIR"
      CODE=$?
      [ "$CODE" = 0 ] || echo "Gazebo stopped with exit code $CODE. Run again with -v to see why."
    '
    ;;

  ui)
    split_run "$@"
    RUN=$(run_dir "$RUN")
    C=$(container); [ "$C" = "$TEMP" ] && trap stop_temp EXIT
    docker exec -w "$PKG" "$C" python3 -m swarm_backend.ui.render "/workspace/$RUNS/$RUN/ui.jsonl" "${ARGS[@]}"
    ;;

  debug)
    split_run "$@"
    RUN=$(run_dir "$RUN")
    C=$(container); [ "$C" = "$TEMP" ] && trap stop_temp EXIT
    docker exec -w "$PKG" "$C" python3 -m swarm_backend.ui.debug_video cut "/workspace/$RUNS/$RUN" "${ARGS[@]}"
    ;;

  cam)
    split_run "$@"
    RUN=$(run_dir "$RUN")
    C=$(container); [ "$C" = "$TEMP" ] && trap stop_temp EXIT
    docker exec -w "$PKG" "$C" python3 -m swarm_backend.ui.camera_video overlay "/workspace/$RUNS/$RUN" "${ARGS[@]}"
    ;;

  screen)
    split_run "$@"
    RUN=$(run_dir "$RUN")
    [ -n "${DISPLAY:-}" ] || { echo "DISPLAY is not set: run this in a terminal on this computer's desktop."; exit 1; }
    # Its own container, not the sim's: x11grab hands the X server a shared-memory segment
    # (MIT-SHM), which only works when the container shares the host's IPC namespace.
    XA="${XAUTHORITY:-$HOME/.Xauthority}"
    XARGS=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
    [ -f "$XA" ] && XARGS+=(-e XAUTHORITY=/tmp/.Xauthority -v "$XA":/tmp/.Xauthority:ro)
    docker run --rm -it --name px4-gazebo-vnc-screen --ipc=host -e NO_VNC=1 "${XARGS[@]}" \
      -v "$REPO":/workspace "$IMAGE" python3 /workspace/recording/screen.py record "/workspace/$RUNS/$RUN" "${ARGS[@]}"
    ;;

  screen-cut)
    split_run "$@"
    RUN=$(run_dir "$RUN")
    C=$(container); [ "$C" = "$TEMP" ] && trap stop_temp EXIT
    docker exec "$C" python3 /workspace/recording/screen.py cut "/workspace/$RUNS/$RUN" "${ARGS[@]}"
    ;;

  compose)
    split_run "$@"
    RUN=$(run_dir "$RUN")
    if ! printf '%s\n' "${ARGS[@]}" | grep -qx -- '--out'; then ARGS+=(--out demo.mp4); fi
    C=$(container); [ "$C" = "$TEMP" ] && trap stop_temp EXIT
    docker exec -w "/workspace/$RUNS/$RUN" "$C" python3 /workspace/recording/compose.py "${ARGS[@]}"
    ;;

  compress)
    [ $# -ge 2 ] || { usage; exit 1; }
    RUN=$(run_dir "${RUN_NAME:-}")
    C=$(container); [ "$C" = "$TEMP" ] && trap stop_temp EXIT
    docker exec -w "/workspace/$RUNS/$RUN" "$C" bash /workspace/recording/compress_to_size.sh "$@"
    ;;

  help|-h|--help) usage ;;
  *) echo "Unknown subcommand: $cmd"; usage; exit 1 ;;
esac
