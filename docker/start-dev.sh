#!/usr/bin/env bash
# LOCAL DEV ON macOS. Builds and starts the Gazebo-VNC container, and
# automatically opens a new Terminal.app window that starts PX4 + Gazebo
# (drone 0) inside it. On the Stellar (Ubuntu) computers use start-stellar.sh.
#
# Usage:
#   ./start-dev.sh                        # default drone (gz_x500) in baylands
#   ./start-dev.sh -m gz_x500_depth        # drone with a depth camera
#   ./start-dev.sh --model gz_x500_lidar_front
#   ./start-dev.sh -w forest               # different bundled world
#   ./start-dev.sh -m gz_x500_depth -w ridge
#   ./start-dev.sh --headless              # Gazebo without its GUI window: faster (the
#                                          # cameras still render, offscreen); RViz and
#                                          # VNC still work
#
# Host ports are bound to 127.0.0.1 only and protected by a random VNC password
# that is printed below. They can be overridden if a port is taken:
#   HOST_NOVNC_PORT=6081 HOST_VNC_PORT=5901 ./start-dev.sh
#
# PX4 gives up if the Gazebo world is not ready within WORLD_TIMEOUT checks
# (about one per second, default 180). Raise it for heavy worlds:
#   WORLD_TIMEOUT=300 ./start-dev.sh -w hive_base
#
# See README.md in this folder for all valid PX4_SIM_MODEL and world values.
set -e

MODEL="gz_x500"
WORLD="baylands"
HOST_NOVNC_PORT="${HOST_NOVNC_PORT:-6080}"
HOST_VNC_PORT="${HOST_VNC_PORT:-5900}"
HOST_UI_PORT="${HOST_UI_PORT:-8090}"  # operator UI (mount-backend.sh ui)
WORLD_TIMEOUT="${WORLD_TIMEOUT:-180}"
HEADLESS_ENV=""

while [ $# -gt 0 ]; do
  case "$1" in
    --headless)
      HEADLESS_ENV="HEADLESS=1 "
      shift
      ;;
    -m|--model)
      MODEL="$2"
      shift 2
      ;;
    -w|--world)
      WORLD="$2"
      shift 2
      ;;
    -h|--help)
      grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "Unknown flag: $1"
      exit 1
      ;;
  esac
done

if [ "$(uname -s)" != "Darwin" ]; then
  echo "start-dev.sh is for macOS (it opens Terminal.app). On Stellar/Ubuntu use start-stellar.sh."
  exit 1
fi

cd "$(dirname "$0")"

echo "Building image (hive-px4-vnc)..."
docker build -t hive-px4-vnc .

docker rm -f px4-gazebo-vnc >/dev/null 2>&1 || true

# Classic VNC auth only uses the first 8 characters. Exported and passed with
# `-e VNC_PASSWORD` (no value) so it never shows up in the process list.
# Random unless you set your own: VNC_PASSWORD=mypass ./start-dev.sh
export VNC_PASSWORD="${VNC_PASSWORD:-$(LC_ALL=C tr -dc 'A-Za-z0-9' </dev/urandom | head -c 8)}"

PX4_CMD="until docker inspect -f '{{.State.Running}}' px4-gazebo-vnc 2>/dev/null | grep -q true; do sleep 1; done; docker exec -it px4-gazebo-vnc bash -c 'cp /workspace/worlds/*.sdf /opt/px4-gazebo/share/gz/worlds/ 2>/dev/null || true; export ${HEADLESS_ENV}PX4_GZ_WORLD_TIMEOUT=${WORLD_TIMEOUT} PX4_GZ_WORLD=${WORLD} PX4_SIM_MODEL=${MODEL} PX4_GZ_MODEL_POSE=0,0,0,0,0,0; /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0'"

echo "Opening a new terminal that waits for the container and starts PX4 (model: ${MODEL}, world: ${WORLD})..."
osascript <<APPLESCRIPT
tell application "Terminal"
    activate
    do script "${PX4_CMD}"
end tell
APPLESCRIPT

echo ""
echo "  noVNC (browser):  http://localhost:${HOST_NOVNC_PORT}/vnc.html"
echo "  VNC client:       vnc://127.0.0.1:${HOST_VNC_PORT}"
echo "  VNC password:     ${VNC_PASSWORD}"
echo "  Operator UI:      http://localhost:${HOST_UI_PORT} (after mount-backend.sh ui)"
echo ""
echo "Starting the container (Ctrl+C here shuts everything down)..."
docker run --rm -it --name px4-gazebo-vnc \
  -p "127.0.0.1:${HOST_NOVNC_PORT}:6080" -p "127.0.0.1:${HOST_VNC_PORT}:5900" \
  -p "127.0.0.1:${HOST_UI_PORT}:8090" \
  -e VNC_PASSWORD \
  -v "$(pwd)/..":/workspace \
  hive-px4-vnc MicroXRCEAgent udp4 -p 8888
