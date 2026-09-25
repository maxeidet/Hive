#!/usr/bin/env bash
# Builds and starts the Gazebo-VNC container, and automatically opens a new
# terminal window that starts PX4 + Gazebo (drone 0) inside it.
#
# Second terminal: Terminal.app on macOS, gnome-terminal / x-terminal-emulator
# on Linux. If none is found, the command to run by hand is printed instead.
#
# Usage:
#   ./start-dev.sh                        # default drone (gz_x500) in baylands
#   ./start-dev.sh -m gz_x500_depth        # drone with a depth camera
#   ./start-dev.sh --model gz_x500_lidar_front
#   ./start-dev.sh -w forest               # different bundled world
#   ./start-dev.sh -m gz_x500_depth -w ridge
#
# Host ports are bound to 127.0.0.1 only. On shared machines (Stellar) another
# user may already hold 6080/5900, so they can be overridden:
#   HOST_NOVNC_PORT=6081 HOST_VNC_PORT=5901 ./start-dev.sh
#
# A random VNC password is generated on every run and printed below.
#
# See README.md in this folder for all valid PX4_SIM_MODEL and world values.
set -e

MODEL="gz_x500"
WORLD="baylands"
HOST_NOVNC_PORT="${HOST_NOVNC_PORT:-6080}"
HOST_VNC_PORT="${HOST_VNC_PORT:-5900}"

while [ $# -gt 0 ]; do
  case "$1" in
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

cd "$(dirname "$0")"

echo "Building image (hive-px4-vnc)..."
docker build -t hive-px4-vnc .

docker rm -f px4-gazebo-vnc >/dev/null 2>&1 || true

# Classic VNC auth only uses the first 8 characters. Exported and passed with
# `-e VNC_PASSWORD` (no value) so it never shows up in the process list.
export VNC_PASSWORD="$(LC_ALL=C tr -dc 'A-Za-z0-9' </dev/urandom | head -c 8)"

PX4_CMD="until docker inspect -f '{{.State.Running}}' px4-gazebo-vnc 2>/dev/null | grep -q true; do sleep 1; done; docker exec -it px4-gazebo-vnc bash -c 'cp /workspace/worlds/*.sdf /opt/px4-gazebo/share/gz/worlds/ 2>/dev/null || true; export PX4_GZ_WORLD=${WORLD} PX4_SIM_MODEL=${MODEL} PX4_GZ_MODEL_POSE=0,0,0,0,0,0; /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0'"

echo "Opening a new terminal that waits for the container and starts PX4 (model: ${MODEL}, world: ${WORLD})..."
case "$(uname -s)" in
  Darwin)
    osascript <<APPLESCRIPT
tell application "Terminal"
    activate
    do script "${PX4_CMD}"
end tell
APPLESCRIPT
    ;;
  Linux)
    if command -v gnome-terminal >/dev/null 2>&1; then
      gnome-terminal -- bash -c "${PX4_CMD}; exec bash"
    elif command -v x-terminal-emulator >/dev/null 2>&1; then
      x-terminal-emulator -e bash -c "${PX4_CMD}; exec bash"
    else
      echo "No terminal emulator found. Run this in a second terminal:"
      echo "${PX4_CMD}"
    fi
    ;;
  *)
    echo "Unsupported OS. Run this in a second terminal:"
    echo "${PX4_CMD}"
    ;;
esac

echo ""
echo "  noVNC (browser):  http://localhost:${HOST_NOVNC_PORT}/vnc.html"
echo "  VNC client:       vnc://127.0.0.1:${HOST_VNC_PORT}"
echo "  VNC password:     ${VNC_PASSWORD}"
echo ""
echo "Starting the container (Ctrl+C here shuts everything down)..."
docker run --rm -it --name px4-gazebo-vnc \
  -p "127.0.0.1:${HOST_NOVNC_PORT}:6080" -p "127.0.0.1:${HOST_VNC_PORT}:5900" \
  -e VNC_PASSWORD \
  -v "$(pwd)/..":/workspace \
  hive-px4-vnc MicroXRCEAgent udp4 -p 8888
