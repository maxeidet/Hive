#!/usr/bin/env bash
# Builds and starts the Gazebo-VNC container, and automatically opens a new
# Terminal window that starts PX4 + Gazebo (drone 0) inside it.
#
# Uses Terminal.app (macOS default).
#
# Usage:
#   ./start-dev.sh                        # default drone (gz_x500) in baylands
#   ./start-dev.sh -m gz_x500_depth        # drone with a depth camera
#   ./start-dev.sh --model gz_x500_lidar_front
#   ./start-dev.sh -w forest               # different bundled world
#   ./start-dev.sh -m gz_x500_depth -w ridge
#
# See README.md in this folder for all valid PX4_SIM_MODEL and world values.
set -e

MODEL="gz_x500"
<<<<<<< Updated upstream
WORLD="baylands"
=======
WORLD="default"
>>>>>>> Stashed changes

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
      grep '^#' "$0" | sed 's/^#//'
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

echo "Opening a new terminal that waits for the container and starts PX4 (model: ${MODEL}, world: ${WORLD})..."
osascript <<APPLESCRIPT
tell application "Terminal"
    activate
<<<<<<< Updated upstream
    do script "until docker inspect -f '{{.State.Running}}' px4-gazebo-vnc 2>/dev/null | grep -q true; do sleep 1; done; docker exec -it px4-gazebo-vnc bash -c 'export PX4_SIM_MODEL=${MODEL} PX4_GZ_WORLD=${WORLD} PX4_GZ_MODEL_POSE=0,0,0,0,0,0; /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0'"
=======
    do script "until docker inspect -f '{{.State.Running}}' px4-gazebo-vnc 2>/dev/null | grep -q true; do sleep 1; done; docker exec -it px4-gazebo-vnc bash -c 'cp /workspace/worlds/*.sdf /opt/px4-gazebo/share/gz/worlds/ 2>/dev/null || true; export PX4_GZ_WORLD=${WORLD} PX4_SIM_MODEL=${MODEL} PX4_GZ_MODEL_POSE=0,0,0,0,0,0; /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0'"
>>>>>>> Stashed changes
end tell
APPLESCRIPT

echo "Starting the container (Ctrl+C here shuts everything down)..."
docker run --rm -it --name px4-gazebo-vnc \
  -p 6080:6080 -p 5900:5900 \
  -v "$(pwd)/..":/workspace \
  hive-px4-vnc MicroXRCEAgent udp4 -p 8888
