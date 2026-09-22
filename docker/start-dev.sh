#!/usr/bin/env bash
# Bygger och startar Gazebo-VNC-containern, och öppnar automatiskt ett nytt
# Terminal-fönster som startar PX4 + Gazebo (drönare 0) däri.
#
# Använd terminal.app (macOS standard).
#
# Användning:
#   ./start-dev.sh                        # standarddrönare (gz_x500)
#   ./start-dev.sh -m gz_x500_depth        # drönare med djupkamera
#   ./start-dev.sh --model gz_x500_lidar_front
#
# Se README.md i denna mapp för alla giltiga PX4_SIM_MODEL-värden.
set -e

MODEL="gz_x500"

while [ $# -gt 0 ]; do
  case "$1" in
    -m|--model)
      MODEL="$2"
      shift 2
      ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^#//'
      exit 0
      ;;
    *)
      echo "Okänd flagga: $1"
      exit 1
      ;;
  esac
done

cd "$(dirname "$0")"

echo "Bygger image (hive-px4-vnc)..."
docker build -t hive-px4-vnc .

docker rm -f px4-gazebo-vnc >/dev/null 2>&1 || true

echo "Öppnar en ny terminal som väntar på containern och startar PX4 (modell: ${MODEL})..."
osascript <<APPLESCRIPT
tell application "Terminal"
    activate
    do script "until docker inspect -f '{{.State.Running}}' px4-gazebo-vnc 2>/dev/null | grep -q true; do sleep 1; done; docker exec -it px4-gazebo-vnc bash -c 'export PX4_SIM_MODEL=${MODEL} PX4_GZ_MODEL_POSE=0,0,0,0,0,0; /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0'"
end tell
APPLESCRIPT

echo "Startar containern (Ctrl+C här stänger ned allt)..."
docker run --rm -it --name px4-gazebo-vnc \
  -p 6080:6080 -p 5900:5900 \
  -v "$(pwd)/..":/workspace \
  hive-px4-vnc MicroXRCEAgent udp4 -p 8888
