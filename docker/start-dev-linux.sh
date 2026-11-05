#!/usr/bin/env bash
# LOCAL DEV ON LINUX (and WSL). Builds and starts the Gazebo-VNC container and opens a
# second terminal that starts PX4 + Gazebo (drone 0) inside it.
#
# Usage:
#   ./start-dev-linux.sh                  # gz_x500 in hive_base
#   ./start-dev-linux.sh -m gz_x500_gimbal -w hive_base
#   ./start-dev-linux.sh --headless       # Gazebo without its GUI window: faster (the
#                                         # cameras still render, offscreen)
set -e

WORLD_TIMEOUT="${WORLD_TIMEOUT:-180}"
MODEL="gz_x500"
WORLD="hive_base"
HEADLESS=""

# ============================================================
# PX4 terminal mode
# ============================================================

if [[ "${1:-}" == "--px4-terminal" ]]; then
    shift

    MODEL="${1:-gz_x500}"
    WORLD="${2:-hive_base}"
    HEADLESS="${3:-}"

    echo "Waiting for PX4 container..."

    until docker inspect -f '{{.State.Running}}' px4-gazebo-vnc 2>/dev/null | grep -q true; do
        sleep 1
    done

    echo "Container is running."
    echo "Starting PX4 + Gazebo"
    echo "  Model: ${MODEL}"
    echo "  World: ${WORLD}"
    echo

    docker exec -it px4-gazebo-vnc bash -c "
        echo '=== Copying custom worlds ==='
        cp /workspace/worlds/*.sdf /opt/px4-gazebo/share/gz/worlds/ 2>/dev/null || true

        echo '=== Available worlds ==='
        ls -lah /opt/px4-gazebo/share/gz/worlds/

        export PX4_GZ_WORLDS=/opt/px4-gazebo/share/gz/worlds
        export GZ_SIM_RESOURCE_PATH=/opt/px4-gazebo/share/gz/models:/opt/px4-gazebo/share/gz/worlds
        export PX4_GZ_WORLD='${WORLD}'
        export PX4_SIM_MODEL='${MODEL}'
        export PX4_GZ_WORLD_TIMEOUT='${WORLD_TIMEOUT}'
        export PX4_GZ_MODEL_POSE=0,5,0,0,0,0
        if [ -n '${HEADLESS}' ]; then export HEADLESS=1; fi

        echo
        echo 'PX4_GZ_WORLD='\${PX4_GZ_WORLD}
        echo 'PX4_SIM_MODEL='\${PX4_SIM_MODEL}
        echo

        /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0
    "

    exit 0
fi

# ============================================================
# Argument parsing
# ============================================================

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

        --headless)
            HEADLESS=1
            shift
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

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "Building image (hive-px4-vnc)..."
docker build -t hive-px4-vnc .

docker rm -f px4-gazebo-vnc >/dev/null 2>&1 || true

# ============================================================
# Open second terminal
# ============================================================

echo
echo "Opening PX4 terminal..."
echo "Model: ${MODEL}"
echo "World: ${WORLD}"
echo

SCRIPT_PATH="$(realpath "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"

if [[ -n "${WSL_DISTRO_NAME:-}" ]] && command -v wt.exe >/dev/null 2>&1; then

    echo "Detected WSL → Windows Terminal"

    wt.exe new-tab \
        --title "PX4 + Gazebo" \
        wsl.exe -d "$WSL_DISTRO_NAME" \
        -- bash "$SCRIPT_PATH" --px4-terminal "$MODEL" "$WORLD" "$HEADLESS"

elif command -v gnome-terminal >/dev/null 2>&1; then

    gnome-terminal \
        --title="PX4 + Gazebo" \
        -- bash "$SCRIPT_PATH" --px4-terminal "$MODEL" "$WORLD" "$HEADLESS"

elif command -v konsole >/dev/null 2>&1; then

    konsole \
        --new-tab \
        -e bash "$SCRIPT_PATH" --px4-terminal "$MODEL" "$WORLD" "$HEADLESS"

elif command -v xfce4-terminal >/dev/null 2>&1; then

    xfce4-terminal \
        --title="PX4 + Gazebo" \
        --command="bash $SCRIPT_PATH --px4-terminal $MODEL $WORLD $HEADLESS"

else
    echo "ERROR: Could not find a supported terminal emulator."
    echo
    echo "Start the container, then run:"
    echo
    echo "  $SCRIPT_PATH --px4-terminal $MODEL $WORLD $HEADLESS"
    exit 1
fi

# ============================================================
# Start Docker container
# ============================================================

echo
echo "Starting the container..."
echo "Ctrl+C here shuts everything down."
echo

docker run --rm -it \
    --name px4-gazebo-vnc \
    -p 6080:6080 \
    -p 5900:5900 \
    -v "$(pwd)/..":/workspace \
    hive-px4-vnc \
    MicroXRCEAgent udp4 -p 8888