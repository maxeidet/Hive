#!/usr/bin/env bash
# Start a Gazebo world with no drones, INSIDE the container (it is at /workspace/docker/).
# The same environment and gz commands as px4-gazebo and PX4's px4-rc.gzsim, so PX4 drones
# started later (mount-backend.sh spawn, the scenarios) attach to the running world.
#
#   /workspace/docker/gz-world.sh WORLD        HEADLESS=1 in the environment: no GUI window
#
# Gazebo runs detached (its own session), with its output in /tmp/gz_server.log, so the
# caller can return. Used by start-stellar.sh --no-drone (which then follows the log, so
# the container outlives a Gazebo restart) and by `mount-backend.sh restart` in a no-drone
# session. Waits until the world's clock runs; exits 1 if it does not within
# WORLD_TIMEOUT seconds (default 300). The world and mode go to /tmp/hive_sim.env, where
# `restart` reads them.
set -e

WORLD="${1:?usage: gz-world.sh WORLD}"
WORLD_TIMEOUT="${WORLD_TIMEOUT:-300}"
G=/opt/px4-gazebo
LOG=/tmp/gz_server.log

cp /workspace/worlds/*.sdf "$G/share/gz/worlds/" 2>/dev/null || true
/workspace/docker/drone-models.sh  # our models, and friendly/hostile collision groups
if [ ! -f "$G/share/gz/worlds/$WORLD.sdf" ]; then
  echo "No world $WORLD ($G/share/gz/worlds/$WORLD.sdf)."
  exit 1
fi

export PX4_GZ_WORLD="$WORLD"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$G/share/gz/models:$G/share/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$G/lib/gz/plugins"
export GZ_SIM_SERVER_CONFIG_PATH="$G/share/gz/server.config"

# As px4-gazebo: gz looks for the unversioned DART physics plugin, which only -dev ships.
d=$(find /usr/lib -maxdepth 3 -type d -name engine-plugins -path '*/gz-physics-7/*' 2>/dev/null | head -1)
if [ -n "$d" ] && [ ! -e "$d/libgz-physics-dartsim-plugin.so" ]; then
  v=$(ls "$d"/libgz-physics*-dartsim-plugin.so.* 2>/dev/null | head -1)
  if [ -n "$v" ]; then ln -sf "$(basename "$v")" "$d/libgz-physics-dartsim-plugin.so"; fi
fi

{
  echo "PX4_GZ_WORLD=$WORLD"
  echo "NO_DRONE=1"
  if [ -n "${HEADLESS:-}" ]; then echo "HEADLESS=1"; fi
} > /tmp/hive_sim.env

source /opt/ros/jazzy/setup.bash
echo "=== Gazebo $WORLD, started $(date +%H:%M:%S) ===" > "$LOG"
setsid nohup gz sim --verbose=1 -r -s "$G/share/gz/worlds/$WORLD.sdf" >> "$LOG" 2>&1 < /dev/null &
if [ -z "${HEADLESS:-}" ]; then
  setsid nohup gz sim -g > /tmp/gz_gui.log 2>&1 < /dev/null &
fi

echo "Waiting for the Gazebo world $WORLD (heavy worlds take a minute)..."
for _ in $(seq "$WORLD_TIMEOUT"); do
  if gz topic -l 2>/dev/null | grep -q "^/world/$WORLD/clock\$"; then
    echo "Gazebo world $WORLD is up, with no drones."
    exit 0
  fi
  sleep 1
done
echo "Gazebo world $WORLD did not come up within $WORLD_TIMEOUT s (log: $LOG in the container)."
exit 1
