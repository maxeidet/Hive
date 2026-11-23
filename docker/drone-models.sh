#!/usr/bin/env bash
# Install the drone models into the container's Gazebo, INSIDE the container (it is at
# /workspace/docker/). Run before drones are spawned: by docker/gz-world.sh and by
# mount-backend.sh (copy_models, before every spawn). Running it again is harmless.
#   - our own models (worlds/models/) copied in, as before
#   - friendly and hostile drones in collision groups that pass through each other, and the
#     x500_hostile model for the hostile drone (docker/drone_collisions.py)
set -e
M=/opt/px4-gazebo/share/gz/models
cp -r /workspace/worlds/models/. "$M/" 2>/dev/null || true
python3 /workspace/docker/drone_collisions.py "$M" > /tmp/drone_models.log
