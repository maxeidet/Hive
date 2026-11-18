#!/usr/bin/env bash
# Scenario "birds": exactly the mpc scenario (mpc.sh: scenario 2 with YOLO, ByteTrack,
# --lock and the MPC intercept), but birds fly in with the two hostile drones. Every kind in
# worlds/birds/ (dove, mourning dove, flock, seagull, gull, eagle, soaring bird) is used
# before any repeats. Each bird starts at a random point 5-25 m (horizontally) and up to 8 m
# (vertically) from one of the hostiles' start points, and flies level and straight over
# the base and 60 m past it, at the hostiles' speed +-30 %. Birds have no collisions and are
# removed when the run ends. They test whether the detector and the lock stay on the
# drones.
#
# Start the container and drone 0 in the hive_base world first, then:
#   scenarios/birds.sh                         ten birds
#   scenarios/birds.sh --birds 25              25 birds
#   scenarios/birds.sh --bird-seed 3           the same start points every time
#   scenarios/birds.sh --no-attack --no-view   same options as scenarios/yolo.sh
#
# Single birds by hand: ./mount-backend.sh bird --help
set -e

if [[ " $* " =~ " -h " || " $* " =~ " --help " ]]; then
  grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'
  exit 0
fi
# --birds 10 first: a --birds given later on the command line wins.
exec "$(dirname "$0")/mpc.sh" --birds 10 "$@"
