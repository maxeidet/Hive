#!/usr/bin/env bash
# Scenario "mpc": exactly the bytetrack scenario (bytetrack.sh: scenario 2 with YOLO,
# ByteTrack and --lock), but the defender attacks with the MPC intercept
# (intercept/mpc_action.py, tests/scenario2_test.py --guidance mpc) instead of pure pursuit.
# On the same detection topics, a Kalman filter estimates the hostile's position and
# velocity; the defender flies at attack.max_speed toward the predicted meeting point and
# the MPC steers the last 5 s. The gimbal is aimed at the estimate. Lost (no report for
# intercept.track_timeout, 3 s) it hovers where it is; after a hit it flies back to the
# guard point above the base for the next hostile.
#
# Start the container and drone 0 in the hive_base world first, then:
#   scenarios/mpc.sh                           same options as scenarios/yolo.sh
#   scenarios/mpc.sh --attack-speed 8 --no-view
#   scenarios/mpc.sh --detect-args --max-lost 10   more ByteTrack options
set -e

if [[ " $* " =~ " -h " || " $* " =~ " --help " ]]; then
  grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'
  exit 0
fi
# --guidance first: bytetrack.sh puts --detect-args (which takes the rest) at the end.
exec "$(dirname "$0")/bytetrack.sh" --guidance mpc "$@"
