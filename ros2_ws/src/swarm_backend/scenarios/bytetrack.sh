#!/usr/bin/env bash
# Scenario "bytetrack": exactly scenario 2 (yolo.sh, scenario2.yaml), but the YOLO
# detector on drone 1 tracks with ByteTrack (detect/live.py --tracker bytetrack) instead of
# the default k-in-a-row Confirmer, and locks on one drone (--lock, TargetLock in
# detect/live.py): only the locked drone is published, also when the other hostile scores
# higher. When ByteTrack gives the locked drone a new id (a new track where the lost one
# should be, within 4 deg + 5 deg/s unseen), the lock is handed over to it at once; unseen
# for --lock-timeout (1 s), the lock goes to the best target. YOLO's --conf is lowered to
# ByteTrack's --low (0.1), so the weak detections reach the tracker; the other thresholds
# (high 0.3, new 0.4, from the 11 Nov threshold replay) are ByteTrack's defaults for YOLO
# (TRACK_SCORES in detect/live.py).
#
# Start the container and drone 0 in the hive_base world first, then:
#   scenarios/bytetrack.sh                     same options as scenarios/yolo.sh
#   scenarios/bytetrack.sh --speed 8 --no-view
#   scenarios/bytetrack.sh --detect-args --high 0.4 --lock-timeout 0.5   more detector options
set -e

if [[ " $* " =~ " -h " || " $* " =~ " --help " ]]; then
  grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'
  exit 0
fi
SCENARIO2="$(dirname "$0")/yolo.sh"
# --detect-args takes everything after it, so the tracker goes at the very end.
if [[ " $* " =~ " --detect-args " ]]; then
  exec "$SCENARIO2" "$@" --tracker bytetrack --lock
else
  exec "$SCENARIO2" "$@" --detect-args --tracker bytetrack --lock
fi
