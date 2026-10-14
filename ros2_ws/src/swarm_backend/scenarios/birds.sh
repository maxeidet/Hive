#!/usr/bin/env bash
# Birds scenario: three birds (dove, mourning dove, a flock) a few metres around drone 0,
# to look at them next to a drone (0.5 m across) and to try the detector on birds.
# Gazebo must be running with drone 0 (docker/start-dev.sh, or ./mount-backend.sh restart).
#
#   scenarios/birds.sh            spawn the three birds around drone 0
#   scenarios/birds.sh --wander   the same, then fly the dove around randomly (Ctrl-C stops)
#   scenarios/birds.sh --remove   take them out again
#
# Positions are relative to drone 0: metres north, east and up. Birds have no physics, so a
# drone flies through them. Single birds: ./mount-backend.sh bird --help
set -e
BACKEND="$(cd "$(dirname "$0")/.." && pwd)/mount-backend.sh"
NEAR=0

case "${1:-}" in
  -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
  --remove)
    for name in bird_dove bird_mourning_dove bird_flock; do "$BACKEND" bird remove "$name" || true; done
    exit 0 ;;
esac

"$BACKEND" bird spawn dove --name bird_dove --near $NEAR --at 3 0 1.5 --heading 90
"$BACKEND" bird spawn mourning_dove --name bird_mourning_dove --near $NEAR --at 0 3 1.5 --heading 0
"$BACKEND" bird spawn flock --name bird_flock --near $NEAR --at -4 -2 2 --heading 45

if [ "${1:-}" = "--wander" ]; then
  # Around a point 6 m north of drone 0, 3 m up, in a 10 m wide area, slowly.
  "$BACKEND" bird wander bird_dove --near $NEAR --around 6 0 3 --speed 2 --size 10
fi
