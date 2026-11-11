#!/usr/bin/env bash
# Compare how drone 4 flies in, in one command. For each path
#   straight       the planning report's straight line
#   weave ±5 m     a sine side to side, 60 m per swing
#   weave ±15 m
#   zigzag ±15 m   straight legs with sharp turns
# and each altitude of drone 4: 10 m below the defenders, level, 10 m above (the defenders
# fly at surveillance.altitude in scenarios/surveillance.yaml, 40 m: drone 4 at 30, 40, 50 m),
# N runs (default 3, seeds 0..N-1) at 4 m/s: 4 x 3 x 3 = 36 runs by default. Each run
# drone 4 starts at a seeded random bearing inside drone 2's sector (the middle 30 degrees
# of the arc) and flies at the base. A seed gives the same start bearing at every altitude
# and on every path, so the paths and altitudes are compared on the same starts, and the
# same seeds give the same test again. The paths are flown one after the other (the
# defenders land after each), then a comparison per path and altitude is printed: hits,
# the median closest pass, and the first detection: how far (3-D, the space diagonal) the
# first defender that confirmed drone 4 was from it.
#
# Everything is saved in data/path_compare/<date-time>/<path>/ in the repo (intercept.jsonl,
# summary.txt per path) and the comparison in data/path_compare/<date-time>/compare.txt.
#
# Start the container and drone 0 first (docker/start-stellar.sh or start-dev.sh), then:
#   scenarios/compare_paths.sh                    3 runs per path and altitude, 36 in all
#   scenarios/compare_paths.sh 5                  5 runs each (seeds 0-4), 60 in all
#   scenarios/compare_paths.sh 3 --seeds 10-12    other start bearings
#   scenarios/compare_paths.sh 3 --altitude-offsets 0   other settings: baseline_eval's own arguments
#   scenarios/compare_paths.sh 3 --sim            the fast simulation, no Gazebo (minutes)
#
# Run it in tmux: in Gazebo 36 runs take about three hours. Ctrl-C stops the path that is
# flying (its defenders land) and skips the rest; the comparison is printed for what was
# flown. A path that fails (e.g. a drone that does not start) is marked in the comparison
# and the next one is flown.

case "${1:-}" in
  -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
esac

RUNS=3
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then
  RUNS=$1
  shift
fi
if [ "$RUNS" -lt 1 ]; then
  echo "The number of runs must be at least 1."
  exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
STAMP=$(date +%Y-%m-%d_%H%M%S)
OUT=/workspace/data/path_compare/$STAMP   # as the container sees the repo
HOST_OUT=$REPO/data/path_compare/$STAMP

# label | baseline_eval arguments
PATHS=(
  "straight|--path straight"
  "weave_5m|--path weave --path-amplitude 5"
  "weave_15m|--path weave --path-amplitude 15"
  "zigzag_15m|--path zigzag --path-amplitude 15"
)

FAILED=()
STOP=0
# Ctrl-C reaches the scenario, which lands the defenders; this script then skips the
# paths still to come and prints the comparison of what was flown.
trap 'STOP=1' INT
for entry in "${PATHS[@]}"; do
  label=${entry%%|*}
  read -r -a path_args <<< "${entry#*|}"
  echo
  echo "=== $label: $RUNS runs per altitude ==============================================="
  # Settings first, the caller's after them (argparse keeps the last value given).
  if ! "$HERE/baseline_eval.sh" intercept --aspects 0 --speeds 4 --seeds "0-$((RUNS - 1))" \
      --altitude-offsets -10 0 10 --sector 2 \
      "${path_args[@]}" --path-wavelength 60 --out "$OUT/$label" "$@"; then
    FAILED+=("$label")
  fi
  if [ "$STOP" = 1 ]; then
    echo "Ctrl-C: stopped after $label; the paths after it are not flown."
    break
  fi
done
trap - INT
mkdir -p "$HOST_OUT"

echo
echo "=== Comparison ======================================================================"
python3 - "$HOST_OUT" "$RUNS" "${FAILED[@]}" <<'EOF' | tee "$HOST_OUT/compare.txt"
import json, statistics, sys
from pathlib import Path

out, runs, failed = Path(sys.argv[1]), int(sys.argv[2]), set(sys.argv[3:])
names = {'straight': 'straight', 'weave_5m': 'weave ±5 m', 'weave_15m': 'weave ±15 m',
         'zigzag_15m': 'zigzag ±15 m'}


def median(values, unit=' m'):
    values = [v for v in values if v is not None]
    return f'{statistics.median(values):.1f}{unit}' if values else '-'


print(f'{"Path":<14} {"Alt":>4} {"Hits":>6} {"Success":>8} {"Closest":>8} {"t_hit":>7} '
      f'{"Seen":>6} {"Detect 3-D":>10} {"horiz.":>8}  outcomes')
print('-' * 96)
for label, name in names.items():
    path = out / label / 'intercept.jsonl'
    rows = [json.loads(line) for line in open(path) if line.strip()] if path.exists() else []
    if not rows:
        print(f'{name:<14} {"":>4} {"-":>6} {"-":>8} {"-":>8} {"-":>7} {"-":>6} {"-":>10} '
              f'{"-":>8}  ' + ('failed' if label in failed else 'not flown'))
        continue
    altitudes = sorted({r.get('hostile_altitude') for r in rows}, key=lambda a: a or 0)
    for altitude in altitudes:
        group = [r for r in rows if r.get('hostile_altitude') == altitude]
        n = len(group)
        hits = sum(r['outcome'] == 'hit' for r in group)
        success = sum(bool(r['success']) for r in group)
        t_hit = [r['time_to_hit'] for r in group if r['time_to_hit'] is not None]
        t = f'{statistics.mean(t_hit):.1f} s' if t_hit else '-'
        seen = [r for r in group if r.get('detect_distance') is not None]
        outcomes = {}
        for r in group:
            outcomes[r['outcome']] = outcomes.get(r['outcome'], 0) + 1
        note = ' (stopped early)' if n < runs else ''
        note += ' (failed)' if label in failed else ''
        alt = '-' if altitude is None else f'{altitude:.0f}'
        print(f'{name:<14} {alt:>4} {hits:>3}/{n:<2} {success:>4}/{n:<3} '
              f'{median([r["closest_miss"] for r in group]):>8} {t:>7} '
              f'{len(seen):>3}/{n:<2} {median([r["detect_distance"] for r in seen]):>10} '
              f'{median([r["detect_horizontal"] for r in seen]):>8}  '
              + ', '.join(f'{k} {v}' for k, v in outcomes.items()) + note)
        name = ''  # the path's name only on its first line
print()
defenders = set()
for label in names:
    path = out / label / 'intercept.jsonl'
    if path.exists():
        defenders |= {json.loads(line).get('defender_altitude') for line in open(path)
                      if line.strip()}
defenders = sorted(defenders - {None})
flying = ', '.join(f'{a:.0f}' for a in defenders) or '?'
print(f'Alt: drone 4\'s altitude in m (the defenders fly at {flying} m). Hits: within 1.5 m (true')
print('positions). Success: a hit more than 100 m from the base. Closest: median over the runs')
print('of the closest the defender came. t_hit: mean time from the first detection to the hit,')
print('over the runs that hit. Seen: runs where a defender\'s detector confirmed drone 4.')
print('Detect 3-D: median true distance (space diagonal) from the first defender that confirmed')
print('drone 4 to it, at that image; horiz.: the same on the ground.')
EOF
echo
echo "Saved in ${HOST_OUT#$REPO/} (repo folder): one folder per path, compare.txt"
