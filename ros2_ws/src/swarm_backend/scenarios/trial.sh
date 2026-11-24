#!/usr/bin/env bash
# A short intercept trial in one command: flies N runs of the baseline evaluation in Gazebo
# (default 3: seeds 0..N-1, head-on at 4 m/s), the defenders land at the end, then the
# metrics of exactly these runs are printed (the same table as baseline_eval.sh summary).
#
# Each trial saves into a folder of its own, data/trials/<date-time>/ in the repo, so runs
# from before never mix in, and nothing is skipped as "already saved".
#
# Start the container and drone 0 first (docker/start-stellar.sh or start-dev.sh), then:
#   scenarios/trial.sh                       3 runs
#   scenarios/trial.sh 5                     5 runs
#   scenarios/trial.sh 3 --speeds 8          other settings: baseline_eval's own arguments
#   scenarios/trial.sh 3 --aspects 30 --seeds 10-12
#   scenarios/trial.sh 3 --path weave        drone 4 swings side to side (sine, ±15 m)
#   scenarios/trial.sh 3 --path zigzag       straight legs with sharp turns
#   scenarios/trial.sh 3 --path weave --path-amplitude 5 --path-wavelength 40
#   scenarios/trial.sh 3 --birds 8          8 birds fly in around drone 4 (Gazebo only)
#   scenarios/trial.sh 3 --sim               the fast simulation, no Gazebo
#
# Run it in tmux: 3 runs in Gazebo take about 10 minutes. Ctrl-C lands the defenders; the
# runs flown so far are in the folder and their metrics are printed.
set -e

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

HERE="$(dirname "${BASH_SOURCE[0]}")"
# The folder as the container sees the repo (it is mounted at /workspace).
OUT=/workspace/data/trials/$(date +%Y-%m-%d_%H%M%S)

# Settings first, the caller's after them: argparse keeps the last value given, so
# `--speeds 8` or `--seeds 10-12` replace these.
"$HERE/baseline_eval.sh" intercept --aspects 0 --speeds 4 --seeds "0-$((RUNS - 1))" \
  --out "$OUT" "$@"

echo
echo "Saved in ${OUT#/workspace/} (repo folder): intercept.jsonl, summary.txt"
