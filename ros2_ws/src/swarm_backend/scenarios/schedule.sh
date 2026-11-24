#!/usr/bin/env bash
# Fly a schedule of intercept tests from a YAML file in scenarios/schedules/ (or any path):
# steps of paths and altitudes, each with its own number of runs. See schedule.py.
#
#   scenarios/schedule.sh paths_altitudes             fly scenarios/schedules/paths_altitudes.yaml
#   scenarios/schedule.sh paths_altitudes --dry-run   only print what it would fly
#   scenarios/schedule.sh paths_altitudes --resume data/schedules/<folder>   go on after a stop
#   scenarios/schedule.sh --help                      everything, with the file format
#
# Start the container first (docker/start-stellar.sh or the dev GUI). Run it in tmux.
exec python3 "$(dirname "${BASH_SOURCE[0]}")/schedule.py" "$@"
