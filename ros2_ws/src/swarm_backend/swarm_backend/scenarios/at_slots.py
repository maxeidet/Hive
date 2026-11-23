"""A surveillance config whose defenders start on the ground right below their slots on
the arc, so they only climb straight up to them (no 150 m transit from the camp first).

    python3 -m swarm_backend.scenarios.at_slots --config scenarios/surveillance.yaml --out X.yaml

Used by `scenarios/baseline_eval.sh --spawn-at-slots` (and so by `compare_paths.sh
--spawn-at-slots` and the dev GUI's single runs). Everything else in the config is kept as
it is; only surveillance.drone_ids' spawn_pose changes. The slots are the scenario's own
(evaluation/arc_eval.slots), so the two cannot disagree. A defender that was started
somewhere else is moved by the scenario itself: check_homes restarts it at its spawn point.

Pure data, no ROS.
"""
import argparse
import copy
import sys
from pathlib import Path

import yaml

from swarm_backend.config import parse_config
from swarm_backend.evaluation.arc_eval import slots


def at_slots(raw: dict) -> dict:
    """A copy of the parsed YAML `raw` with each defender's spawn_pose (Gazebo ENU: east,
    north, up) on the ground below its slot (world NED: north, east)."""
    config = parse_config(raw)
    surveillance = config.surveillance
    out = copy.deepcopy(raw)
    entries = {entry['id']: entry for entry in out.get('drones', [])}
    for drone_id, (north, east, _) in zip(surveillance.drone_ids, slots(surveillance)):
        if drone_id not in entries:
            raise ValueError(f'surveillance: drone {drone_id} is not in drones:')
        entries[drone_id]['spawn_pose'] = [round(east, 3), round(north, 3), 0.0]
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, required=True, help='the surveillance config')
    parser.add_argument('--out', type=Path, required=True, help='where to write the copy')
    args = parser.parse_args(argv)
    data = at_slots(yaml.safe_load(args.config.read_text()))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    header = (f'# Generated from {args.config} by swarm_backend/scenarios/at_slots.py: the\n'
              '# defenders start below their slots. Do not edit; edit the source instead.\n')
    args.out.write_text(header + yaml.safe_dump(data, sort_keys=False))
    defenders = {d['id']: d['spawn_pose'] for d in data['drones']
                 if d['id'] in parse_config(data).surveillance.drone_ids}
    print('Defenders start below their slots (Gazebo east, north): '
          + ', '.join(f'{i}: ({p[0]:.0f}, {p[1]:.0f})' for i, p in defenders.items()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
