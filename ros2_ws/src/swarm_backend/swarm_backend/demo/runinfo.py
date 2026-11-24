"""`run.json`: what a recorded run was, so everything afterwards knows who is who.

`recording/record.sh` writes it into the run folder once the sim is up:

    python3 -m swarm_backend.demo.runinfo write RUN --config CFG [--options JSON] -- CMD ...

It holds the scenario command, its config (copied in, so a later edit of the yaml does not
change the run), the setup and seeds read from the command, and the drones by role as
Gazebo names them (`<model without gz_>_<id>`, as save_images.drone_names). The takes, the
map and the film read the roles from here instead of names written into the tools, so a new
scenario with other drones needs no change in them.
"""
import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from swarm_backend.scenarios.baseline_eval import parse_seeds
from swarm_backend.scenarios.common import DEFAULT_SETUP

FILE = 'run.json'
BIRDS_PREFIX = 'bird_'


@dataclass
class RunInfo:
    command: list[str]
    config: dict
    setup: str | None
    seeds: list[int] | None
    defenders: list[str]
    hostiles: list[str]
    birds_prefix: str = BIRDS_PREFIX
    options: dict = field(default_factory=dict)


def gazebo_name(config: dict, drone_id: int) -> str:
    """The model name PX4 gives drone `drone_id` in Gazebo: <model without gz_>_<id>."""
    drone = next((d for d in config.get('drones', []) if d.get('id') == drone_id), {})
    model = drone.get('model', 'gz_x500')
    return f'{model.removeprefix("gz_")}_{drone_id}'


def roles(config: dict) -> tuple[list[str], list[str]]:
    """(defenders, hostiles) as Gazebo names. Surveillance: surveillance.drone_ids and
    attack.target_id; scenario 2: scenario2.defender_id and scenario2.hostile_ids."""
    s2 = config.get('scenario2') or {}
    if s2.get('hostile_ids'):
        defender_ids = [s2.get('defender_id', (config.get('attack') or {}).get('attacker_id'))]
        hostile_ids = list(s2['hostile_ids'])
    else:
        defender_ids = list((config.get('surveillance') or {}).get('drone_ids', []))
        hostile_ids = [(config.get('attack') or {}).get('target_id')]
    return ([gazebo_name(config, i) for i in defender_ids if i is not None],
            [gazebo_name(config, i) for i in hostile_ids if i is not None])


def _flag(command: list[str], name: str) -> str | None:
    for i, arg in enumerate(command):
        if arg == name and i + 1 < len(command):
            return command[i + 1]
        if arg.startswith(name + '='):
            return arg.split('=', 1)[1]
    return None


def write_run_info(folder: Path, config_path: Path, command: list[str], options: dict) -> dict:
    config = yaml.safe_load(Path(config_path).read_text()) or {}
    defenders, hostiles = roles(config)
    seeds = _flag(command, '--seeds')
    info = {
        'command': list(command),
        'config_path': str(config_path),
        'config': config,
        'setup': _flag(command, '--setup') or DEFAULT_SETUP,
        'seeds': parse_seeds(seeds) if seeds and seeds != 'random' else None,
        'defenders': defenders,
        'hostiles': hostiles,
        'birds_prefix': BIRDS_PREFIX,
        'options': dict(options),
    }
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / FILE).write_text(json.dumps(info, indent=2))
    return info


def read_run_info(folder: Path) -> RunInfo:
    path = Path(folder) / FILE
    if not path.exists():
        raise FileNotFoundError(f'No {FILE} in {folder}: record the run with recording/record.sh.')
    d = json.loads(path.read_text())
    return RunInfo(d['command'], d['config'], d.get('setup'), d.get('seeds'), d['defenders'],
                   d['hostiles'], d.get('birds_prefix', BIRDS_PREFIX), d.get('options', {}))


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    w = sub.add_parser('write')
    w.add_argument('run', type=Path)
    w.add_argument('--config', type=Path, required=True)
    w.add_argument('--options', default='{}', help='JSON of record.sh options')
    # The scenario command comes after `--` and is split off by hand: an argparse.REMAINDER
    # positional right after `run` swallows --config/--options as part of the command.
    argv = list(sys.argv[1:] if argv is None else argv)
    command = argv[argv.index('--') + 1:] if '--' in argv else []
    if '--' in argv:
        argv = argv[:argv.index('--')]
    args = p.parse_args(argv)
    info = write_run_info(args.run, args.config, command, json.loads(args.options))
    print(f'Wrote {args.run / FILE}: defenders {", ".join(info["defenders"])}, '
          f'hostiles {", ".join(info["hostiles"])}, setup {info["setup"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
