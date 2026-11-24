"""A shoot: many runs of scenarios recorded one after the other (recording/shoot.sh).

recording/shoots/<name>.yaml, a list of entries:

    - name: head-on
      run: scenarios/trial.sh 1 --aspects 0 --speeds 4    # any scenario command
      compare: [baseline, yolo-bytetrack-mpc]             # the same run once per setup
      seeds: random                # or 7, 0-2; random = drawn (or pre-screened, prescreen.py)
      want: [ours_hits, baseline_misses]                  # what pre-screening looks for
      repeat: 3
      config: /workspace/.../scenarios/surveillance.yaml  # SWARM_CONFIG (default surveillance)
      options: [--cameras, attacker_watch]                # more record.sh options
    - name: overview
      run: idle 60                 # no scenario: only the cameras, 60 s

    python3 -m swarm_backend.demo.shoot plan SHOOT.yaml [--prescreened F] [--rng-seed N]
    python3 -m swarm_backend.demo.shoot mark SHOOT_DIR RUN_ID done|failed
    python3 -m swarm_backend.demo.shoot status SHOOT_DIR

`plan` prints one tab-separated line per run: its status (todo/done), id, keep_sim (1 when
the next run has the same config, so the sim is kept), config, pair, record.sh options, then
the command. shoot.json in the shoot folder
holds each run's status, so a stopped shoot carries on where it stopped.
"""
import argparse
import json
import random
import shlex
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

from swarm_backend.scenarios.common import SETUPS

DEFAULT_CONFIG = '/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml'
KEYS = {'name', 'run', 'repeat', 'compare', 'seeds', 'want', 'config', 'options'}
OURS = 'yolo-bytetrack-mpc'
STATUS = 'shoot.json'


class ShootError(ValueError):
    pass


@dataclass
class Entry:
    name: str
    run: list[str]
    repeat: int = 1
    compare: list[str] | None = None
    seeds: str | None = None
    want: list[str] = field(default_factory=list)
    config: str | None = None
    options: list[str] = field(default_factory=list)


@dataclass
class Run:
    id: str
    entry: str
    command: list[str]
    config: str
    setup: str | None
    seeds: list[int] | None
    pair: str | None
    options: list[str]


def load_shoot(path: Path) -> list[Entry]:
    path = Path(path)
    data = yaml.safe_load(path.read_text()) or []
    if not isinstance(data, list) or not data:
        raise ShootError(f'{path.name}: a shoot is a list of entries (name, run, ...)')
    entries, names = [], set()
    for i, d in enumerate(data, 1):
        where = f'{path.name}, entry {i}'
        if not isinstance(d, dict) or 'name' not in d or 'run' not in d:
            raise ShootError(f'{where}: needs "name" and "run"')
        extra = set(d) - KEYS
        if extra:
            raise ShootError(f'{where}: unknown key(s) {", ".join(sorted(extra))}; allowed: '
                             f'{", ".join(sorted(KEYS))}')
        if d['name'] in names:
            raise ShootError(f'{where}: the name {d["name"]} is used twice')
        names.add(d['name'])
        compare = d.get('compare')
        bad = [s for s in compare or [] if s not in SETUPS]
        if bad:
            raise ShootError(f'{where}: unknown setup(s) {", ".join(map(str, bad))}; use '
                             f'{", ".join(SETUPS)}')
        entries.append(Entry(str(d['name']), shlex.split(str(d['run'])), int(d.get('repeat', 1)),
                             compare, None if d.get('seeds') is None else str(d['seeds']),
                             list(d.get('want') or []), d.get('config'),
                             [str(o) for o in d.get('options') or []]))
    return entries


def _seed_list(text: str) -> list[int]:
    from swarm_backend.scenarios.baseline_eval import parse_seeds
    return parse_seeds(text)


def is_idle(command: list[str]) -> bool:
    return command[:1] == ['idle']


def expand(entries: list[Entry], rng_seed: int = 0,
           prescreened: dict[str, list[int]] | None = None) -> list[Run]:
    rng = random.Random(rng_seed)
    runs = []
    for e in entries:
        chosen = (prescreened or {}).get(e.name)
        for n in range(1, e.repeat + 1):
            seeds = None
            if e.seeds == 'random':
                seeds = [chosen[n - 1]] if chosen and n <= len(chosen) else [rng.randrange(1000)]
            elif e.seeds is not None:
                seeds = _seed_list(e.seeds)
            base = list(e.run)
            if seeds is not None:
                base += ['--seeds', ','.join(map(str, seeds))]
            config = e.config or DEFAULT_CONFIG
            stem = f'{e.name}-{n}' if e.repeat > 1 or e.compare else e.name + '-1'
            if e.compare:
                for setup in e.compare:
                    tag = 'ours' if setup == OURS else setup
                    runs.append(Run(f'{stem}-{tag}', e.name, base + ['--setup', setup], config,
                                    setup, seeds, stem, list(e.options)))
            else:
                runs.append(Run(stem, e.name, base, config, None, seeds, None, list(e.options)))
    return runs


def same_sim(a: Run, b: Run) -> bool:
    return a.config == b.config


def plan_lines(runs: list[Run], fresh_sim: bool = False) -> list[str]:
    """fresh_sim: every run starts its own sim (keep_sim 0 for all)."""
    lines = []
    for i, r in enumerate(runs):
        keep = int(not fresh_sim and i + 1 < len(runs) and same_sim(r, runs[i + 1]))
        lines.append('\t'.join([r.id, str(keep), r.config, r.pair or '-',
                               shlex.join(r.options) or '-', shlex.join(r.command)]))
    return lines


# ---- status ---------------------------------------------------------------------------------

def read_status(folder: Path) -> dict:
    path = Path(folder) / STATUS
    return json.loads(path.read_text()) if path.exists() else {}


def write_plan(folder: Path, yaml_path: Path, runs: list[Run]) -> dict:
    """shoot.json for a new shoot; an existing one is kept (resume) unless its yaml differs."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    text = Path(yaml_path).read_text()
    old = read_status(folder)
    if old:
        if old.get('yaml') != text:
            raise ShootError(f'{folder} is a shoot of another version of {Path(yaml_path).name}: '
                             'give a new --name')
        return old
    status = {'yaml': text, 'runs': [{**asdict(r), 'status': 'todo'} for r in runs]}
    (folder / STATUS).write_text(json.dumps(status, indent=1) + '\n')
    return status


def mark(folder: Path, run_id: str, state: str) -> None:
    status = read_status(folder)
    for r in status.get('runs', []):
        if r['id'] == run_id:
            r['status'] = state
            break
    else:
        raise ShootError(f'{run_id} is not a run of {folder}')
    (Path(folder) / STATUS).write_text(json.dumps(status, indent=1) + '\n')


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    pl = sub.add_parser('plan')
    pl.add_argument('shoot', type=Path)
    pl.add_argument('--prescreened', type=Path)
    pl.add_argument('--rng-seed', type=int, default=0)
    pl.add_argument('--folder', type=Path, help='write shoot.json here (resume if it exists)')
    pl.add_argument('--fresh-sim', action='store_true', help='every run starts its own sim')
    m = sub.add_parser('mark')
    m.add_argument('folder', type=Path)
    m.add_argument('run_id')
    m.add_argument('state', choices=('done', 'failed', 'todo'))
    st = sub.add_parser('status')
    st.add_argument('folder', type=Path)
    args = p.parse_args(argv)
    try:
        if args.cmd == 'plan':
            pre = json.loads(args.prescreened.read_text()) if args.prescreened else None
            runs = expand(load_shoot(args.shoot), args.rng_seed, pre)
            if args.folder:
                status = write_plan(args.folder, args.shoot, runs)
                done = {r['id'] for r in status['runs'] if r['status'] == 'done'}
                runs = [Run(**{k: v for k, v in r.items() if k != 'status'}) for r in status['runs']]
            else:
                done = set()
            for line, r in zip(plan_lines(runs, args.fresh_sim), runs):
                print(('done\t' if r.id in done else 'todo\t') + line)
        elif args.cmd == 'mark':
            mark(args.folder, args.run_id, args.state)
        else:
            for r in read_status(args.folder).get('runs', []):
                print(f'{r["status"]:6s} {r["id"]}')
    except ShootError as e:
        print(e, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
