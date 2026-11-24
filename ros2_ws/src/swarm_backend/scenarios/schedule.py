#!/usr/bin/env python3
"""Fly a schedule of intercept tests, one step after the other, from a YAML file.

Each step is a path (straight, weave, zigzag) flown at some altitudes, and each altitude
can have its own number of runs. A step is one or more baseline_eval.sh calls (one per
group of altitudes with the same number of runs), so everything that launcher does holds
here too: drones started when missing, Gazebo restarted after a crash, saved runs skipped.

    scenarios/schedule.sh paths_altitudes            scenarios/schedules/paths_altitudes.yaml
    scenarios/schedule.sh my.yaml --dry-run          print the commands and the run count only
    scenarios/schedule.sh paths_altitudes --sim      extra arguments go to every step
    scenarios/schedule.sh paths_altitudes --resume data/schedules/2026-11-24_150000_paths_altitudes
                                                     go on with a stopped schedule (its saved
                                                     runs are skipped)

The schedule (see scenarios/schedules/paths_altitudes.yaml):

    name: paths_altitudes          # the folder: data/schedules/<date-time>_<name>
    defaults:                      # for every step, unless the step sets it
      setup: yolo-bytetrack-mpc    # or baseline
      sector: 2                    # the hostile starts in this defender's sector
      speed: 4                     # m/s
      altitudes: [-10, 0, 10]      # m relative to the defenders (surveillance.altitude)
      runs: 3                      # per altitude; seeds seed_start .. seed_start + runs - 1
    steps:
      - path: straight
        runs: {-10: 2, 0: 5, 10: 2}   # runs per altitude
      - path: zigzag
        amplitude: 15
      - path: weave
        amplitude: 5
        altitudes: [0]
        runs: 1
        args: [--spawn-at-slots]   # anything else baseline_eval.sh takes

Step keys: path, amplitude, wavelength, altitudes, runs, seed_start, speed, aspect,
sector, setup, label (the step's folder; default e.g. zigzag_15m), args.

Results: data/schedules/<date-time>_<name>/<label>/intercept.jsonl per step, and
summary.txt (hits, closest pass, first detection per step and altitude) when the
schedule ends or is stopped. Ctrl-C stops the step that is flying (its defenders land)
and skips the rest. Run it in tmux: in Gazebo a run takes about 5 minutes.
"""
import argparse
import datetime
import json
import shutil
import signal
import statistics
import subprocess
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
SCHEDULES = HERE / 'schedules'
OUT_ROOT = REPO / 'data' / 'schedules'
CONTAINER_REPO = Path('/workspace')  # the repo as the container sees it
MINUTES_PER_RUN = 5  # Gazebo, for the estimate only

PATHS = ('straight', 'weave', 'zigzag')
KEYS = {'path', 'amplitude', 'wavelength', 'altitudes', 'runs', 'seed_start', 'speed',
        'aspect', 'sector', 'setup', 'label', 'args'}
DEFAULTS = {'path': 'straight', 'amplitude': 15.0, 'wavelength': 60.0,
            'altitudes': [-10.0, 0.0, 10.0], 'runs': 3, 'seed_start': 0, 'speed': 4.0,
            'aspect': 0.0, 'sector': 2, 'setup': 'yolo-bytetrack-mpc', 'args': []}


def find_schedule(name: str) -> Path:
    for p in (Path(name), SCHEDULES / name, SCHEDULES / f'{name}.yaml'):
        if p.is_file():
            return p
    raise SystemExit(f'No schedule {name!r} (looked here and in {SCHEDULES})')


def number(text: str) -> str:
    """15.0 -> '15', 2.5 -> '2.5'."""
    return f'{float(text):g}'


def load(path: Path) -> tuple[str, list[dict]]:
    """The schedule's name and its steps, each with every key filled in and checked."""
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict) or not isinstance(data.get('steps'), list) or not data['steps']:
        raise SystemExit(f'{path}: needs a list of steps')
    defaults = {**DEFAULTS, **(data.get('defaults') or {})}
    steps, labels = [], set()
    for i, raw in enumerate(data['steps'], 1):
        if not isinstance(raw, dict):
            raise SystemExit(f'{path}: step {i} is not a mapping')
        step = {**defaults, **raw}
        unknown = set(step) - KEYS
        if unknown:
            raise SystemExit(f'{path}: step {i}: unknown keys {sorted(unknown)} '
                             f'(known: {sorted(KEYS)})')
        if step['path'] not in PATHS:
            raise SystemExit(f'{path}: step {i}: path must be one of {", ".join(PATHS)}')
        runs = step['runs']
        if isinstance(runs, dict):
            groups = {float(alt): int(n) for alt, n in runs.items()}
        else:
            groups = {float(alt): int(runs) for alt in step['altitudes']}
        if not groups or min(groups.values()) < 1:
            raise SystemExit(f'{path}: step {i}: every altitude needs at least 1 run')
        step['groups'] = groups
        if isinstance(step['args'], str):
            step['args'] = step['args'].split()
        if 'label' not in raw:
            step['label'] = step['path'] if step['path'] == 'straight' else (
                f'{step["path"]}_{number(step["amplitude"])}m')
        if step['label'] in labels:
            raise SystemExit(f'{path}: two steps are called {step["label"]!r}; give one a label')
        labels.add(step['label'])
        steps.append(step)
    name = str(data.get('name') or path.stem)
    return name, steps


def commands(step: dict, out: Path, extra: list[str]) -> list[list[str]]:
    """baseline_eval.sh calls for one step: one per group of altitudes with the same runs."""
    by_runs: dict[int, list[float]] = {}
    for alt, n in step['groups'].items():
        by_runs.setdefault(n, []).append(alt)
    seed = int(step['seed_start'])
    calls = []
    for n, alts in sorted(by_runs.items()):
        argv = [str(HERE / 'baseline_eval.sh'), 'intercept',
                '--aspects', number(step['aspect']), '--speeds', number(step['speed']),
                '--seeds', f'{seed}-{seed + n - 1}',
                '--altitude-offsets', *[number(a) for a in sorted(alts)],
                '--sector', str(step['sector']), '--path', step['path']]
        if step['path'] != 'straight':
            argv += ['--path-amplitude', number(step['amplitude']),
                     '--path-wavelength', number(step['wavelength'])]
        argv += ['--setup', step['setup'], '--out', str(out / step['label']),
                 *map(str, step['args']), *extra]  # the caller's last: argparse keeps it
        calls.append(argv)
    return calls


def in_container(path: Path) -> Path:
    return CONTAINER_REPO / path.resolve().relative_to(REPO)


def median(values, unit=' m') -> str:
    values = [v for v in values if v is not None]
    return f'{statistics.median(values):.1f}{unit}' if values else '-'


def summary(out: Path, steps: list[dict], failed: set[str]) -> str:
    """Hits, closest pass and first detection per step and altitude (as compare_paths.sh)."""
    lines = [f'{"Step":<16} {"Alt":>4} {"Hits":>6} {"Success":>8} {"Closest":>8} {"t_hit":>7} '
             f'{"Seen":>6} {"Detect 3-D":>10}  outcomes', '-' * 90]
    for step in steps:
        label = step['label']
        path = out / label / 'intercept.jsonl'
        rows = [json.loads(line) for line in path.open() if line.strip()] if path.exists() else []
        if not rows:
            lines.append(f'{label:<16} {"":>4} {"-":>6} {"-":>8} {"-":>8} {"-":>7} {"-":>6} '
                         f'{"-":>10}  ' + ('failed' if label in failed else 'not flown'))
            continue
        name = label
        for altitude in sorted({r.get('hostile_altitude') for r in rows}, key=lambda a: a or 0):
            group = [r for r in rows if r.get('hostile_altitude') == altitude]
            n = len(group)
            hits = sum(r['outcome'] == 'hit' for r in group)
            success = sum(bool(r['success']) for r in group)
            t_hit = [r['time_to_hit'] for r in group if r['time_to_hit'] is not None]
            seen = [r for r in group if r.get('detect_distance') is not None]
            outcomes: dict[str, int] = {}
            for r in group:
                outcomes[r['outcome']] = outcomes.get(r['outcome'], 0) + 1
            note = ' (failed)' if label in failed else ''
            alt = '-' if altitude is None else f'{altitude:.0f}'
            t = f'{statistics.mean(t_hit):.1f} s' if t_hit else '-'
            lines.append(
                f'{name:<16} {alt:>4} {hits:>3}/{n:<2} {success:>4}/{n:<3} '
                f'{median([r["closest_miss"] for r in group]):>8} {t:>7} {len(seen):>3}/{n:<2} '
                f'{median([r["detect_distance"] for r in seen]):>10}  '
                + ', '.join(f'{k} {v}' for k, v in outcomes.items()) + note)
            name = ''
    lines += ['', "Alt: the hostile's altitude in m. Hits: within 1.5 m. Success: a hit more "
              'than 100 m from the base.', 'Closest: median closest pass. t_hit: mean time from '
              'the first detection to the hit. Seen: runs where a', "defender's detector confirmed "
              'the hostile. Detect 3-D: median distance to it at that image.']
    return '\n'.join(lines) + '\n'


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog='\n'.join(__doc__.splitlines()[2:]))
    p.add_argument('schedule', help='a YAML file, or a name in scenarios/schedules/')
    p.add_argument('--dry-run', action='store_true', help='print the commands, fly nothing')
    p.add_argument('--resume', type=Path, default=None, metavar='DIR',
                   help="a stopped schedule's folder: fly what it does not have yet")
    args, extra = p.parse_known_args(argv)

    file = find_schedule(args.schedule)
    name, steps = load(file)
    if args.resume is not None:
        out = next((d.resolve() for d in (args.resume, REPO / args.resume) if d.is_dir()),
                   args.resume)
        if not out.is_dir():
            raise SystemExit(f'--resume {args.resume}: no such folder')
    else:
        out = OUT_ROOT / f'{datetime.datetime.now():%Y-%m-%d_%H%M%S}_{name}'

    total = sum(sum(s['groups'].values()) for s in steps)
    print(f'Schedule {name} ({file}): {len(steps)} steps, {total} runs'
          + ('' if '--sim' in extra else f', about {total * MINUTES_PER_RUN / 60:.1f} h in Gazebo'))
    plan = []
    for i, step in enumerate(steps, 1):
        calls = commands(step, in_container(out) if not args.dry_run else out, extra)
        per_alt = ', '.join(f'{a:+g} m: {n}' for a, n in sorted(step['groups'].items()))
        print(f'  {i}. {step["label"]:<14} {step["setup"]}, runs per altitude {per_alt}')
        plan.append((step, calls))
    if args.dry_run:
        print('\nCommands:')
        for _, calls in plan:
            for argv in calls:
                print('  ' + ' '.join(argv))
        return 0

    out.mkdir(parents=True, exist_ok=True)
    shutil.copy(file, out / 'schedule.yaml')
    print(f'Saving in {out.relative_to(REPO)}\n')
    failed: set[str] = set()
    stopped = False
    previous = signal.getsignal(signal.SIGINT)
    # Ctrl-C reaches the scenario too (same process group), which lands its defenders;
    # here it only means: no more steps.
    def on_int(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, on_int)
    try:
        for i, (step, calls) in enumerate(plan, 1):
            for argv in calls:
                print(f'=== {i}/{len(plan)} {step["label"]} ' + '=' * 50, flush=True)
                if subprocess.call(argv) != 0 and not stopped:
                    failed.add(step['label'])
                if stopped:
                    break
            if stopped:
                print(f'Ctrl-C: stopped in {step["label"]}; the steps after it are not flown.\n'
                      f'Go on later with: scenarios/schedule.sh {args.schedule} '
                      f'--resume {out.relative_to(REPO)}')
                break
    finally:
        signal.signal(signal.SIGINT, previous)
    text = summary(out, steps, failed)
    (out / 'summary.txt').write_text(text)
    print('\n=== Summary ' + '=' * 60 + '\n' + text)
    print(f'Saved in {out.relative_to(REPO)}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
