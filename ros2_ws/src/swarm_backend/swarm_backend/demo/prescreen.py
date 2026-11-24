"""Pre-screening a shoot's random entries in the fast simulation (no Gazebo, under a second
per run), so only seeds that give the wanted story are flown in Gazebo.

    python3 -m swarm_backend.demo.prescreen SHOOT.yaml --candidates 30 --out DIR
    (recording/shoot.sh --prescreen 30)

For every entry with `seeds: random` and a `want:`, N candidate seeds are flown with
`baseline_eval intercept --sim` once per setup of the entry (its `compare:`, else ours), the
rows turned into the same facts as a recorded take (takes.py), and the seeds that meet every
`want:` kept, one per repeat. DIR/prescreened.json maps the entry to its seeds; the sim's
rows stay in DIR/<entry>/<setup>/. The fast sim has no camera images (its detector is a fake
with the real one's errors), so Gazebo can still turn out differently.
"""
import argparse
import json
import random
import subprocess
import sys
from pathlib import Path

from swarm_backend.demo.shoot import OURS, Entry, load_shoot
from swarm_backend.scenarios.common import PKG

TRIAL_DEFAULTS = ['--aspects', '0', '--speeds', '4']   # what scenarios/trial.sh passes first


def sim_facts(row: dict) -> dict:
    """A fast-sim intercept row as the facts of a recorded take."""
    return {'hit': row.get('outcome') == 'hit',
            'attacker': row.get('killer') or row.get('detected_by'),
            'sighting_range_m': row.get('detect_distance'),
            'sighting_to_hit_s': row.get('time_to_hit'),
            'closest_pass_m': row.get('closest_miss'),
            'setup': row.get('setup')}


def _range(f):
    return (f or {}).get('sighting_range_m') or 0.0


WANTS = {
    'ours_hits': lambda ours, base: bool(ours.get('hit')),
    'ours_misses': lambda ours, base: not ours.get('hit'),
    'baseline_hits': lambda ours, base: base is not None and bool(base.get('hit')),
    'baseline_misses': lambda ours, base: base is not None and not base.get('hit'),
    'ours_sees_earlier': lambda ours, base: base is not None and _range(ours) >= _range(base) + 10,
    'fast_hit': lambda ours, base: bool(ours.get('hit')) and (ours.get('sighting_to_hit_s') or 99) < 5,
}


def sim_args(entry: Entry) -> list[str]:
    """The entry's scenario arguments for baseline_eval: without the script, its run count,
    --seeds and --setup; trial.sh's own defaults first."""
    cmd = list(entry.run)
    out = TRIAL_DEFAULTS[:] if cmd and cmd[0].endswith('trial.sh') else []
    rest = cmd[1:]
    if rest and rest[0].isdigit():
        rest = rest[1:]
    skip = False
    for i, a in enumerate(rest):
        if skip:
            skip = False
            continue
        if a in ('--seeds', '--setup'):
            skip = True
            continue
        out.append(a)
    return out


def run_sim(entry: Entry, setup: str, seeds: list[int], out: Path) -> dict[int, dict]:
    out.mkdir(parents=True, exist_ok=True)
    (out / 'intercept.jsonl').unlink(missing_ok=True)
    cmd = [sys.executable, '-m', 'swarm_backend.scenarios', 'baseline_eval', 'intercept', '--sim',
           '--config', entry.config or str(PKG / 'scenarios' / 'surveillance.yaml'),
           *sim_args(entry), '--setup', setup, '--seeds', ','.join(map(str, seeds)),
           '--out', str(out), '--quiet']
    res = subprocess.run(cmd, cwd=PKG, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f'the fast sim failed for {entry.name} ({setup}): '
                           + (res.stderr.strip().splitlines() or ['?'])[-1])
    rows = {}
    for line in (out / 'intercept.jsonl').read_text().splitlines():
        row = json.loads(line)
        rows[int(row['seed'])] = sim_facts(row)
    return rows


def choose(facts: dict[int, tuple[dict, dict | None]], want: list[str], keep: int,
           name: str) -> tuple[list[int], str]:
    """The seeds meeting every want (in candidate order), topped up with the closest ones."""
    def score(seed):
        ours, base = facts[seed]
        return sum(WANTS[w](ours, base) for w in want)
    order = list(facts)
    full = [s for s in order if score(s) == len(want)]
    chosen = full[:keep]
    if len(chosen) < keep:
        rest = sorted((s for s in order if s not in chosen), key=score, reverse=True)
        chosen += rest[:keep - len(chosen)]
    report = f'{name}: {len(full)} of {len(order)} candidates match {", ".join(want)}'
    if len(full) < keep:
        report += f'; flying the closest: {", ".join(map(str, chosen))}'
    else:
        report += f'; flying {", ".join(map(str, chosen))}'
    return chosen, report


def prescreen(entry: Entry, n_candidates: int, out: Path, rng_seed: int = 0) -> tuple[list[int], str]:
    unknown = [w for w in entry.want if w not in WANTS]
    if unknown:
        raise ValueError(f'{entry.name}: unknown want {", ".join(unknown)}; use {", ".join(WANTS)}')
    rng = random.Random(f'{rng_seed}:{entry.name}')
    candidates = rng.sample(range(1000), n_candidates)
    setups = entry.compare or [OURS]
    by_setup = {s: run_sim(entry, s, candidates, Path(out) / entry.name / s) for s in setups}
    base = 'baseline' if 'baseline' in setups else None
    ours = OURS if OURS in setups else setups[-1]
    facts = {c: (by_setup[ours].get(c, {}), by_setup[base].get(c) if base else None)
             for c in candidates}
    return choose(facts, entry.want, entry.repeat, entry.name)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('shoot', type=Path)
    p.add_argument('--candidates', type=int, default=30)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--rng-seed', type=int, default=0)
    args = p.parse_args(argv)
    chosen = {}
    for e in load_shoot(args.shoot):
        if e.seeds != 'random' or not e.want:
            continue
        if not e.run or not e.run[0].endswith('trial.sh'):
            print(f'{e.name}: only trial.sh runs can be pre-screened; drawing seeds instead')
            continue
        seeds, report = prescreen(e, args.candidates, args.out, args.rng_seed)
        chosen[e.name] = seeds
        print(report, flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'prescreened.json').write_text(json.dumps(chosen, indent=1) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
