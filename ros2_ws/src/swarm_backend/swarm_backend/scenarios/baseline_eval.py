"""Baseline evaluation scenario: the report's 30-seed scenarios and metrics on the arc.

The planning report's seeded scenarios and metrics, flown by the system we have, in
Gazebo or the fast simulation (surveillance.py).

The three gimbal defenders patrol the surveillance arc, the base station decides, each
defender runs its detector (in Gazebo the real camera detector, detect/live.py, started
here; with --sim the fake one with the real one's errors) and the real pure pursuit
(intercept/attack.py, through the patrol). The only truth used is for scoring. The
scenarios and metrics are the evaluation harness's (intercept/scenario.py + metrics.py,
detect/scenario.py + metrics.py), placed on the arc by intercept/arc_eval.py:

  intercept  aspect x speed x seed (default: intercept.eval_aspects x eval_speeds x seeds
             0..eval_runs-1 = 4 x 5 x 30). The hostile drone (attack.target_id, drone 4)
             flies its seeded straight track at its seeded altitude toward the base; the
             base station sends a defender. Scored with intercept.metrics.score_run: hit,
             success (hit outside goal_radius), time to hit, lock-on, distance to base at
             hit, closest to base, closest miss, outcome. The interceptor scored is the
             defender that hit, else the first one sent, else the one the scenario was
             built around. Pure pursuit flies at intercept.interceptor_max_speed.
  detect     seeds 0..detect.eval_runs-1: the hostile drone flies its seeded track toward
             the base at detect.hostile_speed; the defenders only patrol and report (the
             base station sends nobody). Scored with detect.metrics.score_run: missed
             drones, first detection range, warning distance and time, detection
             probability vs range, background false alarms per image.
  summary    print the report's tables from the saved runs (no Gazebo needed).

Every run is appended as one JSON line to --out (default data/baseline_eval/ in the repo,
with --sim data/baseline_eval_sim/: intercept.jsonl, detect.jsonl), and runs already there
are skipped, so a long batch can be stopped (Ctrl-C lands the defenders) and started
again. Each run: the hostile drone is (re)started at its start point, flies its track;
the run ends with a hit (the hostile is shot down, the defender returns to its slot),
when it reaches the base (intercept) or the arc (detect), or at eval_timeout.

In Gazebo, start the surveillance scenario first (scenarios/surveillance.yaml: defenders
1-3 and the hostile drone 4; see the vault note "Baseline evaluation - how to run").

    ./mount-backend.sh scenario baseline_eval intercept --aspects 0 --speeds 4 --seeds 0-4
    ./mount-backend.sh scenario baseline_eval detect
    ./mount-backend.sh scenario baseline_eval summary
    ./mount-backend.sh scenario baseline_eval intercept --sim      # the full grid, minutes

The full intercept grid is 600 runs: a few minutes each in Gazebo (run it in tmux), well
under a second each with --sim.
"""
import argparse
import dataclasses
import datetime
import json
import math
import sys
from pathlib import Path

from swarm_backend.config import load_config
from swarm_backend.detect.camera import GIMBAL_HFOV
from swarm_backend.detect.metrics import DetectRunMetrics
from swarm_backend.detect.metrics import score_run as score_detect
from swarm_backend.detect.metrics import summarize as summarize_detect
from swarm_backend.intercept import arc_eval
from swarm_backend.intercept.metrics import Outcome, RunMetrics, score_run, summarize
from swarm_backend.scenarios import common
from swarm_backend.scenarios.surveillance import Options, SurveillanceWorld

DEFAULT_OUT = common.REPO / 'data' / 'baseline_eval'
DEFAULT_SIM_OUT = common.REPO / 'data' / 'baseline_eval_sim'
DEFAULT_DETECT_RATE = 20.0


# --- saved runs ------------------------------------------------------------------------

def _jsonable(value):
    if isinstance(value, Outcome):
        return value.value
    if isinstance(value, tuple):
        return [_jsonable(v) for v in value]
    return value


def save_run(path: Path, metrics, extra: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    row = {k: _jsonable(v) for k, v in dataclasses.asdict(metrics).items()}
    row.update(extra)
    with open(path, 'a') as f:
        f.write(json.dumps(row) + '\n')


def load_runs(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def as_intercept(row: dict) -> RunMetrics:
    fields = {f.name for f in dataclasses.fields(RunMetrics)}
    values = {k: v for k, v in row.items() if k in fields}
    values['outcome'] = Outcome(values['outcome'])
    return RunMetrics(**values)


def as_detect(row: dict) -> DetectRunMetrics:
    fields = {f.name for f in dataclasses.fields(DetectRunMetrics)}
    values = {k: v for k, v in row.items() if k in fields}
    values['hostile_views'] = tuple(tuple(v) for v in values['hostile_views'])
    return DetectRunMetrics(**values)


def summary_text(out: Path, config, where: str = 'Baseline on the arc') -> str:
    parts = []
    rows = load_runs(out / 'intercept.jsonl')
    intercept = [as_intercept(r) for r in rows]
    if intercept:
        parts.append(summarize(intercept, f'{where}: pure pursuit, detector, base station'))
        parts.append(_intercept_extras(intercept, rows))
    detect = [as_detect(r) for r in load_runs(out / 'detect.jsonl')]
    if detect:
        config_arc = dataclasses.replace(config.detect,
                                         num_defenders=len(config.surveillance.drone_ids))
        parts.append(summarize_detect(detect, config_arc,
                                      f'{where}: detector, patrol sweep, '
                                      f'{config.surveillance.arc_radius:.0f} m arc'))
    return '\n\n'.join(parts) if parts else f'No saved runs in {out}'


def _intercept_extras(results, rows) -> str:
    sent = sum(1 for r in rows if r.get('engaged_by') is not None)
    return (f'  {len(results)} runs; a defender was sent in {sent}; outcomes: '
            + ', '.join(f'{o.value} {sum(r.outcome == o for r in results)}' for o in Outcome))


def parse_seeds(text: str) -> list[int]:
    seeds = []
    for part in text.split(','):
        if '-' in part:
            lo, hi = part.split('-')
            seeds += list(range(int(lo), int(hi) + 1))
        elif part:
            seeds.append(int(part))
    return seeds


# --- the two evaluations -----------------------------------------------------------------

def eval_config(config, mode: str):
    """In detect mode the defenders report and the base station tracks, but nobody is sent."""
    if mode != 'detect':
        return config
    ids = config.surveillance.drone_ids
    return dataclasses.replace(config, surveillance=dataclasses.replace(
        config.surveillance, reserve=len(ids), close_radius_m=-1.0))


def run_intercept(world: SurveillanceWorld, config, scenario, designated: int) -> tuple:
    flight = world.fly(scenario.target_start, scenario.target_goal, scenario.speed,
                       timeout=config.intercept.eval_timeout)
    engaged = None if flight.engaged is None else flight.engaged[1]
    scored = flight.killer or engaged or world.ids[designated]
    samples = arc_eval.intercept_samples(
        flight.truth, flight.records, flight.statuses, world.index[scored], flight.t0,
        flight.t_end, GIMBAL_HFOV, config.detect.target_size, config.detect.match_radius_px)
    if len(samples) < 2:
        raise RuntimeError('Too few samples recorded (is Gazebo publishing poses?)')
    return score_run(samples, scenario, config.intercept), {
        'designated': world.ids[designated], 'scored_defender': scored,
        'engaged_by': engaged, 'killer': flight.killer,
        'images': sum(map(len, flight.records.values()))}


def run_detect(world: SurveillanceWorld, config, scenario) -> tuple:
    bx, by, _ = scenario.base
    ring = scenario.perimeter_radius

    def at_ring(t, hostile):
        return hostile is not None and math.hypot(hostile[0] - bx, hostile[1] - by) <= ring

    flight = world.fly(scenario.hostile.start, scenario.hostile.goal, scenario.hostile.speed,
                       timeout=config.detect.eval_timeout, end=at_ring)
    frames = arc_eval.frame_records(flight.truth, flight.records, scenario, flight.t0,
                                    flight.t_end, GIMBAL_HFOV)
    return score_detect(frames, scenario, config.detect), {
        'engaged_by': None if flight.engaged is None else flight.engaged[1],
        'killer': flight.killer, 'images': sum(map(len, flight.records.values()))}


def jobs(args, config, out_file: Path) -> list:
    if args.mode == 'intercept':
        aspects = args.aspects or config.intercept.eval_aspects
        speeds = args.speeds or config.intercept.eval_speeds
        seeds = args.seeds if args.seeds is not None else range(config.intercept.eval_runs)
        todo = [arc_eval.make_intercept_scenario(a, v, s, config.intercept, config.surveillance)
                for a in aspects for v in speeds for s in seeds]
        done = {(r['aspect_deg'], r['speed'], r['seed']) for r in load_runs(out_file)}
        return [(sc, i) for sc, i in todo
                if args.rerun or (sc.aspect_deg, sc.speed, sc.seed) not in done]
    seeds = args.seeds if args.seeds is not None else range(config.detect.eval_runs)
    done = {r['seed'] for r in load_runs(out_file)}
    return [arc_eval.make_detect_scenario(s, config.detect, config.surveillance)
            for s in seeds if args.rerun or s not in done]


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='baseline_eval', description=__doc__.splitlines()[0])
    p.add_argument('mode', choices=('intercept', 'detect', 'summary'))
    common.add_backend_args(p)
    p.add_argument('--config', default=None,
                   help='swarm.yaml with surveillance:, intercept: and detect: sections '
                        '(default: scenarios/surveillance.yaml)')
    p.add_argument('--aspects', type=float, nargs='+', default=None,
                   help='intercept: degrees (default intercept.eval_aspects)')
    p.add_argument('--speeds', type=float, nargs='+', default=None,
                   help='intercept: hostile m/s (default intercept.eval_speeds)')
    p.add_argument('--seeds', type=parse_seeds, default=None,
                   help='e.g. 0-29 or 0,4,7 (default 0..eval_runs-1)')
    p.add_argument('--rerun', action='store_true', help='fly runs that are already saved')
    p.add_argument('--detect-rate', type=float, default=DEFAULT_DETECT_RATE,
                   help='images per second for each detector')
    p.add_argument('--out', type=Path, default=None,
                   help=f'folder for intercept.jsonl, detect.jsonl and summary.txt (default '
                        f'{DEFAULT_OUT}, with --sim {DEFAULT_SIM_OUT})')
    return p


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.out is None:
        args.out = DEFAULT_SIM_OUT if args.sim else DEFAULT_OUT
    where = 'Baseline on the arc' + (' (fast simulation)' if args.sim else '')
    config = load_config(common.config_path(args.config, common.SURVEILLANCE_CONFIG))

    if args.mode == 'summary':
        print(summary_text(args.out, config, where))
        return 0

    out_file = args.out / f'{args.mode}.jsonl'
    todo = jobs(args, config, out_file)
    if not todo:
        print(f'Every requested run is already in {out_file} (--rerun flies them again).')
        print(summary_text(args.out, config, where))
        return 0
    print(f'{len(todo)} {args.mode} runs to fly; results go to {out_file}', flush=True)

    flown = eval_config(config, args.mode)
    options = Options(detector='real', hit_radius=config.intercept.hit_radius,
                      max_speed=config.intercept.interceptor_max_speed,
                      detect_rate=args.detect_rate, record=True)

    def runs(world):
        for k, job in enumerate(todo):
            if not world.alive():
                world.log.error('No defenders left')
                break
            common_fields = {
                'time': datetime.datetime.now().isoformat(timespec='seconds'),
                'backend': world.backend,
                'detector': ('real (detect/live.py)' if world.backend == 'gazebo'
                             else world.detector),
                'defenders': len(world.alive()), 'arc_radius': config.surveillance.arc_radius}
            if args.mode == 'intercept':
                scenario, designated = job
                world.log.info(f'Run {k + 1}/{len(todo)}: aspect {scenario.aspect_deg:.0f} deg, '
                               f'{scenario.speed} m/s, seed {scenario.seed}, around drone '
                               f'{world.ids[designated]}')
                metrics, extra = run_intercept(world, flown, scenario, designated)
                world.log.info(f'  {metrics.outcome.value}, closest {metrics.closest_miss:.1f} m'
                               + (f', hit {metrics.base_distance_at_hit:.0f} m from the base'
                                  if metrics.base_distance_at_hit is not None else ''))
            else:
                scenario = job
                world.log.info(f'Run {k + 1}/{len(todo)}: detection, seed {scenario.seed}')
                metrics, extra = run_detect(world, flown, scenario)
                world.log.info('  missed' if metrics.missed else
                               f'  detected at {metrics.first_detection_range:.0f} m')
            save_run(out_file, metrics, {**extra, **common_fields})

    code, _ = common.run_surveillance('baseline_eval', flown, options, args, runs)
    text = summary_text(args.out, config, where)
    print(text, flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'summary.txt').write_text(text + '\n')
    return code


if __name__ == '__main__':
    sys.exit(main())
