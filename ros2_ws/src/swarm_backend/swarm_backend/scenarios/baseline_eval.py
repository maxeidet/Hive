"""Baseline evaluation scenario: the report's 30-seed scenarios and metrics on the arc.

The planning report's seeded scenarios and metrics, flown by the system we have, in
Gazebo or the fast simulation (surveillance.py).

The three gimbal defenders patrol the surveillance arc, the base station decides, each
defender runs its detector (in Gazebo the real camera detector, detect/live.py, started
here; with --sim the fake one with the real one's errors) and the real pure pursuit
(intercept/attack.py, through the patrol). The only truth used is for scoring. The
scenarios and metrics are the evaluation harness's (evaluation/scenario.py + metrics.py,
detect/scenario.py + metrics.py), placed on the arc by evaluation/arc_eval.py:

  intercept  aspect x speed x seed (default: intercept.eval_aspects x eval_speeds x seeds
             0..eval_runs-1 = 4 x 5 x 30). The hostile drone (attack.target_id, drone 4)
             flies its seeded straight track at its seeded altitude toward the base; the
             base station sends a defender. Scored with evaluation.metrics.score_run: hit,
             success (hit outside goal_radius), time to hit, lock-on, distance to base at
             hit, closest to base, closest miss, outcome. The interceptor scored is the
             defender that hit, else the first one sent, else the one the scenario was
             built around. Pure pursuit flies at intercept.interceptor_max_speed.
             --altitudes 30 40 50 flies every seed at each altitude (instead of the
             drawn one), --altitude-offsets -10 0 10 the same relative to the
             defenders' (surveillance.altitude); --sector 2 starts the hostile at a seeded bearing inside drone
             2's sector (the same bearing for a seed at every altitude and path). Each
             run also saves the first detection: which defender, when, and the true 3-D
             distance (space diagonal) to the hostile (detect_distance).
  detect     seeds 0..detect.eval_runs-1: the hostile drone flies its seeded track toward
             the base at detect.hostile_speed; the defenders only patrol and report (the
             base station sends nobody). Scored with detect.metrics.score_run: missed
             drones, first detection range, warning distance and time, detection
             probability vs range, background false alarms per image.
  summary    print the report's tables from the saved runs (no Gazebo needed).

--setup picks the detector, tracker and attack: yolo-bytetrack-mpc (the default on this
branch: YOLO, ByteTrack locked on one target, the MPC intercept instead of pure pursuit)
or baseline (the report's). --tracker, --detect-args and --guidance change one part of
it. Every setup but the baseline saves to its own folder (data/baseline_eval_<setup>/),
so its runs and summary never mix with the report's.

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
import statistics
import sys
from pathlib import Path

from swarm_backend.config import load_config
from swarm_backend.detect.camera import GIMBAL_HFOV
from swarm_backend.detect.metrics import DetectRunMetrics
from swarm_backend.detect.metrics import score_run as score_detect
from swarm_backend.detect.metrics import summarize as summarize_detect
from swarm_backend.evaluation import arc_eval
from swarm_backend.evaluation.metrics import Outcome, RunMetrics, score_run, summarize
from swarm_backend.evaluation.paths import INBOUND_PATHS, inbound
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


def summary_text(out: Path, config, where: str = 'Baseline on the arc',
                 attack: str = 'pure pursuit') -> str:
    parts = []
    rows = load_runs(out / 'intercept.jsonl')
    intercept = [as_intercept(r) for r in rows]
    if intercept:
        parts.append(summarize(intercept, f'{where}: {attack}, detector, base station'))
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
    lines = [f'  {len(results)} runs; a defender was sent in {sent}; outcomes: '
             + ', '.join(f'{o.value} {sum(r.outcome == o for r in results)}' for o in Outcome)]
    if any('detect_distance' in r for r in rows):
        lines += ['', f'  {"altitude":>8} {"runs":>4} {"hits":>4} {"detected":>8} '
                      f'{"detect 3-D [m]":>14} {"horizontal":>10} {"height":>7}',
                  '  (first detection by any defender: true distance to the hostile, median)']
        key = lambda r: r.get('hostile_altitude') or 0.0  # noqa: E731
        for altitude in sorted({key(r) for r in rows}):
            group = [r for r in rows if key(r) == altitude]
            seen = [r for r in group if r.get('detect_distance') is not None]
            lines.append(
                f'  {altitude:>6.0f} m {len(group):>4} '
                f'{sum(r["outcome"] == "hit" for r in group):>4} {len(seen):>4}/{len(group):<3} '
                f'{_median_of(seen, "detect_distance"):>14} '
                f'{_median_of(seen, "detect_horizontal"):>10} '
                f'{_median_of(seen, "detect_height"):>7}')
    return '\n'.join(lines)


def _median_of(rows, field) -> str:
    return f'{statistics.median(r[field] for r in rows):.1f}' if rows else '-'


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


def run_intercept(world: SurveillanceWorld, config, scenario, designated: int,
                  way=None) -> tuple:
    """`way(start, goal)`: the hostile's path (paths.inbound); None flies straight."""
    start, goal = scenario.target_start, scenario.target_goal
    flight = world.fly(start, goal, scenario.speed, timeout=config.intercept.eval_timeout,
                       path=None if way is None else way(start, goal))
    engaged = None if flight.engaged is None else flight.engaged[1]
    scored = flight.killer or engaged or world.ids[designated]
    samples = arc_eval.intercept_samples(
        flight.truth, flight.records, flight.statuses, world.index[scored], flight.t0,
        flight.t_end, GIMBAL_HFOV, config.detect.target_size, config.detect.match_radius_px)
    if len(samples) < 2:
        raise RuntimeError('Too few samples recorded (is Gazebo publishing poses?)')
    first = arc_eval.first_detection(
        flight.truth, flight.records, flight.t0, flight.t_end, GIMBAL_HFOV,
        config.detect.target_size, config.detect.match_radius_px)
    return score_run(samples, scenario, config.intercept), {
        'designated': world.ids[designated], 'scored_defender': scored,
        'engaged_by': engaged, 'killer': flight.killer,
        'images': sum(map(len, flight.records.values())),
        'crashes': flight.crashes,  # respawned before the next run (surveillance.py)
        'hostile_altitude': -scenario.target_start[2],
        **detection_fields(first, world.ids)}


def detection_fields(first, ids) -> dict:
    """The first detection (arc_eval.first_detection) as saved fields; None when missed."""
    if first is None:
        return {'detected_by': None, 'detect_t': None, 'detect_distance': None,
                'detect_horizontal': None, 'detect_height': None}
    return {'detected_by': ids[first.defender], 'detect_t': round(first.t, 2),
            'detect_distance': round(first.distance, 2),
            'detect_horizontal': round(first.horizontal, 2),
            'detect_height': round(first.height, 2)}


def run_detect(world: SurveillanceWorld, config, scenario, way=None) -> tuple:
    bx, by, _ = scenario.base
    ring = scenario.perimeter_radius

    def at_ring(t, hostile):
        return hostile is not None and math.hypot(hostile[0] - bx, hostile[1] - by) <= ring

    start, goal = scenario.hostile.start, scenario.hostile.goal
    flight = world.fly(start, goal, scenario.hostile.speed, timeout=config.detect.eval_timeout,
                       end=at_ring, path=None if way is None else way(start, goal))
    frames = arc_eval.frame_records(flight.truth, flight.records, scenario, flight.t0,
                                    flight.t_end, GIMBAL_HFOV)
    return score_detect(frames, scenario, config.detect), {
        'engaged_by': None if flight.engaged is None else flight.engaged[1],
        'killer': flight.killer, 'images': sum(map(len, flight.records.values())),
        'crashes': flight.crashes}


def jobs(args, config, out_file: Path) -> list:
    if args.mode == 'intercept':
        aspects = args.aspects or config.intercept.eval_aspects
        speeds = args.speeds or config.intercept.eval_speeds
        seeds = args.seeds if args.seeds is not None else range(config.intercept.eval_runs)
        altitudes = hostile_altitudes(args, config) or [None]
        sector = sector_index(args, config)
        todo = [arc_eval.make_intercept_scenario(a, v, s, config.intercept, config.surveillance,
                                                 altitude=h, sector=sector)
                for h in altitudes for a in aspects for v in speeds for s in seeds]

        def key(aspect, speed, seed, altitude, start_sector):
            return aspect, speed, seed, round(altitude, 1), start_sector

        done = {key(r['aspect_deg'], r['speed'], r['seed'], r.get('hostile_altitude', -1.0),
                    r.get('start_sector'))
                for r in load_runs(out_file) if r.get('path', 'straight') == args.path}
        return [(sc, i) for sc, i in todo if args.rerun or key(
            sc.aspect_deg, sc.speed, sc.seed, -sc.target_start[2], args.sector) not in done]
    seeds = args.seeds if args.seeds is not None else range(config.detect.eval_runs)
    done = {r['seed'] for r in load_runs(out_file) if r.get('path', 'straight') == args.path}
    return [arc_eval.make_detect_scenario(s, config.detect, config.surveillance)
            for s in seeds if args.rerun or s not in done]


def hostile_altitudes(args, config) -> list[float] | None:
    """--altitudes, then --altitude-offsets from the defenders' altitude; None: the drawn."""
    altitudes = list(args.altitudes or [])
    altitudes += [config.surveillance.altitude + d for d in args.altitude_offsets or []]
    if any(a <= 0 for a in altitudes):
        raise SystemExit(f'Hostile altitudes must be above the ground, got {altitudes}')
    return sorted(set(altitudes)) or None


def sector_index(args, config) -> int | None:
    """--sector's defender id as its index on the arc (left to right), or None."""
    if args.sector is None:
        return None
    ids = list(config.surveillance.drone_ids)
    if args.sector not in ids:
        raise SystemExit(f'--sector {args.sector}: not a defender (surveillance.drone_ids '
                         f'{ids})')
    return ids.index(args.sector)


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
    p.add_argument('--altitudes', type=float, nargs='+', default=None,
                   help='intercept: fly every seed at each of these hostile altitudes (m '
                        'above the ground, e.g. 30 40 50) instead of the drawn one')
    p.add_argument('--altitude-offsets', type=float, nargs='+', default=None,
                   help='intercept: the same, in m relative to the defenders\' altitude '
                        '(surveillance.altitude), e.g. -10 0 10')
    p.add_argument('--sector', type=int, default=None, metavar='DRONE_ID',
                   help='intercept: start the hostile at a seeded random bearing inside this '
                        'defender\'s sector (2: the middle 30 degrees), straight at the base')
    p.add_argument('--rerun', action='store_true', help='fly runs that are already saved')
    p.add_argument('--restart-on-crash', action='store_true',
                   help='Gazebo: a crashed defender is not respawned; the scenario ends with '
                        f'exit code {common.EXIT_RESTART} after the run is saved, for '
                        'baseline_eval.sh to restart Gazebo and go on (it passes this)')
    p.add_argument('--one-run', action='store_true',
                   help=f'Gazebo: end with exit code {common.EXIT_RESTART} after every run '
                        'while runs are left, so Gazebo is restarted between runs '
                        '(baseline_eval.sh --restart-each-run)')
    p.add_argument('--detect-rate', type=float, default=DEFAULT_DETECT_RATE,
                   help='images per second for each detector')
    p.add_argument('--path', choices=INBOUND_PATHS, default='straight',
                   help='how the hostile drone flies in: straight (the report\'s), weave (a '
                        'sine side to side) or zigzag (straight legs, sharp turns); not '
                        'straight saves to its own default folder, e.g. baseline_eval_weave')
    p.add_argument('--path-amplitude', type=float, default=15.0,
                   help='weave/zigzag: metres to either side of the straight line')
    p.add_argument('--path-wavelength', type=float, default=60.0,
                   help='weave/zigzag: metres along the line for one full swing')
    p.add_argument('--out', type=Path, default=None,
                   help=f'folder for intercept.jsonl, detect.jsonl and summary.txt (default '
                        f'{DEFAULT_OUT}, with --sim {DEFAULT_SIM_OUT}, plus _<setup> for any '
                        f'setup but the baseline, e.g. baseline_eval_yolo-bytetrack-mpc)')
    common.add_setup_args(p)
    return p


def main(argv: list[str] | None = None) -> int:
    p = parser()
    args = p.parse_args(argv)
    if args.sim and (args.one_run or args.restart_on_crash):
        p.error('--one-run and --restart-on-crash are for Gazebo, not --sim')
    common.resolve_setup(args)
    tag = common.setup_tag(args, real_detector=not args.sim)
    if args.out is None:
        # never mixed with the report's runs: another setup or path gets its own folder
        args.out = common.with_tag(DEFAULT_SIM_OUT if args.sim else DEFAULT_OUT, tag)
        if args.path != 'straight':
            args.out = args.out.with_name(f'{args.out.name}_{args.path}')
    where = ((tag or 'Baseline') + ' on the arc' + (' (fast simulation)' if args.sim else ''))
    attack = 'MPC' if args.guidance == 'mpc' else 'pure pursuit'
    way = None
    if args.path != 'straight':
        where += (f', {args.path} path (±{args.path_amplitude:g} m, '
                  f'{args.path_wavelength:g} m)')

        def way(start, goal):
            return inbound(start, goal, args.path, args.path_amplitude, args.path_wavelength)
    config = load_config(common.config_path(args.config, common.SURVEILLANCE_CONFIG))

    if args.mode == 'summary':
        print(summary_text(args.out, config, where, attack))
        return 0

    out_file = args.out / f'{args.mode}.jsonl'
    todo = jobs(args, config, out_file)
    if not todo:
        print(f'Every requested run is already in {out_file} (--rerun flies them again).')
        text = summary_text(args.out, config, where, attack)
        print(text)
        (args.out / 'summary.txt').write_text(text + '\n')
        return 0
    print(f'{len(todo)} {args.mode} runs to fly; results go to {out_file}', flush=True)

    flown = eval_config(config, args.mode)
    options = Options(detector='real', hit_radius=config.intercept.hit_radius,
                      max_speed=config.intercept.interceptor_max_speed,
                      detect_rate=args.detect_rate, record=True,
                      **common.setup_options(args))
    print(f'Setup: {options.setup}', flush=True)

    def runs(world):
        world.restart_on_crash = args.restart_on_crash
        world.restart_needed = False
        for k, job in enumerate(todo):
            if not world.alive():
                world.log.error('No defenders left')
                break
            common_fields = {
                'time': datetime.datetime.now().isoformat(timespec='seconds'),
                'backend': world.backend,
                'detector': ('real (detect/live.py)' if world.backend == 'gazebo'
                             else world.detector),
                'defenders': len(world.alive()), 'arc_radius': config.surveillance.arc_radius,
                'path': args.path, 'defender_altitude': config.surveillance.altitude,
                'setup': options.setup}
            if args.mode == 'intercept' and args.sector is not None:
                scenario, _ = job
                bx, by, _ = scenario.base
                sx, sy, _ = scenario.target_start
                common_fields.update(start_sector=args.sector, start_bearing_deg=round(
                    math.degrees(math.atan2(sy - by, sx - bx))
                    - config.surveillance.threat_heading_deg, 2))
            if args.path != 'straight':
                common_fields.update(path_amplitude=args.path_amplitude,
                                     path_wavelength=args.path_wavelength)
            if args.mode == 'intercept':
                scenario, designated = job
                world.log.info(f'Run {k + 1}/{len(todo)}: aspect {scenario.aspect_deg:.0f} deg, '
                               f'{scenario.speed} m/s, seed {scenario.seed}, around drone '
                               f'{world.ids[designated]}, altitude '
                               f'{-scenario.target_start[2]:.0f} m')
                metrics, extra = run_intercept(world, flown, scenario, designated, way)
                world.log.info(f'  {metrics.outcome.value}, closest {metrics.closest_miss:.1f} m'
                               + (f', hit {metrics.base_distance_at_hit:.0f} m from the base'
                                  if metrics.base_distance_at_hit is not None else '')
                               + (f', first seen by drone {extra["detected_by"]} at '
                                  f'{extra["detect_distance"]:.1f} m (3-D)'
                                  if extra['detected_by'] is not None else ', never seen'))
            else:
                scenario = job
                world.log.info(f'Run {k + 1}/{len(todo)}: detection, seed {scenario.seed}')
                metrics, extra = run_detect(world, flown, scenario, way)
                world.log.info('  missed' if metrics.missed else
                               f'  detected at {metrics.first_detection_range:.0f} m')
            save_run(out_file, metrics, {**extra, **common_fields})
            # Also after the last run: a crashed drone would otherwise be left in the world
            # for whatever runs next (another path of compare_paths.sh), and --one-run
            # leaves a fresh world. The launcher then calls again, finds nothing left to
            # fly and prints the summary.
            left = len(todo) - k - 1
            if world.restart_needed:
                raise common.RestartGazebo(f'a defender crashed; {left} runs left')
            if args.one_run:
                raise common.RestartGazebo(f'one run per Gazebo start; {left} runs left')

    code, _ = common.run_surveillance('baseline_eval', flown, options, args, runs)
    if code == common.EXIT_RESTART:
        print('Restarting Gazebo, then the next run (saved runs are skipped).', flush=True)
        return code
    text = summary_text(args.out, config, where, attack)
    print(text, flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'summary.txt').write_text(text + '\n')
    return code


if __name__ == '__main__':
    sys.exit(main())
