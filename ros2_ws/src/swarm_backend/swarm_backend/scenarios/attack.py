"""Surveillance attack scenario: the whole chain, scored from the true positions.

The defenders patrol the arc, a hostile drone flies in toward the base, the base station
sends one defender, and the defender tries to fly into it (surveillance.py has the world,
in Gazebo or the fast simulation).

Settings come from the `surveillance:` and `attack:` sections of the config (default
scenarios/surveillance.yaml): the defenders are surveillance.drone_ids, the hostile drone
is attack.target_id. Each run: the hostile drone starts `--start` metres from the base,
`--bearing` degrees off the threat heading, `--height` metres above the defenders, and
flies straight at the base at `--speed`. The run ends with

  hit           a defender within --hit-radius (1.5 m) of it, from the true positions. The
                hostile drone is shot down, the base station gets a `kill` event and the
                defender, which survives, flies back to its slot
  reached_base  it got to the base

The next run starts when every defender is back on patrol with its sector. Each run adds
one row to --csv (default data/surveillance_attack_runs.csv in the repo, with --sim
data/surveillance_attack_runs_sim.csv; any setup but the baseline adds its name, e.g.
data/surveillance_attack_runs_yolo-bytetrack-mpc.csv).

--setup picks the detector, tracker and attack: yolo-bytetrack-mpc (the default: YOLO,
ByteTrack locked on one target, the MPC intercept) or baseline (the report's: the
baseline detector, the Confirmer, pure pursuit). --tracker, --detect-args and --guidance
change one part of it.

    ./mount-backend.sh scenario attack                          # Gazebo (start it first)
    ./mount-backend.sh scenario attack --bearing -30 0 30 --runs 3
    ./mount-backend.sh scenario attack --fake-detector --realistic
    ./mount-backend.sh scenario attack --sim --runs 3           # no Gazebo, seconds
    ./mount-backend.sh scenario attack --setup baseline         # the report's setup
    ./mount-backend.sh scenario attack --guidance pursuit       # YOLO + ByteTrack, pursuit

A hit means the defender flew into the hostile drone at up to attack.max_speed (5 m/s).
"""
import argparse
import csv
import datetime
import math
import sys
from pathlib import Path

from swarm_backend.config import load_config
from swarm_backend.evaluation.attack_scenario import (
    DEFAULT_HEIGHT_M, DEFAULT_START_M, Inbound, inbound_path)
from swarm_backend.scenarios import common
from swarm_backend.scenarios.surveillance import Options, SurveillanceWorld

DEFAULT_CSV = common.REPO / 'data' / 'surveillance_attack_runs.csv'
DEFAULT_SIM_CSV = common.REPO / 'data' / 'surveillance_attack_runs_sim.csv'
DEFAULT_DETECT_RATE = 20.0
FIELDS = ['time', 'run', 'bearing_deg', 'start_m', 'hostile_speed', 'height', 'detector',
          'gimbal_gain', 'max_speed', 'seed', 'defenders', 'result', 'hit', 'killer',
          'first_sighting_s', 'first_sighting_by', 'engaged_s', 'engaged_by', 'time_to_hit_s',
          'closest_m', 'closest_defender', 'kill_from_base_m', 'return_s', 'setup']


def inbound(args, index: int) -> Inbound:
    return Inbound(args.speed, args.bearing[index % len(args.bearing)], args.start, args.height)


def run(world: SurveillanceWorld, args, index: int) -> dict | None:
    """One run; its CSV row, or None when no defender is left."""
    alive = world.alive()
    if not alive:
        world.log.error('No defenders left')
        return None
    scenario = inbound(args, index)
    any_patrol = world.patrols[world.ids[0]].patrol
    path, _ = inbound_path(scenario, world.surv.base_position, any_patrol.slot_point[2],
                           world.heading)
    start, goal = path.points[0], path.points[1]
    world.log.info(f'Run {index + 1}/{args.runs}: inbound from {scenario.bearing_deg:+.0f} deg, '
                   f'{scenario.start_m:.0f} m out, {scenario.height_m:.0f} m above the '
                   f'defenders, {scenario.speed} m/s, {len(alive)} defenders, setup: '
                   f'{world.options.setup}')
    flight = world.fly(start, goal, scenario.speed)
    row = score(world, scenario, flight, index)
    killer = flight.killer
    world.log.info(f'Run {index + 1}: {row["result"].upper()}'
                   + (f' by drone {killer} {row["kill_from_base_m"]} m from the base, '
                      f'{row["time_to_hit_s"]} s after the first sighting' if killer else '')
                   + f'; closest {row["closest_m"]} m (drone {row["closest_defender"]})')
    return row


def score(world: SurveillanceWorld, scenario: Inbound, flight, index: int) -> dict:
    """The CSV row of one run."""
    a = world.attack_config
    judges, killer = flight.judges, flight.killer
    first = flight.sightings[0] if flight.sightings else None
    closest_id = min(judges, key=lambda i: judges[i].closest)
    bx, by = world.surv.base_position
    hit_time = flight.hit_time(killer) if killer is not None else None
    hostile_at = flight.hostile_at
    return {
        'time': datetime.datetime.now().isoformat(timespec='seconds'), 'run': index + 1,
        'bearing_deg': scenario.bearing_deg, 'start_m': scenario.start_m,
        'hostile_speed': scenario.speed, 'height': scenario.height_m,
        'detector': world.detector, 'gimbal_gain': a.gimbal_gain,
        'max_speed': a.max_speed, 'seed': a.seed,
        'defenders': len(judges), 'result': 'hit' if killer is not None else 'reached_base',
        'hit': int(killer is not None), 'killer': '' if killer is None else killer,
        'first_sighting_s': '' if first is None else round(first[0], 2),
        'first_sighting_by': '' if first is None else first[1],
        'engaged_s': '' if flight.engaged is None else round(flight.engaged[0], 2),
        'engaged_by': '' if flight.engaged is None else flight.engaged[1],
        'time_to_hit_s': (round(hit_time - first[0], 2)
                          if hit_time is not None and first is not None else ''),
        'closest_m': round(judges[closest_id].closest, 2), 'closest_defender': closest_id,
        'kill_from_base_m': (round(math.hypot(hostile_at[0] - bx, hostile_at[1] - by), 1)
                             if killer is not None and hostile_at is not None else ''),
        'return_s': '' if flight.return_s is None else round(flight.return_s, 2),
        'setup': world.options.setup,
    }


def write_row(path: Path, row: dict, log) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        new = not path.exists() or path.stat().st_size == 0
        fields = FIELDS
        if not new:  # keep an older file's columns (it may have no setup column)
            with open(path, newline='') as f:
                fields = next(csv.reader(f), None) or FIELDS
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            if new:
                writer.writeheader()
            writer.writerow(row)
    except OSError as e:
        log.error(f'Could not write {path}: {e}')


def summary(rows: list[dict], args, world: SurveillanceWorld) -> str:
    if not rows:
        return 'No runs finished.'
    hits = [r for r in rows if r['hit']]
    lines = [f'{len(hits)}/{len(rows)} hits (inbound at {args.speed} m/s, detector '
             f'{world.detector}, setup {world.options.setup}, gimbal gain '
             f'{world.attack_config.gimbal_gain})']
    for r in rows:
        what = (f'hit by drone {r["killer"]} {r["kill_from_base_m"]} m from the base'
                if r['hit'] else f'{r["result"]}, closest {r["closest_m"]} m')
        lines.append(f'  run {r["run"]} from {r["bearing_deg"]:+.0f} deg: {what}')
    if args.csv:
        lines.append(f'  one row per run in {args.csv}')
    return '\n'.join(lines)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='attack', description=__doc__.splitlines()[0])
    common.add_backend_args(p)
    p.add_argument('--config', default=None,
                   help='swarm.yaml with a surveillance: section (default: '
                        'scenarios/surveillance.yaml)')
    p.add_argument('--runs', type=int, default=1)
    p.add_argument('--bearing', type=float, nargs='+', default=[0.0],
                   help='degrees off the threat heading the hostile comes from, positive '
                        'to the right (slots of 3 defenders: -30 0 30); several are used '
                        'in turn, one per run')
    p.add_argument('--start', type=float, default=DEFAULT_START_M,
                   help='metres from the base where the hostile drone starts')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT_M,
                   help='metres the hostile drone flies above the defenders')
    p.add_argument('--speed', type=float, default=1.0, help='hostile speed, m/s')
    p.add_argument('--hit-radius', type=float, default=1.5, help='metres')
    p.add_argument('--fake-detector', action='store_true',
                   help="fake detectors from the true poses instead of the camera detector")
    p.add_argument('--realistic', action='store_true',
                   help='with --fake-detector: 0.25 s delay, 1 deg and 20 %% noise, '
                        '5 m blind zone')
    p.add_argument('--detect-rate', type=float, default=DEFAULT_DETECT_RATE,
                   help='images per second for each real detector')
    p.add_argument('--gain', type=float, default=None, help='gimbal gain (attack config)')
    p.add_argument('--max-speed', type=float, default=None, help='m/s (attack config)')
    p.add_argument('--seed', type=int, default=None, help='fake detector noise')
    p.add_argument('--csv', default=None,
                   help=f'append one row per run here (default {DEFAULT_CSV}, with --sim '
                        f'{DEFAULT_SIM_CSV}, plus _<setup> for any setup but the baseline; '
                        f'"" for none)')
    common.add_setup_args(p)
    return p


def options(args) -> Options:
    detector = (('fake-realistic' if args.realistic else 'fake')
                if args.fake_detector else 'real')
    return Options(detector=detector, hit_radius=args.hit_radius, gimbal_gain=args.gain,
                   max_speed=args.max_speed, seed=args.seed, detect_rate=args.detect_rate,
                   **common.setup_options(args))


def main(argv: list[str] | None = None) -> int:
    p = parser()
    args = p.parse_args(argv)
    if args.realistic and not args.fake_detector:
        p.error('--realistic only applies to --fake-detector')
    common.resolve_setup(args)
    if args.csv is None:
        tag = common.setup_tag(args, real_detector=not (args.fake_detector or args.sim))
        args.csv = str(common.with_tag(DEFAULT_SIM_CSV if args.sim else DEFAULT_CSV, tag))
    config = load_config(common.config_path(args.config, common.SURVEILLANCE_CONFIG))
    rows = []

    def runs(world):
        for i in range(args.runs):
            row = run(world, args, i)
            if row is None:
                break
            rows.append(row)
            if args.csv:
                write_row(Path(args.csv), row, world.log)

    code, world = common.run_surveillance('attack_scenario', config, options(args), args, runs)
    if world is not None:
        print(summary(rows, args, world), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
