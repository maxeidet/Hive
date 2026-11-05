"""Baseline evaluation in Gazebo: the report's 30-seed scenarios and metrics on the arc.

The planning report's seeded scenarios and metrics, flown by the system we have
(`./mount-backend.sh baseline-eval`).

The three gimbal defenders patrol the surveillance arc, the base station decides, each
defender runs the real camera detector (detect/live.py, started here) and the real pure
pursuit (intercept/attack.py, through the patrol). No mocks: the only truth used is
Gazebo's, for scoring. The scenarios and metrics are the evaluation harness's
(intercept/scenario.py + metrics.py, detect/scenario.py + metrics.py), placed on the arc
by intercept/arc_eval.py:

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

Every run is appended as one JSON line to --out (default data/baseline_eval/ in the repo:
intercept.jsonl, detect.jsonl), and runs already there are skipped, so a long batch can
be stopped (Ctrl-C lands the defenders) and started again. Each run: the hostile drone is
restarted at its start point, climbs, flies its track; the run ends with a hit (the
hostile is shot down as in attacktest, the defender returns to its slot), when it reaches
the base (intercept) or the arc (detect), or at eval_timeout.

Start the surveillance scenario first (scenarios/surveillance.yaml: defenders 1-3 and the
hostile drone 4; see the vault note "Baseline evaluation - how to run"). Inside the
container:

    python3 .../tests/baseline_eval_test.py intercept --aspects 0 --speeds 4 --seeds 0-4
    python3 .../tests/baseline_eval_test.py detect
    python3 .../tests/baseline_eval_test.py summary

On the host: ./mount-backend.sh baseline-eval [same arguments]. The full intercept grid
is 600 runs of a few minutes each: run it in tmux.
"""
import argparse
import dataclasses
import datetime
import json
import math
import sys
import time
from pathlib import Path

from swarm_backend.config import default_config_path, load_config
from swarm_backend.detect.metrics import DetectRunMetrics
from swarm_backend.detect.metrics import score_run as score_detect
from swarm_backend.detect.metrics import summarize as summarize_detect
from swarm_backend.intercept import arc_eval
from swarm_backend.intercept.interceptor import Status
from swarm_backend.intercept.metrics import Outcome, RunMetrics, score_run, summarize
from swarm_backend.intercept.paths import Path as Track

REPO = Path(__file__).resolve().parents[4]  # /workspace in the container
DEFAULT_OUT = REPO / 'data' / 'baseline_eval'
DEFAULT_DETECT_RATE = 20.0
TRUTH_PERIOD_S = 0.02  # simulation seconds between recorded truth points
HOSTILE_SPEED_MARGIN = 3.0  # m/s above the scenario speed for the hostile's PX4 limit
PX4_MIN_SPEED_LIMIT = 12.0  # PX4's default MPC_XY_VEL_MAX


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


def summary_text(out: Path, config) -> str:
    parts = []
    intercept = [as_intercept(r) for r in load_runs(out / 'intercept.jsonl')]
    if intercept:
        parts.append(summarize(intercept, 'Baseline on the arc: pure pursuit, real detector, '
                                          'base station'))
        parts.append(_intercept_extras(intercept, load_runs(out / 'intercept.jsonl')))
    detect = [as_detect(r) for r in load_runs(out / 'detect.jsonl')]
    if detect:
        config_arc = dataclasses.replace(config.detect,
                                         num_defenders=len(config.surveillance.drone_ids))
        parts.append(summarize_detect(detect, config_arc,
                                      f'Baseline on the arc: real detector, patrol sweep, '
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


# --- the Gazebo runner -----------------------------------------------------------------

def make_runner(node, swarm_config, args):
    """The attack test's machinery (patrols, base station, real detectors, hostile drone
    restarts, kill referee), set up for the evaluation. Imported here so `summary` runs
    without ROS."""
    from attack_test import SETTLE_S, TOLERANCE_M, AttackTest

    from swarm_backend import ground_truth
    from swarm_backend.detect.camera import GIMBAL_HFOV
    from swarm_backend.intercept import surveillance_msgs as msgs
    from swarm_backend.intercept.attack import State
    from swarm_backend.intercept.attack_scenario import ClosestApproach
    from swarm_backend.intercept.base_station import Mode
    from swarm_backend.intercept.mock import HostileFlight
    from swarm_backend.runner import wait_until
    from std_msgs.msg import String

    class BaselineEval(AttackTest):

        def __init__(self):
            config = swarm_config
            if args.mode == 'detect':  # report and track, but nobody is sent
                ids = config.surveillance.drone_ids
                config = dataclasses.replace(config, surveillance=dataclasses.replace(
                    config.surveillance, reserve=len(ids), close_radius_m=-1.0))
            runner_args = argparse.Namespace(
                realistic=False, gain=None, seed=None, fake_detector=False, record=True,
                max_speed=config.intercept.interceptor_max_speed,
                detect_rate=args.detect_rate, hit_radius=config.intercept.hit_radius,
                runs=0, bearing=[0.0], speed=1.0, start=0.0, height=0.0, csv='')
            super().__init__(node, config, runner_args)
            self.config_all = config
            self.index = {d: i for i, d in enumerate(self.ids)}
            self.truth_log: list[arc_eval.TruthPoint] | None = None
            self._last_truth = -math.inf
            self.truth.listeners.append(self._record_truth)
            self._offsets = {d: 0 for d in self.ids}

        # --- recording ----------------------------------------------------------

        def _record_truth(self, stamp, poses) -> None:
            log = self.truth_log
            if log is None or stamp - self._last_truth < TRUTH_PERIOD_S:
                return
            self._last_truth = stamp
            bodies, cameras = {}, {}
            for d in self.ids:
                body = ground_truth.body_position(poses, self.names[d])
                camera = ground_truth.camera_pose(poses, self.names[d], self.cameras[d])
                if body is not None:
                    bodies[self.index[d]] = body
                if camera is not None:
                    cameras[self.index[d]] = camera
            hostile = ground_truth.body_position(poses, self.names[self.hostile_id])
            log.append(arc_eval.TruthPoint(stamp, bodies, cameras, hostile))

        def read_records(self) -> dict:
            """The detector records written since the last call, by defender index."""
            out = {}
            for d in self.ids:
                path = Path(self.record_path(d))
                recs = []
                if path.exists():
                    with open(path) as f:
                        f.seek(self._offsets[d])
                        for line in f:
                            if line.endswith('\n'):
                                recs.append(json.loads(line))
                        self._offsets[d] = f.tell()
                out[self.index[d]] = recs
            return out

        def status(self, drone_id: int) -> Status:
            patrol = self.patrols[drone_id].patrol
            if patrol.mode is Mode.ENGAGE and patrol.attack is not None:
                state = patrol.attack.state
                if state in (State.ATTACK, State.COAST):
                    return Status.TRACKING
                return Status.LOST if state is State.LOST else Status.IDLE
            return Status.LOST if patrol.mode is Mode.RETURN else Status.IDLE

        # --- flying a track -------------------------------------------------------

        def fly(self, start, goal, speed: float, end, timeout: float):
            """Restart the hostile drone at `start` (world NED, its altitude in z), fly it
            straight to `goal` at `speed` while recording; `end(t, hostile)` ends the
            run early. Returns (t0, t_end, killer, first engaged defender id)."""
            self.read_records()  # skip what came before this run
            self.respawn_hostile(start[0], start[1])
            limit = max(PX4_MIN_SPEED_LIMIT, speed + HOSTILE_SPEED_MARGIN)
            self.set_hostile_param('MPC_XY_VEL_MAX', limit)
            altitude = self.hostile.home[2] - start[2]
            self.log.info(f'Hostile taking off to {altitude:.0f} m')
            self.hostile_manager.takeoff(altitude)
            wait_until(self.hostile_manager.all_reached, 900.0, 'the hostile drone to climb')
            velocity = [g - s for g, s in zip(goal, start)]
            yaw = math.atan2(velocity[1], velocity[0])
            self.hostile.set_position(*start, yaw=yaw)
            wait_until(lambda: self.hostile.reached(TOLERANCE_M), 900.0,
                       'the hostile drone at its start point')
            self.wait_patrol()
            self.wait_sim(SETTLE_S)

            track = Track([tuple(start), tuple(goal)], speed)
            flight_time = math.dist(start, goal) / speed
            alive = self.alive()
            judges = {d: ClosestApproach(self.config_all.intercept.hit_radius) for d in alive}
            self.statuses = []
            self.truth_log = []
            self._last_truth = -math.inf
            engaged = None
            killer = None
            self.flight = HostileFlight(self.hostile, track, self.clock)
            self.judges = judges
            self.flight.start()
            t0 = self.clock()
            try:
                while True:
                    now = self.clock()
                    t = now - t0
                    self.statuses.append((now, {self.index[d]: self.status(d) for d in alive}))
                    if engaged is None:
                        engaged = next((d for d in alive
                                        if self.patrols[d].patrol.mode is Mode.ENGAGE), None)
                        if engaged is not None:
                            self.log.info(f'  t={t:6.1f} s drone {engaged} sent')
                    hits = [(j.hit_time, d) for d, j in judges.items() if j.hit]
                    if hits:
                        killer = min(hits)[1]
                        break
                    if t >= flight_time or t >= timeout or end(t, self._hostile_truth):
                        break
                    time.sleep(0.05)
            finally:
                t_end = self.clock()
                self.judges = None
                self.flight.stop()
                self.flight = None
                self.remove_hostile()
                log, self.truth_log = self.truth_log, None
            if killer is not None:
                self._events.publish(String(data=msgs.encode_event(msgs.EventMsg(killer, 'kill'))))
            self.last_truth = log
            try:
                self.wait_patrol()
            except TimeoutError as e:
                self.log.warning(str(e))
            time.sleep(1.0)  # the detectors' last lines
            return t0, t_end, killer, engaged

        # --- the two evaluations -------------------------------------------------------

        def run_intercept(self, scenario, designated: int) -> tuple:
            c = self.config_all
            t0, t_end, killer, engaged = self.fly(
                scenario.target_start, scenario.target_goal, scenario.speed,
                lambda t, h: False, c.intercept.eval_timeout)
            records = self.read_records()
            scored = killer or engaged or self.ids[designated]
            samples = arc_eval.intercept_samples(
                self.last_truth, records, self.statuses, self.index[scored], t0, t_end,
                GIMBAL_HFOV, c.detect.target_size, c.detect.match_radius_px)
            if len(samples) < 2:
                raise RuntimeError('Too few samples recorded (is Gazebo publishing poses?)')
            return score_run(samples, scenario, c.intercept), {
                'designated': self.ids[designated], 'scored_defender': scored,
                'engaged_by': engaged, 'killer': killer, 'images': sum(map(len, records.values()))}

        def run_detect(self, scenario) -> tuple:
            c = self.config_all
            bx, by, _ = scenario.base
            ring = scenario.perimeter_radius

            def at_ring(t, hostile):
                return hostile is not None and math.hypot(hostile[0] - bx, hostile[1] - by) <= ring

            t0, t_end, killer, engaged = self.fly(
                scenario.hostile.start, scenario.hostile.goal, scenario.hostile.speed,
                at_ring, c.detect.eval_timeout)
            records = self.read_records()
            frames = arc_eval.frame_records(self.last_truth, records, scenario, t0, t_end,
                                            GIMBAL_HFOV)
            return score_detect(frames, scenario, c.detect), {
                'engaged_by': engaged, 'killer': killer,
                'images': sum(map(len, records.values()))}

    return BaselineEval()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('mode', choices=('intercept', 'detect', 'summary'))
    parser.add_argument('--config', default=None,
                        help='swarm.yaml with surveillance:, intercept: and detect: sections')
    parser.add_argument('--aspects', type=float, nargs='+', default=None,
                        help='intercept: degrees (default intercept.eval_aspects)')
    parser.add_argument('--speeds', type=float, nargs='+', default=None,
                        help='intercept: hostile m/s (default intercept.eval_speeds)')
    parser.add_argument('--seeds', type=parse_seeds, default=None,
                        help='e.g. 0-29 or 0,4,7 (default 0..eval_runs-1)')
    parser.add_argument('--rerun', action='store_true', help='fly runs that are already saved')
    parser.add_argument('--detect-rate', type=float, default=DEFAULT_DETECT_RATE,
                        help='images per second for each real detector')
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT,
                        help='folder for intercept.jsonl, detect.jsonl and summary.txt')
    args = parser.parse_args()
    config = load_config(args.config or default_config_path())

    if args.mode == 'summary':
        print(summary_text(args.out, config))
        return 0

    if args.mode == 'intercept':
        aspects = args.aspects or config.intercept.eval_aspects
        speeds = args.speeds or config.intercept.eval_speeds
        seeds = args.seeds if args.seeds is not None else range(config.intercept.eval_runs)
        jobs = [arc_eval.make_intercept_scenario(a, v, s, config.intercept, config.surveillance)
                for a in aspects for v in speeds for s in seeds]
        out_file = args.out / 'intercept.jsonl'
        done = {(r['aspect_deg'], r['speed'], r['seed']) for r in load_runs(out_file)}
        jobs = [(sc, i) for sc, i in jobs
                if args.rerun or (sc.aspect_deg, sc.speed, sc.seed) not in done]
    else:
        seeds = args.seeds if args.seeds is not None else range(config.detect.eval_runs)
        out_file = args.out / 'detect.jsonl'
        done = {r['seed'] for r in load_runs(out_file)}
        jobs = [arc_eval.make_detect_scenario(s, config.detect, config.surveillance)
                for s in seeds if args.rerun or s not in done]
    if not jobs:
        print(f'Every requested run is already in {out_file} (--rerun flies them again).')
        print(summary_text(args.out, config))
        return 0
    print(f'{len(jobs)} {args.mode} runs to fly; results go to {out_file}', flush=True)

    import rclpy
    from rclpy.signals import SignalHandlerOptions

    from swarm_backend.runner import spinning

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('baseline_eval')
    try:
        test = make_runner(node, config, args)
    except (RuntimeError, ValueError) as e:
        print(e)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    code = 0
    with spinning(node):
        try:
            test.setup()
            for k, job in enumerate(jobs):
                if not test.alive():
                    test.log.error('No defenders left')
                    break
                common = {'time': datetime.datetime.now().isoformat(timespec='seconds'),
                          'detector': 'real (detect/live.py)', 'defenders': len(test.alive()),
                          'arc_radius': config.surveillance.arc_radius}
                if args.mode == 'intercept':
                    scenario, designated = job
                    test.log.info(f'Run {k + 1}/{len(jobs)}: aspect {scenario.aspect_deg:.0f} deg, '
                                  f'{scenario.speed} m/s, seed {scenario.seed}, around drone '
                                  f'{test.ids[designated]}')
                    metrics, extra = test.run_intercept(scenario, designated)
                    test.log.info(f'  {metrics.outcome.value}, closest {metrics.closest_miss:.1f} m'
                                  + (f', hit {metrics.base_distance_at_hit:.0f} m from the base'
                                     if metrics.base_distance_at_hit is not None else ''))
                else:
                    scenario = job
                    test.log.info(f'Run {k + 1}/{len(jobs)}: detection, seed {scenario.seed}')
                    metrics, extra = test.run_detect(scenario)
                    test.log.info('  missed' if metrics.missed else
                                  f'  detected at {metrics.first_detection_range:.0f} m')
                save_run(out_file, metrics, {**extra, **common})
        except KeyboardInterrupt:
            test.log.info('Ctrl-C')
        except (TimeoutError, RuntimeError) as e:
            test.log.error(str(e))
            code = 1
        finally:
            try:
                test.stop_and_land()
            except (TimeoutError, KeyboardInterrupt) as e:
                test.log.error(f'Did not land cleanly: {e or "Ctrl-C"}')
                code = 1
    node.destroy_node()
    rclpy.try_shutdown()
    text = summary_text(args.out, config)
    print(text, flush=True)
    (args.out / 'summary.txt').parent.mkdir(parents=True, exist_ok=True)
    (args.out / 'summary.txt').write_text(text + '\n')
    return code


if __name__ == '__main__':
    sys.exit(main())
