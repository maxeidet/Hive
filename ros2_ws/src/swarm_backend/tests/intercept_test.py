"""Intercept flight test: one drone chases a hostile drone that flies a pattern.

Both are real PX4 drones in Gazebo at a fixed altitude. The interceptor only knows
what the mock base-station radar measures (range, azimuth and radial velocity, with
noise), and is scored against where the hostile drone really is. Settings come from
the `intercept:` section of swarm.yaml. Inside the container, after
`colcon build` and sourcing install/setup.bash:

    python3 /workspace/ros2_ws/src/swarm_backend/tests/intercept_test.py
    python3 .../intercept_test.py --pattern figure8 --speed 5 --duration 180 --csv /tmp/run.csv

On the host: ./mount-backend.sh intercept [same arguments]. If anything times out,
the drones get within ABORT_RANGE_M of each other, or you press Ctrl-C, both drones
fly back above their start points (at different heights) and land.
"""
import argparse
import csv
import math
import statistics
import time
from dataclasses import replace

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept import guidance
from swarm_backend.intercept.chase import ChaseAction
from swarm_backend.intercept.guidance import Phase
from swarm_backend.intercept.mock import HostileFlight
from swarm_backend.intercept.paths import PATTERNS, make_path
from swarm_backend.intercept.radar import MockRadar, RadarTarget
from swarm_backend.px4.drone import pitch_yaw_deg
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

CLIMB_TIMEOUT_S = 240.0  # 50 m at PX4's climb rate, slower when Gazebo is below real time
ALTITUDE_TOLERANCE_M = 1.0
ABORT_RANGE_M = 3.0
RETURN_GAP_M = 10.0  # the interceptor flies home this much higher than the hostile drone
SAMPLE_PERIOD_S = 0.1


class Run:
    """Everything one test run needs, wired to one ROS node."""

    def __init__(self, node: Node, config, args):
        settings = config.intercept
        overrides = {k: v for k, v in (('hostile_pattern', args.pattern),
                                       ('hostile_speed', args.speed), ('seed', args.seed),
                                       ('altitude', args.altitude)) if v is not None}
        self.settings = settings = replace(settings, **overrides)
        self.args = args
        self.log = node.get_logger()
        rate = config.rate_hz
        self.hostile = PX4Drone(node, config.drone(settings.hostile_id), rate_hz=rate)
        self.hunter = PX4Drone(node, config.drone(settings.interceptor_id), rate_hz=rate)
        self.manager = SwarmManager(Swarm([self.hostile, self.hunter]), ALTITUDE_TOLERANCE_M)

        self.radar = MockRadar(node, self.hostile, settings)
        clock = lambda: self.hunter.stamp  # noqa: E731
        self.chase = ChaseAction(
            self.hunter, [RadarTarget(node, settings, clock)], settings, clock)
        self.flight: HostileFlight | None = None
        node.create_timer(1.0 / rate, self._on_timer)
        self.samples: list[dict] = []

    def _on_timer(self) -> None:
        if self.flight is not None:
            self.flight.step()
        self.chase.step()

    def fly(self) -> None:
        s, args = self.settings, self.args
        self.log.info(f'Hostile drone {self.hostile.id}, interceptor drone {self.hunter.id}')
        try:
            wait_until(self.manager.connected, 30, 'status and position from both drones')
        except TimeoutError:
            raise TimeoutError(
                f'No data from PX4 for drones {self.manager.not_connected()}. Start them with '
                '`./mount-backend.sh spawn`') from None
        wait_until(self.manager.ready, 60, 'pre-flight checks to pass on both drones')

        self.log.info(f'Taking off to {s.altitude} m')
        self.manager.takeoff(s.altitude)
        wait_until(self.manager.all_reached, args.climb_timeout,
                   f'both drones to reach {s.altitude} m')

        # Fly away from the interceptor, so the chase starts with a gap to close.
        heading = guidance.bearing(self.hunter.position, self.hostile.position)
        path = make_path(s.hostile_pattern, self.hostile.position, heading, s.hostile_speed,
                         s.pattern_size, s.seed)
        self.flight = HostileFlight(self.hostile, path, lambda: self.hostile.stamp)
        self.log.info(f'Hostile drone flies {s.hostile_pattern} at {s.hostile_speed} m/s '
                      f'({path.length:.0f} m loop); head start {args.head_start} s')
        self.flight.start()
        time.sleep(args.head_start)

        distance = guidance.horizontal_distance(s.radar_position, self.hostile.position)
        cross = distance * math.radians(s.radar_azimuth_std_deg)
        self.log.info(f'Chasing for {args.duration} s. Radar {distance:.0f} m away: '
                      f'+-{cross:.0f} m across, +-{s.radar_range_std} m along (1 sigma), '
                      f'{s.radar_rate_hz} Hz. Standoff {s.standoff} m, or '
                      f'{s.standoff_sigmas} sigma of the estimate if more')
        self.chase.start()
        start = self.hunter.stamp
        last_phase = None
        while self.hunter.stamp - start < args.duration:
            sample = self._sample(self.hunter.stamp - start)
            if sample['phase'] != last_phase:
                self.log.info(f't={sample["t"]:.1f} s: {sample["phase"].upper()}, '
                              f'range {sample["range"]:.1f} m')
                last_phase = sample['phase']
            if sample['range'] < ABORT_RANGE_M:
                raise RuntimeError(f'Drones within {sample["range"]:.1f} m of each other')
            time.sleep(SAMPLE_PERIOD_S)

    def _sample(self, t: float) -> dict:
        me, truth = self.hunter.position, self.hostile.position
        estimate = self.chase.estimate
        radar, radar_truth = self.radar.last_fix, self.radar.last_true
        sample = {
            't': t,
            'phase': self.chase.phase.value,
            'range': guidance.horizontal_distance(me, truth),
            'standoff': self.chase.standoff,
            'estimate_std': self.chase.estimate_std,
            'estimate_error': math.nan if estimate is None else (
                guidance.horizontal_distance(estimate[0], truth)),
            'radar_error': math.nan if radar is None else (
                guidance.horizontal_distance(radar.position, radar_truth)),
            'gimbal_error': math.nan,
            'hunter_x': me[0], 'hunter_y': me[1], 'hunter_z': me[2],
            'hostile_x': truth[0], 'hostile_y': truth[1], 'hostile_z': truth[2],
        }
        if self.hunter.gimbal is not None and self.hunter.attitude is not None:
            pitch, yaw = self.hunter.gimbal
            heading = pitch_yaw_deg(self.hunter.attitude)[1]
            sample['gimbal_error'] = guidance.pointing_error_deg(pitch, heading + yaw, me, truth)
        self.samples.append(sample)
        return sample

    def stop_and_land(self) -> None:
        self.chase.stop()
        if self.flight is not None:
            self.flight.stop()
        armed = [d for d in (self.hostile, self.hunter) if d.armed]
        if not armed:
            return
        # Straight down from where they are could be into a tree, and their paths home
        # could cross: go home at different heights, then land.
        self.log.info('Returning above the start points')
        for drone, extra in ((self.hostile, 0.0), (self.hunter, RETURN_GAP_M)):
            if drone in armed and drone.offboard:
                x, y, _ = drone.position
                drone.set_position(x, y, drone.home[2] - self.settings.altitude - extra)
        try:
            wait_until(lambda: all(d.reached(ALTITUDE_TOLERANCE_M) for d in armed if d.offboard),
                       60, 'the return height')
            for drone in armed:
                if drone.offboard:
                    _, _, z = drone.target
                    drone.set_position(drone.home[0], drone.home[1], z)
            wait_until(lambda: all(d.reached(2.0) for d in armed if d.offboard),
                       180, 'the drones to get home')
        except (TimeoutError, KeyboardInterrupt) as e:
            self.log.warning(f'Landing where they are ({e or "Ctrl-C"})')
        self.log.info('Landing')
        self.manager.land([d.id for d in armed])
        wait_until(self.manager.all_landed, LAND_TIMEOUT_S, 'both drones to land and disarm')
        self.log.info('Landed')

    def summary(self) -> str:
        s = self.settings
        rows = self.samples
        if not rows:
            return 'No samples recorded.'
        capture = next((r['t'] for r in rows if r['phase'] == Phase.SHADOW.value), None)
        lines = [f'Intercept: {s.hostile_pattern} at {s.hostile_speed} m/s, '
                 f'{rows[-1]["t"]:.0f} s, standoff {s.standoff} m']
        lines.append('  time to capture:      ' + (
            'never' if capture is None else f'{capture:.1f} s'))
        shadow = [r for r in rows if r['phase'] == Phase.SHADOW.value]
        if shadow:
            errors = [abs(r['range'] - r['standoff']) for r in shadow]
            kept = statistics.mean(r['standoff'] for r in shadow)
            lines.append(f'  standoff kept:        mean {kept:.1f} m '
                         f'(configured {s.standoff} m, more while uncertain)')
            lines.append(f'  time in shadow:       {100 * len(shadow) / len(rows):.0f} %')
            lines.append(f'  standoff error:       mean {statistics.mean(errors):.1f} m, '
                         f'p95 {_p95(errors):.1f} m, max {max(errors):.1f} m')
        lines.append(f'  closest approach:     {min(r["range"] for r in rows):.1f} m')
        for key, label, unit in (('radar_error', 'radar error', 'm'),
                                 ('estimate_error', 'tracker error', 'm'),
                                 ('estimate_std', 'tracker 1 sigma', 'm'),
                                 ('gimbal_error', 'gimbal pointing err', 'deg')):
            values = [r[key] for r in (shadow or rows) if not math.isnan(r[key])]
            if values:
                lines.append(f'  {label + ":":<21} mean {statistics.mean(values):.1f} {unit}, '
                             f'p95 {_p95(values):.1f} {unit}')
        return '\n'.join(lines)

    def write_csv(self, path: str) -> None:
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.samples[0]))
            writer.writeheader()
            writer.writerows(self.samples)
        self.log.info(f'Wrote {len(self.samples)} samples to {path}')


def _p95(values: list[float]) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    parser.add_argument('--pattern', choices=PATTERNS, help='hostile pattern (default: config)')
    parser.add_argument('--speed', type=float, help='hostile speed, m/s (default: config)')
    parser.add_argument('--seed', type=int, help='random pattern and radar noise seed')
    parser.add_argument('--altitude', type=float, help='metres (default: config, 50)')
    parser.add_argument('--head-start', type=float, default=10.0,
                        help='seconds the hostile drone flies before the chase starts')
    parser.add_argument('--duration', type=float, default=120.0,
                        help='seconds of chase (simulation time)')
    parser.add_argument('--climb-timeout', type=float, default=CLIMB_TIMEOUT_S,
                        help='wall-clock seconds to wait for the takeoff climb; raise it '
                             'when Gazebo runs far below real time')
    parser.add_argument('--csv', help='write the samples here')
    args = parser.parse_args()

    config = load_config(args.config or default_config_path())

    # Keep Ctrl-C as a KeyboardInterrupt in this thread, so we can still land.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('intercept_test')
    run = Run(node, config, args)

    with spinning(node):
        try:
            run.fly()
        except (TimeoutError, RuntimeError, KeyboardInterrupt) as e:
            run.log.error(f'Aborting ({e or "Ctrl-C"})')
        finally:
            print(run.summary())
            if args.csv and run.samples:
                run.write_csv(args.csv)
            run.stop_and_land()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
