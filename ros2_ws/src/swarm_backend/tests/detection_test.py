"""Baseline detection flight test: how far away does the camera see a drone against the sky?

The watcher (drone 2, gz_x500_gimbal) hovers at --alt and keeps its gimbal on the target
(drone 0) using the true positions, while the target flies straight away from it at
--target-alt, from --near to --far metres. Every processed image is scored against where
the target should be in the image (the camera points at it, corrected for the gimbal's
reported pointing error), so the summary gives detection probability against range and
pixel size, and false confirmed tracks. Inside the container, after colcon build:

    python3 /workspace/ros2_ws/src/swarm_backend/tests/detection_test.py --save /tmp/frames

On the host: ./mount-backend.sh detecttest [same arguments]. Timeouts are in simulation
time, so a slow Gazebo (real-time factor well below 1) only makes the run take longer.
Ctrl-C or a timeout flies both drones home and lands them.
"""
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from swarm_backend.config import default_config_path, load_config
from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.candidates import Settings
from swarm_backend.detect.live import LiveDetector, Result
from swarm_backend.detect.tracking import Confirmer
from swarm_backend.intercept import guidance
from swarm_backend.px4.drone import pitch_yaw_deg, wrap_deg
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import spinning, wait_until
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

TOLERANCE_M = 1.0
CLIMB_TIMEOUT_S = 120.0  # simulation seconds
MOVE_TIMEOUT_S = 120.0
LAND_TIMEOUT_S = 120.0
STEP_S = 0.1
BIN_M = 10.0


class Run:

    def __init__(self, node: Node, config, args):
        self.args = args
        self.log = node.get_logger()
        self.target = PX4Drone(node, config.drone(args.target), rate_hz=config.rate_hz)
        self.watcher = PX4Drone(node, config.drone(args.watcher), rate_hz=config.rate_hz)
        self.manager = SwarmManager(Swarm([self.target, self.watcher]), TOLERANCE_M)
        self.camera = CameraModel()
        self.aiming = False
        self.recording = False
        self.rows: list[dict] = []
        node.create_timer(STEP_S, self._aim)
        self.detector = LiveDetector(
            node, args.watcher,
            Settings(kernel=args.kernel, contrast=args.contrast),
            Confirmer(k=args.k, gate=args.gate), rate_hz=args.rate,
            horizon=self._horizon, on_result=self._on_result,
            save=args.save, save_every=args.save_every)

    # --- callbacks (ROS thread) --------------------------------------------

    def _heading(self) -> float | None:
        if self.watcher.attitude is None:
            return None
        return math.radians(pitch_yaw_deg(self.watcher.attitude)[1])

    def _aim(self) -> None:
        heading = self._heading()
        me, target = self.watcher.position, self.target.position
        if not self.aiming or heading is None or me is None or target is None:
            return
        self.watcher.aim_gimbal(*guidance.gimbal_angles(me, heading, target))

    def _horizon(self) -> float | None:
        if self.watcher.gimbal is None:
            return None
        return self.camera.horizon_row(self.watcher.gimbal[0]) - self.args.horizon_margin

    def expected_pixel(self) -> tuple[float, float] | None:
        """Where the target should be in the image, from the gimbal's reported angles.

        Small-angle model around the image centre; replaced by camera.project in step 1.
        """
        heading = self._heading()
        me, target = self.watcher.position, self.target.position
        if heading is None or self.watcher.gimbal is None or None in (me, target):
            return None
        pitch, yaw = self.watcher.gimbal
        look_az = math.degrees(heading) + yaw
        target_pitch, target_yaw = guidance.gimbal_angles(me, 0.0, target)  # yaw from north
        d_az = math.radians(wrap_deg(target_yaw - look_az))
        d_el = math.radians(target_pitch - pitch)
        cam = self.camera
        return (cam.cx + cam.f * math.tan(d_az) * math.cos(math.radians(target_pitch)),
                cam.cy - cam.f * math.tan(d_el))

    def _on_result(self, result: Result) -> None:
        if not self.recording:
            return
        expected = self.expected_pixel()
        if expected is None:
            return
        me, target = self.watcher.position, self.target.position
        distance = math.dist(me, target)
        u, v = expected

        def offset(box):
            return math.hypot(box.u - u, box.v - v)

        tol = self.args.tolerance
        near = [b for b in result.boxes if offset(b) <= tol]
        confirmed = [t for t in result.tracks if t.confirmed and t.detection is not None]
        hits = [t for t in confirmed if offset(t.detection) <= tol]
        best = min(hits, key=lambda t: offset(t.detection)).detection if hits else None
        self.rows.append({
            't': result.stamp,
            'range': distance,
            'expected_px': self.camera.width_at(distance),
            'expected_u': u, 'expected_v': v,
            'horizon': result.horizon if result.horizon is not None else math.nan,
            'candidates': len(result.boxes),
            'candidate_near': bool(near),
            'confirmed_near': bool(hits),
            'false_confirmed': len(confirmed) - len(hits),
            'box_u': best.u if best else math.nan, 'box_v': best.v if best else math.nan,
            'box_w': best.width if best else math.nan,
            'contrast': best.contrast if best else math.nan,
            'ms': result.ms,
        })

    # --- the flight (main thread) ------------------------------------------

    def wait(self, predicate, sim_timeout: float, what: str) -> None:
        """Like runner.wait_until, but the timeout is in simulation time."""
        start = None
        while not predicate():
            now = self.watcher.stamp
            if start is None:
                start = now
            elif now is not None and now - start > sim_timeout:
                raise TimeoutError(f'Timed out after {sim_timeout:.0f} sim s waiting for {what}')
            time.sleep(0.1)

    def fly(self) -> None:
        args = self.args
        try:
            wait_until(self.manager.connected, 60, 'status and position from both drones')
        except TimeoutError:
            raise TimeoutError(f'No data from drones {self.manager.not_connected()}. '
                               'Start them with ./mount-backend.sh spawn') from None
        self.wait(self.manager.ready, 60, 'pre-flight checks on both drones')

        self.log.info(f'Taking off: watcher {self.watcher.id} to {args.alt} m, '
                      f'target {self.target.id} to {args.target_alt} m')
        self.manager.takeoff(args.alt, [self.watcher.id])
        self.manager.takeoff(args.target_alt, [self.target.id])
        self.wait(self.manager.all_reached, CLIMB_TIMEOUT_S, 'both drones to climb')

        bearing = math.radians(args.bearing)
        wx, wy, _ = self.watcher.position
        self.watcher.set_position(*self.watcher.target, yaw=bearing)
        tz = self.target.target[2]

        def along(distance):
            return wx + distance * math.cos(bearing), wy + distance * math.sin(bearing)

        self.log.info(f'Target to {args.near} m from the watcher, bearing {args.bearing} deg')
        self.target.set_position(*along(args.near), tz)
        self.aiming = True
        self.wait(lambda: self.target.reached(TOLERANCE_M), MOVE_TIMEOUT_S, 'the start point')
        self.wait(lambda: self.watcher.gimbal is not None, 30, 'the gimbal')

        speed = args.speed
        velocity = (speed * math.cos(bearing), speed * math.sin(bearing), 0.0)
        self.log.info(f'Target flies away at {speed} m/s to {args.far} m '
                      f'({self.camera.width_at(args.near):.0f} -> '
                      f'{self.camera.width_at(args.far):.1f} px)')
        self.recording = True
        start = self.target.stamp
        last_log = -1.0
        while True:
            t = self.target.stamp - start
            distance = args.near + speed * t
            if distance >= args.far:
                break
            self.target.set_position(*along(distance), tz, velocity=velocity)
            if t - last_log >= 5.0 and self.rows:
                recent = self.rows[-20:]
                rate = sum(r['confirmed_near'] for r in recent) / len(recent)
                self.log.info(f't={t:.0f} s: range {recent[-1]["range"]:.0f} m '
                              f'(~{recent[-1]["expected_px"]:.1f} px), '
                              f'confirmed {100 * rate:.0f} % of recent images, '
                              f'{recent[-1]["candidates"]} candidates')
                last_log = t
            time.sleep(STEP_S)
        self.recording = False

    def stop_and_land(self) -> None:
        self.recording = False
        self.aiming = False
        self.watcher.release_gimbal()
        armed = [d for d in (self.target, self.watcher) if d.armed]
        if not armed:
            return
        self.log.info('Returning above the start points')
        for drone in armed:
            if drone.offboard:
                _, _, z = drone.target or drone.position
                drone.set_position(drone.home[0], drone.home[1], z)
        try:
            self.wait(lambda: all(d.reached(2.0) for d in armed if d.offboard),
                      MOVE_TIMEOUT_S, 'the drones to get home')
        except (TimeoutError, KeyboardInterrupt) as e:
            self.log.warning(f'Landing where they are ({e or "Ctrl-C"})')
        self.log.info('Landing')
        self.manager.land([d.id for d in armed])
        self.wait(self.manager.all_landed, LAND_TIMEOUT_S, 'both drones to land')
        self.log.info('Landed')

    def summary(self) -> str:
        rows = self.rows
        if not rows:
            return 'No images recorded.'
        lines = [f'Baseline detection, {len(rows)} images, k={self.args.k}, '
                 f'tolerance {self.args.tolerance} px',
                 f'  {"range m":>10} {"~px":>5} {"images":>6} {"candidate":>9} '
                 f'{"confirmed":>9} {"false/img":>9}']
        lo = math.floor(min(r['range'] for r in rows) / BIN_M) * BIN_M
        while lo <= max(r['range'] for r in rows):
            bin_rows = [r for r in rows if lo <= r['range'] < lo + BIN_M]
            if bin_rows:
                n = len(bin_rows)
                lines.append(
                    f'  {f"{lo:.0f}-{lo + BIN_M:.0f}":>10} '
                    f'{self.camera.width_at(lo + BIN_M / 2):5.1f} {n:6d} '
                    f'{100 * sum(r["candidate_near"] for r in bin_rows) / n:8.0f}% '
                    f'{100 * sum(r["confirmed_near"] for r in bin_rows) / n:8.0f}% '
                    f'{sum(r["false_confirmed"] for r in bin_rows) / n:9.2f}')
            lo += BIN_M
        hits = [r for r in rows if r['confirmed_near']]
        if hits:
            du = sorted(r['box_u'] - r['expected_u'] for r in hits)
            dv = sorted(r['box_v'] - r['expected_v'] for r in hits)
            lines.append(f'  box - expected (median): du {du[len(du) // 2]:+.1f} px, '
                         f'dv {dv[len(dv) // 2]:+.1f} px (pointing / frame error)')
        lines.append(f'  processing: {sum(r["ms"] for r in rows) / len(rows):.0f} ms/image')
        return '\n'.join(lines)

    def write_csv(self, path: str) -> None:
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        self.log.info(f'Wrote {len(self.rows)} rows to {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    parser.add_argument('--watcher', type=int, default=2, help='drone with the gimbal camera')
    parser.add_argument('--target', type=int, default=0, help='drone to detect')
    parser.add_argument('--alt', type=float, default=30.0, help='watcher altitude, m')
    parser.add_argument('--target-alt', type=float, default=45.0, help='target altitude, m')
    parser.add_argument('--bearing', type=float, default=0.0,
                        help='degrees from north, direction the target flies away in')
    parser.add_argument('--near', type=float, default=15.0, help='start distance, m (horizontal)')
    parser.add_argument('--far', type=float, default=70.0, help='end distance, m (horizontal)')
    parser.add_argument('--speed', type=float, default=2.0, help='target speed, m/s')
    parser.add_argument('--rate', type=float, default=10.0, help='images processed per second')
    parser.add_argument('--k', type=int, default=3)
    parser.add_argument('--gate', type=float, default=20.0, help='px')
    parser.add_argument('--tolerance', type=float, default=40.0,
                        help='px between a box and the expected target position to count')
    parser.add_argument('--horizon-margin', type=float, default=10.0,
                        help='px above the computed horizon row that are not searched either')
    parser.add_argument('--kernel', type=int, default=Settings.kernel)
    parser.add_argument('--contrast', type=float, default=Settings.contrast)
    parser.add_argument('--save', type=Path, help='folder for raw + annotated images')
    parser.add_argument('--save-every', type=float, default=2.0, help='sim seconds')
    parser.add_argument('--csv', help='write one row per image here')
    args = parser.parse_args()

    config = load_config(args.config or default_config_path())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('detection_test')
    run = Run(node, config, args)

    with spinning(node):
        try:
            run.fly()
        except (TimeoutError, RuntimeError, KeyboardInterrupt) as e:
            run.log.error(f'Aborting ({e or "Ctrl-C"})')
        finally:
            print(run.summary(), flush=True)
            if args.csv and run.rows:
                run.write_csv(args.csv)
            run.stop_and_land()
            run.detector.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
