"""Operator UI preview without ROS, PX4 or Gazebo, light enough for a laptop.

The real attack (intercept/attack.py) flies a simulated defender against the hostile
paths of the attack test (intercept/attack_scenario.py), seen through the fake detector,
and the operator UI page is served from the same handler as nodes/operator_ui.py. The
hostile shows as drone 0, so the overlay can be checked against where it really is.

    python3 tests/ui_preview.py [--realistic] [--port 8090] [--rate 1.0]

Then open http://localhost:8090. The runs loop: head-on, crossing, a fast crossing that
gets away (coast, lost, back to search) and a circle. Needs only PyYAML. It does not test
the ROS side (topics, the UI node's subscriptions): `mount-backend.sh ui` in the sim does.
"""
import argparse
import dataclasses
import math
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE))

from swarm_backend.config import load_config  # noqa: E402
from swarm_backend.intercept.attack import (  # noqa: E402
    AttackAction, DetectionInbox, attack_status, state_label)
from swarm_backend.intercept.attack_scenario import (  # noqa: E402
    ClosestApproach, Scenario, hostile_path)
from swarm_backend.intercept.detection import REALISTIC, FakeDetector  # noqa: E402
from swarm_backend.nodes.operator_ui import STALE_S, STATIC_DIR, make_handler  # noqa: E402
from swarm_backend.px4.drone import Drone  # noqa: E402
from swarm_backend.ui.model import (  # noqa: E402
    SearchArea, attack_view, drone_status, drone_view, load_map, update_search_area)

DT = 0.02  # simulation step, s
RUNS = [Scenario('headon', speed=4.0), Scenario('crossing', speed=4.0),
        Scenario('crossing', speed=9.0), Scenario('circle', speed=4.0)]
AFTER_PATH_S = 10.0  # keep running after the hostile's path ends: shows lost -> search
CIRCLE_S = 45.0
HIT_HOLD_S = 3.0  # show the hit this long before the next run
HIT_RADIUS = 1.5


def yaw_pitch_q(yaw_deg: float, pitch_deg: float = 0.0) -> tuple[float, float, float, float]:
    """Quaternion w, x, y, z of a yaw then a pitch (ZYX), as PX4 reports attitudes."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


class SimDrone(Drone):
    """A point that flies setpoints within PX4-like limits (12 m/s across, 3 up, 1.5
    down, 4 m/s^2), with a gimbal that points at once."""

    MAX_SPEED, MAX_UP, MAX_DOWN, MAX_ACCEL, GAIN = 12.0, 3.0, 1.5, 4.0, 1.0

    def __init__(self, drone_id, position, home):
        super().__init__(drone_id, home)
        self.position, self.velocity, self.yaw = position, (0.0, 0.0, 0.0), 0.0
        self.mode, self.command, self.gimbal_aim = 'position', position, None
        self.connected = self.armed = self.offboard = True

    def set_position(self, x, y, z, yaw=None, velocity=None):
        self.mode, self.command, self.target = 'position', (x, y, z), (x, y, z)
        self.yaw = self.yaw if yaw is None else yaw

    def set_velocity(self, vx, vy, vz, yaw=None):
        self.mode, self.command, self.target = 'velocity', (vx, vy, vz), None
        self.yaw = self.yaw if yaw is None else yaw

    def aim_gimbal(self, pitch, yaw):
        self.gimbal_aim = (pitch, yaw)

    def release_gimbal(self):
        pass  # the gimbal stays where it was, as on PX4

    def arm(self): ...
    def disarm(self): ...
    def start_offboard(self): ...
    def land(self): ...

    @property
    def attitude_q(self):
        return yaw_pitch_q(math.degrees(self.yaw))

    def camera(self):
        pitch, yaw = self.gimbal_aim or (0.0, 0.0)
        p, h = math.radians(pitch), self.yaw + math.radians(yaw)
        forward = (math.cos(p) * math.cos(h), math.cos(p) * math.sin(h), -math.sin(p))
        right = (-math.sin(h), math.cos(h), 0.0)
        down = (forward[1] * right[2] - forward[2] * right[1],
                forward[2] * right[0] - forward[0] * right[2],
                forward[0] * right[1] - forward[1] * right[0])
        return {'position': self.position, 'forward': forward, 'right': right, 'down': down}

    def advance(self, dt):
        if self.mode == 'velocity':
            wanted = list(self.command)
        else:
            wanted = [self.GAIN * (c - p) for c, p in zip(self.command, self.position)]
        wanted = _limit(wanted, self.MAX_SPEED)
        wanted[2] = max(-self.MAX_UP, min(self.MAX_DOWN, wanted[2]))
        change = _limit([w - v for w, v in zip(wanted, self.velocity)], self.MAX_ACCEL * dt)
        self.velocity = tuple(v + c for v, c in zip(self.velocity, change))
        self.position = tuple(p + v * dt for p, v in zip(self.position, self.velocity))


def _limit(vector, limit):
    norm = math.hypot(*vector)
    return list(vector) if norm <= limit else [v * limit / norm for v in vector]


class Preview:
    """Runs the attack runs on a thread and answers the page like OperatorUi does."""

    def __init__(self, config, rate: float):
        self.config = config.attack
        self.rate = rate
        self.home = config.drone(self.config.attacker_id).spawn_ned
        self.hostile_id = self.config.target_id
        self.area = update_search_area(SearchArea(0.0, 90.0, 100.0, revision=-1),
                                       {'heading_deg': self.config.search_heading_deg,
                                        'width_deg': 90.0, 'radius_m': 120.0})
        self._lock = threading.Lock()
        self._views: list[dict] = []
        self.run_name = ''

    # --- what the page asks for ----------------------------------------------

    def state(self) -> dict:
        with self._lock:
            return {'search_area': self.area.to_dict(), 'drones': list(self._views),
                    'map': load_map(STATIC_DIR)}

    def set_search_area(self, data: dict) -> SearchArea:
        with self._lock:
            self.area = update_search_area(self.area, data)
            return self.area

    # --- the runs -------------------------------------------------------------

    def loop(self) -> None:
        while True:
            for scenario in RUNS:
                self._run(scenario)

    def _run(self, scenario: Scenario) -> None:
        c = self.config
        drone = SimDrone(c.attacker_id, None, self.home)
        inbox = DetectionInbox()
        clock = [0.0]
        attack = AttackAction(drone, inbox, c, lambda: clock[0])
        drone.position = drone.command = attack.guard
        path, duration = hostile_path(scenario, attack.guard, math.radians(c.search_heading_deg))
        end = CIRCLE_S if duration is None else duration + AFTER_PATH_S
        detector = FakeDetector(c)
        approach = ClosestApproach(HIT_RADIUS)
        detection = None  # (target relative to the defender, clock time)
        self.run_name = f'{scenario.pattern} at {scenario.speed:g} m/s'
        print(f'Run: {self.run_name}', flush=True)
        attack.start()
        next_step, hit_at = 0.0, None
        while clock[0] < end and (hit_at is None or clock[0] - hit_at < HIT_HOLD_S):
            started = time.monotonic()
            t = clock[0] = round(clock[0] + DT, 6)
            if duration is None or t <= duration:
                hostile, hostile_v = path.at(t)
            else:  # flies on past the end of its path: a fast one gets away
                last, hostile_v = path.at(duration - DT)
                hostile = tuple(p + v * (t - duration + DT) for p, v in zip(last, hostile_v))
            if hit_at is None:
                drone.advance(DT)
                if detector.wants_image(t):
                    detector.observe(t, drone.camera(), hostile)
                for stamp, found in detector.due(t):
                    inbox.put_position(found.position, stamp, t)
                    inbox.put_offset(found.offset, stamp, t)
                    detection = (found.position, t)
                if t >= next_step - 1e-9:
                    attack.step()
                    next_step += 1.0 / 10
                approach.update(drone.position, hostile, t)
                if approach.hit:
                    hit_at = t
                    attack.hit()
                    drone.velocity = (0.0, 0.0, 0.0)
                    print(f'  hit after {t:.1f} s', flush=True)
            self._publish(drone, attack, detection, hostile, hostile_v, t)
            time.sleep(max(0.0, DT / self.rate - (time.monotonic() - started)))
        if hit_at is None:
            print(f'  no hit, closest {approach.closest:.1f} m', flush=True)

    def _publish(self, drone, attack, detection, hostile, hostile_v, t) -> None:
        # The defender, as DroneMonitor.view builds it from the topics.
        gimbal_q = None
        if drone.gimbal_aim is not None:
            pitch, yaw = drone.gimbal_aim
            gimbal_q = yaw_pitch_q(yaw, pitch)  # relative to the nose, like the x500 gimbal
        nav = 'OFFBOARD'
        view = drone_view(drone.id, drone.position, drone.velocity, drone.attitude_q, gimbal_q,
                          False, True, nav, 0.0)
        view['attack_state'] = state_label(attack)
        view['attack'] = attack_view(_json(attack_status(attack, t)))
        view['detection'] = None
        if detection is not None and t - detection[1] < 2.0:
            rel = detection[0]
            view['detection'] = {'north': round(drone.position[0] + rel[0], 2),
                                 'east': round(drone.position[1] + rel[1], 2),
                                 'altitude': round(-(drone.position[2] + rel[2]), 2)}
        view['status'] = drone_status(view['attack_state'], True, nav, 0.0, STALE_S)
        view['hostile'] = False
        # The hostile, as a plain PX4 drone without a gimbal or attack node.
        heading = math.degrees(math.atan2(hostile_v[1], hostile_v[0]))
        enemy = drone_view(self.hostile_id, hostile, hostile_v, yaw_pitch_q(heading), None,
                           False, True, 'AUTO_LOITER', 0.0)
        enemy.update(attack_state=None, attack=None, detection=None, hostile=True,
                     status=drone_status(None, True, 'AUTO_LOITER', 0.0, STALE_S))
        with self._lock:
            self._views = sorted([view, enemy], key=lambda v: v['id'])


def _json(status: dict) -> str:
    import json
    return json.dumps(status)  # through JSON, as the attack node sends it


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=str(PACKAGE / 'config' / 'swarm.yaml'))
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--rate', type=float, default=1.0, help='times real time')
    parser.add_argument('--realistic', action='store_true',
                        help='fake detector like the real one: delay, noise, 5 m blind zone')
    args = parser.parse_args(argv)

    config = load_config(args.config)
    if args.realistic:
        config = dataclasses.replace(config, attack=dataclasses.replace(config.attack,
                                                                        **REALISTIC))
    preview = Preview(config, args.rate)
    threading.Thread(target=preview.loop, daemon=True).start()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(preview))
    print(f'Operator UI preview on http://localhost:{args.port}. Ctrl-C stops it.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
