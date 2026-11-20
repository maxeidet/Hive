"""Operator UI preview without ROS, PX4 or Gazebo, light enough for a laptop.

The defenders hold their slots on the surveillance arc, as in the eval scenarios
(surveillance: in the config, by default scenarios/surveillance.yaml), each flown by the
real patrol (swarm/patrol.py): it sweeps its camera over its sector, and the first to
see the target through its fake detector turns the real attack (intercept/attack.py) on
it. A simplified base station keeps the others on their slots meanwhile and calls back
one whose attack gave up. In each run the hostile flies straight at the base along a
path of the attack test (evaluation/attack_scenario.py). The operator UI page is served from the same
handler as nodes/operator_ui.py, and the event log is the UI node's.

    python3 tests/ui_preview.py [--realistic] [--port 8090] [--rate 1.0] [--config ...]

Then open http://localhost:8090. The runs loop, the hostile at 4 m/s as in the eval
scenarios: head-on, from 30 degrees right, from 30 degrees left and from 15 degrees right.
Needs only PyYAML. It does not test the ROS side (topics, the UI node's subscriptions):
`mount-backend.sh ui` in the sim does.
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
from swarm_backend.evaluation.arc_eval import slots  # noqa: E402
from swarm_backend.swarm.arc import sectors  # noqa: E402
from swarm_backend.intercept.attack import (  # noqa: E402
    DetectionInbox, attack_status, state_label)
from swarm_backend.swarm.base_station import Mode, Order  # noqa: E402
from swarm_backend.swarm.patrol import PatrolAction  # noqa: E402
from swarm_backend.evaluation.attack_scenario import (  # noqa: E402
    ClosestApproach, Inbound, inbound_path)
from swarm_backend.intercept.detection import REALISTIC, FakeDetector  # noqa: E402
from swarm_backend.nodes.operator_ui import STALE_S, STATIC_DIR, make_handler  # noqa: E402
from swarm_backend.px4.drone import Drone  # noqa: E402
from swarm_backend.ui.model import (  # noqa: E402
    EventLog, SearchArea, attack_view, drone_status, drone_view, load_map, update_search_area)

DT = 0.02  # simulation step, s
HOSTILE_SPEED = 4.0  # m/s, as in the eval scenarios (baseline_eval.sh --speeds 4)
RUNS = [Inbound(HOSTILE_SPEED, bearing_deg=b) for b in (0.0, 30.0, -30.0, 15.0)]
AFTER_PATH_S = 10.0  # keep running after the hostile's path ends: shows lost -> search
HIT_HOLD_S = 3.0  # show the hit this long before the next run
HIT_RADIUS = 1.5
START_BEYOND_ARC_M = 100.0  # the hostile starts this far outside the arc
EVENT_S = 0.2  # the event log looks at the drones this often, as the UI node does


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

    @property
    def attitude(self):  # what PX4 would report: the nose, as the patrol reads it
        return self.attitude_q

    @attitude.setter
    def attitude(self, value):
        pass  # Drone.__init__ sets it to None; here it follows the yaw

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
        self.surveillance = s = config.surveillance
        self.rate = rate
        self.hostile_id = self.config.target_id
        self.slots = slots(s)  # left to right, as drone_ids
        self.sectors = sectors(range(len(s.drone_ids)), s.arc_deg)  # degrees off the heading
        self.area = update_search_area(SearchArea(0.0, 90.0, 100.0, revision=-1),
                                       {'heading_deg': s.threat_heading_deg,
                                        'width_deg': s.arc_deg,
                                        'radius_m': s.arc_radius + s.aim_beyond_arc})
        self._lock = threading.Lock()
        self._views: list[dict] = []
        self.run_name = ''
        self.events = EventLog()
        self._next_event = 0.0
        self.started = time.monotonic()

    # --- what the page asks for ----------------------------------------------

    def state(self, events_after: int = 0) -> dict:
        with self._lock:
            return {'search_area': self.area.to_dict(), 'drones': list(self._views),
                    'map': load_map(STATIC_DIR), 'events': self.events.after(events_after),
                    'uptime_s': round(time.monotonic() - self.started, 1)}

    def set_search_area(self, data: dict) -> SearchArea:
        with self._lock:
            self.area = a = update_search_area(self.area, data)
            self.events.add(time.monotonic() - self.started, 'operator',
                            f'Search sector revision {a.revision}',
                            detail=f'{a.heading_deg:03.0f}° ± {a.width_deg / 2:.0f}°, '
                                   f'{a.radius_m:.0f} m')
            return a

    # --- the runs -------------------------------------------------------------

    def loop(self) -> None:
        while True:
            for scenario in RUNS:
                self._run(scenario)

    def _run(self, scenario: Inbound) -> None:
        c, s = self.config, self.surveillance
        scenario = dataclasses.replace(scenario, start_m=s.arc_radius + START_BEYOND_ARC_M)
        clock = [0.0]
        defenders = [Defender.on_slot(i, drone_id, self.slots[i], self.sectors[i], c, s,
                                      lambda: clock[0])
                     for i, drone_id in enumerate(s.drone_ids)]
        path, duration = inbound_path(scenario, s.base_position, self.slots[0][2],
                                      math.radians(s.threat_heading_deg))
        end = duration + AFTER_PATH_S
        self.run_name = f'inbound {scenario.bearing_deg:+g} deg at {scenario.speed:g} m/s'
        print(f'Run: {self.run_name}', flush=True)
        with self._lock:
            self.events.add(time.monotonic() - self.started, 'operator',
                            f'Preview run: {self.run_name}')
        next_step, hit_at = 0.0, None
        while clock[0] < end and (hit_at is None or clock[0] - hit_at < HIT_HOLD_S):
            started = time.monotonic()
            t = clock[0] = round(clock[0] + DT, 6)
            if t <= duration:
                hostile, hostile_v = path.at(t)
            else:  # flies on past the end of its path: a fast one gets away
                last, hostile_v = path.at(duration - DT)
                hostile = tuple(p + v * (t - duration + DT) for p, v in zip(last, hostile_v))
            if hit_at is None:
                step = t >= next_step - 1e-9
                if step:
                    next_step += 1.0 / 10
                for d in defenders:
                    d.advance(t, hostile, step)
                    if d.approach.hit:
                        hit_at = t
                        if d.patrol.attack is not None:
                            d.patrol.attack.hit()
                        d.drone.velocity = (0.0, 0.0, 0.0)
                        print(f'  drone {d.drone.id} hit after {t:.1f} s', flush=True)
                        break
                if step:
                    self._base_station(defenders)
            self._publish(defenders, hostile, hostile_v, t)
            time.sleep(max(0.0, DT / self.rate - (time.monotonic() - started)))
        if hit_at is None:
            closest = min(d.approach.closest for d in defenders)
            print(f'  no hit, closest {closest:.1f} m', flush=True)

    @staticmethod
    def _base_station(defenders) -> None:
        """The base station's part, simplified: one defender engages (if two saw the
        target in the same step, the first keeps it and the other goes back), the others
        may not turn on the target by themselves meanwhile, and one whose attack gave up
        flies back to its slot (and may start again once there)."""
        engaged = [d for d in defenders if d.patrol.mode is Mode.ENGAGE]
        for d in defenders:
            if d.patrol.mode is Mode.ENGAGE and (d.patrol._lost_sent or d is not engaged[0]):
                d.patrol.apply(Order(d.drone.id, Mode.RETURN))
            elif d.patrol.mode is Mode.PATROL:
                may_start = not engaged
                if d.patrol.may_start != may_start:
                    d.patrol.apply(Order(d.drone.id, Mode.PATROL, sector=d.sector,
                                         may_start=may_start))

    def _publish(self, defenders, hostile, hostile_v, t) -> None:
        views = [d.view(t) for d in defenders]
        # The hostile, as a plain PX4 drone without a gimbal or attack node.
        heading = math.degrees(math.atan2(hostile_v[1], hostile_v[0]))
        enemy = drone_view(self.hostile_id, hostile, hostile_v, yaw_pitch_q(heading), None,
                           False, True, 'AUTO_LOITER', 0.0)
        enemy.update(attack_state=None, attack=None, detection=None, hostile=True,
                     status=drone_status(None, True, 'AUTO_LOITER', 0.0, STALE_S))
        views.append(enemy)
        with self._lock:
            self._views = sorted(views, key=lambda v: v['id'])
            now = time.monotonic()
            if now >= self._next_event:
                self._next_event = now + EVENT_S
                self.events.update(now - self.started, self._views)


@dataclasses.dataclass
class Defender:
    """One defender on its slot, flown by the real PatrolAction (which runs the real
    AttackAction once it sees the target), seeing through its own fake detector."""
    drone: SimDrone
    inbox: DetectionInbox
    detector: FakeDetector
    patrol: PatrolAction
    sector: tuple[float, float]
    approach: ClosestApproach
    detection: tuple | None = None  # (target relative to the drone, clock time)

    @classmethod
    def on_slot(cls, index, drone_id, slot, sector, attack_config, surveillance, clock):
        drone = SimDrone(drone_id, slot, (slot[0], slot[1], 0.0))
        drone.yaw = math.radians(surveillance.threat_heading_deg)
        inbox = DetectionInbox()
        patrol = PatrolAction(drone, inbox, attack_config, surveillance, index, clock)
        patrol.start()
        patrol.apply(Order(drone_id, Mode.PATROL, sector=sector, may_start=True))
        return cls(drone, inbox, FakeDetector(attack_config), patrol, sector,
                   ClosestApproach(HIT_RADIUS))

    def advance(self, t, hostile, step: bool) -> None:
        self.drone.advance(DT)
        if self.detector.wants_image(t):
            self.detector.observe(t, self.drone.camera(), hostile)
        for stamp, found in self.detector.due(t):
            self.inbox.put_position(found.position, stamp, t)
            self.inbox.put_offset(found.offset, stamp, t)
            self.detection = (found.position, t)
        if step:
            self.patrol.step()
        self.approach.update(self.drone.position, hostile, t)

    def view(self, t) -> dict:
        """As DroneMonitor.view builds it from the topics."""
        drone, attack = self.drone, self.patrol.attack
        gimbal_q = None
        if drone.gimbal_aim is not None:
            pitch, yaw = drone.gimbal_aim
            gimbal_q = yaw_pitch_q(yaw, pitch)  # relative to the nose, like the x500 gimbal
        view = drone_view(drone.id, drone.position, drone.velocity, drone.attitude_q, gimbal_q,
                          False, True, 'OFFBOARD', 0.0)
        # The patrol node reports 'search' while it sweeps; nothing on the way back.
        if attack is not None:
            view['attack_state'] = state_label(attack)
            view['attack'] = attack_view(_json(attack_status(attack, t)))
        else:
            view['attack_state'] = 'search' if self.patrol.mode is Mode.PATROL else None
            view['attack'] = None
        view['detection'] = None
        if self.detection is not None and t - self.detection[1] < 2.0:
            rel = self.detection[0]
            view['detection'] = {'north': round(drone.position[0] + rel[0], 2),
                                 'east': round(drone.position[1] + rel[1], 2),
                                 'altitude': round(-(drone.position[2] + rel[2]), 2)}
        view['status'] = drone_status(view['attack_state'], True, 'OFFBOARD', 0.0, STALE_S)
        view['hostile'] = False
        return view


def _json(status: dict) -> str:
    import json
    return json.dumps(status)  # through JSON, as the attack node sends it


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=str(PACKAGE / 'scenarios' / 'surveillance.yaml'))
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
