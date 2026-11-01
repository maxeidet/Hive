"""The surveillance plan run on a flat 2D simulation, without ROS or Gazebo.

Three defenders hold points on the arc and sweep their gimbal cameras; hostile drones fly
straight at the base. A defender that sees one is sent by the BaseStation, flies pure
pursuit and hits it, or turns back (leash, lost target). This checks that the rules in
base_station.py work together and gives the numbers the plan asks for: detection and
intercept ranges, and how long each direction on the arc goes unwatched.

Frame: x straight ahead along the threat heading, y to the right, base at the origin,
metres and seconds. Angles are degrees from the heading, positive to the right. Delays,
noise and messages between drones and base station are not modelled yet.

    python3 -m swarm_backend.intercept.surveillance_sim
"""
import argparse
import math
import random
from dataclasses import dataclass, field

from swarm_backend.intercept.base_station import BaseConfig, BaseStation, Mode, Sighting
from swarm_backend.swarm.arc import slot_angles


@dataclass(frozen=True)
class SimConfig:
    radius: float = 60.0  # arc radius, m
    n_drones: int = 3
    defender_speed: float = 5.0  # m/s (the default attack speed)
    detect_range: float = 50.0  # m; the fake detector's range for a 4 pixel target is about 50 m
    fov_deg: float = 20.0  # the narrow camera's width
    sweep_deg_s: float = 30.0  # how fast the aim point moves across the sector, degrees seen from the base
    aim_fraction: float = 0.6  # the camera aims this share of detect_range beyond the arc
    confirm_frames: int = 3  # images in a row before a drone reports a target
    confirmed_hits: int = 10  # images in total before the base counts it as confirmed
    coast_s: float = 2.0  # an attacker that has not seen its target this long gives up
    hit_radius: float = 1.5
    base_radius: float = 5.0  # a hostile this close to the base has got through
    miss_probability: float = 0.0  # chance that a pass at the target does not hit
    dt: float = 0.1
    duration: float = 300.0
    seed: int = 0


@dataclass(frozen=True)
class Hostile:
    name: str
    bearing_deg: float  # where it starts, seen from the base
    range_m: float  # how far out it starts
    speed: float = 1.0  # m/s, straight at the base
    start_s: float = 0.0  # when it appears


@dataclass
class Outcome:
    name: str
    first_seen_s: float | None = None
    first_seen_from_base_m: float | None = None
    destroyed_s: float | None = None
    destroyed_from_base_m: float | None = None
    reached_base_s: float | None = None


@dataclass
class Result:
    outcomes: dict[str, Outcome]
    unwatched_s: dict[int, float]  # 10 degree bin -> longest time nobody looked at it
    drones_lost: int
    events: list[tuple[float, str]] = field(default_factory=list)


class _Defender:

    def __init__(self, drone_id: int, slot_angle: float, config: SimConfig):
        self.id = drone_id
        a = math.radians(slot_angle)
        self.slot = (config.radius * math.cos(a), config.radius * math.sin(a))
        self.slot_angle = slot_angle
        self.pos = self.slot
        self.mode = Mode.PATROL
        self.sector: tuple[float, float] | None = None
        self.target: str | None = None
        self.yaw = slot_angle  # where the camera looks, degrees from the heading
        self.phi = slot_angle  # the direction from the base that the camera aims at
        self.direction = 1.0
        self.streak: dict[str, int] = {}
        self.hits: dict[str, int] = {}
        self.unseen_s = 0.0


def _bearing_deg(frm, to) -> float:
    return math.degrees(math.atan2(to[1] - frm[1], to[0] - frm[0]))


def _angle_diff(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def run(hostiles: list[Hostile], config: SimConfig = SimConfig(),
        base_config: BaseConfig | None = None) -> Result:
    rng = random.Random(config.seed)
    base_config = base_config or BaseConfig()
    ids = list(range(1, config.n_drones + 1))
    defenders = {d.id: d for d in (
        _Defender(i, a, config) for i, a in zip(ids, slot_angles(config.n_drones, base_config.arc_deg)))}
    station = BaseStation(ids, base_config)
    outcomes = {h.name: Outcome(h.name) for h in hostiles}
    positions: dict[str, tuple[float, float]] = {}
    alive: set[str] = set()
    result = Result(outcomes, {}, 0)
    bins = int(base_config.arc_deg // 10)
    last_looked = [0.0] * bins
    worst_gap = [0.0] * bins

    def apply(orders, t):
        for o in orders:
            d = defenders[o.drone_id]
            if o.mode is Mode.ENGAGE:
                d.mode, d.target = Mode.ENGAGE, o.target_id
                d.unseen_s = 0.0
                result.events.append((t, f'drone {d.id} engages {o.target_id}'))
            elif o.mode is Mode.RETURN:
                d.mode, d.target = Mode.RETURN, None
                result.events.append((t, f'drone {d.id} returns'))
            else:
                d.sector = o.sector

    apply(station.tick(0.0), 0.0)
    steps = int(config.duration / config.dt)
    for k in range(1, steps + 1):
        t = k * config.dt
        for h in hostiles:
            if t >= h.start_s and h.name not in positions:
                a = math.radians(h.bearing_deg)
                positions[h.name] = (h.range_m * math.cos(a), h.range_m * math.sin(a))
                alive.add(h.name)
        for h in hostiles:
            if h.name not in alive:
                continue
            x, y = positions[h.name]
            r = math.hypot(x, y)
            step = h.speed * config.dt
            positions[h.name] = (x - x / r * step, y - y / r * step)
            if math.hypot(*positions[h.name]) < config.base_radius:
                alive.discard(h.name)
                outcomes[h.name].reached_base_s = t
                result.events.append((t, f'{h.name} reached the base'))
                station.drop_target(h.name, t)

        sightings = []
        for d in defenders.values():
            if d.mode is Mode.LOST:
                continue
            if d.mode in (Mode.PATROL,):
                _sweep(d, config)
                for name in alive:
                    if _sees(d, positions[name], config):
                        d.streak[name] = d.streak.get(name, 0) + 1
                        d.hits[name] = d.hits.get(name, 0) + 1
                        if d.streak[name] >= config.confirm_frames:
                            sightings.append(Sighting(
                                d.id, name, t, math.hypot(*positions[name]),
                                d.hits[name] >= config.confirmed_hits))
                    else:
                        d.streak[name] = 0
                _log_look(d, t, last_looked, worst_gap, base_config.arc_deg, config)
            elif d.mode is Mode.ENGAGE:
                _fly_attack(d, positions, alive, config, rng, t, station, outcomes, result, apply)
            elif d.mode is Mode.RETURN:
                if _fly_to(d, d.slot, config):
                    d.mode = Mode.PATROL
                    d.phi = d.slot_angle
                    apply(station.on_returned(d.id, t), t)

        for s in sightings:
            o = outcomes[s.target_id]
            if o.first_seen_s is None:
                o.first_seen_s, o.first_seen_from_base_m = t, s.target_range_m
        if sightings:
            apply(station.on_sightings(sightings, t), t)
        for d in defenders.values():
            if d.mode is Mode.ENGAGE:
                apply(station.on_status(d.id, math.hypot(*d.pos), t), t)
        apply(station.tick(t), t)

    result.drones_lost = sum(d.mode is Mode.LOST for d in defenders.values())
    result.unwatched_s = {i * 10: round(g, 1) for i, g in enumerate(worst_gap)}
    return result


def _sweep(d: _Defender, config: SimConfig) -> None:
    """Move the aim point across the sector (angles seen from the base) and point the
    camera at it from where the drone is."""
    lo, hi = d.sector if d.sector is not None else (d.slot_angle, d.slot_angle)
    if hi > lo:
        d.phi += d.direction * config.sweep_deg_s * config.dt
        if d.phi >= hi:
            d.phi, d.direction = hi, -1.0
        elif d.phi <= lo:
            d.phi, d.direction = lo, 1.0
    else:
        d.phi = lo
    aim = config.radius + config.aim_fraction * config.detect_range
    a = math.radians(d.phi)
    d.yaw = _bearing_deg(d.pos, (aim * math.cos(a), aim * math.sin(a)))


def _sees(d: _Defender, target, config: SimConfig) -> bool:
    if math.dist(d.pos, target) > config.detect_range:
        return False
    return abs(_angle_diff(_bearing_deg(d.pos, target), d.yaw)) <= config.fov_deg / 2


def _log_look(d: _Defender, t: float, last_looked, worst_gap, arc_deg: float,
              config: SimConfig) -> None:
    """Which 10 degree bins of the arc this drone's image covers right now (seen from the
    base, at the distance the camera aims at)."""
    aim = config.radius + config.aim_fraction * config.detect_range
    reach = math.dist(d.pos, (aim * math.cos(math.radians(d.phi)), aim * math.sin(math.radians(d.phi))))
    half = math.degrees(math.atan(reach * math.tan(math.radians(config.fov_deg / 2)) / aim))
    for i in range(len(last_looked)):
        centre = -arc_deg / 2 + (i + 0.5) * 10
        if abs(centre - d.phi) <= half:
            worst_gap[i] = max(worst_gap[i], t - last_looked[i])
            last_looked[i] = t


def _fly_to(d: _Defender, point, config: SimConfig) -> bool:
    dx, dy = point[0] - d.pos[0], point[1] - d.pos[1]
    dist = math.hypot(dx, dy)
    step = config.defender_speed * config.dt
    if dist <= step:
        d.pos = point
        return True
    d.pos = (d.pos[0] + dx / dist * step, d.pos[1] + dy / dist * step)
    return False


def _fly_attack(d, positions, alive, config, rng, t, station, outcomes, result, apply) -> None:
    target = d.target
    if target not in alive:
        return  # destroyed by someone else or got through; the base station recalls us
    pos = positions[target]
    if math.dist(d.pos, pos) <= config.detect_range:
        d.unseen_s = 0.0
    else:
        d.unseen_s += config.dt
        if d.unseen_s > config.coast_s:
            apply(station.on_target_lost(d.id, t), t)
            return
    _fly_to(d, pos, config)
    if math.dist(d.pos, pos) <= config.hit_radius:
        if rng.random() < config.miss_probability:
            result.events.append((t, f'drone {d.id} missed {target}'))
            apply(station.on_miss(d.id, drone_alive=True, target_inbound=True, now=t), t)
            return
        alive.discard(target)
        o = outcomes[target]
        o.destroyed_s, o.destroyed_from_base_m = t, math.hypot(*pos)
        d.mode = Mode.LOST
        result.events.append((t, f'drone {d.id} hit {target} at {o.destroyed_from_base_m:.0f} m'))
        apply(station.on_hit(d.id, t), t)


def format_result(result: Result) -> str:
    lines = []
    for o in result.outcomes.values():
        seen = (f'seen at {o.first_seen_from_base_m:.0f} m from the base (t={o.first_seen_s:.1f} s)'
                if o.first_seen_s is not None else 'never seen')
        if o.destroyed_s is not None:
            end = f'hit at {o.destroyed_from_base_m:.0f} m (t={o.destroyed_s:.1f} s)'
        elif o.reached_base_s is not None:
            end = f'REACHED THE BASE (t={o.reached_base_s:.1f} s)'
        else:
            end = 'still flying'
        lines.append(f'{o.name}: {seen}; {end}')
    lines.append(f'defenders lost: {result.drones_lost}')
    worst = max(result.unwatched_s.values(), default=0.0)
    lines.append(f'longest time a 10 degree slice went unwatched: {worst:.1f} s')
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--second-delay', type=float, default=20.0,
                        help='seconds between the two hostiles reaching detection range')
    parser.add_argument('--miss', type=float, default=0.0, help='miss probability')
    parser.add_argument('--events', action='store_true', help='print the event log')
    args = parser.parse_args(argv)
    hostiles = [Hostile('A', -10.0, 150.0), Hostile('B', 25.0, 150.0 + args.second_delay)]
    result = run(hostiles, SimConfig(miss_probability=args.miss),
                 BaseConfig(leash_m=200.0, close_radius_m=100.0))
    if args.events:
        for t, text in result.events:
            print(f'{t:7.1f} s  {text}')
    print(format_result(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
