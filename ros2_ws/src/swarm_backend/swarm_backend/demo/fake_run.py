"""A made-up run folder in the same layout as a real recording, for tests and dry runs.

    python3 -m swarm_backend.demo.fake_run DIR [--takes N] [--no-videos]

Three defenders on the arc (x500_gimbal_1..3), one hostile (x500_4), one bird. Per take, as
gazebo_surveillance.py does it: the hostile is put on the ground at its start point (a jump
of more than 100 m), arms, climbs straight up, hovers, then flies at the base at 4 m/s; the
take's attacker sees it and intercepts; on a hit the hostile is moved to the graveyard and
its link is lost (the Gazebo flow logs no "hit the target": the patrol is called back before
the attack reports one; `hit_event=True` adds it). After the last take the defenders land and
the hostile is put back on its spawn point. The UI events use the
texts of ui/model.py's EventLog, the poses the format of camera_video.PoseFeed, the clips
the .json of debug_video.DroneVideo. The sim runs at half speed (`wall` = 2 x sim).
"""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

from swarm_backend.demo.runinfo import write_run_info

T0 = 100.0            # sim time of the recording's first line
TAKE_S = 60.0         # each take's length
PLACE_AFTER = 5.0     # the hostile is put in place this long into the take
CLIMB = (1.0, 11.0)   # it climbs from the ground to START's height between these (after placing)
WAIT_S = 16.0         # ...and sets off horizontally this long after being placed
SEE_AFTER = 10.0      # the attacker sees it this long after release
HIT_AFTER = 4.0       # ...and hits it this long after the sighting
SPEED = 4.0           # m/s, toward the base
UI_HZ, POSE_HZ, FPS = 10.0, 25.0, 25
WALL0 = 1763460000.0
SLOTS = {1: (-106.0, 106.0, 40.0), 2: (0.0, 150.0, 40.0), 3: (106.0, 106.0, 40.0)}  # ENU
PARK = (-5.0, 0.0, 0.0)
START = (0.0, 300.0, 50.0)
RESET_BEFORE_END = 2.0
MISS_OUT_S = 6.0      # a missed run ends (the hostile taken out) this long after the closest pass  # the hostile back on its spawn point this long before the end
GRAVEYARD = (450.0, -450.0, 50.0)
# hostiles=2: drone 5 flies in from the north-west with drone 4, drone 1 is sent at it
START2 = (-200.0, 220.0, 50.0)
GRAVEYARD2 = (450.0, -430.0, 50.0)   # gazebo_surveillance: drone 5's spot, 20 m from drone 4's
ATTACKER2 = 1
SEE2_AFTER = 2.0                      # drone 1 sees it this long after the first sighting
HIT2_AFTER = 5.0                      # ...and hits it this long after its own sighting
CONFIG2 = Path(__file__).resolve().parents[2] / 'scenarios' / 'surveillance2.yaml'
ATTACKERS = (3, 2)
CAM_POS = (0.0, 100.0, 50.0)  # a fixed "chase camera" looking up the hostile's path
CONFIG = Path(__file__).resolve().parents[2] / 'scenarios' / 'surveillance.yaml'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'


def _cam_q():
    from swarm_backend.ui.camera_video import look_at
    return look_at(CAM_POS, (0.0, 300.0, 50.0))


CAM_Q = _cam_q()


def _lerp(a, b, k):
    k = min(1.0, max(0.0, k))
    return tuple(x + (y - x) * k for x, y in zip(a, b))


def make_run(folder: Path, *, takes: int = 1, hit: list[bool] | None = None,
             teleport: bool = True, videos: bool = True, cam_start_delay: float = 0.0,
             screen: bool = False, hit_event: bool = False, detect: bool = True,
             attackers: tuple[int, ...] = ATTACKERS, boxes: bool = True,
             early_sighting: bool = False, birds_late: bool = False, hostiles: int = 1,
             takeout_delay: float = 0.1) -> dict:
    """hostiles=2: one take, drone 5 too; `hit` is then per hostile ([drone 4, drone 5])."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    two = hostiles == 2
    hit2 = True
    if two:
        takes = 1
        hit, hit2 = ([True, True] if hit is None else list(hit))[:2]
        hit = [hit]
        attackers = tuple(a for a in attackers if a != ATTACKER2) or (3,)
    hit = list(hit) if hit is not None else [True] * takes
    end = T0 + takes * TAKE_S
    plan = []
    for k in range(takes):
        base = T0 + k * TAKE_S
        place = base + PLACE_AFTER if (teleport or k > 0) else T0
        release = place + WAIT_S
        sighting = release + SEE_AFTER
        h = sighting + HIT_AFTER if hit[k] else None
        plan.append({'place': place, 'release': release, 'sighting': sighting, 'hit': h,
                     'attacker': attackers[k % len(attackers)], 'end': base + TAKE_S})

    def take_at(sim):
        for p in reversed(plan):
            if sim >= p['place']:
                return p
        return None

    def hostile(sim):
        p = take_at(sim)
        if p is None or (teleport and sim >= end - RESET_BEFORE_END):
            return PARK
        if p['hit'] is not None and sim >= p['hit'] + takeout_delay:
            return GRAVEYARD
        if p['hit'] is not None and sim >= p['hit']:  # hit, tumbling down until taken out
            hx, hy, hz = _lerp(START, (0.0, 0.0, 50.0), (p['hit'] - p['release']) * SPEED / START[1])
            return (hx, hy, max(0.0, hz - 2.0 * (sim - p['hit'])))
        if p['hit'] is None and sim >= p['sighting'] + HIT_AFTER + MISS_OUT_S:
            return GRAVEYARD  # as Gazebo: the hostile is taken out after every run (fly's finally)
        if sim < p['release']:
            if not (teleport or p is not plan[0]):
                return START
            k = (sim - p['place'] - CLIMB[0]) / (CLIMB[1] - CLIMB[0])
            return _lerp((START[0], START[1], 0.0), START, k)
        return _lerp(START, (0.0, 0.0, 50.0), (sim - p['release']) * SPEED / START[1])

    def defender(i, sim):
        slot = SLOTS[i]
        p = take_at(sim)
        if p is None or p['attacker'] != i or sim < p['sighting']:
            return slot
        meet = p['hit'] if p['hit'] is not None else p['sighting'] + HIT_AFTER
        miss = (1.0, 0.0, 0.0) if p['hit'] is not None else (6.0, 0.0, 0.0)
        goal = tuple(a + b for a, b in zip(hostile(min(meet, p['end']) - 0.2), miss))
        if sim <= meet:
            return _lerp(slot, goal, (sim - p['sighting']) / (meet - p['sighting']))
        return _lerp(goal, slot, (sim - meet) / 10.0)

    q = None  # the second hostile's timeline (hostiles=2), released with the first
    if two:
        s2 = plan[0]['sighting'] + SEE2_AFTER
        q = {'place': plan[0]['place'], 'release': plan[0]['release'], 'sighting': s2,
             'hit': s2 + HIT2_AFTER if hit2 else None, 'end': plan[0]['end']}

    def hostile2(sim):
        if q is None or sim < q['place'] or (teleport and sim >= end - RESET_BEFORE_END):
            return (-5.0, 4.0, 0.0)
        if q['hit'] is not None and sim >= q['hit'] + 0.1:
            return GRAVEYARD2
        if q['hit'] is None and sim >= q['sighting'] + HIT2_AFTER + MISS_OUT_S:
            return GRAVEYARD2
        if sim < q['release']:
            k = (sim - q['place'] - CLIMB[0]) / (CLIMB[1] - CLIMB[0])
            return _lerp((START2[0], START2[1], 0.0), START2, k)
        return _lerp(START2, (0.0, 0.0, 50.0),
                     (sim - q['release']) * SPEED / math.hypot(START2[0], START2[1]))

    defender1 = defender

    def defender(i, sim):  # noqa: F811 - the attacker of the second hostile flies at it
        if q is None or i != ATTACKER2 or sim < q['sighting']:
            return defender1(i, sim)
        slot = SLOTS[i]
        meet = q['hit'] if q['hit'] is not None else q['sighting'] + HIT2_AFTER
        miss = (1.0, 0.0, 0.0) if q['hit'] is not None else (6.0, 0.0, 0.0)
        goal = tuple(a + b for a, b in zip(hostile2(min(meet, q['end']) - 0.2), miss))
        if sim <= meet:
            return _lerp(slot, goal, (sim - q['sighting']) / (meet - q['sighting']))
        return _lerp(goal, slot, (sim - meet) / 10.0)

    def bird(sim):
        a = sim / 8.0
        return (40.0 * math.cos(a), 200.0 + 40.0 * math.sin(a), 30.0)

    # poses.jsonl
    with open(folder / 'poses.jsonl', 'w') as fh:
        n = int((end - T0) * POSE_HZ)
        for j in range(n + 1):
            sim = round(T0 + j / POSE_HZ, 3)
            poses = {f'x500_gimbal_{i}': [*defender(i, sim), 0, 0, 0, 1] for i in SLOTS}
            poses['x500_4'] = [*hostile(sim), 0, 0, 0, 1]
            if two:
                poses['x500_5'] = [*hostile2(sim), 0, 0, 0, 1]
            poses['bird_1'] = [*bird(sim), 0, 0, 0, 1]
            if birds_late and sim >= plan[0]['release']:  # spawned with the release (--birds)
                poses['bird_7'] = [60.0, 120.0 - 2.0 * (sim - plan[0]['release']), 35.0, 0, 0, 0, 1]
            poses['rec_cam_attacker_watch'] = [*CAM_POS, *CAM_Q]
            fh.write(json.dumps({'sim': sim, 'poses': poses}) + '\n')

    # ui.jsonl
    events, next_id = [], [1]

    def event(sim, kind, text, drone, detail='', level=None):
        events.append({'id': next_id[0], 't': round(sim - T0, 1), 'kind': kind, 'text': text,
                       'drone': drone, 'detail': detail, 'level': level, '_sim': sim})
        next_id[0] += 1

    for i in SLOTS:
        event(T0 + 2.0, 'fleet', f'Drone {i} armed', i)
    for p in plan:
        a = p['attacker']
        if teleport or p is not plan[0]:
            event(p['place'] + 0.5, 'fleet', 'Drone 4 armed', 4)
        if early_sighting:  # the hostile seen parked at its start, before the release
            other = next(i for i in SLOTS if i != a)
            event(p['place'] + 1.0, 'detect', f'Drone {other} sees the target', other, '2 m up')
            event(p['place'] + 1.3, 'attack', f'Drone {other} searching → intercept', other, 'bad')
        event(p['sighting'], 'detect', f'Drone {a} sees the target', a, '50 m up')
        event(p['sighting'] + 0.3, 'attack', f'Drone {a} searching → intercept', a, 'bad')
        if p['hit'] is not None:
            if hit_event:
                event(p['hit'], 'attack', f'Drone {a} hit the target', a, '', 'good')
            event(p['hit'] + 2.0, 'fleet', 'Drone 4 link lost', 4, 'last message 2 s ago', 'bad')
            event(p['hit'] + 2.0, 'attack', f'Drone {a} back to searching', a)
    if q is not None:
        a = ATTACKER2
        event(q['sighting'], 'detect', f'Drone {a} sees the target', a, '50 m up')
        event(q['sighting'] + 0.3, 'attack', f'Drone {a} searching → intercept', a, 'bad')
        if q['hit'] is not None:
            event(q['hit'] + 2.0, 'fleet', 'Drone 5 link lost', 5, 'last message 2 s ago', 'bad')
            event(q['hit'] + 2.0, 'attack', f'Drone {a} back to searching', a)
    for i in SLOTS:
        event(end - 3.0, 'fleet', f'Drone {i} landed and disarmed', i)
    events.sort(key=lambda e: e['_sim'])

    def status(i, sim):
        if q is not None and i == ATTACKER2:
            p = q if sim >= q['sighting'] else None
            if p is None or sim < p['sighting'] + 0.3:
                return 'searching'
            if p['hit'] is not None and sim >= p['hit']:
                return 'hit' if sim < p['hit'] + 2.0 else 'searching'
            return 'intercept'
        p = take_at(sim)
        if p is None or p['attacker'] != i or sim < p['sighting'] + 0.3:
            return 'searching'
        if p['hit'] is not None and p['hit'] <= sim < p['hit'] + 2.0:
            return 'hit'
        if p['hit'] is not None and sim >= p['hit'] + 2.0:
            return 'searching'
        return 'intercept'

    def view(name, did, pos, sim, is_hostile):
        e, n, u = pos
        v = {'id': did, 'north': n, 'east': e, 'altitude': u, 'armed': True, 'hostile': is_hostile,
             'nav_state': 'OFFBOARD', 'age_s': 0.1, 'gimbal_heading_deg': None, 'detection': None,
             'attack': None, 'status': 'searching' if not is_hostile else 'flying'}
        return v

    with open(folder / 'ui.jsonl', 'w') as fh:
        k = 0
        n = int((end - T0) * UI_HZ)
        for j in range(n + 1):
            sim = round(T0 + j / UI_HZ, 3)
            drones = []
            for i in SLOTS:
                v = view(f'x500_gimbal_{i}', i, defender(i, sim), sim, False)
                v['gimbal_heading_deg'] = (20.0 * math.sin(sim / 3.0) + 30.0 * (i - 2)) % 360
                v['status'] = status(i, sim)
                p = take_at(sim)
                target = hostile
                if q is not None and i == ATTACKER2:
                    p, target = dict(q, attacker=i), hostile2
                if (detect and p is not None and p['attacker'] == i and sim >= p['sighting']
                        and (p['hit'] is None or sim < p['hit'])):
                    he, hn, hu = target(sim)
                    v['detection'] = {'north': hn + 1.5, 'east': he - 1.0, 'altitude': hu}
                if v['status'] == 'intercept':
                    he, hn, hu = target(sim)
                    de, dn, du = defender(i, sim)
                    n = math.dist((he, hn, hu), (de, dn, du)) or 1.0
                    v['attack'] = {'target': {'north': hn, 'east': he, 'altitude': hu},
                                   'velocity': {'north': 5 * (hn - dn) / n, 'east': 5 * (he - de) / n,
                                                'up': 5 * (hu - du) / n},
                                   'range_m': round(n, 1)}
                drones.append(v)
            drones.append(view('x500_4', 4, hostile(sim), sim, True))
            if two:
                drones.append(view('x500_5', 5, hostile2(sim), sim, True))
            new = []
            while k < len(events) and events[k]['_sim'] <= sim + 1e-9:
                new.append({x: y for x, y in events[k].items() if x != '_sim'})
                k += 1
            state = {'drones': drones, 'events': new, 'uptime_s': round(sim - T0, 1),
                     'search_area': {}}
            fh.write(json.dumps({'sim': sim, 'wall': round(WALL0 + 2 * (sim - T0), 3),
                                 'state': state}) + '\n')

    write_run_info(folder, CONFIG2 if two else CONFIG,
                   ['scenarios/trial.sh', str(takes)] + (['--hostiles', '2'] if two else []),
                   {'fake': True})

    if videos:
        for i in SLOTS:
            _clip(folder, f'debug_d{i}', T0, end, {'drone': i})
        for angle in ('attacker_watch', 'arc_side'):
            _clip(folder, f'cam_{angle}', T0 + cam_start_delay, end,
                  {'drone': 0, 'hfov': 1.2, 'model': f'rec_cam_{angle}', 'angle': angle})
    if screen:
        _screen(folder, T0 + 4.0, end)
    if boxes:  # the detectors' --record files (detect/live.py result_record), 640x360 images
        for p in plan:
            stop = p['hit'] if p['hit'] is not None else p['sighting'] + HIT_AFTER
            with open(folder / f'detect_d{p["attacker"]}.jsonl', 'a') as fh:
                for j in range(int((stop - p['sighting']) / 0.05)):
                    sim = round(p['sighting'] + j * 0.05, 3)
                    k = (sim - p['sighting']) / (stop - p['sighting'])
                    fh.write(json.dumps({'stamp': sim, 'width': 640, 'height': 360, 'boxes': [],
                                         'confirmed': [[200 + 240 * k, 180, 20, 14, 7],
                                                       [500, 100, 12, 9, 9]],
                                         'target': 7}) + '\n')
    return {'release': [p['release'] for p in plan], 'sighting': [p['sighting'] for p in plan],
            'hit': [p['hit'] for p in plan], 'attacker': [p['attacker'] for p in plan],
            'contact': [p['hit'] for p in plan],
            **({'hit2': q['hit'], 'sighting2': q['sighting'], 'attacker2': ATTACKER2}
               if q is not None else {})}


def _clip(folder: Path, stem: str, start: float, end: float, meta: dict) -> None:
    """A test picture with the sim time written in, 25 fps from `start` to `end`."""
    dur = end - start
    text = f"sim %{{eif\\:t+{start:g}\\:d}}"
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'lavfi', '-i',
                    f'testsrc2=size=320x180:rate={FPS}:duration={dur:.3f}',
                    '-vf', f"drawtext=fontfile={FONT}:text='{text}':x=10:y=10:fontsize=24:fontcolor=white",
                    '-c:v', 'libx264', '-preset', 'ultrafast', '-pix_fmt', 'yuv420p',
                    str(folder / f'{stem}.mkv')], check=True)
    (folder / f'{stem}.json').write_text(json.dumps({
        'start': start, 'fps': float(FPS), 'end': end, 'images': int(dur * FPS), 'dropped': 0,
        'width': 320, 'height': 180, **meta}, indent=1) + '\n')


def _screen(folder: Path, start_sim: float, end_sim: float) -> None:
    """screen.mkv on the computer's clock (wall = WALL0 + 2 (sim - T0)), from `start_sim`."""
    wall0 = WALL0 + 2 * (start_sim - T0)
    dur = 2 * (end_sim - start_sim)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'lavfi', '-i',
                    f'testsrc2=size=320x180:rate={FPS}:duration={dur:.3f}',
                    '-c:v', 'libx264', '-preset', 'ultrafast', '-pix_fmt', 'yuv420p',
                    '-output_ts_offset', f'{wall0:.3f}', str(folder / 'screen.mkv')], check=True)
    (folder / 'screen.json').write_text(json.dumps(
        {'display': ':0', 'region': [0, 0, 320, 180], 'fps': FPS}) + '\n')


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('folder', type=Path)
    p.add_argument('--takes', type=int, default=1)
    p.add_argument('--no-videos', action='store_true')
    p.add_argument('--screen', action='store_true')
    args = p.parse_args(argv)
    truth = make_run(args.folder, takes=args.takes, videos=not args.no_videos, screen=args.screen)
    print(f'Wrote a made-up run to {args.folder}: {json.dumps(truth)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
