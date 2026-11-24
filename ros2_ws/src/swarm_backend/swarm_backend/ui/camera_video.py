"""Live camera angles for the demo videos: invisible cameras in Gazebo, filmed on sim time.

Instead of filming a replay of the Gazebo log by hand (video.sh replay), cameras are put in
the running world and recorded as the debug images are (debug_video.py).

record  (`mount-backend.sh camrec [ANGLES]`, during the run, default attacker_watch arc_side)
    Spawns one camera per angle of recording/recording_config.json ("replay_angles"): a
    model with only a camera sensor (no visual, no collision, kinematic, no gravity), so no
    other camera sees it and nothing bumps into it. A director moves them every simulation
    step with one set_pose_vector call (smoothed over live_camera.smooth_s), the recorder
    encodes each one's images into data/recordings/<newest run>/cam_<angle>.mkv (25 fps on
    sim time, as debug_video.py), and every model's true pose goes to poses.jsonl, 25 times
    a sim second, for the overlay. Ctrl-C removes the cameras and closes the files.

overlay (`recording/video.sh cam --angle A --start S --duration D [--plain]`, afterwards)
    cam_<A>_<S>.mp4 from sim time S: each drone marked where it is in the picture, with its
    id, role, status and range to its target from the UI recording (ui.jsonl), and the sim
    time and the newest event in the corner. --plain: the cut without the overlay.

Angles (as in the replay, see recording_config.json):
  {"follow": M, "offset": [x, y, z]}                   behind M in M's heading frame, looking at M
  {"follow": M, "offset": [...], "look_at_model": T}   behind M as seen from T (x = towards T),
                                                       looking at T. M = "@attacker": the
                                                       defender nearest the hostile
  {"position": [x, y, z], "look_at_point": [...], "track_model": T}   a tripod (ENU), turning
                                                       to T (or fixed on the point without it)

Everything here is Gazebo ENU (x east, y north, z up). The geometry and the overlay drawing
are plain Python (tested on the host); Gazebo, OpenCV and ffmpeg are only used in the container.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from swarm_backend.ground_truth import rotate
from swarm_backend.ui import debug_video

CONFIG = Path('/workspace/recording/recording_config.json')
DEFAULT_ANGLES = ['attacker_watch', 'arc_side']
POSE_LOG_HZ = 25.0
ATTACKER_SWITCH = 0.85  # another defender must be this much nearer the hostile to take over
ATTACKER_HOLD_S = 2.0   # and not within this many sim seconds of the last switch
FREEZE_M = 15.0  # nearer than this to its target (or after a hit) a watch angle keeps its direction

Vec = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # x, y, z, w


# ---- geometry (ENU) ------------------------------------------------------------------

def sub(a: Vec, b: Vec) -> Vec:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def add(a: Vec, b: Vec) -> Vec:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def dist(a: Vec, b: Vec) -> float:
    return math.dist(a, b)


def yaw_of(q: Quat) -> float:
    """Heading of a model's +x about the vertical, radians from east (ENU yaw)."""
    fx, fy, _ = rotate(q, (1.0, 0.0, 0.0))
    return math.atan2(fy, fx)


def offset_in_yaw(yaw: float, offset: Vec) -> Vec:
    """`offset` (x forward, y left, z up) turned by `yaw` about the vertical."""
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = offset
    return c * x - s * y, s * x + c * y, z


def look_at(position: Vec, target: Vec) -> Quat:
    """Orientation that points a camera (looking along +x, z up, no roll) from position at target."""
    dx, dy, dz = sub(target, position)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))  # positive pitch = nose down in Gazebo
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return -sy * sp, cy * sp, sy * cp, cy * cp


def blend(a: Vec, b: Vec, k: float) -> Vec:
    return tuple(x + (y - x) * k for x, y in zip(a, b))


# ---- the director --------------------------------------------------------------------

class Tracker:
    """The models' positions. A model at the graveyard (where a hit hostile is moved,
    gazebo_surveillance.GRAVEYARD_ENU) is held where it was before, so the cameras stay on
    the place of the hit; it is let go when it is moved out again (the next run)."""

    def __init__(self, graveyard: tuple[float, float], radius: float = 60.0):
        self.graveyard, self.radius = graveyard, radius
        self.pos: dict[str, Vec] = {}
        self.rot: dict[str, Quat] = {}
        self.held: set[str] = set()

    def update(self, poses: dict[str, tuple[Vec, Quat]]) -> None:
        for name, (p, q) in poses.items():
            if math.hypot(p[0] - self.graveyard[0], p[1] - self.graveyard[1]) < self.radius:
                if name in self.pos:
                    self.held.add(name)  # keep the last place it really was at
                continue
            self.held.discard(name)
            self.pos[name], self.rot[name] = p, q


class Director:
    """Where one angle's camera should be: a smoothed position and a smoothed look-at point."""

    def __init__(self, name: str, spec: dict, defenders: list[str], hostile: str,
                 smooth_s: float):
        self.name, self.spec = name, spec
        self.defenders, self.hostile = defenders, hostile
        self.smooth_s = smooth_s
        self.cam: Vec | None = None
        self.aim: Vec | None = None
        self.attacker: str | None = None
        self._switched_at = -1e9
        self._yaw: float | None = None  # a watch angle's last direction (from the drone to its target)
        self._orbit_t0: float | None = None  # an orbit angle's first sim time

    def _attacker(self, tracker: Tracker, sim: float) -> str | None:
        target = tracker.pos.get(self.spec.get('look_at_model') or self.hostile)
        present = [d for d in self.defenders if d in tracker.pos]
        if not present:
            return None
        if target is None:
            return self.attacker or present[0]
        nearest = min(present, key=lambda d: dist(tracker.pos[d], target))
        if self.attacker not in present:
            self.attacker, self._switched_at = nearest, sim
        elif nearest != self.attacker and sim - self._switched_at >= ATTACKER_HOLD_S and \
                dist(tracker.pos[nearest], target) < ATTACKER_SWITCH * dist(tracker.pos[self.attacker], target):
            self.attacker, self._switched_at = nearest, sim
            self.cam = self.aim = self._yaw = None  # a cut to the new attacker, not a long glide
        return self.attacker

    def goal(self, tracker: Tracker, sim: float) -> tuple[Vec, Vec] | None:
        """(camera position, look-at point) this angle wants now, None if its models are missing."""
        s = self.spec
        if 'orbit' in s:  # circles a point (e.g. the base): an overview, no drone needed
            if self._orbit_t0 is None:
                self._orbit_t0 = sim
            a = 2 * math.pi * (sim - self._orbit_t0) / s.get('period_s', 60.0)
            cx, cy = s['orbit'][:2]
            r = s.get('radius', 200.0)
            return (cx + r * math.cos(a), cy + r * math.sin(a), s.get('height', 70.0)), \
                tuple(s['look_at_point'])
        if 'position' in s:
            aim = tracker.pos.get(s.get('track_model', ''), tuple(s['look_at_point']))
            return tuple(s['position']), aim
        follow = self._attacker(tracker, sim) if s['follow'] == '@attacker' else s['follow']
        if follow not in tracker.pos:
            return None
        base = tracker.pos[follow]
        other = s.get('look_at_model')
        if other:
            if other not in tracker.pos:
                return None
            to = sub(tracker.pos[other], base)
            hit = other in tracker.held
            # Close in, and after a hit (the target held where it was), the direction is
            # kept: the drone flies past the target's point, and turning with it would swing
            # the camera round in front of the drone, looking back at nothing.
            if self._yaw is None or not (hit or math.hypot(to[0], to[1]) < FREEZE_M):
                self._yaw = math.atan2(to[1], to[0]) if math.hypot(to[0], to[1]) > 0.5 \
                    else yaw_of(tracker.rot[follow])
            aim = base if hit else tracker.pos[other]  # after a hit: stay on the drone
            return add(base, offset_in_yaw(self._yaw, tuple(s['offset']))), aim
        return add(base, offset_in_yaw(yaw_of(tracker.rot[follow]), tuple(s['offset']))), base

    def step(self, tracker: Tracker, sim: float, dt: float) -> tuple[Vec, Quat] | None:
        goal = self.goal(tracker, sim)
        if goal is None:
            return None if self.cam is None else (self.cam, look_at(self.cam, self.aim))
        k = 1.0 if self.cam is None or self.smooth_s <= 0 else 1.0 - math.exp(-max(dt, 0.0) / self.smooth_s)
        self.cam = goal[0] if self.cam is None else blend(self.cam, goal[0], k)
        self.aim = goal[1] if self.aim is None else blend(self.aim, goal[1], k)
        if dist(self.cam, self.aim) < 0.1:
            return self.cam, look_at(self.cam, add(self.aim, (1.0, 0.0, 0.0)))
        return self.cam, look_at(self.cam, self.aim)


def camera_model(name: str, topic: str, width: int, height: int, hfov: float, rate: float) -> str:
    """A camera sensor on a kinematic link without gravity, visual or collision: other
    cameras don't see it, nothing hits it, and it stays where set_pose puts it. Not static,
    so it is in dynamic_pose/info like the drones (its true pose for the overlay)."""
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial><mass>0.01</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>
      <sensor name="camera" type="camera">
        <always_on>1</always_on><update_rate>{rate}</update_rate><topic>{topic}</topic>
        <camera><horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{width}</width><height>{height}</height></image>
          <clip><near>0.2</near><far>3000</far></clip></camera>
      </sensor>
    </link>
  </model>
</sdf>'''


def model_name(angle: str) -> str:
    return f'rec_cam_{angle}'


# ---- record (in the container) -------------------------------------------------------

def load_config(path: Path) -> dict:
    return json.loads(Path(path).read_text())


class PoseFeed:
    """Gazebo's dynamic poses: the newest for the director, and a 25 Hz log for the overlay."""

    def __init__(self, world: str, log_path: Path | None, keep):
        """log_path None: no 25 Hz log (`camrec --no-pose-log`, when `posesrec` writes it)."""
        from swarm_backend.nodes.fake_detection import GazeboTruth
        self.lock = threading.Lock()
        self.latest: tuple[float, dict] | None = None
        self.keep = keep
        self._log = open(log_path, 'a', buffering=1) if log_path is not None else None
        self._last_logged = -1e9
        self.truth = GazeboTruth(world)
        self.truth.listeners.append(self._on_poses)

    def _on_poses(self, sim, poses) -> None:
        mine = {name: (p, q) for name, _, p, q in poses if self.keep(name)}
        with self.lock:
            self.latest = (sim, mine)
        if self._log is not None and sim - self._last_logged >= 1.0 / POSE_LOG_HZ - 1e-6:
            self._last_logged = sim
            self._log.write(json.dumps({'sim': round(sim, 3), 'poses': {
                n: [round(v, 4) for v in (*p, *q)] for n, (p, q) in mine.items()}},
                separators=(',', ':')) + '\n')

    def close(self) -> None:
        self.truth.listeners.clear()
        if self._log is not None:
            self._log.close()


class Mover:
    """Moves the cameras from a child process. A blocking Gazebo service call from a process
    that also has Python subscriptions (the images, the poses) stalls the simulation until
    the call times out (gz-transport's Python bindings, seen 2026-11-18: 8 of 12 calls timed
    out, the sim at 0.04 s per 4 s), while the same calls from a process without
    subscriptions take milliseconds. So the recorder only writes the wanted poses to this
    child's stdin; the child sends the newest with set_pose_vector."""

    def __init__(self, world: str):
        self.proc = subprocess.Popen([sys.executable, '-m', 'swarm_backend.ui.camera_video',
                                      'mover', world], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, text=True, bufsize=1)

    def move(self, poses: dict[str, tuple[Vec, Quat]]) -> None:
        line = json.dumps({'poses': {n: [*p, *q] for n, (p, q) in poses.items()}})
        try:
            self.proc.stdin.write(line + '\n')
        except BrokenPipeError:
            pass

    def remove_and_close(self, names: list[str]) -> str:
        try:
            self.proc.stdin.write(json.dumps({'remove': names}) + '\n')
            self.proc.stdin.close()
        except BrokenPipeError:
            pass
        out = self.proc.stdout.read().strip()
        self.proc.wait(timeout=30)
        return out


def mover(world: str) -> int:
    """The child of Mover: reads JSON lines from stdin, sends only the newest poses."""
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # the parent's Ctrl-C: it closes our stdin
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    from swarm_backend.birds import Gazebo

    node = Node()
    lock, wake = threading.Lock(), threading.Event()
    state = {'poses': None, 'remove': None, 'eof': False}

    def read():
        for line in sys.stdin:
            msg = json.loads(line)
            with lock:
                if 'poses' in msg:
                    state['poses'] = msg['poses']
                if 'remove' in msg:
                    state['remove'] = msg['remove']
            wake.set()
        with lock:
            state['eof'] = True
        wake.set()

    threading.Thread(target=read, daemon=True).start()
    moved = failed = 0
    while True:
        wake.wait()
        with lock:
            poses, state['poses'] = state['poses'], None
            remove, eof = state['remove'], state['eof']
            wake.clear()
        if poses:
            request = Pose_V()
            for name, v in poses.items():
                pose = request.pose.add()
                pose.name = name
                pose.position.x, pose.position.y, pose.position.z = v[:3]
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = v[3:]
            ok, reply = node.request(f'/world/{world}/set_pose_vector', request, Pose_V, Boolean, 2000)
            if ok and reply.data:
                moved += 1
            else:
                failed += 1
        if eof:
            break
    removed = [n for n in (remove or []) if Gazebo(world).remove(n)]
    print(f'Moved the cameras {moved} times ({failed} calls failed), removed {len(removed)}.', flush=True)
    return 0


def spawn_camera(gazebo, node, name: str, topic: str, sdf: str, wait_s: float = 30.0) -> None:
    """Spawns a camera and waits for its image topic. The first camera starts Gazebo's
    renderer, which can hold the reply past the service timeout although it worked, so the
    topic decides, not the reply."""
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    request = EntityFactory()
    request.sdf, request.name = sdf, name
    request.pose.position.z = 300.0
    gazebo._call('create', request, EntityFactory, timeout_ms=int(wait_s * 1000))
    deadline = time.monotonic() + wait_s
    while topic not in node.topic_list():
        if time.monotonic() > deadline:
            raise SystemExit(f'Could not spawn the camera {name} in {gazebo.world} '
                             f'(no {topic} after {wait_s:.0f} s).')
        time.sleep(0.2)


def record(args) -> int:
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    from swarm_backend.birds import Gazebo

    cfg = load_config(args.config)
    live = cfg['live_camera']
    width, height = args.width or live['width'], args.height or live['height']
    hfov, rate = args.hfov or live['hfov'], live['rate']
    angles = args.angles or DEFAULT_ANGLES
    unknown = [a for a in angles if a not in cfg['replay_angles']]
    if unknown:
        raise SystemExit(f'Unknown angle(s) {", ".join(unknown)}. In recording_config.json: '
                         f'{", ".join(cfg["replay_angles"])}')
    folder = debug_video.run_folder(args.out)
    folder.mkdir(parents=True, exist_ok=True)

    gazebo = Gazebo()
    world = gazebo.world
    names = {a: model_name(a) for a in angles}
    directors = {a: Director(a, cfg['replay_angles'][a], cfg['defenders'], cfg['hostile'],
                             live['smooth_s']) for a in angles}
    tracker = Tracker(tuple(cfg['graveyard_enu']))
    node = Node()
    # Service calls first, while this process has no subscriptions yet (see Mover).
    for a in angles:
        topic = f'/hive_rec/{a}/image'
        if topic in node.topic_list():  # left over from a recorder that was killed
            gazebo.remove(names[a])
            time.sleep(1.0)
        spawn_camera(gazebo, node, names[a], topic,
                     camera_model(names[a], topic, width, height, hfov, rate))
    mover_proc = Mover(world)
    feed = PoseFeed(world, None if args.no_pose_log else folder / 'poses.jsonl',
                    lambda n: n.startswith('x500') or n.startswith('rec_cam_'))
    videos = {}
    for a in angles:
        topic = f'/hive_rec/{a}/image'
        video = videos[a] = debug_video.DroneVideo(
            0, folder, args.fps, stem=f'cam_{a}', label=f'Camera {a}',
            extra_meta={'angle': a, 'model': names[a], 'hfov': hfov})

        def on_image(msg, video=video):
            if msg.pixel_format_type != debug_video_RGB:
                video.wrong += 1
                return
            row = msg.width * 3
            data = msg.data if msg.step == row else b''.join(
                msg.data[y * msg.step:y * msg.step + row] for y in range(msg.height))
            video.offer(msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9,
                        msg.width, msg.height, bytes(data))

        if not node.subscribe(Image, topic, on_image):
            raise SystemExit(f'Could not subscribe to {topic}')
    print(f'Cameras {", ".join(angles)} in {world} ({width}x{height}, {math.degrees(hfov):.0f} deg, '
          f'{rate:g} Hz), recording into {folder}. Ctrl-C stops it and removes them.', flush=True)

    import signal
    signal.signal(signal.SIGTERM, signal.default_int_handler)  # `kill` stops it like Ctrl-C
    last_sim = None
    try:
        while True:
            with feed.lock:
                latest = feed.latest
            if latest is None or latest[0] == last_sim:
                time.sleep(0.005)
                continue
            sim, poses = latest
            dt = 0.0 if last_sim is None else sim - last_sim
            last_sim = sim
            tracker.update({n: v for n, v in poses.items() if not n.startswith('rec_cam_')})
            wanted = {}
            for a, director in directors.items():
                pose = director.step(tracker, sim, dt)
                if pose is not None:
                    wanted[names[a]] = pose
            if wanted:
                mover_proc.move(wanted)
    except KeyboardInterrupt:
        pass
    finally:
        feed.close()
        for video in videos.values():
            print(video.close(), flush=True)
        print(mover_proc.remove_and_close(list(names.values())), flush=True)
        if not args.no_pose_log:
            print(f'Poses in {folder / "poses.jsonl"}', flush=True)
    # gz-transport's threads abort the interpreter's normal exit ("terminate called without
    # an active exception"); everything is written and closed by now.
    sys.stdout.flush()
    os._exit(0)


debug_video_RGB = 3  # gz.msgs.PixelFormatType.RGB_INT8


# ---- overlay (afterwards) ------------------------------------------------------------

def load_poses(path: Path) -> tuple[list[float], list[dict]]:
    sims, rows = [], []
    for line in path.read_text().splitlines():
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        sims.append(d['sim'])
        rows.append(d['poses'])
    return sims, rows


def pose_at(sims: list[float], rows: list[dict], name: str, sim: float) -> tuple[Vec, Quat] | None:
    """A model's pose at `sim`: linear between the two nearest logged poses."""
    import bisect
    i = bisect.bisect_right(sims, sim)
    before = rows[i - 1].get(name) if i > 0 else None
    after = rows[i].get(name) if i < len(rows) else None
    if before is None or after is None:
        row = before or after
        return None if row is None else (tuple(row[:3]), tuple(row[3:]))
    t0, t1 = sims[i - 1], sims[i]
    k = 0.0 if t1 <= t0 else (sim - t0) / (t1 - t0)
    p = blend(tuple(before[:3]), tuple(after[:3]), k)
    qa, qb = before[3:], after[3:]
    if sum(a * b for a, b in zip(qa, qb)) < 0:
        qb = [-v for v in qb]
    q = [a + (b - a) * k for a, b in zip(qa, qb)]
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return p, tuple(v / n for v in q)


def project(cam_p: Vec, cam_q: Quat, point: Vec, hfov: float, width: int, height: int):
    """(u, v, depth) of a world point in the camera's image (looking along +x, z up), also
    outside the image; None when it is behind the camera."""
    d = sub(point, cam_p)
    depth = dot(d, rotate(cam_q, (1.0, 0.0, 0.0)))
    if depth <= 0.3:
        return None
    f = (width / 2) / math.tan(hfov / 2)
    left = dot(d, rotate(cam_q, (0.0, 1.0, 0.0)))
    up = dot(d, rotate(cam_q, (0.0, 0.0, 1.0)))
    return width / 2 - f * left / depth, height / 2 - f * up / depth, depth


def drone_id(model: str) -> int | None:
    tail = model.rsplit('_', 1)[-1]
    return int(tail) if tail.isdigit() else None


def label_lines(model: str, drone: dict | None, hostile: str) -> tuple[list[str], str]:
    """The label's lines and its kind (hostile, engaged, defender, other)."""
    i = drone_id(model)
    if model == hostile or (drone and drone.get('hostile')):
        return [f'D{i} hostile'], 'hostile'
    if drone is None:
        return [f'D{i}'], 'other'
    status = drone.get('status') or ''
    lines = [f'D{i} {"base" if i == 0 else "defender"}', status]
    attack = drone.get('attack') or {}
    if attack.get('range_m') is not None and status in ('intercept', 'target lost'):
        # The defender's own estimate (est_error in takes.json), not the true distance that the
        # graphs (plot:range) show: often tens of metres apart, so say which it is.
        lines[1] += f'  est. {attack["range_m"]:.0f} m'
    return lines, 'engaged' if status == 'intercept' else 'defender'


def ascii_text(text: str) -> str:
    return text.replace('→', '->').encode('ascii', 'replace').decode()


COLOURS = {'hostile': (60, 60, 255), 'engaged': (0, 165, 255), 'defender': (255, 200, 80),
           'other': (200, 200, 200)}  # BGR


def draw_overlay(img, cam: tuple[Vec, Quat], models: dict[str, Vec], drones: dict[int, dict],
                 hostile: str, hfov: float, sim: float, event: str | None) -> None:
    import cv2
    h, w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = h / 720
    f = (w / 2) / math.tan(hfov / 2)
    for model, p in sorted(models.items()):
        at = project(cam[0], cam[1], p, hfov, w, h)
        if at is None:
            continue
        u, v, depth = at
        if not (-50 <= u < w + 50 and -50 <= v < h + 50):
            continue
        lines, kind = label_lines(model, drones.get(drone_id(model)), hostile)
        colour = COLOURS[kind]
        half = max(12 * scale, f * 0.45 / depth)
        x0, y0, x1, y1 = int(u - half), int(v - half), int(u + half), int(v + half)
        arm = int(max(6 * scale, half * 0.5))
        for (cx, cy, dx, dy) in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
            cv2.line(img, (cx, cy), (cx + dx * arm, cy), colour, max(1, int(2 * scale)), cv2.LINE_AA)
            cv2.line(img, (cx, cy), (cx, cy + dy * arm), colour, max(1, int(2 * scale)), cv2.LINE_AA)
        y = y0 - int(8 * scale)
        for text in reversed([t for t in lines if t]):
            text = ascii_text(text)
            (tw, th), _ = cv2.getTextSize(text, font, 0.55 * scale, 1)
            cv2.rectangle(img, (x0 - 2, y - th - 4), (x0 + tw + 4, y + 4), (20, 20, 20), -1)
            cv2.putText(img, text, (x0 + 1, y), font, 0.55 * scale, colour, 1, cv2.LINE_AA)
            y -= th + 10
    header = [f'sim {sim:6.1f} s'] + ([ascii_text(event)] if event else [])
    y = int(28 * scale)
    for text in header:
        (tw, th), _ = cv2.getTextSize(text, font, 0.65 * scale, 2)
        cv2.rectangle(img, (12, y - th - 8), (24 + tw, y + 8), (20, 20, 20), -1)
        cv2.putText(img, text, (18, y), font, 0.65 * scale, (240, 240, 240), 1, cv2.LINE_AA)
        y += th + 18


def overlay(args) -> int:
    import numpy as np

    from swarm_backend.ui.replay import ReplayUi, load_recording

    folder = args.run
    meta_path = folder / f'cam_{args.angle}.json'
    src = folder / f'cam_{args.angle}.mkv'
    if not src.exists() or not meta_path.exists():
        have = sorted(p.stem for p in folder.glob('cam_*.mkv'))
        raise SystemExit(f'No camera {args.angle} in {folder} (there are: '
                         f'{", ".join(have) or "none"}). Record with `mount-backend.sh camrec`.')
    meta = json.loads(meta_path.read_text())
    out = args.out or folder / f'cam_{args.angle}_{args.start:g}.mp4'
    if args.plain:
        subprocess.run(debug_video.cut_command(src, out, meta['start'], args.start, args.duration,
                                               meta['fps']), check=True)
        print(f'Wrote {out} (no overlay)')
        return 0
    cfg = load_config(args.config)
    sims, rows = load_poses(folder / 'poses.jsonl')
    ui = ReplayUi(load_recording(folder / 'ui.jsonl')) if (folder / 'ui.jsonl').exists() else None
    if ui is None:
        print('No ui.jsonl: labels without status and range.')
    w, h, fps = meta['width'], meta['height'], meta['fps']
    hfov = meta.get('hfov', cfg['live_camera']['hfov'])
    cam_model = meta['model']
    # Decode the cut (sim time from --start on, held first image before the recording)...
    cut = debug_video.cut_command(src, Path('-'), meta['start'], args.start, args.duration, fps)
    cut = cut[:cut.index('-c:v')] + ['-f', 'rawvideo', '-pix_fmt', 'bgr24', '-']
    dec = subprocess.Popen(cut, stdout=subprocess.PIPE)
    # ...and encode the drawn frames.
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                            '-s', f'{w}x{h}', '-r', f'{fps:g}', '-i', '-', '-c:v', 'libx264',
                            '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', str(out)],
                           stdin=subprocess.PIPE)
    events = ui.event_table() if ui else []
    frame_bytes = w * h * 3
    i = 0
    while True:
        buf = dec.stdout.read(frame_bytes)
        if len(buf) < frame_bytes:
            break
        sim = args.start + i / fps
        img = np.frombuffer(buf, np.uint8).reshape(h, w, 3).copy()
        cam = pose_at(sims, rows, cam_model, sim)
        if cam is not None:
            models = {}
            for name in rows[min(len(rows) - 1, max(0, _index(sims, sim)))]:
                if name.startswith('x500'):
                    p = pose_at(sims, rows, name, sim)
                    if p is not None:
                        models[name] = p[0]
            drones = {}
            if ui is not None:
                drones = {d['id']: d for d in ui.at(sim)['state'].get('drones', [])}
            recent = [e for at, _, e in events if sim - 5.0 <= at <= sim]
            draw_overlay(img, cam, models, drones, cfg['hostile'], hfov, sim,
                         recent[-1]['text'] if recent else None)
        enc.stdin.write(img.tobytes())
        i += 1
    enc.stdin.close()
    enc.wait()
    dec.wait()
    if enc.returncode != 0:
        raise SystemExit('ffmpeg could not write the video')
    print(f'Wrote {out}: sim time {args.start:g} to {args.start + i / fps:g} s, {i} frames, overlay')
    return 0


def _index(sims: list[float], sim: float) -> int:
    import bisect
    return bisect.bisect_right(sims, sim) - 1


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--config', type=Path, default=CONFIG)
    sub_p = p.add_subparsers(dest='cmd', required=True)
    r = sub_p.add_parser('record')
    r.add_argument('angles', nargs='*', help=f'angles (default {" ".join(DEFAULT_ANGLES)})')
    r.add_argument('--out', default='auto', help='run folder (default: the newest)')
    r.add_argument('--fps', type=float, default=debug_video.FPS)
    r.add_argument('--width', type=int)
    r.add_argument('--height', type=int)
    r.add_argument('--hfov', type=float, help='radians (default live_camera.hfov)')
    r.add_argument('--no-pose-log', action='store_true',
                   help='do not write poses.jsonl (posesrec writes it, recording/record.sh)')
    o = sub_p.add_parser('overlay')
    o.add_argument('run', type=Path)
    o.add_argument('--angle', required=True)
    o.add_argument('--start', type=float, required=True)
    o.add_argument('--duration', type=float, default=None)
    o.add_argument('--plain', action='store_true', help='no overlay, only the cut')
    o.add_argument('--out', type=Path)
    m = sub_p.add_parser('mover', help='internal: the camera mover process (see Mover)')
    m.add_argument('world')
    args = p.parse_args(argv)
    if args.cmd == 'mover':
        return mover(args.world)
    return record(args) if args.cmd == 'record' else overlay(args)


if __name__ == '__main__':
    sys.exit(main())
