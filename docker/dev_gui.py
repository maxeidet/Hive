#!/usr/bin/env python3
"""Dev GUI: one web page on the host that starts the container and runs things in it.

    python3 docker/dev_gui.py [--port 8095]

Open the URL it prints (it carries a random token: other users on a shared Stellar
computer can reach 127.0.0.1 too, so every API call must send it). Run it inside tmux
on Stellar, since the jobs it starts stop when it stops:

    tmux new -s devgui 'python3 docker/dev_gui.py'

It only wraps the existing scripts, which stay the one place that knows how to start
things:
  container  docker/start-stellar.sh -m MODEL -w WORLD --native-gui|--vnc|--headless --gpu|--no-gpu
  backend    ros2_ws/src/swarm_backend/mount-backend.sh CMD ARGS  (CMD from BACKEND_COMMANDS)
  scenario   mount-backend.sh scenario NAME [--sim] ARGS, or scenarios/NAME.sh ARGS

Each job runs in a pseudo-terminal (the scripts use `docker exec -it`), so you can type
at the pxh> prompt or in the swarm shell. Stop sends Ctrl-C first, so drones land.
Standard library only.
"""
import argparse
import ast
import fcntl
import getpass
import hmac
import http.client
import json
import os
import platform
import pwd
import re
import secrets
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import sys
import termios
import threading
import time
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

REPO = Path(__file__).resolve().parent.parent
DOCKER_DIR = REPO / 'docker'
BACKEND_DIR = REPO / 'ros2_ws' / 'src' / 'swarm_backend'
STATIC_DIR = DOCKER_DIR / 'dev_gui'
CONTAINER = 'px4-gazebo-vnc'
BOOKING_PATH = Path('/etc/stellar/booking.json')
MODELS_DIR = REPO / 'models'  # not in git; the YOLO detector loads the newest models/**/*.onnx

# YOLO weights fetched at startup when missing (GitHub Releases of the repo, see
# ros2_ws/src/swarm_backend/scenarios/README-yolo-bytetrack-mpc.md). Only the model the
# team's live tests use: the detector takes the NEWEST .onnx, so fetching more models
# would silently change which one runs.
RELEASES_URL = 'https://github.com/maxeidet/Hive/releases/download'
WEIGHTS = [
    # (release tag, file, folder under models/)
    ('model-gazebo-only', 'yolo26n-gazebo-only-1280.onnx', 'yolo26n-gazebo-only'),
]

MAX_LINES = 5000  # log lines kept per job
STOP_GRACE_S = 15.0  # after Ctrl-C: wait this long, then SIGINT/SIGTERM/SIGKILL the group
STATUS_POLL_S = 2.0  # container, Gazebo, drones, operator UI
CAMERA_POLL_S = 6.0  # which drones have a camera (gz topic -l is slow)

MODELS = ['gz_x500', 'gz_x500_mono_cam', 'gz_x500_depth', 'gz_x500_lidar_2d',
          'gz_x500_lidar_front', 'gz_x500_lidar_down', 'gz_x500_flow', 'gz_x500_vision',
          'gz_x500_gimbal']
BUNDLED_WORLDS = ['baylands', 'default', 'forest', 'ridge', 'walls', 'lawn', 'windy', 'aruco',
                  'frictionless', 'underwater', 'rover', 'kthspacelab', 'kth_marinarium',
                  'moving_platform']
# mount-backend.sh commands the page may run (see its header for what each does)
BACKEND_COMMANDS = ['shell', 'spawn', 'restart', 'ui', 'mapsnap', 'view', 'fakedetect',
                    'basestation', 'patrol', 'detect', 'attack', 'bird', 'snap', 'predict',
                    'gendata']
# The config every scenario of this page flies (inside the container). The operator UI
# gets it too, so it lists their drones (0-4) and shows drone 4 as the hostile.
SURVEILLANCE_CONFIG = '/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml'

# One-click runs: a scenario launcher with fixed arguments. `sim` adds a --sim dry run (the
# fast simulation, minutes, no Gazebo; it uses a fake detector, so it needs no weights).
QUICK_STARTS = [
    {'id': 'compare-baseline', 'label': '36 runs: baseline',
     'about': 'Blob detector, Confirmer, pure pursuit on the 36 cases (4 paths x 3 altitudes x '
              '3 seeds). About 3 hours in Gazebo. scenarios/README.md',
     'scenario': 'compare_paths.sh', 'args': ['3', '--setup', 'baseline'], 'yolo': False},
    {'id': 'compare-yolo-mpc', 'label': '36 runs: YOLO + ByteTrack + MPC',
     'about': 'YOLO, ByteTrack locked on one target, MPC intercept on the same 36 cases. About '
              '3 hours in Gazebo. scenarios/README-yolo-bytetrack-mpc.md',
     'scenario': 'compare_paths.sh', 'args': ['3', '--setup', 'yolo-bytetrack-mpc'], 'yolo': True},
]

ANSI_RE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)|\x1b[()][0-9A-Za-z]|\x1b[=>78]')
VNC_PASSWORD_RE = re.compile(r'VNC password:\s+(\S+)')
NOVNC_URL_RE = re.compile(r'(http://localhost:\d+/vnc\.html)')
PX4_INSTANCE_RE = re.compile(r'px4(?:-gazebo)? -i (\d+)')
CAMERA_TOPIC_RE = re.compile(r'^/world/[^/]+/model/(?P<model>[^/\n]+)_(?P<id>\d+)/link/[^/\n]+'
                             r'/sensor/(?P<sensor>[^/\n]+)/image$', re.M)
DEPTH_MODEL_RE = re.compile(r'^/world/[^/]+/model/x500_depth_(?P<id>\d+)/', re.M)
NVIDIA_PROC_RE = re.compile(r'^\|\s+\d+\s+\S+\s+\S+\s+(\d+)\s+([CG+]+)\s+(\S+)')


class BadRequest(Exception):
    """A request the page should not have sent: shown to the user as is."""


# ============================================================
# Jobs: one script in a pseudo-terminal each
# ============================================================

class Job:
    def __init__(self, job_id: int, kind: str, title: str, argv: list[str], cwd: Path,
                 env: dict | None = None):
        self.id, self.kind, self.title, self.argv = job_id, kind, title, argv
        self.started = time.time()
        self.exit_code = None
        self.stopping = False
        self._lines = deque(maxlen=MAX_LINES)  # (seq, text)
        self._next_seq = 0
        self._partial = ''  # text after the last newline, e.g. a 'pxh> ' prompt
        self._lock = threading.Lock()

        master, slave = os.openpty()
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 50, 200, 0, 0))
        # Like a fresh terminal (`stty sane`): Enter (\r) ends a line, Ctrl-C is SIGINT,
        # typed text is echoed. A new pty does not always start that way.
        attrs = termios.tcgetattr(slave)
        attrs[0] |= termios.ICRNL
        attrs[1] |= termios.OPOST | termios.ONLCR
        attrs[3] |= termios.ICANON | termios.ECHO | termios.ISIG | termios.IEXTEN
        termios.tcsetattr(slave, termios.TCSANOW, attrs)
        env = dict(os.environ, TERM='dumb', PYTHONUNBUFFERED='1', **(env or {}))

        def as_session_leader():
            # Own session with the pty as controlling terminal: Ctrl-C written to it
            # reaches the job's foreground processes, as in a real terminal.
            os.setsid()
            fcntl.ioctl(0, termios.TIOCSCTTY, 0)

        try:
            self.proc = subprocess.Popen(argv, cwd=cwd, env=env, stdin=slave, stdout=slave,
                                         stderr=slave, preexec_fn=as_session_leader,
                                         close_fds=True)
        except OSError:
            os.close(master)
            os.close(slave)
            raise
        os.close(slave)
        self._master = master
        threading.Thread(target=self._read, daemon=True).start()

    # ---- output ----
    def _read(self):
        while True:
            try:
                data = os.read(self._master, 4096)
            except OSError:  # EIO: every process holding the pty has exited
                break
            if not data:
                break
            self._add(data.decode('utf-8', errors='replace'))
        code = self.proc.wait()
        with self._lock:
            if self._partial:
                self._push(self._partial)
                self._partial = ''
            self.exit_code = code
        os.close(self._master)

    def _add(self, text: str):
        text = ANSI_RE.sub('', text).replace('\r\n', '\n')
        with self._lock:
            parts = (self._partial + text).split('\n')
            for line in parts[:-1]:
                self._push(line)
            self._partial = parts[-1]

    def _push(self, line: str):
        # A bare \r (progress bars, docker build) redraws the line: keep what was drawn last.
        line = line.rstrip('\r').rsplit('\r', 1)[-1]
        self._lines.append((self._next_seq, line))
        self._next_seq += 1

    def log(self, since: int) -> dict:
        with self._lock:
            first = self._lines[0][0] if self._lines else self._next_seq
            lines = [[s, t] for s, t in self._lines if s >= since]
            partial = self._partial.rstrip('\r').rsplit('\r', 1)[-1]
            return {'lines': lines, 'partial': partial, 'next': self._next_seq,
                    'truncated': since < first, 'running': self.running,
                    'exit_code': self.exit_code}

    def text(self) -> str:
        with self._lock:
            return '\n'.join(t for _, t in self._lines) + '\n' + self._partial

    # ---- control ----
    @property
    def running(self) -> bool:
        return self.exit_code is None

    def send(self, data: str):
        if not self.running:
            raise BadRequest('the job has ended')
        try:
            os.write(self._master, data.encode())
        except OSError as e:
            raise BadRequest(f'could not write to the job: {e}') from e

    def stop(self, grace_s: float = STOP_GRACE_S):
        """Ctrl-C like a terminal (docker exec -it forwards it into the container, so
        drones land), then signals to the whole process group if it does not end."""
        if not self.running or self.stopping:
            return
        self.stopping = True
        try:
            os.write(self._master, b'\x03')
        except OSError:
            pass
        threading.Thread(target=self._escalate, args=(grace_s,), daemon=True).start()

    def _escalate(self, grace_s: float):
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            deadline = time.time() + grace_s
            while time.time() < deadline:
                if not self.running:
                    return
                time.sleep(0.1)
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            grace_s = min(grace_s, 5.0)

    def summary(self) -> dict:
        return {'id': self.id, 'kind': self.kind, 'title': self.title,
                'argv': self.argv, 'started': self.started, 'running': self.running,
                'stopping': self.stopping and self.running, 'exit_code': self.exit_code}


class JobManager:
    def __init__(self):
        self._jobs: dict[int, Job] = {}
        self._next_id = 1
        self._lock = threading.Lock()

    def start(self, kind: str, title: str, argv: list[str], cwd: Path,
              env: dict | None = None) -> Job:
        with self._lock:
            if kind == 'container' and any(j.kind == 'container' and j.running
                                           for j in self._jobs.values()):
                raise BadRequest('the container is already started from this page')
            job = Job(self._next_id, kind, title, argv, cwd, env)
            self._jobs[job.id] = job
            self._next_id += 1
            return job

    def get(self, job_id: int) -> Job:
        job = self._jobs.get(job_id)
        if job is None:
            raise BadRequest(f'no job {job_id}')
        return job

    def dismiss(self, job_id: int):
        with self._lock:
            if self.get(job_id).running:
                raise BadRequest('stop the job before dismissing it')
            del self._jobs[job_id]

    def all(self) -> list[Job]:
        return list(self._jobs.values())

    def stop_all(self, wait_s: float = 20.0):
        for job in self.all():
            job.stop(grace_s=5.0)
        deadline = time.time() + wait_s
        while time.time() < deadline and any(j.running for j in self.all()):
            time.sleep(0.2)


# ============================================================
# What the page may run
# ============================================================

def split_args(text) -> list[str]:
    if not text:
        return []
    if not isinstance(text, str):
        raise BadRequest('args must be a string')
    try:
        return shlex.split(text)
    except ValueError as e:
        raise BadRequest(f'could not read the arguments: {e}') from e


def worlds(repo: Path = REPO) -> list[str]:
    ours = sorted(p.stem for p in (repo / 'worlds').glob('*.sdf'))
    if (repo / 'worlds' / 'generate_hive_valley.py').exists() and 'hive_valley' not in ours:
        ours.append('hive_valley')
    return ours + [w for w in BUNDLED_WORLDS if w not in ours]


def scenarios(backend_dir: Path = BACKEND_DIR) -> list[dict]:
    """Module scenarios from swarm_backend/scenarios/__init__.py (read with ast, so no ROS
    imports on the host), and the shell launchers in scenarios/."""
    found = []
    init = backend_dir / 'swarm_backend' / 'scenarios' / '__init__.py'
    try:
        tree = ast.parse(init.read_text())
        for node in tree.body:
            if (isinstance(node, ast.Assign) and len(node.targets) == 1
                    and getattr(node.targets[0], 'id', None) == 'SCENARIOS'):
                for name, (_module, sim) in ast.literal_eval(node.value).items():
                    found.append({'name': name, 'type': 'module', 'sim': bool(sim)})
    except (OSError, SyntaxError, ValueError, TypeError):
        pass
    for script in sorted((backend_dir / 'scenarios').glob('*.sh')):
        if script.name != 'lib.sh':
            found.append({'name': script.name, 'type': 'script', 'sim': True})
    return found


def container_argv(req: dict, darwin: bool, docker_dir: Path = DOCKER_DIR) -> tuple[list[str], str]:
    model, world = req.get('model', 'gz_x500'), req.get('world', 'hive_base')
    for value, what in ((model, 'model'), (world, 'world')):
        if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_.-]+', value):
            raise BadRequest(f'bad {what} name')
    gui = req.get('gui', 'native')
    if gui not in ('native', 'vnc', 'headless'):
        raise BadRequest('gui must be native, vnc or headless')
    gpu = bool(req.get('gpu', True))
    if darwin and (gui == 'native' or gpu):
        raise BadRequest('on macOS only --vnc or --headless, without the GPU')
    drone0 = bool(req.get('drone0'))  # default: the world alone, drones come from spawn/scenarios
    gz_window = bool(req.get('gazebo_window'))  # default: Gazebo headless, RViz still on the display
    argv = [str(docker_dir / 'start-stellar.sh'), '-m', model, '-w', world,
            '--native-gui' if gui == 'native' else f'--{gui}', '--gpu' if gpu else '--no-gpu']
    if not drone0:
        argv.append('--no-drone')
    if gui != 'headless' and not gz_window:
        argv.append('--no-gazebo-gui')
    what = f'drone 0 {model}' if drone0 else 'no drones'
    window = ', Gazebo window' if gz_window and gui != 'headless' else ''
    return argv, f'container {world}, {what} ({gui}{window}{", gpu" if gpu else ""})'


def backend_argv(req: dict, backend_dir: Path = BACKEND_DIR) -> tuple[list[str], str]:
    cmd = req.get('command')
    if cmd not in BACKEND_COMMANDS:
        raise BadRequest(f'unknown backend command: {cmd!r}')
    args = split_args(req.get('args'))
    return [str(backend_dir / 'mount-backend.sh'), cmd, *args], ' '.join(['backend', cmd, *args])


# One path of compare_paths.sh at a time (its PATHS labels), with the defenders starting
# below their slots so they only climb.
SINGLE_PATHS = [('straight', 'straight'), ('weave_5m', 'weave ±5 m'),
                ('weave_15m', 'weave ±15 m'), ('zigzag_15m', 'zigzag ±15 m')]
SINGLE_SETUPS = [('baseline', 'baseline (blob, Confirmer, pure pursuit)'),
                 ('yolo-bytetrack-mpc', 'YOLO + ByteTrack + MPC')]
# drone 4's altitude relative to the defenders (compare_paths.sh flies all three)
SINGLE_ALTITUDES = [('0', 'level with the defenders'), ('-10', '10 m below'),
                    ('10', '10 m above'), ('all', 'all three (below, level, above)')]
MAX_SINGLE_RUNS = 30
# The defender whose RViz a single run opens: the middle one, whose sector the hostile
# starts in (compare_paths.sh --sector 2).
SINGLE_RVIZ_DRONE = 2
AUTO_VIEW_TRIES = 5  # an RViz that ends with an error is opened again, this many times at most


def single_argv(req: dict, backend_dir: Path = BACKEND_DIR,
                models_dir: Path = MODELS_DIR) -> tuple[list[str], str, bool]:
    """compare_paths.sh for one path: (argv, title, uses YOLO)."""
    path, setup, altitude = req.get('path'), req.get('setup'), str(req.get('altitude', '0'))
    if path not in dict(SINGLE_PATHS):
        raise BadRequest(f'unknown path: {path!r}')
    if setup not in dict(SINGLE_SETUPS):
        raise BadRequest(f'unknown setup: {setup!r}')
    if altitude not in dict(SINGLE_ALTITUDES):
        raise BadRequest(f'unknown altitude: {altitude!r}')
    try:
        runs = int(req.get('runs', 1))
    except (TypeError, ValueError) as e:
        raise BadRequest('runs must be a number') from e
    if not 1 <= runs <= MAX_SINGLE_RUNS:
        raise BadRequest(f'runs must be 1-{MAX_SINGLE_RUNS}')
    sim = bool(req.get('sim'))
    yolo = setup != 'baseline'
    if yolo and not sim and not has_weights(models_dir):
        raise BadRequest('no YOLO weights in models/ yet: wait for the download (the YOLO '
                         'weights light), or pick the baseline')
    args = [str(runs), '--paths', path, '--setup', setup]
    if altitude != 'all':
        args += ['--altitude-offsets', altitude]  # after compare_paths' own: argparse keeps the last
    if req.get('spawn_at_slots', True):
        args.append('--spawn-at-slots')
    if not sim:  # a fresh Gazebo world for every run (the fast simulation has none)
        args.append('--restart-each-run')
    argv, _ = scenario_argv({'name': 'compare_paths.sh', 'args': shlex.join(args), 'sim': sim},
                            backend_dir)
    alt = {'0': 'level', '-10': 'below', '10': 'above', 'all': 'all altitudes'}[altitude]
    title = f'{dict(SINGLE_PATHS)[path]}, {setup}, {alt}, {runs} run{"s" if runs > 1 else ""}'
    return argv, title + (' (--sim)' if sim else ''), yolo


# A schedule (scenarios/schedule.py) built in the page: steps of a path, each altitude
# with its own number of runs (0 skips it). Written as YAML to SCHEDULE_DIR, then flown by
# scenarios/schedule.sh like the single runs: defenders below their slots, a fresh Gazebo
# world for every run.
SCHEDULE_PATHS = {'straight': {'path': 'straight'},
                  'weave_5m': {'path': 'weave', 'amplitude': 5},
                  'weave_15m': {'path': 'weave', 'amplitude': 15},
                  'zigzag_15m': {'path': 'zigzag', 'amplitude': 15}}
SCHEDULE_ALTITUDES = ('-10', '0', '10')
MAX_SCHEDULE_STEPS = 20
MAX_SCHEDULE_RUNS = 200  # in all
SCENARIO_KINDS = ('scenario', 'quick', 'single', 'schedule')  # one at a time: they share the drones


def schedule_argv(req: dict, backend_dir: Path = BACKEND_DIR, models_dir: Path = MODELS_DIR,
                  schedule_dir: Path | None = None) -> tuple[list[str], str, bool]:
    """Write the page's schedule as YAML and return (argv for schedule.sh, title, uses YOLO)."""
    setup = req.get('setup')
    if setup not in dict(SINGLE_SETUPS):
        raise BadRequest(f'unknown setup: {setup!r}')
    raw = req.get('steps')
    if not isinstance(raw, list) or not raw:
        raise BadRequest('add at least one step')
    if len(raw) > MAX_SCHEDULE_STEPS:
        raise BadRequest(f'at most {MAX_SCHEDULE_STEPS} steps')
    steps, total, seen = [], 0, {}
    for i, step in enumerate(raw, 1):
        if not isinstance(step, dict) or step.get('path') not in SCHEDULE_PATHS:
            raise BadRequest(f'step {i}: unknown path')
        runs = step.get('runs')
        if not isinstance(runs, dict):
            raise BadRequest(f'step {i}: runs per altitude missing')
        per_alt = {}
        for alt in SCHEDULE_ALTITUDES:
            try:
                n = int(runs.get(alt) or 0)
            except (TypeError, ValueError) as e:
                raise BadRequest(f'step {i}: runs must be numbers') from e
            if n < 0:
                raise BadRequest(f'step {i}: runs must be 0 or more')
            if n:
                per_alt[int(alt)] = n
        if not per_alt:
            raise BadRequest(f'step {i}: give at least one altitude a run')
        total += sum(per_alt.values())
        label = step['path']
        seen[label] = seen.get(label, 0) + 1
        if seen[label] > 1:  # the same path again: its own folder
            label = f'{label}_{seen[label]}'
        steps.append({**SCHEDULE_PATHS[step['path']], 'label': label, 'runs': per_alt})
    if total > MAX_SCHEDULE_RUNS:
        raise BadRequest(f'{total} runs: at most {MAX_SCHEDULE_RUNS} in one schedule')
    sim = bool(req.get('sim'))
    yolo = setup != 'baseline'
    if yolo and not sim and not has_weights(models_dir):
        raise BadRequest('no YOLO weights in models/ yet: wait for the download (the YOLO '
                         'weights light), or pick the baseline')
    schedule_dir = schedule_dir or REPO / 'data' / 'schedules' / 'dev_gui'
    schedule_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y-%m-%d_%H%M%S')
    file = schedule_dir / f'{stamp}.yaml'
    lines = ['# Built in the dev GUI.', 'name: dev_gui', 'defaults:', f'  setup: {setup}',
             '  sector: 2', '  speed: 4', 'steps:']
    for step in steps:
        lines.append(f'  - path: {step["path"]}')
        if 'amplitude' in step:
            lines.append(f'    amplitude: {step["amplitude"]}')
        lines.append(f'    label: {step["label"]}')
        lines.append('    runs: {' + ', '.join(f'{a}: {n}' for a, n in step['runs'].items()) + '}')
    file.write_text('\n'.join(lines) + '\n')
    args = [str(file)]
    if req.get('spawn_at_slots', True):
        args.append('--spawn-at-slots')
    if not sim:
        args.append('--restart-each-run')
    argv, _ = scenario_argv({'name': 'schedule.sh', 'args': shlex.join(args), 'sim': sim},
                            backend_dir)
    title = (f'schedule: {len(steps)} step{"s" if len(steps) > 1 else ""}, {total} '
             f'run{"s" if total > 1 else ""}, {setup}' + (' (--sim)' if sim else ''))
    return argv, title, yolo


def has_weights(models_dir: Path = MODELS_DIR) -> bool:
    """Whether the YOLO detector has weights to load (it takes the newest models/**/*.onnx)."""
    return models_dir.is_dir() and any(models_dir.rglob('*.onnx'))


def quick_argv(req: dict, backend_dir: Path = BACKEND_DIR,
               models_dir: Path = MODELS_DIR) -> tuple[list[str], str, dict]:
    quick = next((q for q in QUICK_STARTS if q['id'] == req.get('id')), None)
    if quick is None:
        raise BadRequest(f'unknown quick start: {req.get("id")!r}')
    sim = bool(req.get('sim'))
    if quick['yolo'] and not sim and not has_weights(models_dir):
        raise BadRequest('no YOLO weights in models/ yet: wait for the download (the YOLO '
                         'weights light), or restart the GUI without --no-weights')
    argv, _ = scenario_argv({'name': quick['scenario'], 'args': shlex.join(quick['args']),
                             'sim': sim}, backend_dir)
    return argv, quick['label'] + (' (--sim)' if sim else ''), quick


def scenario_argv(req: dict, backend_dir: Path = BACKEND_DIR) -> tuple[list[str], str]:
    name = req.get('name')
    known = {s['name']: s for s in scenarios(backend_dir)}
    if name not in known:
        raise BadRequest(f'unknown scenario: {name!r}')
    sim = ['--sim'] if req.get('sim') else []
    if sim and not known[name]['sim']:
        raise BadRequest(f'{name} has no fast simulation')
    args = split_args(req.get('args'))
    if known[name]['type'] == 'module':
        argv = [str(backend_dir / 'mount-backend.sh'), 'scenario', name, *sim, *args]
    else:
        argv = [str(backend_dir / 'scenarios' / name), *args, *sim]
    return argv, ' '.join(['scenario', name, *sim, *args])


# ============================================================
# Stellar rules (booking, GPU, disk) and container status
# ============================================================

def run(argv: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        out = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return out.returncode, out.stdout + out.stderr
    except (OSError, subprocess.TimeoutExpired) as e:
        return -1, str(e)


def booking(path: Path, user: str) -> dict:
    """Whether the machine is booked by someone else (then ask before GPU or long jobs)."""
    try:
        text = path.read_text().strip()
    except OSError:
        return {'booked': False, 'blocks': False, 'text': ''}
    if not text:
        return {'booked': False, 'blocks': False, 'text': ''}
    try:
        data = json.loads(text)
    except ValueError:
        return {'booked': True, 'blocks': True, 'text': text, 'note': 'could not read booking.json'}
    mine = data.get('account') == user if isinstance(data, dict) else False
    shared = bool(data.get('shared')) if isinstance(data, dict) else False
    return {'booked': True, 'blocks': not (mine or shared), 'text': text,
            'account': data.get('account') if isinstance(data, dict) else None, 'shared': shared}


def gpu_info(user: str) -> dict:
    if not shutil.which('nvidia-smi'):
        return {'available': False, 'text': 'nvidia-smi not found: no NVIDIA driver', 'others': []}
    code, text = run(['nvidia-smi'])
    others = []
    for line in text.splitlines():
        m = NVIDIA_PROC_RE.match(line)
        if not m:
            continue
        pid, kind, name = int(m.group(1)), m.group(2), m.group(3)
        try:
            owner = pwd.getpwuid(os.stat(f'/proc/{pid}').st_uid).pw_name
        except (OSError, KeyError):
            owner = '?'
        if owner not in (user, 'root', 'gdm'):
            # G only: a desktop session (Xorg, a browser); C: compute, e.g. training
            others.append({'pid': pid, 'user': owner, 'name': name, 'compute': 'C' in kind})
    return {'available': code == 0, 'text': text, 'others': others}


def preflight(booking_path: Path) -> dict:
    user = getpass.getuser()
    disks = ['/'] + (['/scratch'] if os.path.isdir('/scratch') else [])
    return {'user': user, 'booking': booking(booking_path, user), 'gpu': gpu_info(user),
            'disk': run(['df', '-h', *disks])[1], 'darwin': platform.system() == 'Darwin'}


def camera_drones(gz_topics: str) -> list[dict]:
    """PX4 instances with a camera, from `gz topic -l` (the same topic names as
    SENSOR_TOPIC and DEPTH_TOPIC in swarm_backend/sim/sensor_view.py, which `view` uses)."""
    cams: dict[int, dict] = {}
    for m in CAMERA_TOPIC_RE.finditer(gz_topics):
        drone = cams.setdefault(int(m['id']), {'id': int(m['id']), 'model': m['model'], 'cameras': []})
        if m['sensor'] not in drone['cameras']:
            drone['cameras'].append(m['sensor'])
    if re.search(r'^/depth_camera$', gz_topics, re.M):  # OakD-Lite: one topic for every x500_depth
        for m in DEPTH_MODEL_RE.finditer(gz_topics):
            drone = cams.setdefault(int(m['id']), {'id': int(m['id']), 'model': 'x500_depth', 'cameras': []})
            if 'depth_camera' not in drone['cameras']:
                drone['cameras'].append('depth_camera')
    return [cams[i] for i in sorted(cams)]


def http_answers(port: int, timeout: float = 1.0) -> bool:
    """Whether a web server answers on 127.0.0.1:port. A TCP connect is not enough: with
    rootless Docker, rootlesskit holds every published port while the container runs, and
    only drops the connection when nothing listens inside."""
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=timeout)
    try:
        conn.request('HEAD', '/')
        conn.getresponse()
        return True
    except (OSError, http.client.HTTPException):
        return False
    finally:
        conn.close()


def container_status(ui_port: int, with_cameras: bool) -> dict:
    code, out = run(['docker', 'inspect', '-f', '{{.State.Running}}', CONTAINER])
    status = {'container': code == 0 and out.strip() == 'true', 'gazebo': False, 'drones': [],
              'operator_ui': False}
    if status['container']:
        _, out = run(['docker', 'exec', CONTAINER, 'sh', '-c',
                      "pgrep -f 'gz sim' >/dev/null && echo GAZEBO_UP; ps -eo args"])
        status['gazebo'] = 'GAZEBO_UP' in out
        status['drones'] = sorted({int(i) for i in PX4_INSTANCE_RE.findall(out)})
        status['operator_ui'] = http_answers(ui_port)
        if with_cameras and status['gazebo']:
            _, topics = run(['docker', 'exec', CONTAINER, 'gz', 'topic', '-l'], timeout=15.0)
            status['cameras'] = camera_drones(topics)
    return status


class StatusPoller:
    """Polls the container in the background, so a page request never waits on docker
    (`gz topic -l` alone takes about 2 s). Starts with the first request."""

    def __init__(self, ui_port: int, on_update=None):
        self.ui_port = ui_port
        self.on_update = on_update  # called with each new status, on the poller's thread
        self._value = {'container': False, 'gazebo': False, 'drones': [], 'operator_ui': False,
                       'cameras': []}
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._thread = None

    def get(self) -> dict:
        with self._lock:
            if self._thread is None:
                self._thread = threading.Thread(target=self._loop, daemon=True)
                self._thread.start()
            return dict(self._value)

    def refresh(self):
        """Poll now (after starting or stopping something), not at the next interval."""
        self._wake.set()

    def _loop(self):
        last_cameras = 0.0
        while True:
            with_cameras = time.time() - last_cameras > CAMERA_POLL_S
            value = container_status(self.ui_port, with_cameras)
            if with_cameras:
                last_cameras = time.time()
            with self._lock:
                if 'cameras' not in value:  # keep the last camera list between camera polls
                    value['cameras'] = self._value['cameras'] if value['gazebo'] else []
                self._value = value
            if self.on_update is not None:
                try:
                    self.on_update(dict(value))
                except Exception as e:  # never stop polling
                    print(f'status update: {e}', flush=True)
            self._wake.wait(STATUS_POLL_S)
            self._wake.clear()


class WeightsFetcher:
    """Downloads the missing WEIGHTS into models/ in the background, so the GUI starts at
    once. Writes to a .part file first: a broken download never looks like weights."""

    def __init__(self, models_dir: Path = MODELS_DIR, base_url: str = RELEASES_URL,
                 weights=WEIGHTS, log=None):
        self.models_dir, self.base_url, self.weights = models_dir, base_url, weights
        self._log = log or (lambda msg: print(msg, flush=True))
        self._state = {'state': 'idle', 'files': [], 'message': ''}
        self._lock = threading.Lock()

    def state(self) -> dict:
        with self._lock:
            return dict(self._state)

    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)

    def start(self) -> threading.Thread:
        thread = threading.Thread(target=self.fetch, daemon=True)
        thread.start()
        return thread

    def fetch(self):
        files, failed = [], []
        for tag, name, folder in self.weights:
            dest = self.models_dir / folder / name
            files.append(str(dest.relative_to(self.models_dir.parent)))
            if dest.is_file() and dest.stat().st_size > 0:
                continue
            self._set(state='downloading', files=files, message=f'downloading {name}')
            self._log(f'YOLO weights: downloading {name} ({tag})...')
            try:
                self._download(f'{self.base_url}/{tag}/{name}', dest)
                self._log(f'YOLO weights: saved {dest}')
            except (OSError, ValueError) as e:  # URLError and HTTPError are OSErrors
                failed.append(name)
                self._log(f'YOLO weights: could not download {name}: {e}')
        if failed:
            self._set(state='failed', files=files,
                      message=f'could not download {", ".join(failed)} (see the GUI terminal)')
        else:
            self._set(state='ok', files=files, message='')

    @staticmethod
    def _download(url: str, dest: Path):
        dest.parent.mkdir(parents=True, exist_ok=True)
        part = dest.with_name(dest.name + '.part')
        try:
            with urllib.request.urlopen(url, timeout=30) as r, open(part, 'wb') as f:
                expected = int(r.headers.get('Content-Length') or 0)
                shutil.copyfileobj(r, f, length=1 << 20)
            size = part.stat().st_size
            if size == 0 or (expected and size != expected):
                raise ValueError(f'got {size} of {expected or "?"} bytes')
            part.replace(dest)
        finally:
            part.unlink(missing_ok=True)


# ============================================================
# HTTP
# ============================================================

class DevGui:
    def __init__(self, token: str, booking_path: Path = BOOKING_PATH,
                 darwin: bool = platform.system() == 'Darwin',
                 docker_dir: Path = DOCKER_DIR, backend_dir: Path = BACKEND_DIR,
                 repo: Path = REPO, models_dir: Path = MODELS_DIR):
        self.token = token
        self.booking_path = booking_path
        self.darwin = darwin
        self.docker_dir, self.backend_dir, self.repo = docker_dir, backend_dir, repo
        self.models_dir = models_dir
        self.schedule_dir = repo / 'data' / 'schedules' / 'dev_gui'
        self.jobs = JobManager()
        self.ui_port = int(os.environ.get('HOST_UI_PORT', '8090'))  # as in start-stellar.sh
        self.status = StatusPoller(self.ui_port, on_update=self.auto_views)
        self._auto_views = {}  # single job id -> {'drone', 'job' (its view job), 'tries'}
        self._auto_lock = threading.Lock()
        self.weights = WeightsFetcher(models_dir)

    def config(self) -> dict:
        return {'models': MODELS, 'worlds': worlds(self.repo), 'backend': BACKEND_COMMANDS,
                'scenarios': scenarios(self.backend_dir), 'darwin': self.darwin,
                'ui_port': self.ui_port,
                'quick': [{k: q[k] for k in ('id', 'label', 'about', 'yolo')} for q in QUICK_STARTS],
                'single': {'paths': SINGLE_PATHS, 'setups': SINGLE_SETUPS,
                           'altitudes': SINGLE_ALTITUDES, 'max_runs': MAX_SINGLE_RUNS},
                'schedule': {'altitudes': SCHEDULE_ALTITUDES, 'max_runs': MAX_SCHEDULE_RUNS,
                             'max_steps': MAX_SCHEDULE_STEPS}}

    def start_job(self, req: dict) -> Job:
        kind = req.get('kind')
        env = None
        if kind == 'container':
            argv, title = container_argv(req, self.darwin, self.docker_dir)
            user = getpass.getuser()
            book = booking(self.booking_path, user)
            if book['blocks'] and not req.get('confirm_booking'):
                raise BadRequest(f'the machine is booked by {book.get("account") or "someone else"}: '
                                 'confirm to start anyway')
            if req.get('gpu', True) and not self.darwin and not req.get('confirm_gpu'):
                others = gpu_info(user)['others']
                if others:
                    who = ', '.join(sorted({o['user'] for o in others}))
                    raise BadRequest(f'other users are on the GPU ({who}): confirm, or start without the GPU')
            cwd = self.docker_dir
        elif kind == 'backend':
            argv, title = backend_argv(req, self.backend_dir)
            cwd = self.backend_dir
            if req.get('command') == 'ui' and 'SWARM_CONFIG' not in os.environ:
                env = {'SWARM_CONFIG': SURVEILLANCE_CONFIG}
        elif kind == 'scenario':
            argv, title = scenario_argv(req, self.backend_dir)
            cwd = self.backend_dir
        elif kind in ('quick', 'single', 'schedule'):
            if any(j.kind in SCENARIO_KINDS and j.running for j in self.jobs.all()):
                raise BadRequest('a scenario is already running: two would fight over the drones')
            if not req.get('sim'):  # hours: a long job under the Stellar rules
                book = booking(self.booking_path, getpass.getuser())
                if book['blocks'] and not req.get('confirm_booking'):
                    raise BadRequest(f'the machine is booked by {book.get("account") or "someone else"}: '
                                     'confirm to start a long run anyway')
                if not self.status.get()['gazebo']:
                    raise BadRequest('Gazebo is not running: start the container first (or tick --sim)')
            if kind == 'quick':
                argv, title, _ = quick_argv(req, self.backend_dir, self.models_dir)
            elif kind == 'single':
                argv, title, _ = single_argv(req, self.backend_dir, self.models_dir)
            else:  # last: it writes the schedule file
                argv, title, _ = schedule_argv(req, self.backend_dir, self.models_dir,
                                               self.schedule_dir)
            cwd = self.backend_dir
        else:
            raise BadRequest(f'unknown job kind: {kind!r}')
        job = self.jobs.start(kind, title, argv, cwd, env)
        if kind in ('single', 'schedule') and req.get('rviz', True) and not req.get('sim'):
            with self._auto_lock:
                self._auto_views[job.id] = {'drone': SINGLE_RVIZ_DRONE, 'job': None, 'tries': 0}
        self.status.refresh()
        return job

    def auto_views(self, status: dict):
        """While a single run asked for it: open RViz (`view N`) once drone N's camera is in
        Gazebo (the run spawns it, so not at once). Opened again if it ended with an error,
        but not if it was closed (exit 0) or another view of N runs already."""
        cameras = {c['id'] for c in status.get('cameras', [])}
        with self._auto_lock:
            for single_id, view in list(self._auto_views.items()):
                try:
                    single = self.jobs.get(single_id)
                except BadRequest:
                    single = None
                if single is None or not single.running:
                    del self._auto_views[single_id]  # its RViz stays open
                    continue
                drone = view['drone']
                mine = view['job']
                if mine is not None and (mine.running or mine.exit_code == 0):
                    continue
                if view['tries'] >= AUTO_VIEW_TRIES or drone not in cameras:
                    continue
                if any(j.kind == 'backend' and j.running and j.argv[1:3] == ['view', str(drone)]
                       for j in self.jobs.all()):
                    continue
                argv, title = backend_argv({'command': 'view', 'args': str(drone)}, self.backend_dir)
                view['job'] = self.jobs.start('backend', title, argv, self.backend_dir)
                view['tries'] += 1

    def links(self) -> dict:
        """noVNC URL and password, from the container job's own output."""
        out = {}
        for job in self.jobs.all():
            if job.kind == 'container' and job.running:
                text = job.text()
                if m := NOVNC_URL_RE.search(text):
                    out['novnc'] = m.group(1)
                if m := VNC_PASSWORD_RE.search(text):
                    out['vnc_password'] = m.group(1)
        return out


def make_handler(app: DevGui):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # quiet: the page polls every second
            pass

        def _send(self, code: int, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, data):
            self._send(code, json.dumps(data).encode(), 'application/json')

        def _authorized(self) -> bool:
            given = self.headers.get('X-Token', '')
            if hmac.compare_digest(given.encode(), app.token.encode()):
                return True
            self._json(403, {'error': 'missing or wrong token: open the URL dev_gui.py printed'})
            return False

        def _body(self) -> dict:
            length = int(self.headers.get('Content-Length') or 0)
            if length > 1_000_000:
                raise BadRequest('request too large')
            try:
                data = json.loads(self.rfile.read(length) or b'{}')
            except ValueError as e:
                raise BadRequest('body is not JSON') from e
            if not isinstance(data, dict):
                raise BadRequest('body must be a JSON object')
            return data

        def _job_route(self, path: str):
            m = re.fullmatch(r'/api/jobs/(\d+)/(log|input|stop|dismiss)', path)
            return (int(m.group(1)), m.group(2)) if m else (None, None)

        def do_GET(self):
            url = urlsplit(self.path)
            if url.path in ('/', '/index.html'):
                self._send(200, (STATIC_DIR / 'index.html').read_bytes(), 'text/html; charset=utf-8')
                return
            if not url.path.startswith('/api/'):
                self._json(404, {'error': 'not found'})
                return
            if not self._authorized():
                return
            try:
                if url.path == '/api/config':
                    self._json(200, app.config())
                elif url.path == '/api/preflight':
                    self._json(200, preflight(app.booking_path))
                elif url.path == '/api/status':
                    self._json(200, {**app.status.get(), **app.links(),
                                     'weights': app.weights.state(),
                                     'has_weights': has_weights(app.models_dir),
                                     'jobs': [j.summary() for j in app.jobs.all()]})
                else:
                    job_id, action = self._job_route(url.path)
                    if action != 'log':
                        self._json(404, {'error': 'not found'})
                        return
                    since = int(parse_qs(url.query).get('since', ['0'])[0])
                    self._json(200, app.jobs.get(job_id).log(since))
            except (BadRequest, ValueError) as e:
                self._json(400, {'error': str(e)})

        def do_POST(self):
            path = urlsplit(self.path).path
            if not self._authorized():
                return
            try:
                body = self._body()
                if path == '/api/jobs':
                    self._json(200, app.start_job(body).summary())
                    return
                job_id, action = self._job_route(path)
                if action == 'input':
                    text = body.get('text', '')
                    if not isinstance(text, str):
                        raise BadRequest('text must be a string')
                    # Enter is \r in a terminal; docker exec -it puts the pty in raw mode.
                    app.jobs.get(job_id).send(text if body.get('raw') else text + '\r')
                elif action == 'stop':
                    app.jobs.get(job_id).stop()
                elif action == 'dismiss':
                    app.jobs.dismiss(job_id)
                else:
                    self._json(404, {'error': 'not found'})
                    return
                self._json(200, {'ok': True})
            except BadRequest as e:
                self._json(400, {'error': str(e)})
            except OSError as e:
                self._json(500, {'error': f'could not start: {e}'})

    return Handler


def make_server(app: DevGui, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer(('127.0.0.1', port), make_handler(app))
    server.daemon_threads = True
    return server


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--port', type=int, default=int(os.environ.get('DEV_GUI_PORT', '8095')),
                        help='port on 127.0.0.1 (default 8095, or $DEV_GUI_PORT)')
    parser.add_argument('--no-weights', action='store_true',
                        help='do not fetch missing YOLO weights into models/ at startup')
    args = parser.parse_args(argv)

    app = DevGui(token=secrets.token_urlsafe(18))
    try:
        server = make_server(app, args.port)
    except OSError as e:
        print(f'Could not listen on 127.0.0.1:{args.port} ({e}). Another user may hold it: try --port 8096')
        return 1

    print('Hive dev GUI (only this computer can reach it; the token keeps other users out):')
    print(f'  http://127.0.0.1:{args.port}/?token={app.token}')
    print('Ctrl-C here stops the GUI and every job it started (the container too).', flush=True)
    if not args.no_weights:
        app.weights.start()

    def on_signal(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGHUP, on_signal)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        running = [j for j in app.jobs.all() if j.running]
        if running:
            print(f'\nStopping {len(running)} job(s)...')
        app.jobs.stop_all()
        server.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
