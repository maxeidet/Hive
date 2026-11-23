"""Tests for the dev GUI (docker/dev_gui.py). Host only, no Docker needed:

    python3 -m pytest docker/test_dev_gui.py
"""
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dev_gui  # noqa: E402


def wait_for(cond, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return True
        time.sleep(0.05)
    return False


def text_of(job):
    return job.text()


# ---- jobs ----

def test_job_output_input_and_exit(tmp_path):
    jobs = dev_gui.JobManager()
    job = jobs.start('backend', 'cat', ['cat'], tmp_path)
    job.send('hello pty\r')
    assert wait_for(lambda: text_of(job).count('hello pty') >= 2)  # pty echo + cat
    job.stop(grace_s=2.0)
    assert wait_for(lambda: not job.running)


def test_partial_line_is_shown_and_since_works(tmp_path):
    jobs = dev_gui.JobManager()
    job = jobs.start('backend', 'p', [sys.executable, '-c',
                                      'import sys; print("a"); print("b"); sys.stdout.write("pxh> "); sys.stdout.flush(); input()'],
                     tmp_path)
    assert wait_for(lambda: job.log(0)['partial'] == 'pxh> ')
    log = job.log(0)
    assert [t for _, t in log['lines']] == ['a', 'b']
    assert [t for _, t in job.log(1)['lines']] == ['b']
    job.send('\r')
    assert wait_for(lambda: not job.running)
    assert job.exit_code == 0


def test_stop_sends_ctrl_c_first(tmp_path):
    script = ('import signal, sys, time\n'
              'signal.signal(signal.SIGINT, lambda *a: (print("landing"), sys.exit(3)))\n'
              'print("up", flush=True)\n'
              'time.sleep(30)\n')
    job = dev_gui.JobManager().start('backend', 's', [sys.executable, '-c', script], tmp_path)
    assert wait_for(lambda: 'up' in text_of(job))
    job.stop(grace_s=5.0)
    assert wait_for(lambda: not job.running)
    assert job.exit_code == 3
    assert 'landing' in text_of(job)


def test_stop_escalates_when_ctrl_c_is_ignored(tmp_path):
    script = 'import signal, time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\nprint("up", flush=True)\ntime.sleep(30)\n'
    job = dev_gui.JobManager().start('backend', 's', [sys.executable, '-c', script], tmp_path)
    assert wait_for(lambda: 'up' in text_of(job))
    job.stop(grace_s=0.5)
    assert wait_for(lambda: not job.running, timeout=5.0)


def test_ring_buffer_and_carriage_returns(tmp_path):
    script = ('import sys\n'
              f'for i in range({dev_gui.MAX_LINES + 50}): print(i)\n'
              'sys.stdout.write("10%\\r50%\\r100%\\n")\n')
    job = dev_gui.JobManager().start('backend', 'r', [sys.executable, '-c', script], tmp_path)
    assert wait_for(lambda: not job.running)
    log = job.log(0)
    assert log['truncated']
    assert len(log['lines']) == dev_gui.MAX_LINES
    assert log['lines'][-1][1] == '100%'


def test_only_one_container_job(tmp_path):
    jobs = dev_gui.JobManager()
    first = jobs.start('container', 'c', ['sleep', '5'], tmp_path)
    with pytest.raises(dev_gui.BadRequest):
        jobs.start('container', 'c', ['sleep', '5'], tmp_path)
    first.stop(grace_s=1.0)
    assert wait_for(lambda: not first.running)


# ---- what the page may run ----

def test_backend_whitelist_and_no_shell():
    argv, _ = dev_gui.backend_argv({'command': 'spawn', 'args': '3; rm -rf ~'})
    assert argv[1:] == ['spawn', '3;', 'rm', '-rf', '~']  # literal arguments, never a shell
    with pytest.raises(dev_gui.BadRequest):
        dev_gui.backend_argv({'command': 'rm'})
    with pytest.raises(dev_gui.BadRequest):
        dev_gui.backend_argv({'command': 'spawn', 'args': '"unclosed'})


def test_scenarios_found_and_checked():
    found = {s['name']: s for s in dev_gui.scenarios()}
    assert found['attack']['type'] == 'module' and found['attack']['sim']
    assert 'compare_paths.sh' in found and 'lib.sh' not in found
    argv, _ = dev_gui.scenario_argv({'name': 'attack', 'sim': True, 'args': '--runs 2'})
    assert argv[1:] == ['scenario', 'attack', '--sim', '--runs', '2']
    argv, _ = dev_gui.scenario_argv({'name': 'compare_paths.sh', 'args': '3'})
    assert argv[0].endswith('scenarios/compare_paths.sh') and argv[1:] == ['3']
    with pytest.raises(dev_gui.BadRequest):
        dev_gui.scenario_argv({'name': '../../evil.sh'})


def test_container_args():
    argv, _ = dev_gui.container_argv({'model': 'gz_x500_depth', 'world': 'default', 'gui': 'vnc', 'gpu': False}, darwin=False)
    assert argv[1:] == ['-m', 'gz_x500_depth', '-w', 'default', '--vnc', '--no-gpu', '--no-drone',
                        '--no-gazebo-gui']
    argv, _ = dev_gui.container_argv({'world': 'default', 'gui': 'headless', 'gpu': False, 'drone0': True}, darwin=False)
    assert '--no-drone' not in argv
    with pytest.raises(dev_gui.BadRequest):
        dev_gui.container_argv({'model': 'x; rm', 'world': 'default'}, darwin=False)
    with pytest.raises(dev_gui.BadRequest):
        dev_gui.container_argv({'gui': 'native', 'gpu': False}, darwin=True)


def test_booking(tmp_path):
    path = tmp_path / 'booking.json'
    assert not dev_gui.booking(path, 'me')['blocks']  # no file
    path.write_text('')
    assert not dev_gui.booking(path, 'me')['booked']
    path.write_text(json.dumps({'account': 'me'}))
    assert not dev_gui.booking(path, 'me')['blocks']
    path.write_text(json.dumps({'account': 'other', 'shared': True}))
    assert not dev_gui.booking(path, 'me')['blocks']
    path.write_text(json.dumps({'account': 'other'}))
    assert dev_gui.booking(path, 'me')['blocks']


# ---- HTTP ----

@pytest.fixture
def server(tmp_path):
    booking = tmp_path / 'booking.json'
    booking.write_text(json.dumps({'account': 'someone-else'}))
    app = dev_gui.DevGui(token='secret', booking_path=booking, darwin=False)
    srv = dev_gui.make_server(app, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield app, f'http://127.0.0.1:{srv.server_address[1]}'
    app.jobs.stop_all(wait_s=3)
    srv.shutdown()


def call(url, body=None, token='secret'):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, headers={'X-Token': token, 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_token_required(server):
    _, base = server
    assert call(base + '/api/config', token='')[0] == 403
    assert call(base + '/api/config', token='wrong')[0] == 403
    assert call(base + '/api/jobs', {'kind': 'backend', 'command': 'spawn'}, token='')[0] == 403
    code, cfg = call(base + '/api/config')
    assert code == 200 and 'hive_base' in cfg['worlds']
    with urllib.request.urlopen(base + '/', timeout=5) as r:  # the page itself needs no token
        assert b'Hive dev' in r.read()


def test_container_refused_when_booked(server):
    _, base = server
    code, body = call(base + '/api/jobs', {'kind': 'container', 'gpu': False})
    assert code == 400 and 'booked' in body['error']


def test_unknown_kind_rejected(server):
    _, base = server
    assert call(base + '/api/jobs', {'kind': 'shell', 'cmd': 'id'})[0] == 400
    assert call(base + '/api/jobs', {'kind': 'backend', 'command': 'bash'})[0] == 400


def test_camera_drones_from_gz_topics():
    topics = '\n'.join([
        '/clock',
        '/world/hive_base/model/x500_gimbal_2/link/camera_link/sensor/camera/image',
        '/world/hive_base/model/x500_gimbal_2/link/camera_link/sensor/camera/camera_info',
        '/world/hive_base/model/x500_gimbal_1/link/camera_link/sensor/camera/image',
        '/world/hive_base/model/x500_lidar_3d_4/link/lidar_link/sensor/lidar/scan/points',  # no camera
        '/world/hive_base/model/x500_depth_5/pose/info',
        '/depth_camera',
        '/world/hive_base/model/x500_0/pose/info',
    ]) + '\n'
    found = dev_gui.camera_drones(topics)
    assert [(d['id'], d['model'], d['cameras']) for d in found] == [
        (1, 'x500_gimbal', ['camera']), (2, 'x500_gimbal', ['camera']), (5, 'x500_depth', ['depth_camera'])]
    assert dev_gui.camera_drones('/clock\n') == []


def test_http_answers_needs_a_web_server(server):
    _, base = server
    assert dev_gui.http_answers(int(base.rsplit(':', 1)[1]))
    import socket
    # Accepts the connection but never answers HTTP, like rootlesskit with nothing inside
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        s.listen()
        port = s.getsockname()[1]
        threading.Thread(target=lambda: s.accept()[0].close(), daemon=True).start()
        assert not dev_gui.http_answers(port)
    assert not dev_gui.http_answers(port)


# ---- YOLO weights ----

@pytest.fixture
def release_server(tmp_path):
    """A stand-in for GitHub Releases: serves files from tmp_path/release/<tag>/<file>."""
    import functools
    import http.server
    root = tmp_path / 'release'
    root.mkdir()
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(root))
    handler.log_message = lambda *a: None
    srv = http.server.ThreadingHTTPServer(('127.0.0.1', 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield root, f'http://127.0.0.1:{srv.server_address[1]}'
    srv.shutdown()


def test_weights_downloaded_when_missing(tmp_path, release_server):
    root, url = release_server
    (root / 'tag').mkdir()
    (root / 'tag' / 'm.onnx').write_bytes(b'onnx' * 1000)
    models = tmp_path / 'models'
    f = dev_gui.WeightsFetcher(models, url, [('tag', 'm.onnx', 'm')], log=lambda m: None)
    f.fetch()
    assert (models / 'm' / 'm.onnx').read_bytes() == b'onnx' * 1000
    assert f.state()['state'] == 'ok'
    assert not list(models.rglob('*.part'))


def test_existing_weights_not_downloaded_again(tmp_path):
    models = tmp_path / 'models'
    (models / 'm').mkdir(parents=True)
    (models / 'm' / 'm.onnx').write_bytes(b'mine')
    # an unreachable URL: a download attempt would fail
    f = dev_gui.WeightsFetcher(models, 'http://127.0.0.1:9', [('tag', 'm.onnx', 'm')], log=lambda m: None)
    f.fetch()
    assert f.state()['state'] == 'ok'
    assert (models / 'm' / 'm.onnx').read_bytes() == b'mine'


def test_failed_download_leaves_nothing(tmp_path, release_server):
    _, url = release_server  # the file is not there: 404
    models = tmp_path / 'models'
    f = dev_gui.WeightsFetcher(models, url, [('tag', 'gone.onnx', 'm')], log=lambda m: None)
    f.fetch()
    assert f.state()['state'] == 'failed'
    assert not list(models.rglob('*.onnx*'))


# ---- quick starts ----

def test_quick_start_commands(tmp_path):
    models = tmp_path / 'models'
    argv, title, _ = dev_gui.quick_argv({'id': 'compare-baseline'}, models_dir=models)
    assert argv[0].endswith('scenarios/compare_paths.sh')
    assert argv[1:] == ['3', '--setup', 'baseline']
    argv, title, _ = dev_gui.quick_argv({'id': 'compare-baseline', 'sim': True}, models_dir=models)
    assert argv[1:] == ['3', '--setup', 'baseline', '--sim'] and '--sim' in title
    with pytest.raises(dev_gui.BadRequest):
        dev_gui.quick_argv({'id': 'nope'}, models_dir=models)


def test_yolo_quick_start_needs_weights_except_in_sim(tmp_path):
    models = tmp_path / 'models'
    with pytest.raises(dev_gui.BadRequest, match='weights'):
        dev_gui.quick_argv({'id': 'compare-yolo-mpc'}, models_dir=models)
    argv, _, _ = dev_gui.quick_argv({'id': 'compare-yolo-mpc', 'sim': True}, models_dir=models)
    assert argv[1:] == ['3', '--setup', 'yolo-bytetrack-mpc', '--sim']
    (models / 'm').mkdir(parents=True)
    (models / 'm' / 'w.onnx').write_bytes(b'x')
    argv, _, _ = dev_gui.quick_argv({'id': 'compare-yolo-mpc'}, models_dir=models)
    assert argv[1:] == ['3', '--setup', 'yolo-bytetrack-mpc']


def test_quick_start_refused_when_booked_or_no_gazebo(server):
    app, base = server  # booked by someone-else, and no container in the test
    code, body = call(base + '/api/jobs', {'kind': 'quick', 'id': 'compare-baseline'})
    assert code == 400 and 'booked' in body['error']
    code, body = call(base + '/api/jobs', {'kind': 'quick', 'id': 'compare-baseline', 'confirm_booking': True})
    assert code == 400 and 'Gazebo' in body['error']


def test_quick_start_refused_while_a_scenario_runs(server, tmp_path):
    app, base = server
    flying = app.jobs.start('scenario', 'flying', ['sleep', '10'], tmp_path)
    code, body = call(base + '/api/jobs', {'kind': 'quick', 'id': 'compare-baseline', 'sim': True})
    assert code == 400 and 'already running' in body['error']
    flying.stop(grace_s=1.0)


# ---- single scenarios ----

def test_single_scenario_command(tmp_path):
    models = tmp_path / 'models'
    argv, title, yolo = dev_gui.single_argv(
        {'path': 'zigzag_15m', 'setup': 'baseline', 'altitude': '0', 'runs': '2'}, models_dir=models)
    assert argv[0].endswith('scenarios/compare_paths.sh')
    assert argv[1:] == ['2', '--paths', 'zigzag_15m', '--setup', 'baseline',
                        '--altitude-offsets', '0', '--spawn-at-slots', '--restart-each-run']
    assert not yolo and 'zigzag' in title
    argv, _, _ = dev_gui.single_argv({'path': 'straight', 'setup': 'baseline', 'altitude': 'all',
                                      'spawn_at_slots': False, 'sim': True}, models_dir=models)
    assert argv[1:] == ['1', '--paths', 'straight', '--setup', 'baseline', '--sim']


@pytest.mark.parametrize('req', [
    {'path': 'loop', 'setup': 'baseline'},
    {'path': 'straight', 'setup': 'magic'},
    {'path': 'straight', 'setup': 'baseline', 'altitude': '5'},
    {'path': 'straight', 'setup': 'baseline', 'runs': '0'},
    {'path': 'straight', 'setup': 'baseline', 'runs': '99'},
    {'path': 'straight', 'setup': 'baseline', 'runs': '1; rm'},
])
def test_single_scenario_rejects_bad_values(req, tmp_path):
    with pytest.raises(dev_gui.BadRequest):
        dev_gui.single_argv(req, models_dir=tmp_path)


def test_single_yolo_needs_weights_except_in_sim(tmp_path):
    req = {'path': 'straight', 'setup': 'yolo-bytetrack-mpc'}
    with pytest.raises(dev_gui.BadRequest, match='weights'):
        dev_gui.single_argv(req, models_dir=tmp_path)
    _, _, yolo = dev_gui.single_argv({**req, 'sim': True}, models_dir=tmp_path)
    assert yolo


def test_single_refused_while_a_scenario_runs(server, tmp_path):
    app, base = server
    flying = app.jobs.start('quick', 'flying', ['sleep', '10'], tmp_path)
    code, body = call(base + '/api/jobs', {'kind': 'single', 'path': 'straight', 'setup': 'baseline', 'sim': True})
    assert code == 400 and 'already running' in body['error']
    flying.stop(grace_s=1.0)


def test_job_env(tmp_path):
    job = dev_gui.JobManager().start('backend', 'e', [sys.executable, '-c', 'import os; print(os.environ["SWARM_CONFIG"])'],
                                     tmp_path, env={'SWARM_CONFIG': '/x/surveillance.yaml'})
    assert wait_for(lambda: not job.running)
    assert '/x/surveillance.yaml' in text_of(job)


def test_operator_ui_gets_the_surveillance_config(server, monkeypatch):
    app, _ = server
    monkeypatch.delenv('SWARM_CONFIG', raising=False)
    seen = {}
    monkeypatch.setattr(app.jobs, 'start', lambda kind, title, argv, cwd, env=None: seen.update(env=env) or
                        type('J', (), {'summary': lambda self: {}})())
    app.start_job({'kind': 'backend', 'command': 'ui'})
    assert seen['env'] == {'SWARM_CONFIG': dev_gui.SURVEILLANCE_CONFIG}
    app.start_job({'kind': 'backend', 'command': 'spawn'})
    assert seen['env'] is None


# ---- Gazebo window, and RViz for single runs ----

def test_gazebo_window_off_by_default_but_display_kept():
    argv, _ = dev_gui.container_argv({'world': 'default', 'gui': 'native', 'gpu': False}, darwin=False)
    assert '--native-gui' in argv and '--no-gazebo-gui' in argv
    argv, _ = dev_gui.container_argv({'world': 'default', 'gui': 'native', 'gpu': False,
                                      'gazebo_window': True}, darwin=False)
    assert '--no-gazebo-gui' not in argv
    argv, _ = dev_gui.container_argv({'world': 'default', 'gui': 'headless', 'gpu': False}, darwin=False)
    assert '--headless' in argv and '--no-gazebo-gui' not in argv  # no display at all already


class FakeJobs:
    """JobManager stand-in: records started jobs; `finish(job, code)` ends one."""

    def __init__(self):
        self.jobs = {}
        self.started = []

    def start(self, kind, title, argv, cwd, env=None):
        job = type('Job', (), {})()
        job.id, job.kind, job.argv, job.running, job.exit_code = len(self.jobs) + 1, kind, argv, True, None
        self.jobs[job.id] = job
        self.started.append(job)
        return job

    def get(self, job_id):
        if job_id not in self.jobs:
            raise dev_gui.BadRequest('no job')
        return self.jobs[job_id]

    def all(self):
        return list(self.jobs.values())

    @staticmethod
    def finish(job, code):
        job.running, job.exit_code = False, code


@pytest.fixture
def gui(tmp_path):
    app = dev_gui.DevGui(token='t', booking_path=tmp_path / 'none.json', darwin=False,
                         models_dir=tmp_path)
    app.jobs = FakeJobs()
    app.status.refresh = lambda: None
    app.status.get = lambda: {'gazebo': True}
    single = app.start_job({'kind': 'single', 'path': 'straight', 'setup': 'baseline'})
    return app, single


def views(app):
    return [j for j in app.jobs.started if j.kind == 'backend' and j.argv[1] == 'view']


def cams(*ids):
    return {'cameras': [{'id': i, 'model': 'x500_gimbal', 'cameras': ['camera']} for i in ids]}


def test_rviz_opens_when_drone_2_has_a_camera(gui):
    app, _ = gui
    app.auto_views(cams())  # drone 2 not spawned yet
    assert views(app) == []
    app.auto_views(cams(1, 2, 3))
    assert [v.argv[1:] for v in views(app)] == [['view', '2']]
    app.auto_views(cams(1, 2, 3))  # still open: not again
    assert len(views(app)) == 1


def test_rviz_reopened_after_an_error_but_not_after_closing(gui):
    app, _ = gui
    app.auto_views(cams(2))
    FakeJobs.finish(views(app)[0], 1)  # crashed
    app.auto_views(cams(2))
    assert len(views(app)) == 2
    FakeJobs.finish(views(app)[1], 0)  # the user closed RViz
    app.auto_views(cams(2))
    assert len(views(app)) == 2


def test_rviz_retries_are_limited(gui):
    app, _ = gui
    for _ in range(dev_gui.AUTO_VIEW_TRIES + 3):
        app.auto_views(cams(2))
        FakeJobs.finish(views(app)[-1], 1)
    assert len(views(app)) == dev_gui.AUTO_VIEW_TRIES


def test_no_rviz_when_not_asked_or_in_sim_or_after_the_run(tmp_path, gui):
    app, single = gui
    FakeJobs.finish(single, 0)  # the run is over before drone 2 showed up
    app.auto_views(cams(2))
    assert views(app) == []
    for req in ({'rviz': False}, {'sim': True}):
        other = dev_gui.DevGui(token='t', booking_path=tmp_path / 'none.json', darwin=False,
                               models_dir=tmp_path)
        other.jobs = FakeJobs()
        other.status.refresh = lambda: None
        other.status.get = lambda: {'gazebo': True}
        other.start_job({'kind': 'single', 'path': 'straight', 'setup': 'baseline', **req})
        other.auto_views(cams(2))
        assert views(other) == []


def test_no_second_rviz_when_one_already_runs(gui):
    app, _ = gui
    app.jobs.start('backend', 'view 2', ['mount-backend.sh', 'view', '2'], None)  # opened by hand
    app.auto_views(cams(2))
    assert len(views(app)) == 1
