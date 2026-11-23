"""Points the replay GUI's camera at one angle from recording_config.json ("replay_angles").

Runs INSIDE the sim container, next to `gz sim --playback` (replay.sh starts it).

Angle kinds (see recording_config.json):
  {"follow": MODEL, "offset": [x, y, z]}                       chase cam in MODEL's frame
  {"follow": MODEL, "offset": [...], "look_at_model": OTHER}   rides with MODEL, turns to OTHER
  {"position": [x, y, z], "look_at_point": [...], "track_model": M}   fixed tripod that pans after M
                                                                        (omit track_model for a static shot)

    python3 replay_camera.py <angle> [<pgain, default 0.1>] [<start: sim time, seconds>]

start (Hive): pause the replay at this simulation time of the live run before the camera is
set, so every angle's clip begins at the same sim time as the other clips cut with
--start. Find it with `recording/video.sh events`. Without it, the replay pauses where the
followed drone first exists, as in the original kit.

pgain is the camera smoothing (gz-sim CameraTracking follow_pgain/track_pgain):
lower = softer and laggier, 1.0 = rigidly attached.
"""
import math
import os
import sqlite3
import subprocess
import sys
import time

import kitconf


def gz(*args, timeout=10):
    try:
        return subprocess.run(['gz', *args], capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ''


def vec(v):
    return f'{{ x: {v[0]} y: {v[1]} z: {v[2]} }}'


def look_at(p, a):
    """Roll, pitch, yaw that point a camera (looking along +x) from p at a."""
    dx, dy, dz = a[0] - p[0], a[1] - p[1], a[2] - p[2]
    return 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def track(msg):
    # Publish a few times: the GUI may not have discovered this publisher yet.
    for _ in range(3):
        gz('topic', '-t', '/gui/track', '-m', 'gz.msgs.CameraTrack', '-p', msg)
        time.sleep(0.5)


def follow_old(target, offset):
    """Older GUIs without /gui/track: plain follow (no smoothing gain, no look-at)."""
    gz('service', '-s', '/gui/follow', '--reqtype', 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.Boolean',
       '--timeout', '3000', '--req', f'data: "{target}"')
    gz('service', '-s', '/gui/follow/offset', '--reqtype', 'gz.msgs.Vector3d', '--reptype', 'gz.msgs.Boolean',
       '--timeout', '3000', '--req', f'x: {offset[0]}, y: {offset[1]}, z: {offset[2]}')


def wait_for_model(name, max_s=60):
    """PX4 spawns the drone a few seconds INTO the log, so at the start of a replay it does
    not exist yet and a camera following it gets an empty target. Steps the paused replay
    forward one second at a time until the model is there. Returns the sim second it stopped
    at (this is where your recording will start: your sync offset), or None if it was
    already there. Only steps FORWARD: seeking backwards in gz-sim 8 playback makes models
    vanish."""
    def present():
        return f'- {name}' in gz('model', '--list')

    if present():
        return None
    control = next((s for s in gz('service', '-l').split() if s.endswith('/playback/control')), None)
    if not control:
        return None
    for sec in range(1, max_s + 1):
        gz('service', '-s', control, '--reqtype', 'gz.msgs.LogPlaybackControl', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req', f'pause: true seek {{ sec: {sec} }}')
        time.sleep(0.5)
        if present():
            return sec
    print(f'{name} did not appear in the first {max_s} s of the log.')
    return None


def log_t0(log_dir):
    """The live run's sim time at the start of the log (seconds). LogRecord starts with the
    world, so this is about 0; the replay's own clock counts from the start of the log."""
    try:
        db = sqlite3.connect(f'file:{os.path.join(log_dir, "state.tlog")}?mode=ro', uri=True)
        t0 = db.execute('SELECT MIN(time_recv) FROM messages').fetchone()[0]
        db.close()
    except sqlite3.Error as e:
        print(f'Could not read the log start time ({e}); assuming 0.')
        return 0.0
    t0 = int(t0 or 0) * 1e-9
    if t0 > 1e6:  # a wall-clock stamp, not sim time
        print(f'The log is stamped with wall-clock time ({t0:.0f}); assuming it starts at sim time 0.')
        return 0.0
    return t0


def playback_control():
    return next((s for s in gz('service', '-l').split() if s.endswith('/playback/control')), None)


def seek(sim):
    """Pauses the replay at `sim` seconds of the replay's clock (from the start of the log).
    Forward only: seeking backwards in gz-sim 8 playback makes models vanish."""
    control = playback_control()
    if not control:
        print('No /playback/control service: could not move the replay to the start time.')
        return False
    sec = int(sim)
    nsec = int(round((sim - sec) * 1e9))
    out = gz('service', '-s', control, '--reqtype', 'gz.msgs.LogPlaybackControl', '--reptype',
             'gz.msgs.Boolean', '--timeout', '5000', '--req',
             f'pause: true seek {{ sec: {sec} nsec: {nsec} }}')
    time.sleep(1.0)
    return 'true' in out


def replay_clock():
    """The paused replay's clock, seconds, as Gazebo reports it (for the terminal)."""
    world = kitconf.load()['world']
    out = gz('topic', '-e', '-n', '1', '-t', f'/world/{world}/clock', timeout=5)
    sec = nsec = 0
    in_sim = False
    for line in out.splitlines():
        line = line.strip()
        if line.startswith('sim'):
            in_sim = True
        elif in_sim and line.startswith('sec:'):
            sec = int(line.split(':')[1])
        elif in_sim and line.startswith('nsec:'):
            nsec = int(line.split(':')[1])
        elif in_sim and line.startswith('}'):
            return sec + nsec * 1e-9
    return None


def main(angle, pgain=0.1, start=None):
    cfg = kitconf.load()
    angles = cfg['replay_angles']
    if angle not in angles:
        sys.exit(f'Unknown angle {angle!r}. Angles in recording_config.json: {", ".join(angles)}')
    a = angles[angle]
    if str(a.get('follow', '')).startswith('@'):
        sys.exit(f'{angle} follows {a["follow"]}, which only the live cameras know '
                 '(mount-backend.sh camrec). Pick another angle for the replay.')

    # Wait for the GUI's CameraTracking plugin to come up.
    for _ in range(240):
        if '/gui/move_to/pose' in gz('service', '-l'):
            break
        time.sleep(1)
    else:
        sys.exit('The Gazebo GUI did not come up, so the camera angle was not set.')
    time.sleep(2)

    needed = {m for m in (a.get('follow'), a.get('look_at_model'), a.get('track_model')) if m}
    t0 = log_t0(os.environ.get('LOG_DIR', '.'))
    if start is not None:
        if start < t0:
            sys.exit(f'--start {start} is before the log starts (sim time {t0:.2f} s).')
        seek(start - t0)
        shown = replay_clock()
        print(f'Replay paused at sim time {start:.2f} s of the run (replay clock '
              f'{"?" if shown is None else f"{shown:.2f}"} s, log starts at {t0:.2f} s).')
        missing = [m for m in needed if f'- {m}' not in gz('model', '--list')]
        if missing:
            print(f'Not in the world at that time: {", ".join(missing)}. The camera may not follow.')
    else:
        for m in needed:
            at = wait_for_model(m)
            if at is not None:
                print(f'{m} appears at replay time {at} s = sim time {t0 + at:.2f} s of the run: '
                      f'the replay is paused there. Use --start {t0 + at:.2f} for the other clips.')

    gains = f'follow_pgain: {pgain} track_pgain: {pgain}'
    has_track = '/gui/track' in gz('topic', '-l')
    if not has_track:
        print('This Gazebo GUI has no /gui/track: using plain follow (no look-at, tripods stay fixed).')

    if 'follow' in a:
        target, offset, other = a['follow'], a['offset'], a.get('look_at_model')
        if not has_track:
            follow_old(target, offset)
        elif other:
            track(f'track_mode: FOLLOW_LOOK_AT follow_target {{ name: "{target}" }} '
                  f'track_target {{ name: "{other}" }} follow_offset {vec(offset)} {gains}')
        else:
            track(f'track_mode: FOLLOW follow_target {{ name: "{target}" }} follow_offset {vec(offset)} {gains}')
    else:
        p, target = a['position'], a['look_at_point']
        q = quat(*look_at(p, target))
        gz('service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req',
           f'pose {{ position {vec(p)} orientation {{ x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]} }} }}')
        time.sleep(2)
        if has_track and a.get('track_model'):
            track(f'track_mode: TRACK track_target {{ name: "{a["track_model"]}" }} '
                  f'track_offset {{ x: 0 y: 0 z: 0 }} {gains}')
    print(f'Camera set to {angle}. Click the video recorder button, pick mp4, then press play.')


if __name__ == '__main__':
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.1,
         float(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] else None)
