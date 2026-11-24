"""Logs the true poses of the drones, birds and recording cameras to poses.jsonl, always.

    ./mount-backend.sh posesrec [--out auto|DIR]      (recording/record.sh starts it)

The takes, the moments and the tactical map need every run's poses, also when no live
camera runs (`camrec` used to be the only writer, and only of x500* and rec_cam_*). Same
format and rate as camera_video.PoseFeed: {"sim": S, "poses": {name: [x, y, z, qx, qy, qz, qw]}}
at 25 Hz, Gazebo ENU. Which models: names starting with recording_config.json's
pose_log.prefixes, plus every drone named in the run's run.json. Ctrl-C stops it.
"""
import argparse
import json
import signal
import sys
import threading

from swarm_backend.scenarios.common import REPO

CONFIG = REPO / 'recording' / 'recording_config.json'
DEFAULT_PREFIXES = ['x500', 'bird_', 'rec_cam_']


def keep_names(prefixes: list[str], roles: list[str]):
    """Which pose names to log: those starting with a prefix, and the exact role names."""
    exact = set(roles)
    starts = tuple(prefixes)
    return lambda name: name in exact or name.startswith(starts)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--out', default='auto', help='run folder (default: the newest)')
    args = p.parse_args(argv)

    from swarm_backend.birds import world_name
    from swarm_backend.demo.runinfo import read_run_info
    from swarm_backend.ui.camera_video import PoseFeed
    from swarm_backend.ui.debug_video import run_folder

    folder = run_folder(args.out)
    folder.mkdir(parents=True, exist_ok=True)
    try:
        prefixes = json.loads(CONFIG.read_text()).get('pose_log', {}).get('prefixes', DEFAULT_PREFIXES)
    except (OSError, ValueError):
        prefixes = DEFAULT_PREFIXES
    try:
        info = read_run_info(folder)
        roles = info.defenders + info.hostiles
    except FileNotFoundError:
        roles = []
    world = world_name()
    feed = PoseFeed(world, folder / 'poses.jsonl', keep_names(prefixes, roles))
    print(f'Logging poses of {world} to {folder / "poses.jsonl"} (Ctrl-C stops)', flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while not stop.wait(1.0):
            pass
    except KeyboardInterrupt:
        pass
    feed.close()
    print(f'Stopped logging poses ({folder / "poses.jsonl"})', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
