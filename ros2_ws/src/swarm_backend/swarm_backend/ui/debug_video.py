"""Recording the detectors' debug images (the boxes drawn) on Gazebo's clock, for the demo videos.

Each defender's detector (detect/live.py) publishes its camera picture with the boxes drawn
on /<ns>/detection/debug, stamped with the camera image's simulation time. Nothing else saves
it: the Gazebo log has only poses, and in a replay the cameras do not run.

record  (`mount-backend.sh debugrec [IDS]`, during the run, next to `ui --record auto`)
    Subscribes to the debug topics of drones IDS (default 1 2 3) and encodes each into
    data/recordings/<newest run>/debug_d<ID>.mkv at 25 fps on simulation time: frame i
    shows the newest debug image stamped at or before START + i / 25, where START (the
    first image's stamp) is in debug_d<ID>.json next to it. Matroska, so a killed recorder
    still leaves a playable file. Ctrl-C stops it.

cut     (`recording/video.sh debug --id ID --start S --duration D`, afterwards)
    Cuts drone ID's recording to start at sim time S: debug_d<ID>_<S>.mp4. Its first frame
    shows the same sim time as `video.sh ui --start S` and `video.sh replay --start S`, so it
    composes with them (`video.sh compose ... --gazebo debug_d3_120.mp4`).

Only `record` needs ROS; the rest runs without a simulation.
"""
import argparse
import json
import math
import queue
import subprocess
import sys
import threading
from pathlib import Path

RECORDINGS = Path('/workspace/data/recordings')  # /workspace = the repo; one folder per run
FPS = 25.0
QUEUE_IMAGES = 30  # images waiting for the encoder per drone; more are dropped (and counted)


class FrameClock:
    """Turns images at irregular simulation times into frames at a constant rate.

    `add(sim)` for each new image returns how many frames of the PREVIOUS image to write
    now: every frame whose time START + i / fps lies before `sim`. `finish()` returns the
    frames still owed to the last image (one: the frame at or after its own time). An image
    older than the last one returns None (drop it)."""

    def __init__(self, fps: float = FPS):
        self.fps = fps
        self.start: float | None = None
        self.last: float | None = None
        self.written = 0

    def _frames_before(self, sim: float) -> int:
        # frames i >= 0 with START + i / fps < sim; the epsilon keeps 0.04 * 25 at 1, not 1.0000001
        return max(0, math.ceil((sim - self.start) * self.fps - 1e-6))

    def add(self, sim: float) -> int | None:
        if self.start is None:
            self.start = self.last = sim
            return 0
        if sim <= self.last:
            return None
        self.last = sim
        n = self._frames_before(sim) - self.written
        self.written += n
        return n

    def finish(self) -> int:
        if self.start is None:
            return 0
        n = max(1, self._frames_before(self.last) - self.written + 1)
        self.written += n
        return n


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def image_rgb(msg) -> bytes | None:
    """sensor_msgs/Image (rgb8 or bgr8) as packed RGB rows, or None for other encodings."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        return None
    row = msg.width * 3
    data = bytes(msg.data)
    if msg.step != row:
        data = b''.join(data[r * msg.step:r * msg.step + row] for r in range(msg.height))
    if msg.encoding == 'bgr8':
        import numpy as np
        data = np.frombuffer(data, np.uint8).reshape(-1, 3)[:, ::-1].tobytes()
    return data


def encoder_command(width: int, height: int, fps: float, out: Path) -> list[str]:
    # veryfast and two threads per drone: the machine is shared, and the sim runs beside it
    return ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', f'{width}x{height}', '-r', f'{fps:g}', '-i', '-',
            '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20', '-threads', '2',
            '-pix_fmt', 'yuv420p', str(out)]


class DroneVideo:
    """One drone's debug video: a queue fed by the ROS callback, an ffmpeg fed by a thread."""

    def __init__(self, drone_id: int, folder: Path, fps: float = FPS, stem: str | None = None,
                 label: str | None = None, extra_meta: dict | None = None):
        """stem: the file names (default debug_d<ID>); label: in messages (default Drone ID);
        extra_meta: more fields for the .json (camera_video.py: the camera's field of view)."""
        self.id = drone_id
        stem = stem or f'debug_d{drone_id}'
        self.label = label or f'Drone {drone_id}'
        self.extra_meta = extra_meta or {}
        self.video = folder / f'{stem}.mkv'
        self.meta = folder / f'{stem}.json'
        self.fps = fps
        self.clock = FrameClock(fps)
        self.size: tuple[int, int] | None = None
        self.images = self.dropped = self.wrong = 0
        self._queue: queue.Queue = queue.Queue(QUEUE_IMAGES)
        self._ffmpeg = None
        self._held: bytes | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def offer(self, sim: float, width: int, height: int, rgb: bytes) -> None:
        """From the ROS callback: never blocks, so a slow encoder cannot stall the others."""
        try:
            self._queue.put_nowait((sim, width, height, rgb))
        except queue.Full:
            self.dropped += 1

    def _write(self, frames: int) -> None:
        for _ in range(frames):
            self._ffmpeg.stdin.write(self._held)

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                break
            sim, width, height, rgb = item
            if self.size is None:
                self.size = (width, height)
                self._ffmpeg = subprocess.Popen(encoder_command(width, height, self.fps, self.video),
                                                stdin=subprocess.PIPE)
            elif (width, height) != self.size:
                self.wrong += 1
                continue
            n = self.clock.add(sim)
            if n is None:
                continue
            if self._held is not None:
                self._write(n)  # 0 when two images fall inside one frame: the newer one wins
            else:
                self._write_meta()
                print(f'{self.label}: first image at sim time {sim:.2f} s, '
                      f'{width}x{height}, recording to {self.video}', flush=True)
            self._held = rgb
            self.images += 1

    def _write_meta(self) -> None:
        self.meta.write_text(json.dumps({
            'drone': self.id, 'start': self.clock.start, 'fps': self.fps,
            'end': (None if self.clock.start is None
                    else self.clock.start + self.clock.written / self.fps),
            'images': self.images, 'dropped': self.dropped,
            'width': self.size[0] if self.size else None,
            'height': self.size[1] if self.size else None, **self.extra_meta}, indent=1) + '\n')

    def close(self) -> str:
        self._queue.put(None)
        self._thread.join()
        if self._ffmpeg is None:
            return f'{self.label}: no images (was it running?)'
        self._write(self.clock.finish())
        self._ffmpeg.stdin.close()
        self._ffmpeg.wait()
        self._write_meta()
        seconds = self.clock.written / self.fps
        note = f', {self.dropped} dropped (encoder too slow)' if self.dropped else ''
        note += f', {self.wrong} of another size skipped' if self.wrong else ''
        return (f'{self.label}: {self.images} images, sim time {self.clock.start:.1f} to '
                f'{self.clock.start + seconds:.1f} s ({seconds:.0f} s) in {self.video}{note}')


def run_folder(arg: str) -> Path:
    if arg != 'auto':
        return Path(arg)
    runs = sorted(p for p in RECORDINGS.glob('*') if p.is_dir()) if RECORDINGS.is_dir() else []
    if not runs:
        raise SystemExit(f'No run folder in {RECORDINGS}. Start the sim with '
                         'docker/start-stellar.sh --record first, or give --out DIR.')
    return runs[-1]


def debug_topic(drone_id: int) -> str:
    """As detect/live.py's offset_topic(id, 'debug'), without importing the detector."""
    from swarm_backend.config import default_namespace
    ns = default_namespace(drone_id)
    return f'/{ns}/detection/debug' if ns else '/detection/debug'


def record(args) -> int:
    folder = run_folder(args.out)
    folder.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node('debug_video_recorder')
    # Reliable like the detector's publisher (big images over best effort lose fragments);
    # depth 1 on its side, so it never waits for us.
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    videos = {}
    for drone_id in args.ids:
        video = videos[drone_id] = DroneVideo(drone_id, folder, args.fps)

        def on_image(msg, video=video):
            rgb = image_rgb(msg)
            if rgb is None:
                video.wrong += 1
                return
            video.offer(stamp_seconds(msg.header.stamp), msg.width, msg.height, rgb)

        node.create_subscription(Image, debug_topic(drone_id), on_image, qos)
    print(f'Recording the debug images of drones {", ".join(map(str, args.ids))} '
          f'({", ".join(debug_topic(i) for i in args.ids)}) into {folder}. '
          'Waiting for the detectors; Ctrl-C stops it.', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C (Jazzy: the latter)
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for video in videos.values():
            print(video.close(), flush=True)
    return 0


def cut_command(src: Path, out: Path, rec_start: float, start: float, duration: float | None,
                fps: float = FPS) -> list[str]:
    """ffmpeg: drone's video from sim time `start` on. Before the recording began, the first
    image is held (so the clip still starts at `start` and stays in sync)."""
    offset = start - rec_start
    filters = [f'fps={fps:g}']
    if offset < 0:
        filters.append(f'tpad=start_duration={-offset:.3f}:start_mode=clone')
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-i', str(src)]
    cmd += ['-vf', ','.join(filters)]
    if offset > 0:
        cmd += ['-ss', f'{offset:.3f}']  # after -i: frame exact (decodes from the start)
    if duration is not None:
        cmd += ['-t', f'{duration:.3f}']
    return cmd + ['-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p',
                  '-an', str(out)]


def cut(args) -> int:
    folder = args.run
    src = folder / f'debug_d{args.id}.mkv'
    meta_path = folder / f'debug_d{args.id}.json'
    if not src.exists() or not meta_path.exists():
        have = sorted(p.name for p in folder.glob('debug_d*.mkv'))
        raise SystemExit(f'No debug recording of drone {args.id} in {folder} '
                         f'(there are: {", ".join(have) or "none"}). Record one with '
                         '`mount-backend.sh debugrec` during the run.')
    meta = json.loads(meta_path.read_text())
    end = meta.get('end')
    if end is not None and args.start >= end:
        raise SystemExit(f'Drone {args.id}\'s debug video ends at sim time {end:.1f} s, '
                         f'before --start {args.start:g}.')
    out = args.out or folder / f'debug_d{args.id}_{args.start:g}.mp4'
    subprocess.run(cut_command(src, out, meta['start'], args.start, args.duration, meta['fps']),
                   check=True)
    print(f'Wrote {out} (drone {args.id}\'s debug images from sim time {args.start:g} s)')
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record', help='during the run: save the debug images as video')
    r.add_argument('ids', type=int, nargs='*', default=[1, 2, 3],
                   help='drones whose detectors to record (default 1 2 3)')
    r.add_argument('--out', default='auto',
                   help='run folder (default auto: the newest data/recordings/<run>/)')
    r.add_argument('--fps', type=float, default=FPS)
    c = sub.add_parser('cut', help='afterwards: one drone\'s video from sim time --start')
    c.add_argument('run', type=Path, help='the run folder')
    c.add_argument('--id', type=int, required=True)
    c.add_argument('--start', type=float, required=True, help='sim time of the first frame')
    c.add_argument('--duration', type=float, default=None)
    c.add_argument('--out', type=Path, default=None,
                   help='mp4 (default: debug_d<ID>_<start>.mp4 in the run folder)')
    args = parser.parse_args(argv)
    return record(args) if args.cmd == 'record' else cut(args)


if __name__ == '__main__':
    sys.exit(main())
