"""Save a drone's camera images from Gazebo as PNG files (`mount-backend.sh snap ID`, or
snap(id, count) in the shell).

Reads the camera's Gazebo topic directly (no ROS bridge) and writes every image, or one
every --every seconds, to --out until --count images are saved or Ctrl-C. The file name
carries the simulation time of the image. Next to each PNG it writes a .json with the truth
from Gazebo at that time (ground_truth.py): where the camera drone, its camera and every
other drone really are, and the distance and direction from the camera to each of them;
and under `objects` every other drone and every bird with its class and projected box, as
training labels (yolo_labels.py turns them into YOLO label files).
It also holds `px4_camera`: the camera orientation PX4 reports for that time, and its error
against the truth (null if PX4 had not reported yet).
PNGs are written with the standard library only, so nothing extra is needed in the container.
"""
import argparse
import bisect
import json
import struct
import subprocess
import sys
import threading
import time
import zlib
from pathlib import Path

from swarm_backend.sim import ground_truth
from swarm_backend.detect.orientation import axes_error_deg
from swarm_backend.sim.sensor_view import MODEL_TOPIC, find_sensors

RGB_INT8 = 3  # gz.msgs.PixelFormatType
GIMBAL_HFOV = 2.0  # radians, <horizontal_fov> of the camera in PX4's gimbal model
POSE_HISTORY = 400  # pose messages kept, to find the one from when an image was taken


def png_bytes(rgb: bytes, width: int, height: int) -> bytes:
    """An 8-bit RGB PNG from `height` rows of `width` * 3 bytes."""
    if len(rgb) != width * height * 3:
        raise ValueError(f'Expected {width * height * 3} bytes, got {len(rgb)}')
    row = width * 3
    # Each row starts with its filter type; 0 = the bytes as they are.
    raw = b''.join(b'\x00' + rgb[y * row:(y + 1) * row] for y in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body))

    header = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)  # 8 bit, colour type RGB
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header)
            + chunk(b'IDAT', zlib.compress(raw, 3)) + chunk(b'IEND', b''))


class ImageSaver:

    def __init__(self, gz_topic: str, out: Path, prefix: str, every: float, count: int,
                 truth=None):
        """`truth(stamp, width, height)` returns what to save as .json with the image."""
        from gz.msgs10.image_pb2 import Image
        from gz.transport13 import Node

        self.done = threading.Event()
        self.saved = 0
        self._out = out
        self._prefix = prefix
        self._every = every
        self._count = count
        self._last = None  # simulation time of the last saved image
        self._truth = truth
        self._node = Node()
        if not self._node.subscribe(Image, gz_topic, self._on_image):
            raise RuntimeError(f'Could not subscribe to {gz_topic}')

    def _on_image(self, msg) -> None:
        if self.done.is_set():
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        if self._last is not None and 0 <= stamp - self._last < self._every:
            return
        if msg.pixel_format_type != RGB_INT8:
            print(f'Skipping an image with pixel format {msg.pixel_format_type} (not RGB)')
            return
        # Rows can be padded: keep the first width * 3 bytes of each `step`.
        row = msg.width * 3
        rgb = msg.data if msg.step == row else b''.join(
            msg.data[y * msg.step:y * msg.step + row] for y in range(msg.height))
        path = self._out / f'{self._prefix}_{stamp:010.3f}.png'
        note = ''
        if self._truth is not None:
            data = self._truth(stamp, msg.width, msg.height)
            if data is None:
                return  # no pose from Gazebo for that time (yet): take the next image
            path.with_suffix('.json').write_text(
                json.dumps({'image': path.name, 'time': round(stamp, 3), **data}, indent=2))
            note = '  ' + ', '.join(
                f"{o['name']}: {o['distance']:.1f} m" + ('' if o['box'] else ' (out of view)')
                for o in data['objects'])
        path.write_bytes(png_bytes(rgb, msg.width, msg.height))
        self._last = stamp
        self.saved += 1
        print(f'{self.saved:4d}  {path}  ({msg.width}x{msg.height}){note}')
        if self._count and self.saved >= self._count:
            self.done.set()


class PoseHistory:
    """Gazebo's recent poses of everything that moves (`dynamic_pose/info`), or of every
    model including static ones such as the birds (`pose/info`), to look up by simulation
    time."""

    def __init__(self, world: str, topic: str = 'dynamic_pose/info'):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self._lock = threading.Lock()
        self._stamps: list[float] = []
        self._messages: list = []
        self._node = Node()
        topic = f'/world/{world}/{topic}'
        if not self._node.subscribe(Pose_V, topic, self._on_poses):
            raise RuntimeError(f'Could not subscribe to {topic}')

    def _on_poses(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self._lock:
            if self._stamps and stamp < self._stamps[-1]:
                self._stamps.clear()  # the simulation was reset
                self._messages.clear()
            self._stamps.append(stamp)
            self._messages.append(msg)
            if len(self._stamps) > POSE_HISTORY:
                del self._stamps[0], self._messages[0]

    def at(self, stamp: float, tolerance: float = 0.05):
        """(time, poses) closest to `stamp`, or None if nothing is within `tolerance` s."""
        with self._lock:
            if not self._stamps:
                return None
            i = bisect.bisect_left(self._stamps, stamp)
            best = min((j for j in (i - 1, i) if 0 <= j < len(self._stamps)),
                       key=lambda j: abs(self._stamps[j] - stamp))
            time_, msg = self._stamps[best], self._messages[best]
        if abs(time_ - stamp) > tolerance:
            return None
        return time_, [(p.name, p.id, (p.position.x, p.position.y, p.position.z),
                        (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                       for p in msg.pose]


def drone_names(gz_topics: list[str]) -> tuple[str | None, dict[int, str]]:
    """The Gazebo world and the model name of every PX4 drone in it, by instance id."""
    world, names = None, {}
    for topic in gz_topics:
        match = MODEL_TOPIC.match(topic)
        if match:
            world = match['world']
            names[int(match['id'])] = f"{match['model']}_{match['id']}"
    return world, names


DEFAULT_OUT = Path('/workspace/data/Manualimages')  # /workspace is the repo on the host
STALL_S = 60.0  # give up when no image arrives for this long (wall clock; Gazebo can be slow)


def snap(drone_id: int, count: int = 1, every: float = 1.0, label: str = '',
         out: Path = DEFAULT_OUT, sensor: str = 'camera', node=None) -> int:
    """Save `count` images from the drone's camera (0 = until Ctrl-C), `every` simulation
    seconds apart (0 = every image). Blocks until done; returns how many were saved.

    node: a spinning ROS node to read PX4 with, so the .json also holds the camera
    orientation PX4 reports next to the truth (`px4_camera`). None leaves it out."""
    gz_topics = subprocess.run(
        ['gz', 'topic', '-l'], capture_output=True, text=True, check=True).stdout.split()
    cameras = [s for s in find_sensors(gz_topics, drone_id) if s.kind == 'image']
    camera = next((s for s in cameras if s.name == sensor), None)
    if camera is None:
        names = ', '.join(s.name for s in cameras) or 'none'
        print(f'Drone {drone_id} has no image sensor named {sensor} (it has: {names}). '
              'Start a camera drone with: ./mount-backend.sh spawn gimbal')
        return 0

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    prefix = f'drone{drone_id}' + (f'_{label}' if label else '')
    print(f'Saving {camera.gz_topic}\n  to {out}/{prefix}_<sim time>.png, Ctrl-C to stop')
    world, names = drone_names(gz_topics)
    history = PoseHistory(world)
    statics = PoseHistory(world, 'pose/info')  # birds are static models: only listed here
    px4 = None
    if node is not None:
        from swarm_backend.detect.px4_camera import PX4Camera
        px4 = PX4Camera(node, drone_id)
        deadline = time.monotonic() + 10.0  # let PX4 report first, so the first image has it too
        while (px4.clock.value is None or px4.earth_frame is None) and time.monotonic() < deadline:
            time.sleep(0.1)

    def truth(stamp: float, width: int, height: int) -> dict | None:
        # The image can arrive a moment before the poses of the same instant.
        deadline = time.monotonic() + 1.0
        while (found := history.at(stamp)) is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if found is None:
            return None
        pose_time, poses = found
        # Birds are moved by setting their pose (birds.py), not simulated, so a looser match
        # in time is fine; without one they are just missing from the .json.
        found_statics = statics.at(stamp, tolerance=0.25)
        if found_statics is not None:
            poses = poses + ground_truth.bird_poses(found_statics[1])
        data = ground_truth.truth(poses, names, drone_id, camera.frame, GIMBAL_HFOV, width, height)
        if data is None:
            return None
        if px4 is not None:
            # PX4's report for this moment can arrive a little after the image.
            deadline = time.monotonic() + 1.0
            while (axes := px4.axes(stamp)) is None and time.monotonic() < deadline:
                time.sleep(0.05)
            data['px4_camera'] = px4_camera(axes, data['camera'])
        return {'pose_time': round(pose_time, 3), **data}

    saver = ImageSaver(camera.gz_topic, out, prefix, every, count, truth)
    last_saved, last_change = 0, time.monotonic()
    try:
        while not saver.done.wait(0.2):
            if saver.saved != last_saved:
                last_saved, last_change = saver.saved, time.monotonic()
            elif time.monotonic() - last_change > STALL_S + 20 * every:
                print(f'No image for {STALL_S:.0f} s. Is Gazebo paused?')
                break
    except KeyboardInterrupt:
        pass
    saver.done.set()
    if px4 is not None:
        px4.close()
    print(f'Saved {saver.saved} images.')
    return saver.saved


def px4_camera(axes, true_camera: dict) -> dict | None:
    """What PX4 says about the camera's orientation, and how far that is from the truth."""
    if axes is None:
        return None  # PX4 had not reported yet
    forward_error, roll_error = axes_error_deg(
        axes, (true_camera['forward'], true_camera['right'], true_camera['down']))
    return {'forward': [round(c, 6) for c in axes[0]], 'right': [round(c, 6) for c in axes[1]],
            'down': [round(c, 6) for c in axes[2]],
            'forward_error_deg': round(forward_error, 3), 'roll_error_deg': round(roll_error, 3)}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--id', type=int, default=2, help='PX4 instance number of the drone')
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT, help='folder for the PNGs')
    parser.add_argument('--every', type=float, default=1.0,
                        help='simulation seconds between saved images; 0 saves every image')
    parser.add_argument('--count', type=int, default=0, help='stop after this many; 0 = until Ctrl-C')
    parser.add_argument('--label', default='', help='extra word in the file names, e.g. sky_30m')
    parser.add_argument('--sensor', default='camera', help='Gazebo sensor name')
    parser.add_argument('--no-px4', action='store_true',
                        help="don't save PX4's camera orientation next to the truth")
    args = parser.parse_args(argv)
    if args.no_px4:
        return 0 if snap(args.id, args.count, args.every, args.label, args.out, args.sensor) else 1
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    from swarm_backend.runner import spinning
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl-C stops the saving
    node = rclpy.create_node(f'snap_{args.id}')
    try:
        with spinning(node):
            saved = snap(args.id, args.count, args.every, args.label, args.out, args.sensor, node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if saved else 1


if __name__ == '__main__':
    sys.exit(main())
