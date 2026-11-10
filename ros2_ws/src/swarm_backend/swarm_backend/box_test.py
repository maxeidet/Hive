"""Try Gazebo's bounding-box camera for training labels (`mount-backend.sh boxtest`).

A small test before writing the training-data generator ("Training data and YOLO training"
in the vault): does the bounding-box camera, with the Label system on our models, give boxes
that sit on the drone and the bird in the camera image, down to a few pixels across?

It spawns, in the running world:
  - box_test_camera: a bare camera without PX4, with the gimbal camera's settings (1280x720,
    2.0 rad wide), HEIGHT metres up at the origin, looking north. Three sensors at the same
    spot: an RGB camera, a bounding-box camera with visible boxes (what shows in the pixels)
    and one with full boxes (also the hidden part);
  - box_test_drone: a static x500 (no PX4) with label 1;
  - box_test_bird: a static dove with label 2.
Then it moves the drone and the bird to a few distances ahead of the camera, waits for an
image and both kinds of boxes from after the move, and writes to --out:
  <step>.png         the RGB image
  <step>_boxes.png   the same with the boxes: visible green, full red, blue the box
                     ground_truth.py projects (the fallback labels), and a yellow cross
                     where the pinhole model puts the model's origin
  results.json       every box, with the expected width and centre from the pinhole model
and prints a table. The models are removed at the end (--keep leaves them).

It also prints the clip planes, anti-aliasing and noise of PX4's x500_gimbal camera, which
decide how far away the real camera can see a target and how clean a few pixels look.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

WIDTH, HEIGHT_PX, HFOV = 1280, 720, 2.0  # the x500_gimbal camera
F = (WIDTH / 2) / math.tan(HFOV / 2)  # focal length, px (about 411)
DRONE, BIRD = 1, 2  # labels (class numbers) for the Label system
LABEL_NAMES = {DRONE: 'drone', BIRD: 'bird'}
SIZES = {DRONE: 0.5, BIRD: 0.45}  # metres across, for the expected width
TOPIC = 'box_test'
DEFAULT_OUT = Path('/workspace/data/box_test')  # /workspace is the repo on the host
DISTANCES = (3, 5, 10, 20, 40, 60, 100, 150)
SIDE = 0.15  # drone this far left of straight ahead per metre of distance, bird as far right
RISE = 0.10  # both this far above the camera per metre, so they show against the sky


def camera_sdf(name: str, topic: str = TOPIC, rate: float = 5.0, far: float = 1000.0,
               anti_aliasing: int = 0) -> str:
    """A static model with an RGB camera and two bounding-box cameras at the same spot
    (near clip 0.05 m as on the gimbal camera, so the own drone's parts close by show),
    all looking along the model's +x with the gimbal camera's image size and field of view.
    anti_aliasing: MSAA samples for the RGB camera (0 = not set, Gazebo's default, as in
    PX4's gimbal camera); without it, parts thinner than a pixel can vanish."""
    aa = f'<anti_aliasing>{anti_aliasing}</anti_aliasing>' if anti_aliasing else ''
    def sensor(sensor_name: str, kind: str, sensor_topic: str, extra: str = '') -> str:
        return (f'<sensor name="{sensor_name}" type="{kind}"><always_on>1</always_on>'
                f'<update_rate>{rate}</update_rate><topic>{sensor_topic}</topic>'
                f'<camera><horizontal_fov>{HFOV}</horizontal_fov>'
                f'<image><width>{WIDTH}</width><height>{HEIGHT_PX}</height>'
                f'{aa if kind == "camera" else ""}</image>'
                f'<clip><near>0.05</near><far>{far}</far></clip>{extra}</camera></sensor>')

    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      {sensor('rgb', 'camera', f'{topic}/image')}
      {sensor('visible', 'boundingbox_camera', f'{topic}/visible', '<box_type>2d</box_type>')}
      {sensor('full', 'boundingbox_camera', f'{topic}/full', '<box_type>full_2d</box_type>')}
    </link>
  </model>
</sdf>'''


def drone_sdf(name: str, model: str = 'x500_base', label: int | None = DRONE) -> str:
    """PX4's x500 body as a static model (no PX4, no physics) with a label, or without one
    (label=None: the bounding-box cameras ignore it)."""
    plugin = '' if label is None else (
        '\n    <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
        f'<label>{label}</label></plugin>')
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <include merge="true"><uri>{model}</uri></include>{plugin}
  </model>
</sdf>'''


def steps(height: float) -> list[tuple[str, tuple, tuple]]:
    """(name, drone position, bird position), positions as (north, east, up) in metres."""
    away = (-50.0, 0.0, height)  # behind the camera: nothing in view
    out = [('empty', away, (-50.0, 5.0, height))]
    for d in DISTANCES:
        up = height + RISE * d
        out.append((f'{d:03d}m', (float(d), -SIDE * d, up), (float(d), SIDE * d, up)))
    # The bird 2 m behind the drone and a little to the side: partly hidden, so its visible
    # box must be smaller than its full box.
    up = height + RISE * 10
    out.append(('overlap', (10.0, 0.0, up), (12.0, 0.12, up)))
    return out


def expected(height: float, position: tuple, size: float) -> dict | None:
    """Where the pinhole model puts a model's origin in the image, and how wide it is, for
    the camera at (0, 0, height) looking north and level. None when behind the camera."""
    north, east, up = position
    if north <= 0:
        return None
    return {'u': round(WIDTH / 2 + F * east / north, 1),
            'v': round(HEIGHT_PX / 2 + F * (height - up) / north, 1),
            'width': round(F * size / north, 1)}


def projected(height: float, position: tuple, label: int) -> dict | None:
    """The box ground_truth.py would save as a training label (projected from HALF_SIZE)
    for a model at `position` facing north, seen by the test camera. Compared with the
    full boxes of the bounding-box camera, it shows whether HALF_SIZE is right. It is
    centred on the model's origin, which for the drone may sit below its body."""
    from swarm_backend import ground_truth
    from swarm_backend.birds import yaw_quaternion
    camera = {'position': (0.0, 0.0, -height), 'forward': (1.0, 0.0, 0.0),
              'right': (0.0, 1.0, 0.0), 'down': (0.0, 0.0, 1.0)}
    north, east, up = position
    points = ground_truth.corners((north, east, -up), yaw_quaternion(0.0),
                                  ground_truth.HALF_SIZE[LABEL_NAMES[label]])
    box = ground_truth.project_box(camera, points, HFOV, WIDTH, HEIGHT_PX)
    if box is None:
        return None
    return {**box, 'label': label, 'width': round(box['x1'] - box['x0'], 1),
            'height': round(box['y1'] - box['y0'], 1)}


def box_dict(box) -> dict:
    """An AnnotatedAxisAligned2DBox as plain numbers (px). Gazebo's corners are the first
    and last pixel of the object (both inclusive), so the box's right and bottom edges are
    one pixel further: without the +1 a box is a pixel too narrow and too low, which shows
    as parts of the drone outside its box in enlarged cut-outs (checked 18 Oct on 107
    objects: the last dark pixel is exactly at max_corner)."""
    x0, y0 = box.box.min_corner.x, box.box.min_corner.y
    x1, y1 = box.box.max_corner.x + 1, box.box.max_corner.y + 1
    return {'label': box.label, 'x0': round(x0, 1), 'y0': round(y0, 1),
            'x1': round(x1, 1), 'y1': round(y1, 1),
            'width': round(x1 - x0, 1), 'height': round(y1 - y0, 1),
            'u': round((x0 + x1) / 2, 1), 'v': round((y0 + y1) / 2, 1)}


def camera_settings(sdf_text: str) -> list[dict]:
    """Clip planes, anti-aliasing and noise of every camera sensor in a model's SDF."""
    found = []
    for sensor in ET.fromstring(sdf_text).iter('sensor'):
        if sensor.get('type') not in ('camera', 'rgbd_camera', 'depth_camera'):
            continue
        cam = sensor.find('camera')
        if cam is None:
            continue

        def text(path: str):
            node = cam.find(path)
            return None if node is None else node.text.strip()
        found.append({'sensor': sensor.get('name'), 'hfov': text('horizontal_fov'),
                      'width': text('image/width'), 'height': text('image/height'),
                      'clip_near': text('clip/near'), 'clip_far': text('clip/far'),
                      'anti_aliasing': text('image/anti_aliasing'),
                      'noise': text('noise/type'), 'noise_stddev': text('noise/stddev')})
    return found


def find_model_sdf(model: str) -> Path | None:
    """`model`/model.sdf on Gazebo's resource path, as Gazebo would find it."""
    for root in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':'):
        path = Path(root) / model / 'model.sdf'
        if root and path.exists():
            return path
    return None


def included_models(sdf_text: str) -> list[str]:
    """Names of the models an SDF includes (<include><uri>model://NAME</uri>)."""
    return [uri.text.strip().removeprefix('model://') for uri in
            ET.fromstring(sdf_text).iter('uri') if uri.text and '/' not in
            uri.text.strip().removeprefix('model://')]


def print_gimbal_camera(model: str = 'x500_gimbal') -> None:
    path = find_model_sdf(model)
    if path is None:
        print(f'{model}/model.sdf is not on GZ_SIM_RESOURCE_PATH; camera settings not checked.')
        return
    # PX4's x500_gimbal includes the camera from its `gimbal` model.
    paths = [path] + [p for p in map(find_model_sdf, included_models(path.read_text())) if p]
    for cam, path in ((c, p) for p in paths for c in camera_settings(p.read_text())):
        print(f'{model} camera "{cam["sensor"]}" ({path}):')
        print(f'  clip near/far: {cam["clip_near"]} / {cam["clip_far"]} m '
              '(no far = Gazebo default)')
        print(f'  anti-aliasing: {cam["anti_aliasing"] or "not set (Gazebo default)"}')
        print(f'  noise: {cam["noise"] or "none"}'
              + (f', stddev {cam["noise_stddev"]}' if cam['noise_stddev'] else ''))


def unique_name(base: str) -> str:
    """A model name not used before in this Gazebo session. Gazebo (gz-rendering 8) crashes
    when a bounding-box camera is created again under a name it had before, even after the
    old one was removed ("A node definition with name 'AutoGen ...' already exists"), so
    every camera with a bounding-box sensor gets a new name."""
    return f'{base}_{time.time_ns() // 1_000_000 % 100_000_000}'


class Streams:
    """The newest images and boxes from a camera, by simulation time. By default the test
    camera's topics (camera_sdf: <topic>/image, /visible, /full); or any image topic with
    the topics of two bounding-box cameras that see the same (visible and full boxes), or
    none (visible=full=None: images only)."""

    def __init__(self, topic: str = TOPIC, image: str | None = None,
                 visible: str | None = '', full: str | None = ''):
        from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
        from gz.msgs10.image_pb2 import Image
        from gz.transport13 import Node

        topics = {'visible': f'{topic}/visible' if visible == '' else visible,
                  'full': f'{topic}/full' if full == '' else full}
        self._lock = threading.Lock()
        self.images: dict[float, object] = {}
        self.boxes: dict[str, dict[float, list[dict]]] = {
            kind: {} for kind, t in topics.items() if t}
        self.newest = 0.0
        self._node = Node()
        image = image or f'{topic}/image'
        ok = self._node.subscribe(Image, image, self._on_image)
        for kind in self.boxes:
            ok &= self._node.subscribe(AnnotatedAxisAligned2DBox_V, topics[kind],
                                       lambda msg, k=kind: self._on_boxes(k, msg))
        if not ok:
            raise RuntimeError(f'Could not subscribe to {image} or the box topics')

    @staticmethod
    def _stamp(msg) -> float:
        return round(msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, 3)

    def _on_image(self, msg) -> None:
        with self._lock:
            stamp = self._stamp(msg)
            self.images[stamp] = msg
            self.newest = max(self.newest, stamp)
            for old in [s for s in self.images if s < stamp - 5]:
                del self.images[old]

    def _on_boxes(self, kind: str, msg) -> None:
        with self._lock:
            stamp = self._stamp(msg)
            self.boxes[kind][stamp] = [box_dict(b) for b in msg.annotated_box]
            for old in [s for s in self.boxes[kind] if s < stamp - 5]:
                del self.boxes[kind][old]

    def _same_time(self, kind: str, stamp: float, tolerance: float = 0.002):
        near = [s for s in self.boxes[kind] if abs(s - stamp) <= tolerance]
        return None if not near else self.boxes[kind][min(near, key=lambda s: abs(s - stamp))]

    def after(self, t: float, timeout: float):
        """The first image taken after simulation time t that has boxes from the same moment
        (the box cameras may run slower than the camera): (stamp, image, visible, full).
        Without box topics, the first image after t with visible = full = None. If no box
        message comes at all for 2 s after an image, that image with None boxes. None
        after `timeout` wall-clock seconds."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                later = sorted(s for s in self.images if s > t)
                if later and not self.boxes:
                    return later[0], self.images[later[0]], None, None
                for s in later:
                    found = {kind: self._same_time(kind, s) for kind in self.boxes}
                    if all(v is not None for v in found.values()):
                        return s, self.images[s], found.get('visible'), found.get('full')
                newest_box = max((max(b, default=0.0) for b in self.boxes.values()), default=0.0)
                if later and newest_box < later[0] and self.newest > later[0] + 2.0:
                    return later[0], self.images[later[0]], None, None
            time.sleep(0.05)
        return None


def rgb_array(msg):
    import numpy as np
    row = msg.width * 3
    data = msg.data if msg.step == row else b''.join(
        msg.data[y * msg.step:y * msg.step + row] for y in range(msg.height))
    return np.frombuffer(data, np.uint8).reshape(msg.height, msg.width, 3)


def draw(rgb, visible: list[dict], full: list[dict], crosses: list[dict],
         projections: list[dict] = ()):
    """A BGR copy with the boxes and the expected centres drawn on."""
    import cv2
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for boxes, colour in ((projections, (255, 128, 0)), (full, (0, 0, 255)),
                          (visible, (0, 255, 0))):
        for b in boxes:
            pad = 3  # draw a little outside, so a 2 px box is still seen
            cv2.rectangle(bgr, (int(b['x0']) - pad, int(b['y0']) - pad),
                          (int(math.ceil(b['x1'])) + pad, int(math.ceil(b['y1'])) + pad), colour, 1)
            cv2.putText(bgr, LABEL_NAMES.get(b['label'], str(b['label'])),
                        (int(b['x0']), max(10, int(b['y0']) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)
    for c in crosses:
        cv2.drawMarker(bgr, (int(c['u']), int(c['v'])), (0, 255, 255), cv2.MARKER_CROSS, 12, 1)
    return bgr


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--height', type=float, default=40.0,
                        help='metres above the ground for the camera (default %(default)s)')
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--drone-model', default='x500_base',
                        help='Gazebo model to include as the drone (default %(default)s)')
    parser.add_argument('--bird', default='seagull', help='bird kind (default %(default)s)')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='wall-clock seconds to wait for each image (default %(default)s)')
    parser.add_argument('--keep', action='store_true', help='leave the models in the world')
    parser.add_argument('--aa', type=int, default=0,
                        help='anti-aliasing samples for the RGB camera, e.g. 4 or 8 (default: '
                             'not set, as on the real gimbal camera)')
    args = parser.parse_args(argv)

    import cv2

    from swarm_backend.birds import KINDS, Gazebo, bird_sdf

    print_gimbal_camera()
    gazebo = Gazebo()
    names = (unique_name('box_test_camera'), 'box_test_drone', 'box_test_bird')
    for name in names[1:]:
        gazebo.remove(name)  # left over from an earlier run
    streams = Streams()
    first = steps(args.height)[0]
    spawned = [gazebo.spawn_sdf(names[0], camera_sdf(names[0], anti_aliasing=args.aa), 0.0, 0.0, args.height),
               gazebo.spawn_sdf(names[1], drone_sdf(names[1], args.drone_model), *first[1]),
               gazebo.spawn_sdf(names[2], bird_sdf(names[2], KINDS[args.bird], BIRD), *first[2])]
    for name, ok in zip(names, spawned):
        print(f'{name}: ' + ('spawned' if ok else 'FAILED to spawn'))
    if not all(spawned):
        return 1

    args.out.mkdir(parents=True, exist_ok=True)
    results = {'camera': {'height': args.height, 'width': WIDTH, 'height_px': HEIGHT_PX,
                          'hfov': HFOV, 'f': round(F, 1)},
               'drone_model': args.drone_model, 'bird': args.bird, 'steps': []}
    print(f'\n{"step":8} {"what":6} {"expected w":>10} {"visible w x h":>14} '
          f'{"full w x h":>12} {"proj w x h":>12} {"centre off":>10}')
    try:
        deadline = time.monotonic() + args.timeout
        while not streams.images and time.monotonic() < deadline:
            time.sleep(0.1)
        if not streams.images:
            print(f'No image on {TOPIC}/image after {args.timeout:.0f} s. Is Gazebo paused, '
                  'or does the world lack the Sensors system?')
            return 1
        for name, drone_at, bird_at in steps(args.height):
            ok = gazebo.move(names[1], *drone_at) and gazebo.move(names[2], *bird_at)
            moved_at = streams.newest + 0.3  # simulation time; later images show the move
            got = streams.after(moved_at, args.timeout) if ok else None
            if got is None:
                print(f'{name:8} no image after the move' if ok else f'{name:8} move FAILED')
                continue
            stamp, image, visible, full = got
            if visible is None or full is None:
                print(f'{name:8} no box message with this image'
                      + (' (none expected)' if name == 'empty' else ''))
            visible, full = visible or [], full or []
            want = {DRONE: expected(args.height, drone_at, SIZES[DRONE]),
                    BIRD: expected(args.height, bird_at, KINDS[args.bird].span)}
            proj = {DRONE: projected(args.height, drone_at, DRONE),
                    BIRD: projected(args.height, bird_at, BIRD)}
            rgb = rgb_array(image)
            cv2.imwrite(str(args.out / f'{name}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(args.out / f'{name}_boxes.png'),
                        draw(rgb, visible, full, [w for w in want.values() if w],
                             [p for p in proj.values() if p]))
            results['steps'].append({'step': name, 'time': stamp, 'drone_at': drone_at,
                                     'bird_at': bird_at, 'expected': want, 'projected': proj,
                                     'visible': visible, 'full': full})
            for label in (DRONE, BIRD):
                w = want[label]
                vis = [b for b in visible if b['label'] == label]
                ful = [b for b in full if b['label'] == label]
                v = f'{vis[0]["width"]:.0f} x {vis[0]["height"]:.0f}' if vis else 'none'
                fu = f'{ful[0]["width"]:.0f} x {ful[0]["height"]:.0f}' if ful else 'none'
                pr = proj[label]
                pj = f'{pr["width"]:.0f} x {pr["height"]:.0f}' if pr else 'none'
                off = (f'{math.hypot(vis[0]["u"] - w["u"], vis[0]["v"] - w["v"]):.1f} px'
                       if vis and w else '')
                print(f'{name:8} {LABEL_NAMES[label]:6} {w["width"] if w else "-":>10} '
                      f'{v:>14} {fu:>12} {pj:>12} {off:>10}')
    finally:
        (args.out / 'results.json').write_text(json.dumps(results, indent=2))
        print(f'\nWrote {len(results["steps"])} steps to {args.out}')
        if not args.keep:
            for name in names:
                gazebo.remove(name)
    return 0


if __name__ == '__main__':
    sys.exit(main())
