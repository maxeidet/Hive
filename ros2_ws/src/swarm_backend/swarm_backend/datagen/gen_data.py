"""Labelled training images for YOLO from Gazebo (`mount-backend.sh gendata`).

    ./mount-backend.sh gendata --run r01 --count 500        one run of 500 images
    ./mount-backend.sh gendata --run r02 --count 500 --seed 2
    ./mount-backend.sh gendata --yaml --val r05 --test r06   data.yaml over all runs

The placed-stills generator of "Training data and YOLO training" in the vault. It needs a
running world (hive_base: trees and a forest behind the targets) and nothing else; no PX4
drone flies. Per run it spawns a bare camera with the x500_gimbal camera's settings
(1280x720, 2.0 rad wide; the same sensors as box_test.py) and a few static, labelled x500s
(and birds with --birds). Then, for every image:
  - the camera goes to a random spot, 20-120 m up, facing a random way, tilted between
    looking a little down (trees and ground behind) and up (sky);
  - the sun gets a random direction from high (day) to just above the horizon (dusk),
    dimmer and warmer the lower it is;
  - 0-3 drones are put at random places in the image (picked from --variants drones of
    random kinds and colours spawned for the run, see target_models.py), each at the
    distance that makes it
    the drawn number of pixels across (2-100 px, mostly around 4 and 20 px). About
    --empty of the images have nothing in view (empty sky, trees: negatives);
  - an unlabelled x500 is put around the camera where the camera drone's own body would
    be (gimbal turned a little from the nose, body tilted as in flight), so its legs and
    propellers show where they do in flight and the model learns they are not targets
    (--no-own-body leaves it out);
  - after the move, the image and Gazebo's visible boxes are saved.

Output in --out/<run>/ (Ultralytics finds labels/ next to images/):
  images/<run>_00000.png
  labels/<run>_00000.txt   YOLO: class x_centre y_centre width height (fractions);
                           0 drone, 1 bird; empty file = nothing to find
  meta/<run>_00000.json    camera pose, sun, and per object its distance, visible and
                           full box and visibility (visible area / full area)
  objects.csv              one row per object, also the ignored ones (under --min-px)
Train / val / test are split by run, never by image: `--yaml` writes data.yaml with the
runs given to --val and --test and every other run for training.
"""
import argparse
import csv
import json
import math
import random
import sys
import time
from pathlib import Path

from swarm_backend.datagen.box_test import (BIRD, DRONE, F, HEIGHT_PX, HFOV, WIDTH, Streams,
                                    camera_sdf, drone_sdf, rgb_array, unique_name)

DEFAULT_OUT = Path('/workspace/data/gen')  # /workspace is the repo on the host
CLASSES = {DRONE: 0, BIRD: 1}  # Gazebo label -> YOLO class
NAMES = {0: 'drone', 1: 'bird'}
TOPIC = 'gen_data'
SUN = 'sun'  # the directional light's name in hive_base.sdf
TARGET_WIDTH = {DRONE: 0.6, BIRD: 0.45}  # metres across, to turn pixel size into distance
AREA = 100.0  # camera within this many metres of the world origin, north and east
MIN_CLEARANCE = 2.0  # metres: targets stay at least this far above the ground
PARK = (0.0, 0.0, -100.0)  # where unused models wait: Gazebo ENU, under the ground
# Where the gimbal camera sits on the x500, in the drone's own frame (x forward, y left,
# z up; metres from the model's origin at its feet): PX4's x500_gimbal mounts the gimbal
# 0.26 m up, turned half a turn, and the camera is 0.0412 m behind and 0.162 m below the
# gimbal's origin, so 0.04 m ahead of and 0.10 m above the feet. The gimbal's yaw and pitch
# joints turn the camera about (nearly) that point, so it stays there.
CAMERA_ON_BODY = (0.0412, 0.0, 0.098)


# ----- random choices (pure, seeded) -----

def pixel_size(rng: random.Random) -> float:
    """Width in the image (px) of a target: 2-100 px, weighted towards 4 px (where the
    report's detector must find it) and 20 px (where it must tell drone from bird)."""
    pick = rng.random()
    if pick < 0.35:
        return min(max(rng.gauss(4.0, 1.2), 2.0), 8.0)
    if pick < 0.65:
        return min(max(rng.gauss(20.0, 5.0), 10.0), 35.0)
    return math.exp(rng.uniform(math.log(2.0), math.log(100.0)))


def camera_pose(rng: random.Random) -> dict:
    """Where the camera is (north, east, up; metres) and where it looks: heading (radians
    from north, clockwise) and pitch (radians, up positive)."""
    return {'north': rng.uniform(-AREA, AREA), 'east': rng.uniform(-AREA, AREA),
            'up': rng.uniform(20.0, 120.0), 'heading': rng.uniform(0.0, 2 * math.pi),
            'pitch': math.radians(rng.uniform(-15.0, 35.0))}


def sun(rng: random.Random) -> dict:
    """A sun from high (day) to just above the horizon (dusk): elevation and azimuth
    (degrees), the direction its light travels (Gazebo ENU) and its colour."""
    elevation = rng.uniform(2.0, 70.0)
    azimuth = rng.uniform(0.0, 360.0)
    e, a = math.radians(elevation), math.radians(azimuth)
    towards_sun = (math.cos(e) * math.sin(a), math.cos(e) * math.cos(a), math.sin(e))  # ENU
    strength = min(max(math.sin(e) / math.sin(math.radians(30)), 0.25), 1.0)
    warm = min(max((elevation - 2.0) / 25.0, 0.0), 1.0)  # 0 at dusk, 1 by 27 degrees
    colour = (strength, strength * (0.75 + 0.25 * warm), strength * (0.55 + 0.45 * warm))
    return {'elevation_deg': round(elevation, 1), 'azimuth_deg': round(azimuth, 1),
            'direction': tuple(round(-c, 4) for c in towards_sun),
            'diffuse': tuple(round(c, 3) for c in colour)}


def own_body(rng: random.Random, camera: dict) -> dict:
    """The camera drone's own body for a camera pose: the gimbal turned up to about 60
    degrees from the nose (mostly near it), the body pitched as in flight (nose down when
    flying forward) and rolled a little. Its parts (legs, propellers) then show in the
    image where they would in flight, as background the model must learn to ignore."""
    yaw_offset = math.radians(min(max(rng.gauss(0.0, 20.0), -60.0), 60.0))
    return {'heading': (camera['heading'] - yaw_offset) % (2 * math.pi),
            'pitch': math.radians(rng.uniform(-15.0, 8.0)),
            'roll': math.radians(rng.uniform(-8.0, 8.0))}


def body_pose(camera: dict, body: dict) -> tuple[tuple, tuple]:
    """Gazebo position (ENU) and orientation of the own body so that its camera mount
    (CAMERA_ON_BODY) is where the camera is."""
    from swarm_backend.sim.ground_truth import rotate
    q = quaternion(body['heading'], body['pitch'], body['roll'])
    offset = rotate(q, CAMERA_ON_BODY)
    cam = enu((camera['north'], camera['east'], -camera['up']))
    return tuple(c - o for c, o in zip(cam, offset)), q


# ----- geometry (pure) -----

def axes(heading: float, pitch: float):
    """The camera's forward, right and down directions in world NED."""
    ch, sh, cp, sp = math.cos(heading), math.sin(heading), math.cos(pitch), math.sin(pitch)
    forward = (cp * ch, cp * sh, -sp)
    right = (-sh, ch, 0.0)
    down = (sp * ch, sp * sh, cp)  # forward x right
    return forward, right, down


def camera_axes(camera: dict):
    """forward, right, down of a camera dict: given directly ('axes', e.g. from Gazebo's
    poses of a real gimbal camera) or from its heading and pitch."""
    return camera['axes'] if 'axes' in camera else axes(camera['heading'], camera['pitch'])


def quaternion(heading: float, pitch: float = 0.0, roll: float = 0.0):
    """Gazebo (x, y, z, w) of a model whose +x points along heading (from north, clockwise)
    and pitch (up positive), rolled about that axis (right side down positive)."""
    yaw, gz_pitch, gz_roll = math.pi / 2 - heading, -pitch, roll  # ENU, y to the left
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(gz_pitch / 2), math.sin(gz_pitch / 2)
    cr, sr = math.cos(gz_roll / 2), math.sin(gz_roll / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def enu(ned: tuple) -> tuple:
    return (ned[1], ned[0], -ned[2])


def place(camera: dict, u: float, v: float, distance: float) -> tuple:
    """The world point (north, east, up) `distance` metres from the camera along the ray
    through pixel (u, v)."""
    forward, right, down = camera_axes(camera)
    x, y = (u - WIDTH / 2) / F, (v - HEIGHT_PX / 2) / F
    ray = tuple(f + x * r + y * d for f, r, d in zip(forward, right, down))
    norm = math.sqrt(sum(c * c for c in ray))
    north, east, down_ = (distance * c / norm for c in ray)
    return (camera['north'] + north, camera['east'] + east, camera['up'] - down_)


def project_back(camera: dict, at: tuple) -> tuple[float, float]:
    """Pixel (u, v) of a world point (north, east, up) for the camera; used to match a
    Gazebo box to the model placed there. Points behind the camera give the centre."""
    forward, right, down = camera_axes(camera)
    rel = (at[0] - camera['north'], at[1] - camera['east'], camera['up'] - at[2])
    depth = sum(a * b for a, b in zip(rel, forward))
    if depth <= 0:
        return (WIDTH / 2, HEIGHT_PX / 2)
    return (WIDTH / 2 + F * sum(a * b for a, b in zip(rel, right)) / depth,
            HEIGHT_PX / 2 + F * sum(a * b for a, b in zip(rel, down)) / depth)


def targets(rng: random.Random, camera: dict, kinds: list[int], tries: int = 20,
            widths: list[float] | None = None):
    """For each label in `kinds`: (label, pixel size drawn, position, heading, pitch, roll),
    placed somewhere in the image at the distance that gives that size (for an object
    `widths[i]` metres across, default TARGET_WIDTH of its label), above the ground.
    Targets that find no place in `tries` draws come back as None, so the list stays in
    step with `kinds`."""
    out = []
    for i, kind in enumerate(kinds):
        width = widths[i] if widths else TARGET_WIDTH[kind]
        found = None
        for _ in range(tries):
            size = pixel_size(rng)
            distance = F * width / size
            margin = size / 2 + 2
            u, v = rng.uniform(margin, WIDTH - margin), rng.uniform(margin, HEIGHT_PX - margin)
            at = place(camera, u, v, distance)
            if at[2] >= MIN_CLEARANCE and distance > 1.5:
                tilt = 15.0 if kind == DRONE else 25.0  # degrees: drones fly, birds bank
                found = (kind, size, at, rng.uniform(0, 2 * math.pi),
                         math.radians(rng.uniform(-tilt, tilt)),
                         math.radians(rng.uniform(-tilt, tilt)))
                break
        out.append(found)
    return out


class TargetPool:
    """The models a run places in front of the camera: `variants` labelled drones of
    random kinds and colours (target_models.py) and `birds` labelled birds of random kinds
    from `bird_kinds` (birds.py), spawned once out of sight; per image a few of them are
    moved into view and the rest parked."""

    def __init__(self, gazebo, rng: random.Random, variants: int, kinds: list[str],
                 recolour: float, birds: int, bird_kinds: list[str]):
        from swarm_backend.datagen import target_models
        self.gazebo = gazebo
        self.names = target_names('gen_drone', variants)
        self.variants = [target_models.pick(rng, kinds, recolour) for _ in range(variants)]
        self.birds = [f'gen_bird_{i}' for i in range(birds)]
        from swarm_backend.sim.birds import KINDS as BIRD_KINDS
        self.bird_kinds = [rng.choice(bird_kinds) for _ in range(birds)]
        self._bird_spans = [BIRD_KINDS[k].span for k in self.bird_kinds]
        self._library = target_models.Library()

    def spawn(self) -> bool:
        from swarm_backend.sim.birds import KINDS, bird_sdf
        for name in self.names + self.birds:
            self.gazebo.remove(name)  # left over from an earlier run
        ok = all(self.gazebo.spawn_sdf(n, self._library.sdf(n, v, DRONE), 0.0, 0.0, PARK[2])
                 for n, v in zip(self.names, self.variants))
        return ok and all(self.gazebo.spawn_sdf(n, bird_sdf(n, KINDS[k], BIRD), 0.0, 0.0, PARK[2])
                          for n, k in zip(self.birds, self.bird_kinds))

    def place(self, rng: random.Random, camera: dict, drones: int, birds: int) -> list[dict]:
        """Pick `drones` drone variants and `birds` birds and put them at random places in
        the camera's image: dicts with model, label, variant, size (drawn px), at (north,
        east, up), heading, pitch, roll. Moves them in Gazebo and parks the others."""
        chosen = [(n, DRONE, v.name, v.width) for n, v in rng.sample(
            list(zip(self.names, self.variants)), min(drones, len(self.names)))]
        chosen += [(n, BIRD, k, span) for n, k, span in rng.sample(
            list(zip(self.birds, self.bird_kinds, self._bird_spans)), min(birds, len(self.birds)))]
        spots = targets(rng, camera, [c[1] for c in chosen], widths=[c[3] for c in chosen])
        placed = [{'model': n, 'label': label, 'variant': variant,
                   'size': spot[1], 'at': spot[2], 'heading': spot[3], 'pitch': spot[4],
                   'roll': spot[5]}
                  for (n, label, variant, _), spot in zip(chosen, spots) if spot is not None]
        self.ok = True
        for p in placed:
            self.ok &= self.gazebo.set_pose(p['model'], enu((p['at'][0], p['at'][1], -p['at'][2])),
                                            quaternion(p['heading'], p['pitch'], p['roll']))
        used = {p['model'] for p in placed}
        for name in self.names + self.birds:
            if name not in used:
                self.ok &= self.gazebo.set_pose(name, PARK, (0.0, 0.0, 0.0, 1.0))
        return placed

    def remove(self) -> None:
        for name in self.names + self.birds:
            self.gazebo.remove(name)


# ----- labels (pure) -----

def yolo_line(box: dict) -> str:
    """One YOLO label line from a visible box (box_test.box_dict) with a Gazebo label."""
    return (f"{CLASSES[box['label']]} {box['u'] / WIDTH:.6f} {box['v'] / HEIGHT_PX:.6f} "
            f"{box['width'] / WIDTH:.6f} {box['height'] / HEIGHT_PX:.6f}")


def objects(visible: list[dict], full: list[dict], min_px: float):
    """(label lines, rows for objects.csv) from the boxes of one image. Visible and full
    boxes are paired by label and nearest centre. Objects whose visible box is under
    `min_px` (or that are fully hidden) get no label line and are marked ignored."""
    lines, rows, unused = [], [], list(visible)
    for f in full:
        same = [b for b in unused if b['label'] == f['label']]
        vis = min(same, key=lambda b: math.dist((b['u'], b['v']), (f['u'], f['v'])),
                  default=None)
        if vis is not None:
            unused.remove(vis)
        area = f['width'] * f['height']
        visibility = (vis['width'] * vis['height'] / area) if vis and area > 0 else 0.0
        size = max(vis['width'], vis['height']) if vis else 0.0
        keep = vis is not None and size >= min_px
        if keep:
            lines.append(yolo_line(vis))
        rows.append({'class': NAMES[CLASSES[f['label']]], 'visible_px': round(size, 1),
                     'full_px': round(max(f['width'], f['height']), 1),
                     'visibility': round(min(visibility, 1.0), 3), 'labelled': keep,
                     'visible': vis, 'full': f})
    return lines, rows


def match(rows: list[dict], camera: dict, placed: list[tuple]) -> None:
    """Adds to each row the distance and drawn size of the placed model (label, drawn px,
    position) of its class nearest its full box, for P(detect) vs range later."""
    cam = (camera['north'], camera['east'], camera['up'])
    for row in rows:
        same = [p for p in placed if NAMES[CLASSES[p[0]]] == row['class']]
        box = (row['full']['u'], row['full']['v'])
        near = min(same, key=lambda p: math.dist(project_back(camera, p[2]), box), default=None)
        row['distance'] = round(math.dist(cam, near[2]), 2) if near else None
        row['drawn_px'] = round(near[1], 2) if near else None


def size_histogram(sizes: list[float]) -> list[tuple[str, int]]:
    """Counts per pixel-size bin, as the report asks for (recall vs pixel size)."""
    edges = [2, 4, 8, 16, 32, 64, 128]
    bins = [(f'{a}-{b} px', sum(a <= s < b for s in sizes)) for a, b in zip(edges, edges[1:])]
    return [('<2 px', sum(s < 2 for s in sizes))] + bins + [('128+ px', sum(s >= 128 for s in sizes))]


def data_yaml(root: Path, runs: list[str], val: list[str], test: list[str]) -> str:
    """Ultralytics data.yaml: every run not in val or test is for training."""
    def listing(names):
        return '\n'.join(f'  - {name}/images' for name in names) or '  []'
    train = [r for r in runs if r not in val and r not in test]
    return (f'# Written by gen_data.py. Split by run.\npath: {root}\n'
            f'train:\n{listing(train)}\nval:\n{listing(val)}\ntest:\n{listing(test)}\n'
            'names:\n  0: drone\n  1: bird\n')


def target_names(base: str, count: int) -> list[str]:
    """Model names for the placed drones, ending in a letter, not a number: a model with
    sensors gets Gazebo topics, and then a name like gen_drone_0 would be taken for PX4
    drone 0 by sensor_view / save_images (they match <model>_<id>). The look-only copies of
    target_models.py have no sensors, but the names stay safe either way."""
    return [f'{base}_{chr(ord("a") + i)}' for i in range(count)]


def add_variety_options(parser: argparse.ArgumentParser) -> None:
    """The options for which drones are placed (gen_data.py and gen_live.py)."""
    from swarm_backend.datagen import target_models
    parser.add_argument('--kinds', nargs='+', choices=list(target_models.KINDS),
                        default=list(target_models.KINDS),
                        help='drone models to place (default: all of them)')
    parser.add_argument('--variants', type=int, default=8,
                        help='different drones (kind + colour) spawned for the run '
                             '(default %(default)s)')
    parser.add_argument('--recolour', type=float, default=0.5,
                        help='share of them repainted in a plain colour (default %(default)s)')


def add_bird_options(parser: argparse.ArgumentParser) -> None:
    from swarm_backend.sim import birds
    parser.add_argument('--bird-kinds', nargs='+', choices=list(birds.KINDS),
                        default=birds.FLYING,
                        help='bird models to place (default: the ones in a flying pose)')


# ----- writing a run -----

def prepare_run(out: Path) -> bool:
    """Make out/images, labels and meta; False if the run already has images."""
    if (out / 'images').exists() and any((out / 'images').iterdir()):
        print(f'{out} already has images: pick another --run name or move it away.')
        return False
    for sub in ('images', 'labels', 'meta'):
        (out / sub).mkdir(parents=True, exist_ok=True)
    return True


class RunWriter:
    """Writes one run: images/, labels/ (YOLO), meta/ (JSON) and objects.csv, and prints
    progress and the pixel-size histogram (used by gen_data.py and gen_live.py)."""

    FIELDS = ['image', 'class', 'visible_px', 'full_px', 'visibility', 'labelled',
              'distance', 'drawn_px']

    def __init__(self, out: Path, count: int, min_px: float):
        self.out, self.count, self.min_px = out, count, min_px
        self._csv = open(out / 'objects.csv', 'a', newline='')
        self._writer = csv.DictWriter(self._csv, self.FIELDS)
        if self._csv.tell() == 0:
            self._writer.writeheader()
        self.sizes: list[float] = []
        self.saved = 0
        self._started = time.monotonic()

    def save(self, name: str, image, visible: list[dict], full: list[dict], camera: dict,
             placed: list[tuple], meta: dict) -> None:
        """One image (a gz Image message) with its boxes (box_test.box_dict) and what was
        placed where ((label, drawn px, position) per model), plus `meta` for its JSON."""
        import cv2
        lines, rows = objects(visible, full, self.min_px)
        cv2.imwrite(str(self.out / 'images' / f'{name}.png'),
                    cv2.cvtColor(rgb_array(image), cv2.COLOR_RGB2BGR))
        (self.out / 'labels' / f'{name}.txt').write_text(''.join(line + '\n' for line in lines))
        match(rows, camera, placed)
        (self.out / 'meta' / f'{name}.json').write_text(json.dumps(
            {'image': f'{name}.png', **meta, 'objects': rows}, indent=1))
        for row in rows:
            self._writer.writerow({'image': name, **{k: row[k] for k in self.FIELDS
                                                     if k != 'image'}})
            if row['labelled']:
                self.sizes.append(row['visible_px'])
        self.saved += 1
        if self.saved % 10 == 0 or self.saved == self.count:
            rate = self.saved / (time.monotonic() - self._started)
            print(f'{self.saved}/{self.count} images ({rate:.2f}/s), '
                  f'{len(self.sizes)} objects labelled', flush=True)

    def close(self) -> None:
        self._csv.close()
        print(f'\n{self.saved} images in {self.out}. Labelled objects by visible size:')
        for label, count in size_histogram(self.sizes):
            print(f'  {label:>9}: {count}')


# ----- the run (needs Gazebo) -----

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--run', help='name of this run (folder and file names), e.g. r01')
    parser.add_argument('--count', type=int, default=100, help='images (default %(default)s)')
    parser.add_argument('--seed', type=int, help='random seed (default: from the run name)')
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--drones', type=int, default=3, help='most drones in one image')
    add_variety_options(parser)
    parser.add_argument('--birds', type=int, default=2,
                        help='most birds in one image (0-this many; default %(default)s)')
    add_bird_options(parser)
    parser.add_argument('--no-own-body', action='store_true',
                        help='leave out the camera drone\'s own body (legs, propellers)')
    parser.add_argument('--empty', type=float, default=0.15,
                        help='share of images with nothing in view (default %(default)s)')
    parser.add_argument('--min-px', type=float, default=2.0,
                        help='objects smaller than this get no label (default %(default)s)')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='wall-clock seconds to wait for an image (default %(default)s)')
    parser.add_argument('--yaml', action='store_true',
                        help='only write --out/data.yaml from the runs there')
    parser.add_argument('--val', nargs='*', default=[], help='runs for validation (--yaml)')
    parser.add_argument('--test', nargs='*', default=[], help='runs for testing (--yaml)')
    args = parser.parse_args(argv)

    if args.yaml:
        runs = sorted(p.name for p in args.out.iterdir() if (p / 'images').is_dir())
        missing = [r for r in args.val + args.test if r not in runs]
        if missing:
            print(f'No such runs in {args.out}: {", ".join(missing)}')
            return 1
        (args.out / 'data.yaml').write_text(data_yaml(args.out, runs, args.val, args.test))
        print(f'Wrote {args.out / "data.yaml"}: {len(runs) - len(args.val) - len(args.test)} '
              f'training runs, val {args.val}, test {args.test}')
        return 0
    if not args.run:
        parser.error('--run is needed (or --yaml)')
    return generate(args)


def generate(args) -> int:
    from swarm_backend.sim.birds import Gazebo

    rng = random.Random(args.seed if args.seed is not None else args.run)
    out = args.out / args.run
    if not prepare_run(out):
        return 1

    gazebo = Gazebo()
    camera_name = unique_name('gen_camera')
    self_name = 'gen_own_body'  # unlabelled: never boxed, only seen
    pool = TargetPool(gazebo, rng, max(args.variants, args.drones), args.kinds, args.recolour,
                      args.birds, args.bird_kinds)
    gazebo.remove(self_name)
    streams = Streams(TOPIC)
    spawned = [gazebo.spawn_sdf(camera_name, camera_sdf(camera_name, TOPIC), 0.0, 0.0, 50.0)]
    spawned.append(pool.spawn())
    if not args.no_own_body:
        spawned.append(gazebo.spawn_sdf(self_name, drone_sdf(self_name, label=None),
                                        0.0, 0.0, PARK[2]))
    if not all(spawned):
        print('Could not spawn the camera or the models (see box_test for the same setup).')
        return 1
    print('Drones this run: ' + ', '.join(v.name for v in pool.variants))

    deadline = time.monotonic() + args.timeout
    while not streams.images and time.monotonic() < deadline:
        time.sleep(0.1)
    if not streams.images:
        print(f'No image on {TOPIC}/image after {args.timeout:.0f} s. Is Gazebo paused?')
        return 1

    writer = RunWriter(out, args.count, args.min_px)
    try:
        for i in range(args.count):
            name = f'{args.run}_{i:05d}'
            camera = camera_pose(rng)
            light = sun(rng)
            empty = rng.random() < args.empty
            body = None if args.no_own_body else own_body(rng, camera)

            ok = gazebo.set_pose(camera_name, enu((camera['north'], camera['east'], -camera['up'])),
                                 quaternion(camera['heading'], camera['pitch']))
            if body is not None:
                ok &= gazebo.set_pose(self_name, *body_pose(camera, body))
            gazebo.set_sun(SUN, light['direction'], light['diffuse'])
            placed = pool.place(rng, camera, 0 if empty else rng.randint(1, max(1, args.drones)),
                                0 if empty or not args.birds else rng.randint(0, args.birds))
            if not ok:
                print(f'{name}: moving the camera failed, skipped')
                continue
            if not pool.ok:
                # The labels still come from what Gazebo shows, so the image stays usable;
                # only its 'placed' list may be off.
                print(f'{name}: moving a target failed (Gazebo slow?), kept anyway')
            got = streams.after(streams.newest + 0.3, args.timeout)
            if got is None:
                print(f'{name}: no image after the move, skipped')
                continue
            stamp, image, visible, full = got
            writer.save(name, image, visible or [], full or [], camera,
                        [(p['label'], p['size'], p['at']) for p in placed], {
                'time': stamp, 'run': args.run, 'index': i,
                'camera': {**{k: round(v, 4) for k, v in camera.items()},
                           'hfov_rad': HFOV, 'width': WIDTH, 'height': HEIGHT_PX},
                'sun': light, 'empty': empty,
                'own_body': body and {k: round(math.degrees(v), 1) for k, v in body.items()},
                'placed': [{'model': p['model'], 'variant': p['variant'],
                            'drawn_px': round(p['size'], 1)} for p in placed]})
    except KeyboardInterrupt:
        print('Stopped.')
    finally:
        writer.close()
        for model in [camera_name, self_name]:
            gazebo.remove(model)
        pool.remove()
    return 0


if __name__ == '__main__':
    sys.exit(main())
