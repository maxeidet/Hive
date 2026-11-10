"""Labelled training images taken by a real flying gimbal drone (`mount-backend.sh genlive`).

    ./mount-backend.sh spawn gimbal_labels          the camera drone, once (drone id N)
    ./mount-backend.sh genlive N --run L01 --count 200
    ./mount-backend.sh genlive 2 --run L02 --labels projected     the stock gz_x500_gimbal

The "live" generator of "Training data and YOLO training" in the vault. Where gen_data.py
moves a bare camera, here PX4 flies the camera drone and its own gimbal takes the photos, so
the images are what the drone sees in flight: its own legs and propellers, the body's
tilt, the gimbal's lag and wobble. The drone climbs to each of --heights in turn and hovers
there; for every image it turns to a new heading and the gimbal to a random pitch, and then
static, labelled drones (and birds with --birds) are put at random places in the image,
each at the distance that makes it the drawn size (gen_data.py's choices), around where the
camera is at that moment (Gazebo's true pose). After the move, the image and its labels are
saved. It lands at the end, or on Ctrl-C.

Labels (--labels):
  visible    Gazebo's visible boxes, from the two bounding-box cameras on the gimbal camera
             of the x500_gimbal_labels model (worlds/models/). Same quality as gen_data.py.
  projected  boxes projected from a box around each placed model (ground_truth.py), with
             the camera's true pose at the image's time. Works with the stock
             gz_x500_gimbal; a bit loose, and blind to trees in the way.
Output: the same as gen_data.py, in --out/<run>/ (images/, labels/, meta/, objects.csv);
`gendata --yaml` makes data.yaml over runs of both generators.
"""
import argparse
import math
import random
import subprocess
import sys
import time
from pathlib import Path

from swarm_backend import ground_truth
from swarm_backend.box_test import BIRD, DRONE, HFOV, HEIGHT_PX, WIDTH, Streams
from swarm_backend.gen_data import (DEFAULT_OUT, RunWriter, TargetPool, add_bird_options,
                                    add_variety_options, prepare_run, quaternion)

TOLERANCE_M = 1.0
CLIMB_TIMEOUT_S = 120.0  # simulation seconds
LAND_TIMEOUT_S = 120.0
# Where the box around a placed model is centred, in the model's own frame (x forward,
# y left, z up): PX4's x500 has its origin at its feet and its body about 0.17 m higher
# (box_test, 17 Oct); the bird meshes are centred.
BOX_CENTRE = {DRONE: (0.0, 0.0, 0.17), BIRD: (0.0, 0.0, 0.0)}
CLASS_NAME = {DRONE: 'drone', BIRD: 'bird'}


# ----- pure parts -----

def camera_from_truth(truth_camera: dict) -> dict:
    """gen_data's camera dict (north, east, up, axes) from ground_truth.camera_pose()."""
    north, east, down = truth_camera['position']
    return {'north': north, 'east': east, 'up': -down,
            'axes': (truth_camera['forward'], truth_camera['right'], truth_camera['down'])}


def projected_boxes(truth_camera: dict, placed: list[dict]) -> list[dict]:
    """Boxes in box_test.box_dict's form for placed models (gen_data.TargetPool.place's
    dicts: label, at (north, east, up), heading, pitch, roll), projected from HALF_SIZE
    boxes around them. HALF_SIZE is the x500's, so only x500s get boxes that fit."""
    boxes = []
    for p in placed:
        label, at = p['label'], p['at']
        q = quaternion(p['heading'], p['pitch'], p['roll'])
        offset = ground_truth.enu_to_ned(ground_truth.rotate(q, BOX_CENTRE[label]))
        centre = (at[0] + offset[0], at[1] + offset[1], -at[2] + offset[2])
        points = ground_truth.corners(centre, q, ground_truth.HALF_SIZE[CLASS_NAME[label]])
        box = ground_truth.project_box(truth_camera, points, HFOV, WIDTH, HEIGHT_PX)
        if box is None or box['in_image'] < 0.5:
            continue
        w, h = box['x1'] - box['x0'], box['y1'] - box['y0']
        boxes.append({'label': label, 'x0': box['x0'], 'y0': box['y0'], 'x1': box['x1'],
                      'y1': box['y1'], 'width': round(w, 1), 'height': round(h, 1),
                      'u': round((box['x0'] + box['x1']) / 2, 1),
                      'v': round((box['y0'] + box['y1']) / 2, 1)})
    return boxes


def box_topics(gz_topics: list[str], model: str) -> tuple[str | None, str | None]:
    """The visible and full box topics of a drone model's bounding-box cameras
    (boxes_visible, boxes_full in x500_gimbal_labels), or None."""
    def find(sensor: str) -> str | None:
        found = [t for t in gz_topics if f'/model/{model}/' in t and f'/sensor/{sensor}/' in t
                 and not t.endswith('/image')]
        return min(found, key=len, default=None)
    return find('boxes_visible'), find('boxes_full')


def schedule(count: int, heights: list[float]) -> list[float]:
    """The height for each image: `count` images shared out over the heights in turn."""
    per = [count // len(heights) + (i < count % len(heights)) for i in range(len(heights))]
    return [h for h, n in zip(heights, per) for _ in range(n)]


# ----- the flight (needs Gazebo, PX4 and ROS) -----

class Flight:

    def __init__(self, node, config, drone_id: int):
        from swarm_backend.px4.px4_drone import PX4Drone
        from swarm_backend.swarm.manager import SwarmManager
        from swarm_backend.swarm.swarm import Swarm
        self.drone = PX4Drone(node, config.drone(drone_id), rate_hz=config.rate_hz)
        self.manager = SwarmManager(Swarm([self.drone]), TOLERANCE_M)
        self.log = node.get_logger()

    def wait(self, predicate, sim_timeout: float, what: str) -> None:
        """Until predicate() is true; the timeout is in simulation time (PX4's clock)."""
        start = None
        while not predicate():
            now = self.drone.stamp
            if start is None:
                start = now
            elif now is not None and now - start > sim_timeout:
                raise TimeoutError(f'Timed out after {sim_timeout:.0f} sim s waiting for {what}')
            time.sleep(0.1)

    def sleep(self, sim_seconds: float) -> None:
        start = self.drone.stamp
        self.wait(lambda: self.drone.stamp is not None and start is not None
                  and self.drone.stamp - start >= sim_seconds, sim_seconds + 30, 'time to pass')

    def start(self, height: float) -> None:
        from swarm_backend.runner import wait_until
        wait_until(self.manager.connected, 180, f'status and position from drone {self.drone.id}')
        self.wait(self.manager.ready, 60, 'pre-flight checks')
        self.log.info(f'Taking off to {height} m')
        self.manager.takeoff(height, [self.drone.id])
        self.wait(self.manager.all_reached, CLIMB_TIMEOUT_S, 'the climb')

    def go_to_height(self, height: float, heading: float) -> None:
        x, y, _ = self.drone.home
        self.drone.set_position(x, y, self.drone.home[2] - height, yaw=heading)
        self.wait(lambda: self.drone.reached(TOLERANCE_M), CLIMB_TIMEOUT_S, f'{height} m')

    def turn(self, heading: float, gimbal_pitch: float, gimbal_yaw: float) -> None:
        """New heading (radians from north) and gimbal angles (degrees; pitch from the
        horizon, yaw from the nose), keeping the position."""
        self.drone.set_position(*self.drone.target, yaw=heading)
        self.drone.aim_gimbal(gimbal_pitch, gimbal_yaw)

    def land(self) -> None:
        self.drone.release_gimbal()
        if not self.drone.armed:
            return
        self.log.info('Landing')
        self.manager.land([self.drone.id])
        self.wait(self.manager.all_landed, LAND_TIMEOUT_S, 'the landing')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('id', type=int, help='PX4 instance number of the camera drone')
    parser.add_argument('--run', required=True, help='name of this run, e.g. L01')
    parser.add_argument('--count', type=int, default=100, help='images (default %(default)s)')
    parser.add_argument('--heights', type=float, nargs='+', default=[25, 50, 80, 120],
                        help='metres, flown in turn (default %(default)s)')
    parser.add_argument('--labels', choices=('visible', 'projected'), default='visible')
    parser.add_argument('--pitch', type=float, nargs=2, default=(-15.0, 35.0),
                        metavar=('MIN', 'MAX'), help='gimbal pitch range, degrees from the '
                        'horizon (default %(default)s)')
    parser.add_argument('--settle', type=float, default=1.5,
                        help='simulation seconds to turn before each image (default %(default)s)')
    parser.add_argument('--seed', type=int, help='random seed (default: from the run name)')
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--drones', type=int, default=3, help='most drones in one image')
    add_variety_options(parser)
    parser.add_argument('--birds', type=int, default=2,
                        help='most birds in one image (0-this many; default %(default)s)')
    add_bird_options(parser)
    parser.add_argument('--empty', type=float, default=0.15,
                        help='share of images with nothing placed (default %(default)s)')
    parser.add_argument('--min-px', type=float, default=2.0,
                        help='objects smaller than this get no label (default %(default)s)')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='wall-clock seconds to wait for an image (default %(default)s)')
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    from swarm_backend.birds import Gazebo
    from swarm_backend.config import default_config_path, load_config
    from swarm_backend.runner import spinning
    from swarm_backend.save_images import PoseHistory, drone_names
    from swarm_backend.sensor_view import find_sensors

    gz_topics = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                               check=True).stdout.split()
    world, names = drone_names(gz_topics)
    model = names.get(args.id)
    camera = next((s for s in find_sensors(gz_topics, args.id) if s.name == 'camera'), None)
    if model is None or camera is None:
        print(f'Drone {args.id} has no gimbal camera in Gazebo. Start one with '
              './mount-backend.sh spawn gimbal_labels (or spawn gimbal with --labels projected)')
        return 1
    visible_topic, full_topic = box_topics(gz_topics, model)
    if args.labels == 'visible' and not (visible_topic and full_topic):
        print(f'{model} has no bounding-box cameras: spawn the x500_gimbal_labels model '
              '(./mount-backend.sh spawn gimbal_labels), or use --labels projected.')
        return 1
    if args.labels == 'projected':
        visible_topic = full_topic = None

    rng = random.Random(args.seed if args.seed is not None else args.run)
    out = args.out / args.run
    if not prepare_run(out):
        return 1
    if args.labels == 'projected' and (args.kinds != ['x500'] or args.recolour):
        print('--labels projected: placing only unpainted x500s (the projected box size is '
              "the x500's)")
        args.kinds, args.recolour = ['x500'], 0.0
    gazebo = Gazebo(world)
    pool = TargetPool(gazebo, rng, max(args.variants, args.drones), args.kinds, args.recolour,
                      args.birds, args.bird_kinds)
    if not pool.spawn():
        print('Could not spawn the target models.')
        return 1
    print('Drones this run: ' + ', '.join(v.name for v in pool.variants))
    streams = Streams(image=camera.gz_topic, visible=visible_topic, full=full_topic)
    poses = PoseHistory(world)

    config = load_config(args.config or default_config_path())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('gen_live')
    flight = Flight(node, config, args.id)
    writer = RunWriter(out, args.count, args.min_px)
    heights = schedule(args.count, args.heights)
    heading = rng.uniform(0, 2 * math.pi)
    with spinning(node):
        try:
            flight.start(heights[0])
            for i, height in enumerate(heights):
                if i == 0 or height != heights[i - 1]:
                    flight.go_to_height(height, heading)
                name = f'{args.run}_{i:05d}'
                heading = (heading + math.radians(rng.uniform(20, 70))) % (2 * math.pi)
                gimbal = (rng.uniform(*args.pitch), max(-45.0, min(45.0, rng.gauss(0.0, 15.0))))
                flight.turn(heading, *gimbal)
                flight.sleep(args.settle)

                found = poses.at(streams.newest, tolerance=0.2)
                truth_camera = found and ground_truth.camera_pose(found[1], model, camera.frame)
                if truth_camera is None:
                    print(f'{name}: no camera pose from Gazebo yet, skipped')
                    continue
                cam = camera_from_truth(truth_camera)
                empty = rng.random() < args.empty
                placed = pool.place(rng, cam, 0 if empty else rng.randint(1, max(1, args.drones)),
                                    0 if empty or not args.birds else rng.randint(0, args.birds))
                if not pool.ok:
                    # labels still come from what Gazebo shows; only 'placed' may be off
                    print(f'{name}: moving a target failed (Gazebo slow?), kept anyway')
                got = streams.after(streams.newest + 0.3, args.timeout)
                if got is None:
                    print(f'{name}: no image after the move, skipped')
                    continue
                stamp, image, visible, full = got
                at_image = poses.at(stamp, tolerance=0.05)
                image_camera = at_image and ground_truth.camera_pose(at_image[1], model,
                                                                     camera.frame)
                if args.labels == 'projected':
                    if image_camera is None:
                        print(f'{name}: no camera pose for the image time, skipped')
                        continue
                    visible = full = projected_boxes(image_camera, placed)
                used = camera_from_truth(image_camera) if image_camera else cam
                writer.save(name, image, visible or [], full or [], used,
                            [(p['label'], p['size'], p['at']) for p in placed], {
                                'time': stamp, 'run': args.run, 'index': i,
                                'generator': 'live', 'labels': args.labels, 'drone': model,
                                'height': height,
                                'camera': {'position': (image_camera or truth_camera)['position'],
                                           'forward': (image_camera or truth_camera)['forward'],
                                           'hfov_rad': HFOV, 'width': WIDTH,
                                           'height': HEIGHT_PX},
                                'heading_deg': round(math.degrees(heading), 1),
                                'gimbal_cmd_deg': [round(g, 1) for g in gimbal],
                                'gimbal_reported_deg': flight.drone.gimbal, 'empty': empty,
                                'placed': [{'model': p['model'], 'variant': p['variant'],
                                            'drawn_px': round(p['size'], 1)}
                                           for p in placed]})
        except (TimeoutError, RuntimeError, KeyboardInterrupt) as e:
            flight.log.error(f'Stopping ({e or "Ctrl-C"})')
        finally:
            writer.close()
            pool.remove()
            try:
                flight.land()
            except (TimeoutError, KeyboardInterrupt) as e:
                flight.log.error(f'Landing did not finish ({e or "Ctrl-C"})')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
