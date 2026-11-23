"""Take one picture of the running Gazebo world from straight above, as the operator UI's
background map (`mount-backend.sh mapsnap`).

Spawns a camera high above the world (a static model with one camera sensor, looking
straight down, north up in the image), takes one image, removes the camera again and
writes map.png and map.json next to the operator UI's page. map.json holds the area the
image covers, in world NED metres, so the UI can put it under the drones at the right
place and scale. Works for any world: it only needs the world running, no drones.

It is a picture for the operator, nothing more: no drone reads it.

    python3 -m swarm_backend.sim.map_snapshot [--half-size 400] [--pixels 2048]
"""
import argparse
import json
import math
import re
import subprocess
import sys
import threading
from pathlib import Path

from swarm_backend.datagen.save_images import RGB_INT8, png_bytes

MODEL_NAME = 'hive_overhead_camera'
IMAGE_TOPIC = '/hive_overhead/image'
DEFAULT_HFOV = 0.6  # radians; narrow, so trees and tents lean outward little at the edges
OUT_DIR = Path(__file__).resolve().parent / 'ui' / 'static'
WORLD_TOPIC = re.compile(r'^/world/(?P<world>[^/]+)/')
SERVICE_TIMEOUT_MS = 5000
IMAGE_TIMEOUT_S = 180.0  # a big image takes a while without a GPU


def camera_height(half_size: float, hfov: float) -> float:
    """Height that makes a camera with `hfov` (radians) see `half_size` metres to each side."""
    return half_size / math.tan(hfov / 2)


def map_extent(centre_north: float, centre_east: float, half_size: float) -> dict:
    """The square the image covers, world NED metres (the same frame as the drones)."""
    return {'north_min': centre_north - half_size, 'north_max': centre_north + half_size,
            'east_min': centre_east - half_size, 'east_max': centre_east + half_size}


def camera_sdf(centre_north: float, centre_east: float, height: float, hfov: float,
               pixels: int) -> str:
    """A static model with one camera looking straight down, north at the top of the image.

    It only draws what is less than 1000 m above the ground, which keeps depth precise when
    the camera is kilometres up (otherwise water and ground flicker into each other).

    Gazebo is ENU (x east, y north) and a camera looks along its x axis. Pitch 90 degrees
    turns x to straight down; yaw 90 degrees then puts the image's top (the camera's z
    before the pitch) to north and its left (the camera's y) to west.
    """
    pose = f'{centre_east} {centre_north} {height} 0 {math.pi / 2} {math.pi / 2}'
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{MODEL_NAME}">
    <static>true</static>
    <pose>{pose}</pose>
    <link name="link">
      <sensor name="overhead" type="camera">
        <topic>{IMAGE_TOPIC}</topic>
        <update_rate>2</update_rate>
        <always_on>true</always_on>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{pixels}</width><height>{pixels}</height><format>R8G8B8</format></image>
          <clip><near>{max(1.0, height - 1000):.0f}</near><far>{height + 1000:.0f}</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""


def find_world() -> str:
    topics = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                            check=True).stdout.split()
    for t in topics:
        match = WORLD_TOPIC.match(t)
        if match:
            return match['world']
    raise RuntimeError('No Gazebo world running (no /world/... topics). Start it first.')


def gz_service(service: str, reqtype: str, request: str) -> None:
    result = subprocess.run(
        ['gz', 'service', '-s', service, '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
         '--timeout', str(SERVICE_TIMEOUT_MS), '--req', request],
        capture_output=True, text=True)
    if result.returncode != 0 or 'true' not in result.stdout:
        raise RuntimeError(f'{service} failed: {result.stdout.strip()} {result.stderr.strip()}')


def take_image(timeout: float) -> tuple[bytes, int, int]:
    """The first RGB image on IMAGE_TOPIC: (rows of RGB bytes, width, height)."""
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    got: dict = {}
    done = threading.Event()

    def on_image(msg) -> None:
        if done.is_set() or msg.pixel_format_type != RGB_INT8:
            return
        row = msg.width * 3
        rgb = msg.data if msg.step == row else b''.join(
            msg.data[y * msg.step:y * msg.step + row] for y in range(msg.height))
        got['image'] = (rgb, msg.width, msg.height)
        done.set()

    node = Node()
    if not node.subscribe(Image, IMAGE_TOPIC, on_image):
        raise RuntimeError(f'Could not subscribe to {IMAGE_TOPIC}')
    if not done.wait(timeout):
        raise TimeoutError(f'No image on {IMAGE_TOPIC} in {timeout:.0f} s. Is the simulation '
                           'running (not paused)?')
    return got['image']


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--half-size', type=float, default=400.0,
                        help='metres the map reaches from the centre to each side')
    parser.add_argument('--north', type=float, default=0.0, help='centre, world north (m)')
    parser.add_argument('--east', type=float, default=0.0, help='centre, world east (m)')
    parser.add_argument('--pixels', type=int, default=2048, help='image width and height')
    parser.add_argument('--hfov', type=float, default=DEFAULT_HFOV, help='radians')
    parser.add_argument('--out', type=Path, default=OUT_DIR, help='folder for map.png/json')
    args = parser.parse_args(argv)

    world = find_world()
    height = camera_height(args.half_size, args.hfov)
    sdf = camera_sdf(args.north, args.east, height, args.hfov, args.pixels)
    print(f'World {world}: camera {height:.0f} m up, map {2 * args.half_size:.0f} m across',
          flush=True)
    gz_service(f'/world/{world}/create', 'gz.msgs.EntityFactory', 'sdf: ' + json.dumps(sdf))
    try:
        rgb, width, height_px = take_image(IMAGE_TIMEOUT_S)
    finally:
        gz_service(f'/world/{world}/remove', 'gz.msgs.Entity',
                   f'name: "{MODEL_NAME}" type: MODEL')
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'map.png').write_bytes(png_bytes(rgb, width, height_px))
    info = {'world': world, **map_extent(args.north, args.east, args.half_size)}
    (args.out / 'map.json').write_text(json.dumps(info, indent=2))
    print(f'Wrote {args.out / "map.png"} ({width}x{height_px}) and map.json. Restart the '
          'operator UI to show it.', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
