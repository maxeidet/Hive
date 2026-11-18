"""Draw the scanned 3D map (OctoMap) from straight above, as the operator UI's background
map (`mount-backend.sh scanmap`).

Reads an OctoMap binary file (.bt, as explore saves it in data/map.bt) and writes map.png
and map.json next to the operator UI's page, in the same form as map_snapshot.py: map.json
holds the area the image covers in world NED metres, so the UI puts it under the drones.
The UI reloads the picture when map.png changes, so it needs no restart.

Each pixel shows the highest occupied voxel above it: near the ground in earth tones,
higher up (trees, tents) in greens that get lighter with height, with hill shading so
shapes stand out. Space the drone saw as empty but with nothing occupied under it is a
pale tint, and what it never saw is transparent.

With --watch N it asks the running octomap_server (`map` or `explore`) for its map every
N seconds instead, so the operator sees the map grow while the drone flies.

    python3 -m swarm_backend.octomap_snapshot [data/map.bt] [--watch 10] [--ns mapping]
"""
import argparse
import json
import math
import struct
import subprocess
import sys
import tempfile
import time
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

OUT_DIR = Path(__file__).resolve().parent / 'ui' / 'static'
DEFAULT_MAP = 'data/map.bt'
TREE_DEPTH = 16  # OctoMap's fixed depth: keys run 0..2^16-1, 2^15 is the origin
KEY_OFFSET = 1 << (TREE_DEPTH - 1)
SAVE_TIMEOUT_S = 60.0


@dataclass
class Leaves:
    """The leaves of an OctoMap as boxes in Gazebo ENU metres (x east, y north, z up).

    lo: (n, 3) lower corners, size: (n,) edge lengths. Inner nodes are not leaves; a leaf
    higher up in the tree is a bigger box with one state for all of it.
    """
    resolution: float
    occupied_lo: np.ndarray
    occupied_size: np.ndarray
    free_lo: np.ndarray
    free_size: np.ndarray


def read_bt(data: bytes) -> Leaves:
    """Parse an OctoMap binary file (OcTree::writeBinary).

    After a text header ending in 'data', each node is two bytes with two bits per child
    (children 0-3, then 4-7): 01 occupied leaf, 10 free leaf, 11 inner node, 00 no child.
    The inner children follow depth first in child order. Child i takes the upper half of
    its parent in x if bit 0 of i is set, in y for bit 1, in z for bit 2.
    """
    resolution = None
    pos = 0
    while True:
        end = data.index(b'\n', pos)
        line = data[pos:end].decode('ascii', 'replace').strip()
        pos = end + 1
        if line.startswith('res '):
            resolution = float(line.split()[1])
        elif line.startswith('id ') and line.split()[1] != 'OcTree':
            raise ValueError(f'Not an OcTree binary file: {line}')
        elif line == 'data':
            break
    if resolution is None:
        raise ValueError('No resolution in the OctoMap header')

    occupied: list[tuple[int, int, int, int]] = []
    free: list[tuple[int, int, int, int]] = []
    body = memoryview(data)[pos:]
    cursor = 0

    def node(kx: int, ky: int, kz: int, size: int) -> None:
        nonlocal cursor
        bits = body[cursor] | (body[cursor + 1] << 8)
        cursor += 2
        half = size >> 1
        inner = []
        for i in range(8):
            code = (bits >> (2 * i)) & 3
            if not code:
                continue
            child = (kx + (half if i & 1 else 0), ky + (half if i & 2 else 0),
                     kz + (half if i & 4 else 0), half)
            if code == 3:
                inner.append(child)
            elif code == 2:
                occupied.append(child)
            else:
                free.append(child)
        for child in inner:
            node(*child)

    if len(body) >= 2:
        node(0, 0, 0, 1 << TREE_DEPTH)

    def boxes(keys):
        if not keys:
            return np.zeros((0, 3)), np.zeros(0)
        arr = np.array(keys, dtype=np.float64)
        return (arr[:, :3] - KEY_OFFSET) * resolution, arr[:, 3] * resolution

    return Leaves(resolution, *boxes(occupied), *boxes(free))


@dataclass
class TopView:
    """The map from above on a grid of `cell` metres. Row 0 is the north edge."""
    top: np.ndarray  # highest occupied z per cell (NaN: nothing occupied)
    seen: np.ndarray  # True where any voxel (free or occupied) is known
    east_min: float
    north_max: float
    cell: float

    @property
    def extent(self) -> dict:
        rows, cols = self.top.shape
        return {'north_min': self.north_max - rows * self.cell, 'north_max': self.north_max,
                'east_min': self.east_min, 'east_max': self.east_min + cols * self.cell}


def top_view(leaves: Leaves, cell: float | None = None, max_cells: int = 4096,
             bounds: tuple[float, float, float, float] | None = None) -> TopView:
    """Project the leaves onto the ground. `bounds` (east_min, east_max, north_min,
    north_max) crops; by default the grid covers everything that is known. The cell grows
    when needed so neither side is longer than max_cells."""
    lo = np.vstack([leaves.occupied_lo, leaves.free_lo])
    size = np.concatenate([leaves.occupied_size, leaves.free_size])
    if not len(size):
        raise ValueError('The map is empty')
    if bounds is None:
        bounds = (lo[:, 0].min(), (lo[:, 0] + size).max(), lo[:, 1].min(), (lo[:, 1] + size).max())
    east_min, east_max, north_min, north_max = bounds
    cell = cell or leaves.resolution
    cell = max(cell, (east_max - east_min) / max_cells, (north_max - north_min) / max_cells)
    cols = max(1, math.ceil((east_max - east_min) / cell))
    rows = max(1, math.ceil((north_max - north_min) / cell))
    top = np.full((rows, cols), -np.inf)
    seen = np.zeros((rows, cols), dtype=bool)

    def cells(lo_xy: np.ndarray, sz: np.ndarray):
        """Pixel ranges (r0, r1, c0, c1) of each box, clipped; boxes outside are dropped."""
        c0 = np.floor((lo_xy[:, 0] - east_min) / cell).astype(int)
        c1 = np.ceil((lo_xy[:, 0] + sz - east_min) / cell).astype(int)
        r0 = np.floor((north_max - (lo_xy[:, 1] + sz)) / cell).astype(int)
        r1 = np.ceil((north_max - lo_xy[:, 1]) / cell).astype(int)
        c0, r0 = np.clip(c0, 0, cols), np.clip(r0, 0, rows)
        c1, r1 = np.clip(c1, 0, cols), np.clip(r1, 0, rows)
        keep = (c1 > c0) & (r1 > r0)
        return r0[keep], r1[keep], c0[keep], c1[keep], keep

    # Small boxes are one pixel each, which numpy does in one go; bigger ones by slicing.
    for box_lo, box_size, is_occupied in ((leaves.free_lo, leaves.free_size, False),
                                          (leaves.occupied_lo, leaves.occupied_size, True)):
        if not len(box_size):
            continue
        r0, r1, c0, c1, keep = cells(box_lo[:, :2], box_size)
        z_top = (box_lo[:, 2] + box_size)[keep]
        single = (r1 - r0 == 1) & (c1 - c0 == 1)
        seen[r0[single], c0[single]] = True
        if is_occupied:
            np.maximum.at(top, (r0[single], c0[single]), z_top[single])
        for i in np.flatnonzero(~single):
            seen[r0[i]:r1[i], c0[i]:c1[i]] = True
            if is_occupied:
                view = top[r0[i]:r1[i], c0[i]:c1[i]]
                np.maximum(view, z_top[i], out=view)
    top[np.isinf(top)] = np.nan
    return TopView(fill_gaps(top), seen, east_min, north_max, cell)


def fill_gaps(top: np.ndarray, passes: int = 2, min_neighbours: int = 3) -> np.ndarray:
    """Fill the thin gaps between the depth camera's sampled rays: an empty cell with at
    least `min_neighbours` of its 8 neighbours occupied gets their mean height."""
    top = top.copy()
    for _ in range(passes):
        occupied = ~np.isnan(top)
        padded_h = np.pad(np.where(occupied, top, 0.0), 1)
        padded_n = np.pad(occupied.astype(np.int32), 1)
        total = np.zeros_like(top)
        count = np.zeros(top.shape, dtype=np.int32)
        rows, cols = top.shape
        for dr in (0, 1, 2):
            for dc in (0, 1, 2):
                if dr == dc == 1:
                    continue
                total += padded_h[dr:dr + rows, dc:dc + cols]
                count += padded_n[dr:dr + rows, dc:dc + cols]
        fill = ~occupied & (count >= min_neighbours)
        if not fill.any():
            break
        top[fill] = total[fill] / count[fill]
    return top


# Colour ramp over the height above the ground (metres, RGB).
GROUND_RAMP = [(-1.0, (120, 110, 95)), (0.4, (170, 150, 115)), (1.5, (120, 140, 80)),
               (5.0, (50, 110, 50)), (12.0, (90, 160, 80)), (20.0, (190, 220, 170))]
FREE_RGBA = (215, 225, 235, 110)  # seen, but nothing occupied below


def colourize(view: TopView, ground_z: float) -> np.ndarray:
    """RGBA image (rows, cols, 4) of the top view."""
    height = view.top - ground_z
    occupied = ~np.isnan(height)
    h = np.where(occupied, height, 0.0)
    stops = np.array([s for s, _ in GROUND_RAMP])
    colours = np.array([c for _, c in GROUND_RAMP], dtype=float)
    rgb = np.stack([np.interp(h, stops, colours[:, k]) for k in range(3)], axis=-1)

    # Hill shading, light from the north-west, so tree crowns and edges read as shapes.
    surface = np.where(occupied, view.top, np.nanmin(view.top) if occupied.any() else 0.0)
    gy, gx = np.gradient(np.pad(surface, 1, mode='edge'), view.cell)  # pad: works for 1 px
    gy, gx = gy[1:-1, 1:-1], gx[1:-1, 1:-1]
    slope = np.pi / 2 - np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    azimuth, altitude = math.radians(315), math.radians(45)
    shade = (np.sin(altitude) * np.sin(slope)
             + np.cos(altitude) * np.cos(slope) * np.cos(azimuth - aspect))
    rgb *= (0.55 + 0.45 * np.clip(shade, 0, 1))[..., None]

    image = np.zeros(view.top.shape + (4,), dtype=np.uint8)
    image[view.seen & ~occupied] = FREE_RGBA
    image[occupied, :3] = np.clip(rgb[occupied], 0, 255).astype(np.uint8)
    image[occupied, 3] = 255
    return image


def png_rgba(image: np.ndarray) -> bytes:
    """An 8-bit RGBA PNG of an (rows, cols, 4) uint8 array."""
    rows, cols, _ = image.shape
    raw = np.hstack([np.zeros((rows, 1), dtype=np.uint8), image.reshape(rows, cols * 4)])

    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body))

    header = struct.pack('>IIBBBBB', cols, rows, 8, 6, 0, 0, 0)  # 8 bit, colour type RGBA
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header)
            + chunk(b'IDAT', zlib.compress(raw.tobytes(), 6)) + chunk(b'IEND', b''))


def write_snapshot(bt: bytes, out: Path, ground_z: float, cell: float | None,
                   bounds: tuple | None, world: str | None) -> dict:
    """Render `bt` into out/map.png and out/map.json; returns what map.json holds."""
    leaves = read_bt(bt)
    view = top_view(leaves, cell=cell, bounds=bounds)
    image = colourize(view, ground_z)
    out.mkdir(parents=True, exist_ok=True)
    # Write next to the target and rename, so the UI never reads half a file.
    tmp = out / 'map.png.tmp'
    tmp.write_bytes(png_rgba(image))
    info = {'world': world, 'source': 'octomap', **view.extent,
            'occupied_voxels': int(len(leaves.occupied_size))}
    (out / 'map.json.tmp').write_text(json.dumps(info, indent=2))
    (out / 'map.json.tmp').replace(out / 'map.json')
    tmp.replace(out / 'map.png')
    return info


def save_from_server(ns: str, path: Path) -> bytes:
    """Ask the octomap_server in namespace `ns` for its map (octomap_saver_node)."""
    from ament_index_python.packages import get_package_prefix
    saver = Path(get_package_prefix('octomap_server')) / 'lib' / 'octomap_server' / 'octomap_saver_node'
    path.unlink(missing_ok=True)
    subprocess.run([str(saver), '--ros-args', '-r', f'__ns:=/{ns.strip("/")}',
                    '-p', f'octomap_path:={path}'],
                   timeout=SAVE_TIMEOUT_S, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    if not path.is_file():
        raise RuntimeError(f'No map from /{ns}/octomap_full. Is `map` or `explore` running?')
    return path.read_bytes()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('map', nargs='?', default=DEFAULT_MAP, help='.bt file (default data/map.bt)')
    parser.add_argument('--watch', type=float, default=None, metavar='S',
                        help='take the map from the running octomap_server every S seconds')
    parser.add_argument('--ns', default='mapping', help='octomap_server namespace for --watch')
    parser.add_argument('--ground', type=float, default=0.0, help='ground height, ENU z (m)')
    parser.add_argument('--cell', type=float, default=None,
                        help='metres per pixel (default the map resolution)')
    parser.add_argument('--bounds', type=float, nargs=4, default=None,
                        metavar=('E_MIN', 'E_MAX', 'N_MIN', 'N_MAX'),
                        help='crop to this area, world metres (default: all that is known)')
    parser.add_argument('--world', default=None, help='world name for map.json')
    parser.add_argument('--out', type=Path, default=OUT_DIR, help='folder for map.png/json')
    args = parser.parse_args(argv)

    def render(bt: bytes) -> None:
        info = write_snapshot(bt, args.out, args.ground, args.cell, args.bounds, args.world)
        print(f'{time.strftime("%H:%M:%S")} {info["occupied_voxels"]} occupied voxels, '
              f'east {info["east_min"]:.0f}..{info["east_max"]:.0f} m, '
              f'north {info["north_min"]:.0f}..{info["north_max"]:.0f} m -> '
              f'{args.out / "map.png"}', flush=True)

    if args.watch is None:
        render(Path(args.map).read_bytes())
        return 0
    tmp = Path(tempfile.gettempdir()) / 'octomap_snapshot.bt'
    print(f'Drawing /{args.ns}/octomap_full every {args.watch:g} s. Ctrl-C stops.', flush=True)
    try:
        while True:
            started = time.monotonic()
            try:
                render(save_from_server(args.ns, tmp))
            except (RuntimeError, ValueError, subprocess.TimeoutExpired) as e:
                print(f'{time.strftime("%H:%M:%S")} {e}', flush=True)
            time.sleep(max(0.0, args.watch - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
