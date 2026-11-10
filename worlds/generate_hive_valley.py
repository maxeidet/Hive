#!/usr/bin/env python3
"""Generates worlds/hive_valley.sdf: a 4 x 4 km Swedish valley with hills, a river, dirt
roads, a bridge and the field camp in a clearing at the origin.

Detail falls off from the centre, so the world is big without being slow:

  centre, 1.2 x 1.2 km   fine ground (6 m), the camp, river, bridge and dense forest
  out to about 1.1 km    the forest thins out and the trees get simpler
  out to 2 km            coarse ground (25 m), forest only painted on, hills toward the
                         edges so the end of the world is not in view

Built to load fast. Gazebo makes one entity per visual and collision, and that is what
makes hive_base slow (thousands of trees and parts). Here the static world is baked into
a few meshes instead:

  terrain.obj / terrain_collision.obj   the centre's ground, painted by terrain.png
  outer.obj / outer_collision.obj       the ground around it (a hole where the centre
                                        is), painted by outer.png
  forest_<i>.obj / forest_<i>_col.obj   every tree in a 550 x 550 m chunk merged into one
                                        low-poly mesh, and their trunks for physics

so the whole world is a few dozen entities. Plain Python, no packages:

    python3 worlds/generate_hive_valley.py

The drones spawn at the origin, so the camp clearing there is flat at height 0 (spawn
poses in swarm.yaml are all within it). North of the camp is an open meadow toward the
default search heading, so the defender sees the hostile against the sky. Seeded: the
same script always gives the same world.
"""

import math
import random
import struct
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAME = "hive_valley"
OUT_SDF = HERE / f"{NAME}.sdf"
MESH_DIR = HERE / NAME
URI = f"/workspace/worlds/{NAME}"  # the repo is mounted at /workspace in the container
rng = random.Random(1206)

HALF = 2000.0  # metres from the centre to each edge (Gazebo ENU: x east, y north)
INNER = 600.0  # the fine centre: -INNER..INNER both ways
GRID = 3.0  # centre height samples; the visual mesh uses every 2nd, collision every 4th
OUTER_GRID = 25.0  # outer ground; INNER and HALF are multiples, so the meshes meet
TEX = 1024  # centre texture (1.2 m per pixel)
OUTER_TEX = 1024  # outer texture (3.9 m per pixel)
SKIRT = 4.0  # the centre mesh hangs this far down at its edge, to hide seams
WATER_Z = -5.0
CAMP_R = 60.0  # flat at z = 0 within this radius, blending to the hills by CAMP_BLEND
CAMP_BLEND = 140.0
TREE_FULL = 650.0  # full forest within this (square) distance from the centre...
TREE_END = 1100.0  # ...thinning out to none here; beyond, forest is only painted
SIMPLE_TREES = 850.0  # beyond this, one cone per tree instead of trunk and crown tiers
TREE_TARGET = 11000
MIN_TREE_D = 4.5
CHUNK = 550.0  # forest chunk size


# --- noise -------------------------------------------------------------------

def _hash(ix, iy, seed):
    h = (ix * 374761393 + iy * 668265263 + seed * 2147483647) & 0xFFFFFFFF
    h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
    return ((h ^ (h >> 16)) & 0xFFFF) / 65535.0


def value_noise(x, y, scale, seed):
    """Smooth noise in [-1, 1] with features about `scale` metres across."""
    x, y = x / scale, y / scale
    ix, iy = math.floor(x), math.floor(y)
    fx, fy = x - ix, y - iy
    sx, sy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
    a, b = _hash(ix, iy, seed), _hash(ix + 1, iy, seed)
    c, d = _hash(ix, iy + 1, seed), _hash(ix + 1, iy + 1, seed)
    return 2 * (a + (b - a) * sx + (c - a) * sy + (a - b - c + d) * sx * sy) - 1


def fbm(x, y, scale, seed, octaves=4):
    total, amp, norm = 0.0, 1.0, 0.0
    for o in range(octaves):
        # Each octave turned by a different angle, so the blobs do not line up with the grid.
        a = 0.9 * o + 0.3 * seed
        c, s = math.cos(a), math.sin(a)
        total += amp * value_noise(c * x - s * y, s * x + c * y, scale, seed + 17 * o)
        norm += amp
        amp *= 0.5
        scale *= 0.5
    return total / norm


def smoothstep(a, b, x):
    t = min(1.0, max(0.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return a + (b - a) * t


def square_dist(x, y):
    return max(abs(x), abs(y))


# --- layout ------------------------------------------------------------------

def river_y(x):
    """The river's centre line: west to east, south of the camp."""
    return -215.0 + 50.0 * math.sin(x / 160.0) + 18.0 * math.sin(x / 53.0 + 1.0)


def river_dist(x, y):
    slope = (river_y(x + 1) - river_y(x - 1)) / 2
    return abs(y - river_y(x)) / math.sqrt(1 + slope * slope)


def road_y(x):
    """Main road: west to east, just south of the camp."""
    return -42.0 + 9.0 * math.sin(x / 70.0)


def south_road_x(y):
    """Side road: from the main road south over the bridge."""
    return 40.0 + 20.0 * math.sin(y / 60.0)


ROAD_W = 5.0
BRIDGE_Y = river_y(south_road_x(-215.0))


def road_dist(x, y):
    d = abs(y - road_y(x))
    if y < road_y(x):
        d = min(d, abs(x - south_road_x(y)))
    return d


BUMPS = [  # (x, y, radius, height): extra hills
    (350, -380, 160, 38), (-380, 260, 140, 32), (-320, -430, 150, 28), (420, 200, 120, 24),
    (900, -700, 300, 55), (-1000, -800, 350, 60), (-900, 700, 280, 50), (1100, 900, 320, 65),
]


def natural_height(x, y):
    h = 10.0 * fbm(x, y, 260.0, 1)
    h += 50.0 * smoothstep(180, 520, y) * (0.65 + 0.35 * fbm(x, y, 150.0, 2))  # ridge north
    for bx, by, br, bh in BUMPS:
        d2 = ((x - bx) ** 2 + (y - by) ** 2) / (br * br)
        if d2 < 6:
            h += bh * math.exp(-2.0 * d2)
    # Hills rise toward the edges, so the end of the world is not in view.
    h += 90.0 * smoothstep(1100, 1900, square_dist(x, y)) * (0.6 + 0.4 * fbm(x, y, 350.0, 5))
    if h < -1.5:  # keep hollows above the water, so the river is the only water
        h = -1.5 - 0.2 * (-1.5 - h)
    # River valley: an 8 m deep channel with sloping banks, in a wider valley.
    d = river_dist(x, y)
    channel = lerp(-8.0, -3.0, smoothstep(8.0, 28.0, d))
    h = lerp(channel, max(h, -3.0) if d < 160 else h, smoothstep(28.0, 160.0, d))
    return h


def height(x, y):
    r = math.hypot(x, y)
    return natural_height(x, y) * smoothstep(CAMP_R, CAMP_BLEND, r)


def meadow(x, y):
    """1 in open ground with no trees: the camp clearing, the meadow north of it, and
    open patches where the noise says so."""
    camp = 1 - smoothstep(38.0, 48.0, math.hypot(x, y))
    north = (1 - smoothstep(45.0, 60.0, abs(x))) * smoothstep(10.0, 20.0, y) * (
        1 - smoothstep(190.0, 220.0, y))
    patches = smoothstep(0.25, 0.4, fbm(x, y, 180.0, 7, 3))
    return max(camp, north, patches)


def forest(x, y):
    """How dense the forest is here, 0..1 (stands and gaps), ignoring meadows."""
    return smoothstep(-0.35, 0.1, fbm(x, y, 120.0, 11, 3))


# --- height grids ------------------------------------------------------------

class Grid:
    """Heights on a square grid from -half to half, with bilinear lookup."""

    def __init__(self, half, step):
        self.half, self.step = half, step
        self.n = int(round(2 * half / step)) + 1
        self.h = [[height(-half + i * step, -half + j * step) for i in range(self.n)]
                  for j in range(self.n)]  # h[j][i]: j north, i east

    def at(self, x, y):
        fx = min(max((x + self.half) / self.step, 0.0), self.n - 1.001)
        fy = min(max((y + self.half) / self.step, 0.0), self.n - 1.001)
        i, j = int(fx), int(fy)
        tx, ty = fx - i, fy - j
        h = self.h
        return (h[j][i] * (1 - tx) * (1 - ty) + h[j][i + 1] * tx * (1 - ty)
                + h[j + 1][i] * (1 - tx) * ty + h[j + 1][i + 1] * tx * ty)

    def slope(self, x, y):
        s = self.step
        dx = (self.at(x + s, y) - self.at(x - s, y)) / (2 * s)
        dy = (self.at(x, y + s) - self.at(x, y - s)) / (2 * s)
        return dx, dy


print("Height grids...", flush=True)
FINE = Grid(INNER, GRID)
COARSE = Grid(HALF, OUTER_GRID)


def ground(x, y):
    """The grid that has the ground at (x, y): the fine one in the centre."""
    return FINE if square_dist(x, y) <= INNER else COARSE


def ground_height(x, y):
    return ground(x, y).at(x, y)


def ground_slope(x, y):
    dx, dy = ground(x, y).slope(x, y)
    return math.hypot(dx, dy)


# --- writing meshes ----------------------------------------------------------

def write_terrain_obj(path, grid, step, mtl, hole=None, skirt=0.0):
    """`grid` every `step` samples as an OBJ with normals (and UVs over the grid when
    textured with `mtl`). `hole`: leave out the quads inside this square distance.
    `skirt`: hang the edge down this far, so a coarser mesh beside it shows no gaps."""
    idx = list(range(0, grid.n, step))
    if idx[-1] != grid.n - 1:
        idx.append(grid.n - 1)
    n, H, s = len(idx), grid.h, grid.step
    xs = [-grid.half + i * s for i in idx]
    v, vt, vn = [], [], []
    for b, j in enumerate(idx):
        for a, i in enumerate(idx):
            v.append((xs[a], xs[b], H[j][i]))
            vt.append((i / (grid.n - 1), j / (grid.n - 1)))
            i0, i1 = max(i - 1, 0), min(i + 1, grid.n - 1)
            j0, j1 = max(j - 1, 0), min(j + 1, grid.n - 1)
            dx = (H[j][i1] - H[j][i0]) / ((i1 - i0) * s)
            dy = (H[j1][i] - H[j0][i]) / ((j1 - j0) * s)
            k = 1 / math.sqrt(dx * dx + dy * dy + 1)
            vn.append((-dx * k, -dy * k, k))
    faces = []
    for b in range(n - 1):
        for a in range(n - 1):
            if hole and square_dist((xs[a] + xs[a + 1]) / 2, (xs[b] + xs[b + 1]) / 2) < hole:
                continue
            v00 = b * n + a + 1
            v10, v01, v11 = v00 + 1, v00 + n, v00 + n + 1
            faces += [(v00, v10, v11), (v00, v11, v01)]
    if skirt:
        ring = ([a for a in range(n)] + [a * n + n - 1 for a in range(1, n)]
                + [n * n - 1 - a for a in range(1, n)] + [(n - 1 - a) * n for a in range(1, n)])
        first = len(v) + 1
        for k, vi in enumerate(ring):
            x, y, z = v[vi]
            v.append((x, y, z - skirt))
            vt.append(vt[vi])
            vn.append(vn[vi])
        for k in range(len(ring)):
            top_a, top_b = ring[k] + 1, ring[(k + 1) % len(ring)] + 1
            low_a, low_b = first + k, first + (k + 1) % len(ring)
            # Both windings, so it shows from either side.
            faces += [(top_a, low_a, low_b), (top_a, low_b, top_b),
                      (top_a, low_b, low_a), (top_a, top_b, low_b)]
    lines = [f"mtllib {mtl}", "usemtl ground"] if mtl else []
    lines += [f"v {x:.2f} {y:.2f} {z:.2f}" for x, y, z in v]
    if mtl:
        lines += [f"vt {a:.5f} {b:.5f}" for a, b in vt]
    # Normals in every mesh: Gazebo's physics (DART) drops a mesh without one per vertex,
    # and then crashes on the empty shape.
    lines += [f"vn {x:.4f} {y:.4f} {z:.4f}" for x, y, z in vn]
    if mtl:
        lines += [f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}" for a, b, c in faces]
    else:
        lines += [f"f {a}//{a} {b}//{b} {c}//{c}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")
    return len(faces)


def write_png(path, rgb_rows, width, height):
    raw = b"".join(b"\x00" + row for row in rgb_rows)

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(
            ">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    path.write_bytes(b"\x89PNG\r\n\x1a\n"
                     + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


class MeshBuilder:
    """Triangles grouped by material, written as one OBJ with smooth-ish normals."""

    def __init__(self):
        self.v, self.vn = [], []
        self.faces = {}  # material -> [(a, b, c)], 1-based vertex indices

    def vertex(self, p, normal):
        self.v.append(p)
        k = 1 / math.sqrt(sum(c * c for c in normal)) if any(normal) else 0
        self.vn.append(tuple(c * k for c in normal))
        return len(self.v)

    def cone(self, mat, cx, cy, z0, radius, length, sides):
        """A cone from z0 up (or, with a negative length, down) `length`."""
        apex = self.vertex((cx, cy, z0 + length), (0, 0, 1 if length > 0 else -1))
        ring = []
        for k in range(sides):
            a = 2 * math.pi * k / sides
            ring.append(self.vertex((cx + radius * math.cos(a), cy + radius * math.sin(a), z0),
                                    (math.cos(a), math.sin(a), radius / length)))
        f = self.faces.setdefault(mat, [])
        for k in range(sides):  # counter-clockwise seen from outside, also when pointing down
            a, b = ring[k], ring[(k + 1) % sides]
            f.append((a, b, apex) if length > 0 else (b, a, apex))

    def prism(self, mat, cx, cy, z0, radius, length, sides):
        bottom, top = [], []
        for k in range(sides):
            a = 2 * math.pi * k / sides
            c, s = math.cos(a), math.sin(a)
            bottom.append(self.vertex((cx + radius * c, cy + radius * s, z0), (c, s, 0)))
            top.append(self.vertex((cx + radius * c, cy + radius * s, z0 + length), (c, s, 0)))
        f = self.faces.setdefault(mat, [])
        for k in range(sides):
            k1 = (k + 1) % sides
            f.append((bottom[k], bottom[k1], top[k1]))
            f.append((bottom[k], top[k1], top[k]))

    def write(self, path, mtl):
        lines = [f"mtllib {mtl}"] if mtl else []
        lines += [f"v {x:.2f} {y:.2f} {z:.2f}" for x, y, z in self.v]
        lines += [f"vn {x:.3f} {y:.3f} {z:.3f}" for x, y, z in self.vn]
        for mat, faces in self.faces.items():
            if mtl:
                lines.append(f"usemtl {mat}")
            lines += [f"f {a}//{a} {b}//{b} {c}//{c}" for a, b, c in faces]
        path.write_text("\n".join(lines) + "\n")
        return sum(len(f) for f in self.faces.values())


# --- ground meshes -----------------------------------------------------------

MESH_DIR.mkdir(exist_ok=True)
for old in MESH_DIR.glob("forest_*.obj"):  # chunk numbering changes with the layout
    old.unlink()
for name, png in (("centre", "terrain.png"), ("outer", "outer.png")):
    (MESH_DIR / f"{NAME}_{name}.mtl").write_text(
        f"newmtl ground\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd {png}\n")
tris = write_terrain_obj(MESH_DIR / "terrain.obj", FINE, 2, f"{NAME}_centre.mtl", skirt=SKIRT)
write_terrain_obj(MESH_DIR / "terrain_collision.obj", FINE, 4, None)
outer_tris = write_terrain_obj(MESH_DIR / "outer.obj", COARSE, 1, f"{NAME}_outer.mtl", hole=INNER)
write_terrain_obj(MESH_DIR / "outer_collision.obj", COARSE, 1, None, hole=INNER)
print(f"Ground: centre {tris} triangles, outer {outer_tris}", flush=True)

# --- trees -------------------------------------------------------------------

TREE_KINDS = {
    # crown material, bark material, height range (m), crown radius / height
    "pine": ("needles_pine", "bark_pine", (14.0, 24.0), 0.16),
    "spruce": ("needles_spruce", "bark_pine", (10.0, 20.0), 0.22),
    "birch": ("leaves_birch", "bark_birch", (8.0, 14.0), 0.28),
}


def tree_ok(x, y):
    if meadow(x, y) > 0.3 or road_dist(x, y) < ROAD_W / 2 + 3 or river_dist(x, y) < 30:
        return False
    return ground_slope(x, y) < 0.7


chunks = {}
cells = {}
placed = tries = 0
print("Placing trees...", flush=True)
while placed < TREE_TARGET and tries < 1000000:
    tries += 1
    x, y = rng.uniform(-TREE_END, TREE_END), rng.uniform(-TREE_END, TREE_END)
    keep = forest(x, y) * (1 - smoothstep(TREE_FULL, TREE_END, square_dist(x, y)))
    if rng.random() > keep or not tree_ok(x, y):
        continue
    gx, gy = int(x // 5), int(y // 5)
    if any((px - x) ** 2 + (py - y) ** 2 < MIN_TREE_D ** 2
           for ix in range(gx - 1, gx + 2) for iy in range(gy - 1, gy + 2)
           for px, py in cells.get((ix, iy), ())):
        continue
    cells.setdefault((gx, gy), []).append((x, y))
    if river_dist(x, y) < 70 and rng.random() < 0.6:
        kind = "birch"
    else:
        kind = "spruce" if rng.random() < 0.45 else "pine"
    crown_mat, bark_mat, (h0, h1), crown_k = TREE_KINDS[kind]
    h = rng.uniform(h0, h1)
    z = ground_height(x, y) - 0.3
    key = (int((x + TREE_END) // CHUNK), int((y + TREE_END) // CHUNK))
    vis, col = chunks.setdefault(key, (MeshBuilder(), MeshBuilder()))
    if square_dist(x, y) > SIMPLE_TREES:  # far out: one cone, seen from far away anyway
        vis.cone(crown_mat, x, y, z, h * crown_k, h, 5)
    elif kind == "pine":  # tall bare trunk, crown in the top third
        vis.prism(bark_mat, x, y, z, 0.25, h * 0.75, 5)
        vis.cone(crown_mat, x, y, z + h * 0.55, h * crown_k, h * 0.45, 6)
    elif kind == "spruce":  # dark cone almost to the ground, in two tiers
        vis.prism(bark_mat, x, y, z, 0.2, h * 0.2, 5)
        vis.cone(crown_mat, x, y, z + h * 0.12, h * crown_k, h * 0.6, 7)
        vis.cone(crown_mat, x, y, z + h * 0.45, h * crown_k * 0.65, h * 0.55, 7)
    else:  # birch: white trunk, rounder crown (a cone pointing down under one pointing up)
        vis.prism(bark_mat, x, y, z, 0.15, h * 0.7, 5)
        r = h * crown_k
        vis.cone(crown_mat, x, y, z + h * 0.6, r, h * 0.4, 7)
        vis.cone(crown_mat, x, y, z + h * 0.6, r, -h * 0.25, 7)
    # Physics: one thin prism per tree through trunk and crown core, as in hive_base.
    col.prism("c", x, y, z, max(0.3, h * crown_k * 0.35), h, 6)
    placed += 1
print(f"Trees: {placed} in {len(chunks)} chunks", flush=True)

(MESH_DIR / f"{NAME}_trees.mtl").write_text("".join(
    f"newmtl {m}\nKa {c}\nKd {c}\nKs 0 0 0\n" for m, c in (
        ("needles_pine", "0.16 0.27 0.13"), ("needles_spruce", "0.08 0.17 0.09"),
        ("leaves_birch", "0.35 0.52 0.18"), ("bark_pine", "0.45 0.30 0.20"),
        ("bark_birch", "0.86 0.85 0.80"))))
forest_files = []
for k, (key, (vis, col)) in enumerate(sorted(chunks.items())):
    vis.write(MESH_DIR / f"forest_{k}.obj", f"{NAME}_trees.mtl")
    col.write(MESH_DIR / f"forest_{k}_col.obj", None)
    forest_files.append(k)

# --- textures ----------------------------------------------------------------

MOSS = (70, 86, 46)
MEADOW = (104, 118, 62)
FOREST_FLOOR = (52, 62, 36)
CANOPY = (34, 50, 30)  # painted forest, where there are no tree meshes
ROCK = (118, 114, 106)
BANK = (122, 108, 78)
RIVERBED = (60, 70, 60)
DIRT = (110, 88, 62)
RUT = (80, 63, 45)
GRAVEL = (132, 122, 104)


def mix(a, b, t):
    return tuple(int(ca + (cb - ca) * t) for ca, cb in zip(a, b))


def ground_color(x, y, grid):
    h = grid.at(x, y)
    dx, dy = grid.slope(x, y)
    col = mix(FOREST_FLOOR, MEADOW, meadow(x, y))
    col = mix(col, MOSS, 0.5 + 0.5 * value_noise(x, y, 23.0, 31))
    # Where the tree meshes have thinned out, paint the forest on: dark, dotted canopy.
    painted = smoothstep(TREE_FULL, TREE_END, square_dist(x, y)) * forest(x, y) * (1 - meadow(x, y))
    if painted > 0.02:
        dots = 0.5 + 0.5 * value_noise(x, y, 7.0, 51)
        col = mix(col, CANOPY, painted * (0.6 + 0.4 * dots))
    col = mix(col, ROCK, smoothstep(0.45, 0.8, math.hypot(dx, dy)))
    if h < -2.0:  # river banks and bed
        col = mix(col, BANK, smoothstep(-2.0, -4.0, h))
        col = mix(col, RIVERBED, smoothstep(WATER_Z, WATER_Z - 1.5, h))
    rd = road_dist(x, y)
    if rd < ROAD_W / 2 + 0.8:
        col = mix(col, DIRT, smoothstep(ROAD_W / 2 + 0.8, ROAD_W / 2 - 0.3, rd))
        if 0.7 < rd < 1.5:
            col = mix(col, RUT, 0.6)
    if math.hypot(x - 14, y + 14) < 9:  # vehicle park
        col = GRAVEL
    # Hill shading baked in (light from the north-west), so the relief shows from above.
    lit = (dx * 0.55 - dy * 0.55 + 0.63) / math.sqrt(dx * dx + dy * dy + 1)
    shade = (0.55 + 0.7 * lit) * (1.0 + 0.06 * value_noise(x, y, 6.0, 41))
    return bytes(max(0, min(255, int(v * shade))) for v in col)


def paint(path, grid, half, size):
    rows = []
    for r in range(size):
        y = half - (r + 0.5) * 2 * half / size  # row 0 is north (texture v = 1)
        rows.append(b"".join(ground_color(-half + (c + 0.5) * 2 * half / size, y, grid)
                             for c in range(size)))
    write_png(path, rows, size, size)


print("Painting the textures...", flush=True)
paint(MESH_DIR / "terrain.png", FINE, INNER, TEX)
paint(MESH_DIR / "outer.png", COARSE, HALF, OUTER_TEX)

# --- SDF ---------------------------------------------------------------------


def f(v):
    return f"{v:.3f}".rstrip("0").rstrip(".")


def pose(*p):
    return " ".join(f(float(v)) for v in p)


def material(rgba):
    c = " ".join(f(float(v)) for v in rgba)
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>0.05 0.05 0.05 1</specular></material>"


def mesh_model(name, visual, collision, shadows):
    cs = "" if shadows else "<cast_shadows>false</cast_shadows>"
    col = (f'<collision name="collision"><geometry><mesh><uri>{URI}/{collision}</uri></mesh>'
           f"</geometry></collision>") if collision else ""
    return (f'    <model name="{name}"><static>true</static><link name="link">\n'
            f'      <visual name="visual"><geometry><mesh><uri>{URI}/{visual}</uri></mesh></geometry>{cs}</visual>\n'
            f"      {col}\n    </link></model>\n")


class Parts:
    """A static model of SDF primitives (the camp and the bridge: few parts)."""

    def __init__(self, name, p=(0, 0, 0, 0, 0, 0)):
        self.name, self.pose, self.parts, self.n = name, p, [], 0

    def add(self, kind, dims, p, color, collide=False):
        self.n += 1
        g = {"box": lambda: f"<box><size>{pose(*dims)}</size></box>",
             "cyl": lambda: f"<cylinder><radius>{f(dims[0])}</radius><length>{f(dims[1])}</length></cylinder>",
             "cone": lambda: f"<cone><radius>{f(dims[0])}</radius><length>{f(dims[1])}</length></cone>"}[kind]()
        self.parts.append(f'<visual name="v{self.n}"><pose>{pose(*p)}</pose><geometry>{g}</geometry>{material(color)}</visual>')
        if collide:
            cg = f"<cylinder><radius>{f(dims[0] * 0.6)}</radius><length>{f(dims[1])}</length></cylinder>" if kind == "cone" else g
            self.parts.append(f'<collision name="c{self.n}"><pose>{pose(*p)}</pose><geometry>{cg}</geometry></collision>')

    def sdf(self):
        return (f'    <model name="{self.name}"><static>true</static><pose>{pose(*self.pose)}</pose>'
                f'<link name="link">\n        ' + "\n        ".join(self.parts) + "\n    </link></model>\n")


TENT, TENT_DARK = (0.27, 0.31, 0.20, 1), (0.20, 0.23, 0.15, 1)
FM_GREEN, FM_DARK, BLACK = (0.24, 0.30, 0.20, 1), (0.12, 0.15, 0.10, 1), (0.08, 0.08, 0.07, 1)
WOOD, STEEL, ASH = (0.50, 0.36, 0.22, 1), (0.25, 0.25, 0.25, 1), (0.20, 0.19, 0.18, 1)
CONCRETE = (0.55, 0.54, 0.50, 1)

camp = Parts("camp")
for tx, ty in ((-14, 10), (-4, 17), (7, 16), (-18, -2), (17, 6)):
    camp.add("cyl", (3.0, 0.6), (tx, ty, 0.3, 0, 0, 0), TENT_DARK, True)
    camp.add("cone", (3.15, 2.6), (tx, ty, 1.9, 0, 0, 0), TENT, True)
camp.add("cyl", (0.75, 0.03), (-3, 6, 0.015, 0, 0, 0), ASH)  # fire pit
camp.add("box", (2.0, 0.9, 0.05), (-9, -5.5, 0.75, 0, 0, 0), WOOD, True)  # staff table
camp.add("box", (5.0, 4.0, 0.02), (-9, -5.5, 2.5, 0, 0, 0.3), FM_DARK)  # camouflage net
for px, py in ((-11, -7), (-7, -7), (-11, -4), (-7, -4)):
    camp.add("cyl", (0.04, 2.5), (px, py, 1.25, 0, 0, 0), WOOD)
for vx, vy, yaw in ((10, -13, 1.45), (17, -14, 1.6)):  # two vehicles as plain blocks
    c, s = math.cos(yaw), math.sin(yaw)
    camp.add("box", (6.5, 2.2, 1.4), (vx, vy, 0.9, 0, 0, yaw), FM_GREEN, True)
    camp.add("box", (2.0, 2.1, 1.2), (vx + 2.2 * c, vy + 2.2 * s, 2.2, 0, 0, yaw), FM_GREEN, True)
for k in range(4):
    camp.add("box", (1.2, 0.6, 0.5), (8 + k * 1.3, -19, 0.25, 0, 0, 0), FM_GREEN, True)

# Bridge where the side road crosses the river: deck above the water, on four piers.
bx = south_road_x(BRIDGE_Y)
bridge = Parts("bridge")
deck_z = -2.8  # top level with the banks (about -3 m)
bridge.add("box", (ROAD_W + 1, 70.0, 0.6), (bx, BRIDGE_Y, deck_z, 0, 0, 0), CONCRETE, True)
for side in (-1, 1):
    bridge.add("box", (0.2, 70.0, 1.0), (bx + side * (ROAD_W / 2 + 0.4), BRIDGE_Y, deck_z + 0.8, 0, 0, 0), STEEL)
for py in (-21, -7, 7, 21):
    bridge.add("box", (1.2, 1.2, 7.0), (bx, BRIDGE_Y + py, deck_z - 3.8, 0, 0, 0), CONCRETE, True)

HEADER = f"""<?xml version="1.0" ?>
<!--
  GENERATED FILE - do not edit by hand.
  Source: worlds/generate_hive_valley.py  (run: python3 worlds/generate_hive_valley.py)
  4 x 4 km valley: hills, a river with a bridge, dirt roads, {placed} trees and the field
  camp at the origin. Meshes in worlds/{NAME}/, read from /workspace (the mounted repo).
-->
<sdf version="1.11">
  <world name="{NAME}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"></plugin>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"></plugin>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"></plugin>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"></plugin>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.55 0.58 0.6</ambient>
      <background>0.62 0.72 0.82</background>
      <sky></sky>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>1 0.95 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.6 0.4 -0.55</direction>
    </light>

"""

FOOTER = """
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""

models = [mesh_model("terrain", "terrain.obj", "terrain_collision.obj", False),
          mesh_model("terrain_outer", "outer.obj", "outer_collision.obj", False)]
# A flat box under the spawn area, so drones stand and land on a simple shape.
pad = Parts("spawn_pad")
pad.parts.append('<collision name="c"><pose>0 0 -0.5 0 0 0</pose><geometry><box><size>60 60 1</size></box></geometry></collision>')
models.append(pad.sdf())
models.append(
    f'    <model name="river"><static>true</static><link name="link"><visual name="water">'
    f"<pose>0 0 {WATER_Z} 0 0 0</pose><geometry><box><size>{2 * HALF} {2 * HALF} 0.05</size></box></geometry>"
    f"{material((0.16, 0.30, 0.36, 1))}<transparency>0.15</transparency><cast_shadows>false</cast_shadows>"
    f"</visual></link></model>\n")
models += [mesh_model(f"forest_{k}", f"forest_{k}.obj", f"forest_{k}_col.obj", False)
           for k in forest_files]
models += [camp.sdf(), bridge.sdf()]
OUT_SDF.write_text(HEADER + "".join(models) + FOOTER)
size = sum(p.stat().st_size for p in MESH_DIR.iterdir()) / 1e6
print(f"Wrote {OUT_SDF.name} ({len(models)} models) and {NAME}/ ({size:.1f} MB)")
