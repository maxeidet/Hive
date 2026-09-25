#!/usr/bin/env python3
"""Genererar worlds/hive_base.sdf: ett militärt tältläger i en svensk barrskog.

Allt byggs av SDF-primitiver (box, cylinder, cone, ellipsoid, capsule), så världen
kräver inga nedladdningar från Gazebo Fuel. Kräver Gazebo Harmonic (sdformat 14).

Kör från repots rot:  python3 worlds/generate_hive_base.py
Slumpen är seedad, så samma skript ger alltid samma värld.
"""

import math
import random
from pathlib import Path

OUT = Path(__file__).with_name("hive_base.sdf")
rng = random.Random(206)

# ----------------------------------------------------------------------------
# Färger (RGBA)
# ----------------------------------------------------------------------------
MOSS = (0.23, 0.30, 0.15, 1)
SPRUCE = (0.10, 0.20, 0.10, 1)
SPRUCE_DARK = (0.07, 0.15, 0.08, 1)
PINE_BARK = (0.55, 0.33, 0.20, 1)
PINE_CROWN = (0.16, 0.27, 0.13, 1)
SPRUCE_BARK = (0.30, 0.22, 0.16, 1)
BIRCH_BARK = (0.88, 0.87, 0.82, 1)
BIRCH_LEAF = (0.35, 0.52, 0.18, 1)
GRANITE = (0.45, 0.44, 0.43, 1)
GRANITE_LICHEN = (0.55, 0.56, 0.50, 1)
BLUEBERRY = (0.18, 0.32, 0.14, 1)
DIRT = (0.40, 0.32, 0.22, 1)
GRAVEL = (0.50, 0.45, 0.38, 1)
RUT = (0.30, 0.24, 0.17, 1)
TENT = (0.27, 0.31, 0.20, 1)
TENT_DARK = (0.20, 0.23, 0.15, 1)
FM_GREEN = (0.24, 0.30, 0.20, 1)   # svensk försvarsmaktsgrön
FM_DARK = (0.12, 0.15, 0.10, 1)
FM_BROWN = (0.33, 0.27, 0.18, 1)
FM_BLACK = (0.08, 0.08, 0.07, 1)
RUBBER = (0.05, 0.05, 0.05, 1)
STEEL = (0.25, 0.25, 0.25, 1)
GLASS = (0.10, 0.14, 0.17, 1)
CANVAS = (0.30, 0.33, 0.22, 1)
WOOD = (0.50, 0.36, 0.22, 1)
ASH = (0.20, 0.19, 0.18, 1)
SE_BLUE = (0.0, 0.32, 0.58, 1)
SE_YELLOW = (1.0, 0.80, 0.0, 1)
HELIPAD = (0.12, 0.12, 0.12, 1)
WHITE = (0.9, 0.9, 0.9, 1)


def f(x):
    return f"{x:.3f}".rstrip("0").rstrip(".") if isinstance(x, float) else str(x)


def pose(*p):
    return " ".join(f(float(v)) for v in p)


def material(c):
    rgba = " ".join(f(float(v)) for v in c)
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.05 0.05 0.05 1</specular></material>"


def geom(kind, *dims):
    if kind == "box":
        return f"<box><size>{pose(*dims)}</size></box>"
    if kind == "cyl":
        return f"<cylinder><radius>{f(float(dims[0]))}</radius><length>{f(float(dims[1]))}</length></cylinder>"
    if kind == "cone":
        return f"<cone><radius>{f(float(dims[0]))}</radius><length>{f(float(dims[1]))}</length></cone>"
    if kind == "sphere":
        return f"<sphere><radius>{f(float(dims[0]))}</radius></sphere>"
    if kind == "ell":
        return f"<ellipsoid><radii>{pose(*dims)}</radii></ellipsoid>"
    if kind == "cap":
        return f"<capsule><radius>{f(float(dims[0]))}</radius><length>{f(float(dims[1]))}</length></capsule>"
    raise ValueError(kind)


class Model:
    """Statisk modell med en länk; delar läggs till som visual (+ ev. collision)."""

    def __init__(self, name, p=(0, 0, 0, 0, 0, 0)):
        self.name, self.pose, self.parts, self.n = name, p, [], 0

    def add(self, kind, dims, p, color, collide=False, shadows=True):
        self.n += 1
        g = geom(kind, *dims)
        # DART saknar kon-kollision; en smalare cylinder räcker gott
        cg = geom("cyl", dims[0] * 0.6, dims[1]) if kind == "cone" else g
        cs = "" if shadows else "<cast_shadows>false</cast_shadows>"
        self.parts.append(
            f'<visual name="v{self.n}"><pose>{pose(*p)}</pose><geometry>{g}</geometry>{material(color)}{cs}</visual>'
        )
        if collide:
            self.parts.append(
                f'<collision name="c{self.n}"><pose>{pose(*p)}</pose><geometry>{cg}</geometry></collision>'
            )

    def sdf(self):
        body = "\n        ".join(self.parts)
        return (
            f'    <model name="{self.name}">\n'
            f"      <static>true</static>\n"
            f"      <pose>{pose(*self.pose)}</pose>\n"
            f'      <link name="link">\n        {body}\n      </link>\n'
            f"    </model>\n"
        )


models = []

# ----------------------------------------------------------------------------
# Layout
# ----------------------------------------------------------------------------
# Lägret ligger i en glänta kring origo (helipaden, där drönaren spawnar).
# Grusvägen går öst-väst söder om lägret med en svag kurva, och en infart
# leder norrut in i gläntan.
CLEARING_R = 30.0
WORLD_R = 140.0


def road_y(x):
    return -38.0 + 7.0 * math.sin(x / 45.0)


ROAD_X = (-150.0, 150.0)
ROAD_W = 5.0
ACCESS_X = 12.0  # infartens x-läge


def dist_to_road(x, y):
    return abs(y - road_y(x))


def in_access(x, y, margin=0.0):
    return abs(x - ACCESS_X) < 3.5 + margin and road_y(ACCESS_X) - 2 < y < -8


# ----------------------------------------------------------------------------
# Mark
# ----------------------------------------------------------------------------
ground = Model("ground_plane")
ground.parts.append(
    "<collision name=\"collision\"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>"
    "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>"
)
ground.parts.append(
    f"<visual name=\"visual\"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>{material(MOSS)}</visual>"
)
# Gläntan: lite ljusare, upptrampat gräs
ground.add("cyl", (CLEARING_R - 2, 0.01), (0, 0, 0.005, 0, 0, 0), (0.33, 0.38, 0.20, 1), shadows=False)
models.append(ground)

# ----------------------------------------------------------------------------
# Grusväg (segment längs en kurva) + infart
# ----------------------------------------------------------------------------
road = Model("dirt_road")
step = 6.0
x = ROAD_X[0]
while x < ROAD_X[1]:
    x2 = x + step
    y1, y2 = road_y(x), road_y(x2)
    L = math.hypot(x2 - x, y2 - y1) + 0.4
    yaw = math.atan2(y2 - y1, x2 - x)
    cx, cy = (x + x2) / 2, (y1 + y2) / 2
    road.add("box", (L, ROAD_W, 0.04), (cx, cy, 0.02, 0, 0, yaw), DIRT, shadows=False)
    # Hjulspår och grusad mittsträng
    for off, col in ((-1.1, RUT), (1.1, RUT), (0.0, GRAVEL)):
        ox, oy = -math.sin(yaw) * off, math.cos(yaw) * off
        w = 0.5 if col is RUT else 0.6
        road.add("box", (L, w, 0.02), (cx + ox, cy + oy, 0.045, 0, 0, yaw), col, shadows=False)
    x = x2
# Infart från vägen upp till gläntan
ay0 = road_y(ACCESS_X)
alen = -6 - ay0
road.add("box", (4.0, alen, 0.04), (ACCESS_X, ay0 + alen / 2, 0.021, 0, 0, 0), DIRT, shadows=False)
for off in (-0.9, 0.9):
    road.add("box", (0.45, alen, 0.02), (ACCESS_X + off, ay0 + alen / 2, 0.046, 0, 0, 0), RUT, shadows=False)
# Uppställningsplats för fordon i gläntan (grus)
road.add("box", (16, 10, 0.04), (14, -14, 0.022, 0, 0, 0), GRAVEL, shadows=False)
models.append(road)

# ----------------------------------------------------------------------------
# Skog: gran, tall och björk. En modell per ~sektor håller antalet entiteter nere.
# ----------------------------------------------------------------------------


def tree_spot_ok(x, y, placed, min_d):
    r = math.hypot(x, y)
    if r < CLEARING_R + rng.uniform(-3, 4) or r > WORLD_R:
        return False
    if dist_to_road(x, y) < ROAD_W / 2 + 2.5 or in_access(x, y, 2.0):
        return False
    for px, py in placed:
        if (px - x) ** 2 + (py - y) ** 2 < min_d * min_d:
            return False
    return True


def add_spruce(m, x, y, s):
    h = rng.uniform(14, 22) * s
    m.add("cyl", (0.22 * s, h * 0.35), (x, y, h * 0.175, 0, 0, 0), SPRUCE_BARK, collide=True)
    # Granens krona: staplade koner, bredast nedtill
    tiers = 5
    base_z = h * 0.12
    crown_h = h - base_z
    for i in range(tiers):
        t = i / tiers
        rad = (2.6 - 2.0 * t) * s * rng.uniform(0.9, 1.1)
        th = crown_h * 0.38
        z = base_z + crown_h * t * 0.85 + th / 2
        col = SPRUCE if i % 2 else SPRUCE_DARK
        m.add("cone", (rad, th), (x, y, z, 0, 0, 0), col, collide=(i == 0))


def add_pine(m, x, y, s):
    h = rng.uniform(16, 24) * s
    trunk_h = h * 0.72
    lean = rng.uniform(-0.04, 0.04)
    m.add("cyl", (0.2 * s, trunk_h), (x, y, trunk_h / 2, lean, 0, 0), PINE_BARK, collide=True)
    # Tallens krona: några oregelbundna "moln" högt upp
    for _ in range(3):
        m.add(
            "ell",
            (rng.uniform(1.8, 2.8) * s, rng.uniform(1.8, 2.8) * s, rng.uniform(1.0, 1.6) * s),
            (x + rng.uniform(-1, 1), y + rng.uniform(-1, 1), trunk_h + rng.uniform(0, 3) * s, 0, 0, 0),
            PINE_CROWN,
        )


def add_birch(m, x, y, s):
    h = rng.uniform(10, 15) * s
    m.add("cyl", (0.13 * s, h * 0.8), (x, y, h * 0.4, 0, 0, 0), BIRCH_BARK, collide=True)
    # Svarta fläckar på stammen
    for i in range(4):
        z = h * (0.1 + 0.15 * i)
        m.add("cyl", (0.135 * s, 0.08), (x, y, z, 0, 0, 0), FM_BLACK)
    m.add("ell", (2.0 * s, 2.0 * s, 3.2 * s), (x, y, h * 0.75, 0, 0, 0), BIRCH_LEAF)


N_SECTORS = 8
forests = [Model(f"forest_{i}") for i in range(N_SECTORS)]
placed = []
target = 420
tries = 0
while len(placed) < target and tries < 40000:
    tries += 1
    # Tätare skog nära gläntan (sämplas i r med viss bias)
    r = CLEARING_R + (WORLD_R - CLEARING_R) * math.sqrt(rng.random())
    a = rng.uniform(0, 2 * math.pi)
    x, y = r * math.cos(a), r * math.sin(a)
    if not tree_spot_ok(x, y, placed, 4.2):
        continue
    placed.append((x, y))
    sector = int(((a % (2 * math.pi)) / (2 * math.pi)) * N_SECTORS) % N_SECTORS
    m = forests[sector]
    s = rng.uniform(0.8, 1.15)
    roll = rng.random()
    # Björk vid vägkanten och gläntan, annars mest gran och tall
    edge = dist_to_road(x, y) < 12 or r < CLEARING_R + 8
    if roll < (0.35 if edge else 0.12):
        add_birch(m, x, y, s)
    elif roll < 0.62:
        add_spruce(m, x, y, s)
    else:
        add_pine(m, x, y, s)
models.extend(forests)

# ----------------------------------------------------------------------------
# Markvegetation: blåbärsris, granitblock, fallna stockar
# ----------------------------------------------------------------------------
under = Model("undergrowth")
for i in range(260):
    r = rng.uniform(CLEARING_R - 4, WORLD_R)
    a = rng.uniform(0, 2 * math.pi)
    x, y = r * math.cos(a), r * math.sin(a)
    if dist_to_road(x, y) < ROAD_W / 2 + 0.8 or in_access(x, y):
        continue
    under.add(
        "ell",
        (rng.uniform(0.6, 1.4), rng.uniform(0.6, 1.4), rng.uniform(0.2, 0.35)),
        (x, y, 0.05, 0, 0, rng.uniform(0, 3.14)),
        BLUEBERRY,
        shadows=False,
    )
models.append(under)

rocks = Model("granite_boulders")
for i in range(45):
    r = rng.uniform(CLEARING_R - 6, WORLD_R)
    a = rng.uniform(0, 2 * math.pi)
    x, y = r * math.cos(a), r * math.sin(a)
    if dist_to_road(x, y) < ROAD_W / 2 + 1.5 or in_access(x, y, 1.5):
        continue
    sz = rng.uniform(0.6, 2.2)
    col = GRANITE if rng.random() < 0.6 else GRANITE_LICHEN
    rocks.add(
        "ell",
        (sz * rng.uniform(1.0, 1.6), sz * rng.uniform(0.8, 1.3), sz * rng.uniform(0.5, 0.8)),
        (x, y, sz * 0.2, rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), rng.uniform(0, 3.14)),
        col,
        collide=True,
    )
# Ett stort flyttblock i gläntans kant
rocks.add("ell", (3.2, 2.4, 1.8), (-24, -12, 0.6, 0, 0.05, 0.4), GRANITE, collide=True)
models.append(rocks)

logs = Model("fallen_logs")
for i in range(18):
    r = rng.uniform(CLEARING_R + 2, WORLD_R)
    a = rng.uniform(0, 2 * math.pi)
    x, y = r * math.cos(a), r * math.sin(a)
    if dist_to_road(x, y) < ROAD_W / 2 + 2 or in_access(x, y, 2):
        continue
    L = rng.uniform(4, 9)
    logs.add("cyl", (rng.uniform(0.15, 0.3), L), (x, y, 0.22, 0, math.pi / 2, rng.uniform(0, 3.14)), SPRUCE_BARK, collide=True)
models.append(logs)

# ----------------------------------------------------------------------------
# Helipad (drönarens start, origo)
# ----------------------------------------------------------------------------
pad = Model("helipad", (0, 0, 0, 0, 0, 0))
pad.add("box", (4, 4, 0.1), (0, 0, 0.05, 0, 0, 0), HELIPAD, collide=True)
pad.add("box", (0.3, 1.8, 0.01), (-0.5, 0, 0.105, 0, 0, 0), WHITE, shadows=False)
pad.add("box", (0.3, 1.8, 0.01), (0.5, 0, 0.105, 0, 0, 0), WHITE, shadows=False)
pad.add("box", (0.8, 0.3, 0.01), (0, 0, 0.105, 0, 0, 0), WHITE, shadows=False)
models.append(pad)

# ----------------------------------------------------------------------------
# Tält 20 (fem stycken): tolvkantigt pyramidtält med mittstång, låga väggar,
# ingång och kaminrör.
# ----------------------------------------------------------------------------
TENTS = [(-14, 10, 0.3), (-4, 17, -0.2), (7, 16, 0.5), (-18, -2, 1.2), (17, 6, 2.4)]
for i, (tx, ty, yaw) in enumerate(TENTS, 1):
    t = Model(f"tent_{i}", (tx, ty, 0, 0, 0, yaw))
    R = 3.0
    wall_h = 0.6
    roof_h = 2.6
    t.add("cyl", (R, wall_h), (0, 0, wall_h / 2, 0, 0, 0), TENT_DARK, collide=True)
    t.add("cone", (R + 0.15, roof_h), (0, 0, wall_h + roof_h / 2, 0, 0, 0), TENT, collide=True)
    # Topp och kaminrör
    t.add("cyl", (0.05, 0.4), (0, 0, wall_h + roof_h + 0.1, 0, 0, 0), WOOD)
    t.add("cyl", (0.07, 1.4), (1.0, 0.6, wall_h + roof_h - 0.2, 0, 0, 0), STEEL)
    t.add("cyl", (0.1, 0.12), (1.0, 0.6, wall_h + roof_h + 0.5, 0, 0, 0), STEEL)
    # Ingång (uppslagen tältduk) mot lägrets mitt (+x i tältets ram)
    t.add("box", (0.12, 1.1, 1.4), (R - 0.05, 0, 0.7, 0, -0.35, 0), FM_BLACK)
    t.add("box", (0.05, 0.6, 1.5), (R + 0.15, 0.65, 0.75, 0, -0.25, 0.4), TENT_DARK)
    # Tältlinor med pinnar
    for k in range(6):
        a = k * math.pi / 3 + math.pi / 6
        gx, gy = math.cos(a) * (R + 1.4), math.sin(a) * (R + 1.4)
        ex, ey = math.cos(a) * R, math.sin(a) * R
        L = math.hypot(math.hypot(gx - ex, gy - ey), wall_h)
        pitch = math.atan2(math.hypot(gx - ex, gy - ey), wall_h)
        t.add("cyl", (0.01, L), ((gx + ex) / 2, (gy + ey) / 2, wall_h / 2, 0, pitch, a + math.pi), (0.6, 0.6, 0.5, 1), shadows=False)
        t.add("cyl", (0.03, 0.25), (gx, gy, 0.1, 0, 0, 0), WOOD, shadows=False)
    models.append(t)

# ----------------------------------------------------------------------------
# Lägerutrustning
# ----------------------------------------------------------------------------
camp = Model("camp_gear")
# Eldstad med stenring och sittstockar
for k in range(10):
    a = k * 2 * math.pi / 10
    camp.add("ell", (0.25, 0.2, 0.15), (-3 + 0.9 * math.cos(a), 6 + 0.9 * math.sin(a), 0.08, 0, 0, a), GRANITE)
camp.add("cyl", (0.75, 0.03), (-3, 6, 0.015, 0, 0, 0), ASH, shadows=False)
for k in range(3):
    a = k * 2.1
    camp.add("cyl", (0.08, 0.9), (-3, 6, 0.2, 0.6 * math.cos(a), 0.6 * math.sin(a), 0), WOOD)
for (lx, ly, lyaw) in ((-3, 8.3, 0), (-5.3, 6, 1.57), (-0.7, 6.3, 1.4)):
    camp.add("cyl", (0.2, 2.4), (lx, ly, 0.2, 0, math.pi / 2, lyaw), SPRUCE_BARK, collide=True)
# Vedtrave
for row in range(3):
    for k in range(6 - row):
        camp.add("cyl", (0.1, 0.6), (-10 + k * 0.2 + row * 0.1, 18.5, 0.1 + row * 0.18, math.pi / 2, 0, 0), WOOD)
# Flaggstång med svensk flagga
camp.add("cyl", (0.05, 8), (0, 9, 4, 0, 0, 0), WHITE, collide=True)
camp.add("sphere", (0.08,), (0, 9, 8.05, 0, 0, 0), SE_YELLOW)
FX, FZ = 0.05 + 0.8, 7.2
camp.add("box", (1.6, 0.02, 1.0), (FX, 9, FZ, 0, 0, 0), SE_BLUE)
camp.add("box", (1.62, 0.03, 0.2), (FX, 9, FZ, 0, 0, 0), SE_YELLOW)
camp.add("box", (0.2, 0.03, 1.02), (FX - 0.2, 9, FZ, 0, 0, 0), SE_YELLOW)
# Vapenställ / lådor och drivmedelsdunkar vid fordonsplatsen
for k in range(4):
    camp.add("box", (1.2, 0.6, 0.5), (8 + k * 1.3, -19, 0.25, 0, 0, 0), FM_GREEN, collide=True)
camp.add("box", (1.2, 0.6, 0.5), (8.6, -19, 0.75, 0, 0, 0.1), FM_GREEN, collide=True)
for k in range(6):
    camp.add("box", (0.17, 0.35, 0.47), (20 + k * 0.22, -18.5, 0.235, 0, 0, 0), FM_DARK, collide=True)
# Kamouflagenät på stolpar över ett stabsbord
for (px, py) in ((-12, -8), (-6, -8), (-12, -3), (-6, -3)):
    camp.add("cyl", (0.04, 2.5), (px, py, 1.25, 0, 0, 0), WOOD)
for k in range(12):
    col = (FM_GREEN, FM_BROWN, FM_DARK)[k % 3]
    camp.add(
        "box",
        (rng.uniform(1.5, 2.5), rng.uniform(1.5, 2.5), 0.02),
        (-9 + rng.uniform(-2.5, 2.5), -5.5 + rng.uniform(-2, 2), 2.5 + rng.uniform(-0.05, 0.05), 0, 0, rng.uniform(0, 3.14)),
        col,
    )
camp.add("box", (2.0, 0.9, 0.05), (-9, -5.5, 0.75, 0, 0, 0), WOOD, collide=True)
for (lx, ly) in ((-9.9, -5.9), (-8.1, -5.9), (-9.9, -5.1), (-8.1, -5.1)):
    camp.add("cyl", (0.03, 0.75), (lx, ly, 0.375, 0, 0, 0), STEEL)
camp.add("box", (0.5, 0.35, 0.3), (-9.4, -5.5, 0.93, 0, 0, 0.2), FM_DARK)  # radio
camp.add("cyl", (0.01, 2.5), (-9.25, -5.4, 2.2, 0, 0, 0), FM_BLACK)  # antenn
models.append(camp)

# ----------------------------------------------------------------------------
# Fordon
# ----------------------------------------------------------------------------


def camo_patches(m, cx, cy, z, sx, sy, n):
    """Oregelbundna fläckar i M90-liknande färger ovanpå en yta."""
    for _ in range(n):
        m.add(
            "box",
            (rng.uniform(0.3, 0.9), rng.uniform(0.15, 0.4), 0.01),
            (cx + rng.uniform(-sx, sx), cy + rng.uniform(-sy, sy), z, 0, 0, rng.uniform(0, 3.14)),
            FM_BROWN if rng.random() < 0.5 else FM_DARK,
            shadows=False,
        )


def track_unit(m, x0, length, width, gauge):
    """Ett bandpar (höger/vänster) centrerat på x0. Returnerar bandets topphöjd."""
    r = 0.33
    for side in (-1, 1):
        y = side * gauge / 2
        m.add("box", (length - 2 * r, width, 2 * r), (x0, y, r, 0, 0, 0), RUBBER, collide=True)
        for ex in (-1, 1):
            m.add("cyl", (r, width), (x0 + ex * (length / 2 - r), y, r, math.pi / 2, 0, 0), RUBBER, collide=True)
        # Bärhjul synliga på utsidan
        for k in range(4):
            wx = x0 - length / 2 + r + 0.2 + k * (length - 2 * r - 0.4) / 3
            m.add("cyl", (0.22, 0.04), (wx, y + side * (width / 2 + 0.01), 0.3, math.pi / 2, 0, 0), STEEL)
        # Stålgaller/klackar
        for k in range(10):
            gx = x0 - length / 2 + r + k * (length - 2 * r) / 9
            m.add("box", (0.06, width + 0.02, 0.02), (gx, y, 2 * r + 0.005, 0, 0, 0), STEEL, shadows=False)
    return 2 * r


def build_bv206(name, p):
    """Bandvagn 206: två bandgående vagnar sammankopplade med styrled."""
    m = Model(name, p)
    L1, L2, W, gauge = 3.3, 3.0, 0.62, 1.2
    body_w = 1.85
    # Främre vagn (hytt + motor), centrerad på x = +2.0
    fx = 2.0
    top = track_unit(m, fx, L1, W, gauge)
    m.add("box", (L1 - 0.2, body_w, 0.35), (fx, 0, top + 0.1, 0, 0, 0), FM_GREEN, collide=True)
    # Motorhuv sluttar framåt
    m.add("box", (0.95, body_w - 0.05, 0.45), (fx + 1.0, 0, top + 0.47, 0, 0.25, 0), FM_GREEN, collide=True)
    m.add("box", (0.4, body_w - 0.1, 0.35), (fx + 1.55, 0, top + 0.2, 0, -0.5, 0), FM_GREEN)
    # Hytt
    cab_h = 1.05
    cab_z = top + 0.27 + cab_h / 2
    m.add("box", (1.9, body_w, cab_h), (fx - 0.45, 0, cab_z, 0, 0, 0), FM_GREEN, collide=True)
    m.add("box", (1.95, body_w + 0.02, 0.05), (fx - 0.45, 0, cab_z + cab_h / 2 + 0.02, 0, 0, 0), FM_DARK)
    # Framruta (lutad) och sidorutor
    m.add("box", (0.05, body_w - 0.2, 0.5), (fx + 0.52, 0, cab_z + 0.18, 0, -0.3, 0), GLASS)
    for side in (-1, 1):
        m.add("box", (0.7, 0.02, 0.4), (fx - 0.05, side * (body_w / 2 + 0.005), cab_z + 0.2, 0, 0, 0), GLASS)
        m.add("box", (0.6, 0.02, 0.4), (fx - 0.95, side * (body_w / 2 + 0.005), cab_z + 0.2, 0, 0, 0), GLASS)
    # Strålkastare, takräcke, antenn
    for side in (-1, 1):
        m.add("cyl", (0.09, 0.06), (fx + 1.5, side * 0.65, top + 0.55, 0, math.pi / 2, 0), (0.8, 0.8, 0.7, 1))
    m.add("box", (1.5, 1.5, 0.04), (fx - 0.5, 0, cab_z + cab_h / 2 + 0.12, 0, 0, 0), FM_BLACK)
    m.add("cyl", (0.01, 2.2), (fx - 1.2, 0.8, cab_z + cab_h / 2 + 1.1, 0, 0, 0), FM_BLACK)
    camo_patches(m, fx - 0.45, 0, cab_z + cab_h / 2 + 0.05, 0.8, 0.7, 10)

    # Styrled mellan vagnarna
    m.add("box", (0.9, 0.35, 0.3), (fx - L1 / 2 - 0.3, 0, top + 0.1, 0, 0, 0), FM_BLACK)
    m.add("cyl", (0.2, 0.3), (fx - L1 / 2 - 0.3, 0, top + 0.1, 0, 0, 0), STEEL)

    # Bakre vagn (trupputrymme), centrerad på x = -2.0
    rx = -2.0
    track_unit(m, rx, L2, W, gauge)
    m.add("box", (L2 - 0.1, body_w, 0.35), (rx, 0, top + 0.1, 0, 0, 0), FM_GREEN, collide=True)
    rear_h = 1.2
    rz = top + 0.27 + rear_h / 2
    m.add("box", (L2 - 0.3, body_w, rear_h), (rx - 0.05, 0, rz, 0, 0, 0), FM_GREEN, collide=True)
    m.add("box", (L2 - 0.25, body_w + 0.02, 0.05), (rx - 0.05, 0, rz + rear_h / 2 + 0.02, 0, 0, 0), FM_DARK)
    for side in (-1, 1):
        for k in range(2):
            m.add("box", (0.55, 0.02, 0.35), (rx + 0.6 - k * 0.9, side * (body_w / 2 + 0.005), rz + 0.25, 0, 0, 0), GLASS)
    # Bakdörrar
    m.add("box", (0.02, 0.75, 0.9), (rx - L2 / 2 + 0.09, -0.42, rz - 0.05, 0, 0, 0), FM_DARK)
    m.add("box", (0.02, 0.75, 0.9), (rx - L2 / 2 + 0.09, 0.42, rz - 0.05, 0, 0, 0), FM_DARK)
    camo_patches(m, rx, 0, rz + rear_h / 2 + 0.05, 1.2, 0.7, 12)
    models.append(m)


def build_tgb40(name, p):
    """Terrängbil 40 (Scania SBA111): 4x4 med frambyggd hytt och kapellflak."""
    m = Model(name, p)
    wr, ww = 0.6, 0.45
    track = 2.1
    axles = (2.35, -1.6)
    # Hjul
    for ax in axles:
        for side in (-1, 1):
            m.add("cyl", (wr, ww), (ax, side * track / 2, wr, math.pi / 2, 0, 0), RUBBER, collide=True)
            m.add("cyl", (0.3, ww + 0.02), (ax, side * track / 2, wr, math.pi / 2, 0, 0), FM_DARK)
            # Mönster (klackar) på däcket
            for k in range(8):
                a = k * math.pi / 4
                m.add(
                    "box",
                    (0.12, ww + 0.02, 0.05),
                    (ax + math.cos(a) * wr, side * track / 2, wr + math.sin(a) * wr, 0, -a, 0),
                    RUBBER,
                    shadows=False,
                )
    # Ram och axlar
    m.add("box", (6.2, 0.9, 0.3), (0.3, 0, 1.0, 0, 0, 0), FM_BLACK, collide=True)
    for ax in axles:
        m.add("cyl", (0.12, track), (ax, 0, wr, math.pi / 2, 0, 0), FM_BLACK)
    # Stänkskärmar
    for ax in axles:
        for side in (-1, 1):
            m.add("box", (1.45, 0.55, 0.06), (ax, side * track / 2, 1.3, 0, 0, 0), FM_GREEN)
    # Frambyggd hytt
    cx, cw, ch = 2.35, 2.35, 1.55
    cz = 1.25 + ch / 2
    m.add("box", (1.9, cw, ch), (cx, 0, cz, 0, 0, 0), FM_GREEN, collide=True)
    m.add("box", (0.25, cw - 0.3, 0.45), (cx + 1.05, 0, 1.3, 0, 0, 0), FM_DARK)  # kylargrill
    m.add("box", (0.2, cw + 0.2, 0.2), (cx + 1.15, 0, 1.0, 0, 0, 0), FM_BLACK)   # stötfångare
    m.add("box", (0.05, cw - 0.2, 0.65), (cx + 0.96, 0, cz + 0.35, 0, -0.08, 0), GLASS)
    for side in (-1, 1):
        m.add("box", (0.8, 0.02, 0.55), (cx + 0.3, side * (cw / 2 + 0.005), cz + 0.3, 0, 0, 0), GLASS)
        m.add("box", (0.05, 0.3, 0.35), (cx + 1.0, side * (cw / 2 + 0.2), cz + 0.3, 0, 0, 0), FM_BLACK)  # backspegel
        m.add("cyl", (0.11, 0.08), (cx + 1.12, side * 0.9, 1.55, 0, math.pi / 2, 0), (0.8, 0.8, 0.7, 1))
    m.add("box", (1.95, cw + 0.02, 0.06), (cx, 0, cz + ch / 2 + 0.03, 0, 0, 0), FM_DARK)
    camo_patches(m, cx, 0, cz + ch / 2 + 0.07, 0.8, 1.0, 10)
    # Flak med kapell på bågar
    bx, bl, bw = -1.0, 4.3, 2.4
    m.add("box", (bl, bw, 0.15), (bx, 0, 1.25, 0, 0, 0), FM_GREEN, collide=True)
    for side in (-1, 1):
        m.add("box", (bl, 0.05, 0.5), (bx, side * bw / 2, 1.55, 0, 0, 0), FM_GREEN)
    m.add("box", (0.05, bw, 0.5), (bx - bl / 2, 0, 1.55, 0, 0, 0), FM_GREEN)
    # Kapell: box + halvcylinder-tak
    m.add("box", (bl - 0.05, bw - 0.05, 1.0), (bx, 0, 2.3, 0, 0, 0), CANVAS, collide=True)
    m.add("ell", ((bl - 0.05) / 2, (bw - 0.05) / 2, 0.35), (bx, 0, 2.8, 0, 0, 0), CANVAS)
    for k in range(4):
        m.add("box", (0.06, bw, 0.02), (bx - bl / 2 + 0.4 + k * (bl - 0.8) / 3, 0, 3.1, 0, 0, 0), TENT_DARK, shadows=False)
    m.add("box", (0.02, bw - 0.3, 0.9), (bx - bl / 2 - 0.01, 0, 2.25, 0, 0, 0), TENT_DARK)
    # Reservhjul bakom hytten, avgasrör
    m.add("cyl", (0.55, 0.4), (cx - 1.15, 0, 2.4, 0, math.pi / 2, 0), RUBBER)
    m.add("cyl", (0.06, 1.8), (cx - 1.0, 1.05, 2.4, 0, 0, 0), STEEL)
    models.append(m)


# Bv 206 på uppställningsplatsen, Tgb 40 bredvid, en till Bv 206 ute på vägen
build_bv206("bv206_1", (10, -13, 0, 0, 0, 1.45))
build_tgb40("tgb40_1", (17, -14, 0, 0, 0, 1.6))
_rx = -40.0
_ryaw = math.atan2(road_y(_rx + 1) - road_y(_rx - 1), 2)
build_bv206("bv206_2", (_rx, road_y(_rx) + 1.1, 0, 0, 0, _ryaw))

# ----------------------------------------------------------------------------
# Skriv SDF
# ----------------------------------------------------------------------------
HEADER = """<?xml version="1.0" ?>
<!--
  GENERERAD FIL - redigera inte för hand.
  Källa: worlds/generate_hive_base.py  (kör: python3 worlds/generate_hive_base.py)

  Militärt tältläger i svensk barrskog: fem tält 20, grusväg, Bv 206 och Tgb 40.
  Byggd enbart av SDF-primitiver (kräver Gazebo Harmonic), ingen nedladdning behövs.
-->
<sdf version="1.11">
  <world name="hive_base">
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

    <!-- Låg nordisk sol -->
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

OUT.write_text(HEADER + "".join(m.sdf() for m in models) + FOOTER)
print(f"Wrote {OUT} ({len(placed)} trees, {sum(m.n for m in models)} parts)")
