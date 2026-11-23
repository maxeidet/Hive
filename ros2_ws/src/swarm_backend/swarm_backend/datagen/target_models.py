"""Target drones of several kinds and colours for the training-data generators.

The generators (gen_data.py, gen_live.py) place static, labelled drones in front of the
camera. With only PX4's black x500 the model could learn "x500" instead of "small drone",
so this builds look-only copies of several of PX4's drone models, optionally repainted:

    x500            PX4's x500 quadcopter (0.66 m across with propellers)
    px4vision       Holybro PX4 Vision: small quadcopter with a boxy body (~0.47 m)
    omniquad        quadcopter with tilted arms (~0.6 m)
    quadtailsitter  tailsitter VTOL: a flying wing with four rotors (~0.7 m)
    standard_vtol   fixed-wing VTOL plane (~2.1 m wingspan)

A copy has every <visual> of the original (meshes, primitives and their materials), moved
into one static link, and nothing else: no sensors, plugins, joints or physics, so it costs
no simulation time and gets no Gazebo topics. With a colour, every visual gets that plain
colour instead of its own material. The label (the Label system) makes Gazebo's
bounding-box cameras box it. The meshes are read from PX4's model folders through
Gazebo's resource path, so this runs in the simulation container.
"""
import math
import random
import xml.etree.ElementTree as ET
from dataclasses import dataclass

Pose = tuple[float, float, float, float, float, float]  # x y z roll pitch yaw, as in SDF


@dataclass(frozen=True)
class Kind:
    model: str  # PX4 model folder with the visuals
    width: float  # metres across (propellers or wings): turns drawn pixels into distance
    weight: float  # how often it is picked


KINDS = {
    'x500': Kind('x500_base', 0.66, 0.35),
    'px4vision': Kind('px4vision', 0.47, 0.20),
    'omniquad': Kind('omniquad_base', 0.60, 0.15),
    'quadtailsitter': Kind('quadtailsitter', 0.70, 0.15),
    'standard_vtol': Kind('standard_vtol', 2.10, 0.15),
}
# Plain colours (RGB 0-1) a copy can be repainted with. Real small drones are mostly black,
# grey or white; a few brighter ones keep the model from learning "dark = drone".
COLOURS = {
    'black': (0.05, 0.05, 0.05), 'dark_grey': (0.25, 0.25, 0.25), 'grey': (0.5, 0.5, 0.5),
    'white': (0.9, 0.9, 0.9), 'red': (0.6, 0.08, 0.06), 'olive': (0.33, 0.36, 0.2),
}
ORIGINAL = 'original'


@dataclass(frozen=True)
class Variant:
    kind: str
    colour: str  # a COLOURS key, or ORIGINAL for the model's own materials

    @property
    def width(self) -> float:
        return KINDS[self.kind].width

    @property
    def name(self) -> str:
        return f'{self.kind}/{self.colour}'


# ----- poses -----

def quat_from_rpy(roll: float, pitch: float, yaw: float):
    """(x, y, z, w) of SDF's fixed-axis roll, pitch, yaw."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def rpy_from_quat(q) -> tuple[float, float, float]:
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def compose(parent: Pose, child: Pose) -> Pose:
    """The child pose (given in the parent's frame) in the parent's parent frame."""
    from swarm_backend.sim.ground_truth import multiply, rotate
    qp, qc = quat_from_rpy(*parent[3:]), quat_from_rpy(*child[3:])
    offset = rotate(qp, child[:3])
    position = tuple(p + o for p, o in zip(parent[:3], offset))
    return (*position, *rpy_from_quat(multiply(qp, qc)))


def parse_pose(text: str | None) -> Pose:
    values = [float(v) for v in (text or '').split()]
    return tuple(values + [0.0] * (6 - len(values)))[:6] if values else (0.0,) * 6


# ----- building a copy -----

def visuals(sdf_text: str, model: str) -> list[tuple[Pose, str, str | None]]:
    """Every visual of a model's SDF as (pose in the model frame, <geometry> XML,
    <material> XML or None). Relative mesh paths become model://<model>/..."""
    root = ET.fromstring(sdf_text)
    model_el = root.find('model') if root.tag == 'sdf' else root
    out = []
    for link in model_el.findall('link'):
        link_pose = parse_pose(link.findtext('pose'))
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            if geometry is None:
                continue
            for uri in geometry.iter('uri'):
                if uri.text and '://' not in uri.text and not uri.text.startswith('/'):
                    uri.text = f'model://{model}/{uri.text.strip()}'
            material = visual.find('material')
            out.append((compose(link_pose, parse_pose(visual.findtext('pose'))),
                        ET.tostring(geometry, encoding='unicode').strip(),
                        None if material is None else
                        ET.tostring(material, encoding='unicode').strip()))
    return out


def paint(rgb: tuple[float, float, float]) -> str:
    r, g, b = rgb
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            '<specular>0.2 0.2 0.2 1</specular></material>')


def target_sdf(name: str, parts: list[tuple[Pose, str, str | None]], label: int | None,
               colour: str = ORIGINAL) -> str:
    """A static model with only these visuals, repainted unless colour is ORIGINAL, and
    the Label system with `label` (None: no label)."""
    body = []
    for i, (pose, geometry, material) in enumerate(parts):
        mat = (material or '') if colour == ORIGINAL else paint(COLOURS[colour])
        body.append(f'<visual name="v{i}"><pose>{" ".join(f"{p:.6g}" for p in pose)}</pose>'
                    f'{geometry}{mat}</visual>')
    plugin = '' if label is None else (
        '<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
        f'<label>{label}</label></plugin>')
    return (f'<?xml version="1.0"?>\n<sdf version="1.9"><model name="{name}">'
            f'<static>true</static><link name="body">{"".join(body)}</link>{plugin}'
            '</model></sdf>')


def pick(rng: random.Random, kinds: list[str], recolour: float) -> Variant:
    """A random variant: kind by KINDS weight (among `kinds`), repainted with a random
    colour with probability `recolour`, else its own materials."""
    kind = rng.choices(kinds, weights=[KINDS[k].weight for k in kinds])[0]
    colour = rng.choice(list(COLOURS)) if rng.random() < recolour else ORIGINAL
    return Variant(kind, colour)


class Library:
    """Reads each PX4 model's visuals once (from Gazebo's resource path)."""

    def __init__(self):
        self._parts: dict[str, list] = {}

    def parts(self, kind: str) -> list:
        if kind not in self._parts:
            from swarm_backend.datagen.box_test import find_model_sdf
            model = KINDS[kind].model
            path = find_model_sdf(model)
            if path is None:
                raise FileNotFoundError(f'{model}/model.sdf is not on GZ_SIM_RESOURCE_PATH')
            self._parts[kind] = visuals(path.read_text(), model)
        return self._parts[kind]

    def sdf(self, name: str, variant: Variant, label: int | None) -> str:
        return target_sdf(name, self.parts(variant.kind), label, variant.colour)
