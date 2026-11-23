#!/usr/bin/env python3
"""Friendly drones and the hostile drone pass through each other in Gazebo; both still hit
the ground, the trees and the camp. Run inside the container by docker/drone-models.sh.

    python3 drone_collisions.py MODELS_DIR

Why: a hit is decided from the true positions at 1.5 m (scenarios/surveillance.py), not
from contact, but a defender on a collision course reaches the hostile's body within a
tenth of a second after that, faster than the hostile can be taken out of the world. The
bodies then collided and the defender fell (crash cause 'intercept').

How: Gazebo only lets two collision shapes touch when their collide_bitmasks share a bit.
Everything not changed here keeps the default, 0xffff (every bit):
  - every collision shape of x500_base and gimbal, which every PX4 x500 model and so
    every defender is built from: FRIENDLY_BITMASK
  - x500_hostile, generated here as a copy of x500 (+ x500_base as x500_hostile_base, so
    it looks the same to the cameras and YOLO): HOSTILE_BITMASK
Friendly & hostile = 0: they pass through each other. Either & 0xffff (the ground, trees,
tents, birds) != 0: they collide. Friendlies still collide with each other.

Generated from the models installed in the image, so it follows PX4's x500 if that
changes. Running it again gives the same result.
"""
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

FRIENDLY_BITMASK = 1
HOSTILE_BITMASK = 2
FRIENDLY_MODELS = ('x500_base', 'gimbal')  # every defender model includes these
HOSTILE = 'x500_hostile'
HOSTILE_BASE = 'x500_hostile_base'


def set_bitmask(sdf_text: str, bitmask: int) -> tuple[str, int]:
    """`sdf_text` with every <collision>'s surface/contact/collide_bitmask set to
    `bitmask`, and how many collisions there were."""
    root = ET.fromstring(sdf_text)
    collisions = root.findall('.//collision')
    for collision in collisions:
        surface = collision.find('surface')
        if surface is None:
            surface = ET.SubElement(collision, 'surface')
        contact = surface.find('contact')
        if contact is None:
            contact = ET.SubElement(surface, 'contact')
        element = contact.find('collide_bitmask')
        if element is None:
            element = ET.SubElement(contact, 'collide_bitmask')
        element.text = str(bitmask)
    return ET.tostring(root, encoding='unicode'), len(collisions)


def rename_model(sdf_text: str, old: str, new: str) -> str:
    """The model's name and its includes of model://old_base become new ones."""
    root = ET.fromstring(sdf_text)
    model = root.find('model')
    if model is not None and model.get('name') == old:
        model.set('name', new)
    for uri in root.iter('uri'):
        if uri.text and uri.text.strip() == 'model://x500_base':
            uri.text = f'model://{HOSTILE_BASE}'
    return ET.tostring(root, encoding='unicode')


def copy_model(models: Path, source: str, target: str) -> Path:
    """models/source copied to models/target, with its model.config named target."""
    dest = models / target
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(models / source, dest)
    config = dest / 'model.config'
    if config.exists():
        text = config.read_text()
        config.write_text(re.sub(r'<name>[^<]*</name>', f'<name>{target}</name>', text, count=1))
    return dest


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__.split('\n\n')[1])
        return 2
    models = Path(argv[0])
    for name in FRIENDLY_MODELS:
        path = models / name / 'model.sdf'
        text, n = set_bitmask(path.read_text(), FRIENDLY_BITMASK)
        path.write_text(text)
        print(f'{name}: {n} collisions in the friendly group ({FRIENDLY_BITMASK})')

    base = copy_model(models, 'x500_base', HOSTILE_BASE) / 'model.sdf'
    text, n = set_bitmask(rename_model(base.read_text(), 'x500_base', HOSTILE_BASE), HOSTILE_BITMASK)
    base.write_text(text)
    hostile = copy_model(models, 'x500', HOSTILE) / 'model.sdf'
    hostile.write_text(rename_model(hostile.read_text(), 'x500', HOSTILE))
    print(f'{HOSTILE}: x500 with {n} collisions in the hostile group ({HOSTILE_BITMASK})')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
