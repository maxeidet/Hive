"""docker/drone_collisions.py: friendly and hostile drones in collision groups that pass
through each other. Host only: `python3 -m pytest docker/test_drone_collisions.py`."""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drone_collisions as dc  # noqa: E402

BASE = """<?xml version="1.0"?>
<sdf version="1.9"><model name="x500_base">
  <link name="base_link">
    <visual name="v"><geometry><mesh><uri>model://x500_base/meshes/body.dae</uri></mesh></geometry></visual>
    <collision name="c0"><geometry><box><size>1 1 1</size></box></geometry></collision>
    <collision name="c1"><geometry><box><size>1 1 1</size></box></geometry>
      <surface><friction><ode><mu>0.5</mu></ode></friction></surface></collision>
  </link>
  <link name="rotor_0"><collision name="r"><geometry><cylinder><radius>0.1</radius><length>0.01</length></cylinder></geometry>
    <surface><contact><collide_bitmask>65535</collide_bitmask></contact></surface></collision></link>
</model></sdf>"""
X500 = """<sdf version="1.9"><model name="x500">
  <include merge="true"><uri>model://x500_base</uri></include>
  <plugin filename="gz-sim-multicopter-motor-model-system" name="m"/>
</model></sdf>"""
GIMBAL = """<sdf version="1.9"><model name="gimbal"><link name="g">
  <collision name="g"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></sdf>"""


def bitmasks(text):
    return [int(c.find('surface/contact/collide_bitmask').text)
            for c in ET.fromstring(text).iter('collision')]


def models(tmp_path):
    for name, sdf in (('x500_base', BASE), ('x500', X500), ('gimbal', GIMBAL)):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'model.sdf').write_text(sdf)
        (tmp_path / name / 'model.config').write_text(f'<model><name>{name}</name></model>')
    return tmp_path


def test_groups_pass_through_each_other_but_not_the_ground():
    default = 0xffff  # the ground, trees, tents: not changed
    assert dc.FRIENDLY_BITMASK & dc.HOSTILE_BITMASK == 0
    assert dc.FRIENDLY_BITMASK & default and dc.HOSTILE_BITMASK & default
    assert dc.FRIENDLY_BITMASK & dc.FRIENDLY_BITMASK  # defenders still collide with each other


def test_every_collision_gets_the_bitmask_and_keeps_its_surface():
    text, n = dc.set_bitmask(BASE, 7)
    assert n == 3 and bitmasks(text) == [7, 7, 7]
    friction = ET.fromstring(text).find(".//collision[@name='c1']/surface/friction/ode/mu")
    assert friction is not None and friction.text == '0.5'


def test_main_puts_friendlies_and_the_hostile_in_their_groups(tmp_path):
    m = models(tmp_path)
    assert dc.main([str(m)]) == 0
    assert set(bitmasks((m / 'x500_base' / 'model.sdf').read_text())) == {dc.FRIENDLY_BITMASK}
    assert set(bitmasks((m / 'gimbal' / 'model.sdf').read_text())) == {dc.FRIENDLY_BITMASK}
    hostile_base = (m / 'x500_hostile_base' / 'model.sdf').read_text()
    assert set(bitmasks(hostile_base)) == {dc.HOSTILE_BITMASK}
    assert ET.fromstring(hostile_base).find('model').get('name') == 'x500_hostile_base'
    # the same meshes as the x500, so it looks the same to the cameras and YOLO
    assert 'model://x500_base/meshes/body.dae' in hostile_base
    hostile = ET.fromstring((m / 'x500_hostile' / 'model.sdf').read_text())
    assert hostile.find('model').get('name') == 'x500_hostile'
    assert [u.text for u in hostile.iter('uri')] == ['model://x500_hostile_base']
    assert hostile.find('model/plugin') is not None  # the motors are still there
    assert '<name>x500_hostile</name>' in (m / 'x500_hostile' / 'model.config').read_text()


def test_running_twice_gives_the_same(tmp_path):
    m = models(tmp_path)
    dc.main([str(m)])
    first = {p: p.read_text() for p in m.rglob('*') if p.is_file()}
    dc.main([str(m)])
    assert {p: p.read_text() for p in m.rglob('*') if p.is_file()} == first
