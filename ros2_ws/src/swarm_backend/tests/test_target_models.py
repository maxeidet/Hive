import math
import random
import xml.etree.ElementTree as ET

import pytest

from swarm_backend.datagen import target_models as tm

SDF = '''<sdf version="1.9"><model name="toy">
  <link name="base_link">
    <visual name="body"><pose>0 0 0.1 0 0 1.5708</pose>
      <geometry><mesh><uri>meshes/body.dae</uri><scale>0.001 0.001 0.001</scale></mesh></geometry>
      <material><diffuse>0 0 1 1</diffuse></material></visual>
    <sensor name="imu" type="imu"/>
  </link>
  <link name="rotor_0"><pose>0.2 -0.2 0.05 0 0 0</pose>
    <visual name="prop"><geometry><box><size>0.2 0.02 0.002</size></box></geometry></visual>
  </link>
  <joint name="j" type="revolute"><parent>base_link</parent><child>rotor_0</child></joint>
</model></sdf>'''


def test_visuals_move_link_poses_in_and_fix_relative_mesh_paths():
    parts = tm.visuals(SDF, 'toy')
    assert len(parts) == 2
    (body_pose, body_geom, body_mat), (prop_pose, _, prop_mat) = parts
    assert body_pose == pytest.approx((0, 0, 0.1, 0, 0, 1.5708), abs=1e-9)
    assert 'model://toy/meshes/body.dae' in body_geom
    assert body_mat is not None and prop_mat is None
    assert prop_pose[:3] == pytest.approx((0.2, -0.2, 0.05))


def test_compose_turns_the_child_with_the_parent():
    parent = (1.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2)  # facing +y
    child = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)            # 1 m ahead of it
    x, y, z, roll, pitch, yaw = tm.compose(parent, child)
    assert (x, y, z) == pytest.approx((1.0, 1.0, 0.0), abs=1e-9)
    assert yaw == pytest.approx(math.pi / 2)


def test_copy_is_static_look_only_labelled_and_can_be_repainted():
    parts = tm.visuals(SDF, 'toy')
    model = ET.fromstring(tm.target_sdf('t', parts, 1, 'white')).find('model')
    assert model.find('static').text == 'true'
    assert model.find('link/sensor') is None and model.find('joint') is None
    assert model.find('plugin/label').text == '1'
    diffuse = [v.find('material/diffuse').text for v in model.iter('visual')]
    assert diffuse == ['0.9 0.9 0.9 1'] * 2
    original = ET.fromstring(tm.target_sdf('t', parts, None)).find('model')
    assert original.find('plugin') is None
    assert [v.find('material') is not None for v in original.iter('visual')] == [True, False]


def test_pick_follows_kinds_and_recolour():
    rng = random.Random(0)
    picks = [tm.pick(rng, ['x500', 'standard_vtol'], 0.5) for _ in range(400)]
    assert {p.kind for p in picks} == {'x500', 'standard_vtol'}
    painted = sum(p.colour != tm.ORIGINAL for p in picks) / len(picks)
    assert 0.4 < painted < 0.6
    assert all(tm.pick(rng, ['px4vision'], 0.0).colour == tm.ORIGINAL for _ in range(20))
    assert tm.Variant('standard_vtol', 'red').width == 2.1
