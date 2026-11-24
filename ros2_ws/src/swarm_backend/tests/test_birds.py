import math
import xml.etree.ElementTree as ET

import pytest

from swarm_backend import birds


def test_world_frame_to_gazebo():
    assert birds.enu(10.0, 3.0, 25.0) == (3.0, 10.0, 25.0)  # north, east, up -> east, north, up


@pytest.mark.parametrize('heading_deg, facing', [(0, (0, 1)), (90, (1, 0)), (180, (0, -1))])
def test_heading_turns_the_bird_toward_it(heading_deg, facing):
    x, y, z, w = birds.yaw_quaternion(math.radians(heading_deg))
    yaw = 2 * math.atan2(z, w)  # Gazebo yaw, from east counter-clockwise
    assert (math.cos(yaw), math.sin(yaw)) == pytest.approx(facing, abs=1e-9)  # (east, north)


def test_sdf_is_a_static_mesh_without_collision():
    root = ET.fromstring(birds.bird_sdf('bird_7', birds.KINDS['dove']))
    model = root.find('model')
    assert model.get('name') == 'bird_7'
    assert model.find('static').text == 'true'
    assert model.find('link/collision') is None
    uri = model.find('link/visual/geometry/mesh/uri').text
    assert uri == '/workspace/worlds/birds/dove.glb'


@pytest.mark.parametrize('kind', list(birds.KINDS))
def test_every_kind_has_its_mesh_and_a_sensible_scale(kind):
    from pathlib import Path
    k = birds.KINDS[kind]
    assert (Path(__file__).resolve().parents[4] / 'worlds' / 'birds' / k.mesh).exists()
    assert 0 < k.scale < 1


def test_label_adds_the_label_system():
    model = ET.fromstring(birds.bird_sdf('b', birds.KINDS['dove'], label=2)).find('model')
    assert model.find('plugin').get('name') == 'gz::sim::systems::Label'
    assert model.find('plugin/label').text == '2'
    assert ET.fromstring(birds.bird_sdf('b', birds.KINDS['dove'])).find('model/plugin') is None


def test_flying_birds_are_the_ones_the_generators_use():
    assert set(birds.FLYING) == {'dove', 'seagull', 'gull', 'eagle', 'soaring_bird'}
    assert all(0.3 < birds.KINDS[k].span < 2.5 for k in birds.FLYING)


def test_flying_birds_are_dynamic_so_the_pose_log_sees_them():
    # Static models are not in Gazebo's dynamic_pose/info, which the pose log (posesrec) and
    # so the films' map and labels read: the scenarios' birds are kinematic, without gravity.
    model = ET.fromstring(birds.bird_sdf('bird_1', birds.KINDS['dove'], dynamic=True)).find('model')
    assert model.find('static').text == 'false'
    link = model.find('link')
    assert link.find('kinematic').text == 'true' and link.find('gravity').text == 'false'
    assert model.find('link/collision') is None
