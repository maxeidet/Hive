import xml.etree.ElementTree as ET

import pytest

from swarm_backend.datagen import box_test


def test_camera_has_rgb_and_both_box_sensors_with_the_gimbal_settings():
    root = ET.fromstring(box_test.camera_sdf('cam'))
    sensors = {s.get('name'): s for s in root.iter('sensor')}
    assert {n: s.get('type') for n, s in sensors.items()} == {
        'rgb': 'camera', 'visible': 'boundingbox_camera', 'full': 'boundingbox_camera'}
    assert sensors['visible'].find('camera/box_type').text == '2d'
    assert sensors['full'].find('camera/box_type').text == 'full_2d'
    for s in sensors.values():
        assert float(s.find('camera/horizontal_fov').text) == box_test.HFOV
        assert int(s.find('camera/image/width').text) == box_test.WIDTH


def test_drone_is_static_and_labelled():
    model = ET.fromstring(box_test.drone_sdf('d')).find('model')
    assert model.find('static').text == 'true'
    assert model.find('include/uri').text == 'x500_base'
    assert model.find('plugin/label').text == str(box_test.DRONE)


def test_expected_pixel_and_width():
    assert box_test.expected(40, (10.0, 0.0, 40.0), 0.5) == {
        'u': 640.0, 'v': 360.0, 'width': pytest.approx(box_test.F * 0.05, abs=0.1)}
    right = box_test.expected(40, (10.0, 1.0, 41.0), 0.5)
    assert right['u'] > 640 and right['v'] < 360  # east is right, higher is up
    assert box_test.expected(40, (-5.0, 0.0, 40.0), 0.5) is None


def test_steps_keep_both_targets_in_view_and_start_empty():
    steps = box_test.steps(40.0)
    assert steps[0][0] == 'empty'
    for name, drone, bird in steps[1:]:
        for pos in (drone, bird):
            e = box_test.expected(40.0, pos, 0.5)
            assert 0 <= e['u'] < box_test.WIDTH and 0 <= e['v'] < box_test.HEIGHT_PX / 2


def test_camera_settings_reads_clip_and_noise():
    sdf = ('<sdf><model><link><sensor name="cam" type="camera"><camera>'
           '<clip><near>0.1</near><far>100</far></clip>'
           '<noise><type>gaussian</type><stddev>0.007</stddev></noise>'
           '</camera></sensor></link></model></sdf>')
    (cam,) = box_test.camera_settings(sdf)
    assert (cam['clip_far'], cam['noise'], cam['noise_stddev']) == ('100', 'gaussian', '0.007')
    assert cam['anti_aliasing'] is None


def test_included_models_reads_model_uris_only():
    sdf = ('<sdf><model><include><uri>model://x500</uri></include>'
           '<include><uri>gimbal</uri></include><link><visual><geometry><mesh>'
           '<uri>/workspace/worlds/birds/dove.glb</uri></mesh></geometry></visual></link>'
           '</model></sdf>')
    assert box_test.included_models(sdf) == ['x500', 'gimbal']


def test_anti_aliasing_only_on_the_rgb_camera_and_only_when_asked():
    plain = ET.fromstring(box_test.camera_sdf('cam'))
    assert all(s.find('camera/image/anti_aliasing') is None for s in plain.iter('sensor'))
    root = ET.fromstring(box_test.camera_sdf('cam', anti_aliasing=8))
    found = {s.get('name'): s.find('camera/image/anti_aliasing') for s in root.iter('sensor')}
    assert found['rgb'].text == '8' and found['visible'] is None and found['full'] is None


def test_box_dict_counts_the_last_pixel():
    from types import SimpleNamespace as NS
    msg = NS(label=1, box=NS(min_corner=NS(x=100.0, y=50.0), max_corner=NS(x=103.0, y=51.0)))
    box = box_test.box_dict(msg)
    assert (box['width'], box['height']) == (4.0, 2.0)  # pixels 100..103 and 50..51
    assert (box['u'], box['v']) == (102.0, 51.0)
