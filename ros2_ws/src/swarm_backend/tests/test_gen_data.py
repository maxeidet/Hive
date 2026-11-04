import math
import random

import pytest

from swarm_backend import gen_data, ground_truth
from swarm_backend.box_test import BIRD, DRONE, F, HEIGHT_PX, WIDTH


@pytest.mark.parametrize('heading, pitch', [(0, 0), (90, 0), (200, 20), (45, -10)])
def test_quaternion_points_the_model_along_the_camera_axes(heading, pitch):
    h, p = math.radians(heading), math.radians(pitch)
    forward, right, down = gen_data.axes(h, p)
    q = gen_data.quaternion(h, p)
    assert ground_truth.enu_to_ned(ground_truth.rotate(q, (1.0, 0.0, 0.0))) == pytest.approx(forward, abs=1e-9)
    # Gazebo's camera: image right is the model's -y, image down its -z
    assert ground_truth.enu_to_ned(ground_truth.rotate(q, (0.0, -1.0, 0.0))) == pytest.approx(right, abs=1e-9)
    assert ground_truth.enu_to_ned(ground_truth.rotate(q, (0.0, 0.0, -1.0))) == pytest.approx(down, abs=1e-9)


def test_place_and_project_back_round_trip():
    camera = {'north': 10.0, 'east': -5.0, 'up': 50.0, 'heading': 1.0, 'pitch': 0.2}
    at = gen_data.place(camera, 900.0, 200.0, 40.0)
    assert math.dist(at, (10.0, -5.0, 50.0)) == pytest.approx(40.0)
    assert gen_data.project_back(camera, at) == pytest.approx((900.0, 200.0), abs=1e-6)


def test_targets_get_the_drawn_size_and_stay_in_view_above_ground():
    rng = random.Random(1)
    for _ in range(50):
        camera = gen_data.camera_pose(rng)
        for kind, size, at, *_ in filter(None, gen_data.targets(rng, camera, [DRONE, DRONE, BIRD])):
            distance = math.dist(at, (camera['north'], camera['east'], camera['up']))
            assert F * gen_data.TARGET_WIDTH[kind] / distance == pytest.approx(size, rel=1e-6)
            u, v = gen_data.project_back(camera, at)
            assert 0 < u < WIDTH and 0 < v < HEIGHT_PX
            assert at[2] >= gen_data.MIN_CLEARANCE


def test_pixel_sizes_cover_2_to_100_and_peak_near_4_and_20():
    rng = random.Random(0)
    sizes = [gen_data.pixel_size(rng) for _ in range(5000)]
    assert 2.0 <= min(sizes) and max(sizes) <= 100.0
    near4 = sum(3 <= s <= 5 for s in sizes) / len(sizes)
    near20 = sum(16 <= s <= 24 for s in sizes) / len(sizes)
    far = sum(50 <= s <= 100 for s in sizes) / len(sizes)
    assert near4 > 0.2 and near20 > 0.15 and 0.02 < far < 0.15


def test_sun_is_dimmer_and_warmer_near_the_horizon():
    rng = random.Random(3)
    suns = [gen_data.sun(rng) for _ in range(200)]
    low = min(suns, key=lambda s: s['elevation_deg'])
    high = max(suns, key=lambda s: s['elevation_deg'])
    assert sum(low['diffuse']) < sum(high['diffuse'])
    assert low['diffuse'][2] / low['diffuse'][0] < high['diffuse'][2] / high['diffuse'][0]
    assert all(s['direction'][2] < 0 for s in suns)  # light travels down


def box(label, u, v, w, h):
    return {'label': label, 'u': u, 'v': v, 'width': w, 'height': h}


def test_objects_label_visible_boxes_and_ignore_tiny_or_hidden_ones():
    visible = [box(DRONE, 100, 100, 10, 6), box(BIRD, 500, 300, 1, 1)]
    full = [box(DRONE, 100, 101, 12, 8), box(BIRD, 500, 300, 3, 2), box(DRONE, 900, 50, 20, 10)]
    lines, rows = gen_data.objects(visible, full, min_px=2.0)
    assert lines == [f'0 {100 / WIDTH:.6f} {100 / HEIGHT_PX:.6f} {10 / WIDTH:.6f} {6 / HEIGHT_PX:.6f}']
    assert [r['labelled'] for r in rows] == [True, False, False]  # tiny bird, hidden drone
    assert rows[0]['visibility'] == pytest.approx(60 / 96, abs=1e-3)
    assert rows[2]['visibility'] == 0.0


def test_match_finds_the_placed_model_behind_each_box():
    camera = {'north': 0.0, 'east': 0.0, 'up': 50.0, 'heading': 0.0, 'pitch': 0.0}
    a, b = gen_data.place(camera, 300, 300, 20.0), gen_data.place(camera, 1000, 400, 60.0)
    rows = [{'class': 'drone', 'full': box(DRONE, 1001, 399, 4, 3)},
            {'class': 'drone', 'full': box(DRONE, 301, 300, 12, 8)}]
    gen_data.match(rows, camera, [(DRONE, 12.3, a), (DRONE, 4.1, b)])
    assert (rows[0]['distance'], rows[0]['drawn_px']) == (60.0, 4.1)
    assert (rows[1]['distance'], rows[1]['drawn_px']) == (20.0, 12.3)


def test_data_yaml_splits_by_run(tmp_path):
    text = gen_data.data_yaml(tmp_path, ['r01', 'r02', 'r03', 'r04'], ['r03'], ['r04'])
    train = text.split('train:')[1].split('val:')[0]
    assert 'r01/images' in train and 'r02/images' in train and 'r03' not in train
    assert '  - r03/images' in text.split('val:')[1].split('test:')[0]
    assert "0: drone" in text and "1: bird" in text


def test_histogram_bins():
    bins = dict(gen_data.size_histogram([1.0, 3.0, 5.0, 20.0, 200.0]))
    assert bins['<2 px'] == 1 and bins['2-4 px'] == 1 and bins['4-8 px'] == 1
    assert bins['16-32 px'] == 1 and bins['128+ px'] == 1


def test_own_body_puts_its_camera_mount_on_the_camera():
    rng = random.Random(5)
    for _ in range(20):
        camera = gen_data.camera_pose(rng)
        body = gen_data.own_body(rng, camera)
        position, q = gen_data.body_pose(camera, body)
        mount = tuple(p + o for p, o in zip(position, ground_truth.rotate(q, gen_data.CAMERA_ON_BODY)))
        assert mount == pytest.approx(gen_data.enu((camera['north'], camera['east'], -camera['up'])))
        # the body's nose is within 60 degrees of where the camera looks
        diff = (camera['heading'] - body['heading'] + math.pi) % (2 * math.pi) - math.pi
        assert abs(diff) <= math.radians(60) + 1e-9


def test_own_body_has_no_label():
    import xml.etree.ElementTree as ET
    from swarm_backend.box_test import drone_sdf
    assert ET.fromstring(drone_sdf('me', label=None)).find('model/plugin') is None


def test_target_names_are_not_taken_for_px4_drones():
    from swarm_backend.sensor_view import MODEL_TOPIC
    for name in gen_data.target_names('gen_drone', 5):
        assert not MODEL_TOPIC.match(f'/world/w/model/{name}/link/base_link/sensor/imu_sensor/imu')


def test_targets_use_the_given_widths_and_stay_in_step():
    rng = random.Random(2)
    camera = {'north': 0.0, 'east': 0.0, 'up': 60.0, 'heading': 0.0, 'pitch': 0.3}
    spots = gen_data.targets(rng, camera, [DRONE, DRONE], widths=[2.1, 0.47])
    assert len(spots) == 2
    for (kind, size, at, *_), width in zip(spots, [2.1, 0.47]):
        distance = math.dist(at, (0.0, 0.0, 60.0))
        assert F * width / distance == pytest.approx(size, rel=1e-6)
