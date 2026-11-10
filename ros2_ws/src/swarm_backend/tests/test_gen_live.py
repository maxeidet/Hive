import math

import pytest

from swarm_backend import gen_data, gen_live, ground_truth
from swarm_backend.box_test import BIRD, DRONE


def truth_camera(heading_deg=30.0, pitch_deg=10.0, at=(5.0, -3.0, -40.0)):
    forward, right, down = gen_data.axes(math.radians(heading_deg), math.radians(pitch_deg))
    return {'position': at, 'forward': forward, 'right': right, 'down': down}


def test_camera_from_truth():
    cam = gen_live.camera_from_truth(truth_camera())
    assert (cam['north'], cam['east'], cam['up']) == (5.0, -3.0, 40.0)
    assert cam['axes'][0] == truth_camera()['forward']


def test_projected_box_lands_on_the_placed_pixel_with_the_right_size():
    tc = truth_camera()
    cam = gen_live.camera_from_truth(tc)
    at = gen_data.place(cam, 700.0, 300.0, 20.0)
    # drone placed with its box centre (0.17 m above its origin) lifted out: place the
    # origin 0.17 m lower so the box centre is on the pixel
    at = (at[0], at[1], at[2] - 0.17)
    # facing the same way as the camera, so it is seen square on
    (box,) = gen_live.projected_boxes(tc, [{'label': DRONE, 'size': 14.0, 'at': at,
                                            'heading': math.radians(30.0), 'pitch': 0.0,
                                            'roll': 0.0}])
    assert (box['u'], box['v']) == pytest.approx((700.0, 300.0), abs=1.0)
    f = gen_data.F
    assert box['width'] == pytest.approx(f * 0.66 / 20.0, rel=0.25)
    assert box['label'] == DRONE


def test_projected_boxes_skip_what_is_out_of_view():
    tc = truth_camera(heading_deg=0.0, pitch_deg=0.0, at=(0.0, 0.0, -40.0))
    behind = (-20.0, 0.0, 40.0)
    assert gen_live.projected_boxes(tc, [{'label': BIRD, 'size': 5.0, 'at': behind,
                                          'heading': 0.0, 'pitch': 0.0, 'roll': 0.0}]) == []


def test_box_topics_pick_the_drone_and_skip_images():
    topics = ['/world/w/model/x500_gimbal_labels_2/link/label_camera_link/sensor/boxes_visible/boundingbox_camera',
              '/world/w/model/x500_gimbal_labels_2/link/label_camera_link/sensor/boxes_visible/boundingbox_camera/image',
              '/world/w/model/x500_gimbal_labels_2/link/label_camera_link/sensor/boxes_full/boundingbox_camera',
              '/world/w/model/x500_gimbal_labels_3/link/label_camera_link/sensor/boxes_full/boundingbox_camera']
    visible, full = gen_live.box_topics(topics, 'x500_gimbal_labels_2')
    assert visible == topics[0] and full == topics[2]
    assert gen_live.box_topics(topics, 'x500_gimbal_2') == (None, None)


def test_schedule_shares_images_over_heights_in_order():
    assert gen_live.schedule(10, [20, 50, 80]) == [20] * 4 + [50] * 3 + [80] * 3
    assert len(gen_live.schedule(7, [25])) == 7


def test_half_size_matches_box_test():
    assert 2 * ground_truth.HALF_SIZE['drone'][0] == pytest.approx(0.66, abs=0.05)
