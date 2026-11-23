import pytest

from swarm_backend.datagen import make_crops
from swarm_backend.detect.detectors import Detection


def test_read_labels_gives_pixel_corners(tmp_path):
    (tmp_path / 'a.txt').write_text('0 0.5 0.5 0.01 0.02\n')
    assert make_crops.read_labels(tmp_path / 'a.txt', 1000, 500) == [
        (0, pytest.approx(495.0), pytest.approx(245.0), pytest.approx(505.0), pytest.approx(255.0))]


def test_window_is_centred_and_stays_inside():
    assert make_crops.window(500, 300, 64, 1280, 720) == (468, 268, 64)
    assert make_crops.window(10, 710, 64, 1280, 720) == (0, 656, 64)  # corner: moved in
    assert make_crops.window(640, 360, 2000, 1280, 720)[2] == 720  # never wider than the image


def test_scaled_cut_outs_follow_the_candidate_size_with_a_minimum():
    assert make_crops.side_for(2, 'scaled') == 12          # minimum
    assert make_crops.side_for(4, 'scaled') == 12          # 3x = the minimum
    assert make_crops.side_for(20, 'scaled') == 60         # 3x
    assert make_crops.side_for(20, 'scaled', context=4) == 80
    assert make_crops.side_for(3, 'scaled', min_crop=16) == 16


def test_fixed_cut_outs_stay_the_same_unless_the_candidate_is_big():
    assert make_crops.side_for(4, 'fixed') == 64
    assert make_crops.side_for(20, 'fixed') == 64
    assert make_crops.side_for(50, 'fixed') == 100
    assert make_crops.side_for(4, 'fixed', crop=96) == 96


def test_crop_labels_move_and_scale_the_box_and_drop_mostly_outside_ones():
    boxes = [(0, 495, 295, 505, 305), (1, 530, 290, 560, 300)]  # second: mostly outside
    lines = make_crops.crop_labels(boxes, 468, 268, 64)
    assert len(lines) == 1
    c, u, v, w, h = (float(x) for x in lines[0].split())
    assert (c, u, v, w, h) == pytest.approx((0, 32 / 64, 32 / 64, 10 / 64, 10 / 64))


def test_found_by_uses_centre_distance():
    box = (0, 98, 98, 102, 102)  # 4 px drone at (100, 100)
    near = Detection(101, 100, 3, 3, 10.0)   # centre (102.5, 101.5): 2.9 px away
    far = Detection(110, 100, 3, 3, 10.0)
    assert make_crops.found_by(box, [near])
    assert not make_crops.found_by(box, [far])


def test_dedupe_keeps_the_strongest_of_close_candidates():
    a, b, c = Detection(100, 100, 4, 4, 5.0), Detection(105, 100, 4, 4, 9.0), Detection(300, 100, 4, 4, 1.0)
    assert make_crops.dedupe([a, b, c], 16) == [b, c]


def test_split_yaml_keeps_the_runs_split(tmp_path):
    src = tmp_path / 'data.yaml'
    src.write_text('# Written by gen_data.py. Split by run.\npath: /workspace/data/gen\n'
                   'train:\n  - r01/images\n  - r04/images\nval:\n  - r02/images\n'
                   'test:\n  - r03/images\nnames:\n  0: drone\n  1: bird\n')
    text = make_crops.split_yaml(src, tmp_path / 'crops', ['r01', 'r02', 'r03'])
    assert f'path: {tmp_path / "crops"}' in text
    assert 'r04' not in text  # not cut
    assert text.split('val:')[1].split('test:')[0].strip() == '- r02/images'
    assert text.count('# Written') == 1


def test_cut_through_finds_objects_only_partly_inside():
    box = (0, 100, 100, 140, 120)  # 40 x 20
    assert make_crops.cut_through([box], 125, 90, 40)          # a quarter inside
    assert not make_crops.cut_through([box], 90, 90, 64)       # all inside
    assert not make_crops.cut_through([box], 300, 300, 64)     # not touched
