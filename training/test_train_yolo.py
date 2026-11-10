from pathlib import Path

import train_yolo

YAML = '''# Written by gen_data.py. Split by run.
path: /workspace/data/gen
train:
  - r01/images
val:
  - r02/images
test:
  - r03/images
names:
  0: drone
  1: bird
'''


def test_local_yaml_points_at_its_own_folder(tmp_path):
    data = tmp_path / 'data.yaml'
    data.write_text(YAML)
    local = train_yolo.local_yaml(data, tmp_path / 'local.yaml')
    assert f'path: {tmp_path.resolve()}' in local.read_text()
    assert train_yolo.split_dirs(local, 'test') == [tmp_path.resolve() / 'r03/images']
    assert train_yolo.split_dirs(local, 'train') == [tmp_path.resolve() / 'r01/images']


def test_read_labels_in_pixels(tmp_path):
    label = tmp_path / 'a.txt'
    label.write_text('0 0.5 0.25 0.01 0.02\n')
    assert train_yolo.read_labels(label, 1000, 500) == [(0, 500.0, 125.0, 10.0, 10.0)]
    assert train_yolo.read_labels(tmp_path / 'missing.txt', 1000, 500) == []


def test_match_by_centre_and_class():
    truth = [(0, 100, 100, 4, 4), (0, 500, 300, 30, 20), (1, 800, 50, 10, 10)]
    found = [(0, 102, 101, 5, 5, 0.9),   # 2.2 px off a 4 px drone: within 3 px
             (0, 520, 300, 30, 20, 0.8),  # 20 px off a 30 px drone: too far (limit 15)
             (0, 800, 50, 10, 10, 0.7)]   # right place, wrong class
    hit, false_alarms = train_yolo.match(truth, found)
    assert hit == [True, False, False]
    assert false_alarms == 2


def test_each_detection_finds_one_object_most_confident_first():
    truth = [(0, 100, 100, 6, 6)]
    hit, false_alarms = train_yolo.match(truth, [(0, 100, 100, 6, 6, 0.3), (0, 101, 100, 6, 6, 0.9)])
    assert hit == [True] and false_alarms == 1


def test_size_bins():
    assert train_yolo.size_bin(3.9) == '2-4 px'
    assert train_yolo.size_bin(20) == '16-32 px'
    assert train_yolo.size_bin(1.5) is None
