import json

import pytest

from swarm_backend.datagen import yolo_labels


def truth(*objects):
    return {'camera': {'width': 1000, 'height': 500}, 'objects': list(objects)}


def obj(name, kind, box, distance=10.0):
    return {'name': name, 'class': kind, 'box': box, 'distance': distance}


def test_line_is_class_and_box_as_fractions():
    box = {'x0': 100.0, 'y0': 200.0, 'x1': 120.0, 'y1': 210.0, 'in_image': 1.0}
    lines, ignored = yolo_labels.yolo_lines(truth(obj('bird_1', 'bird', box)))
    assert ignored == []
    cls, u, v, w, h = lines[0].split()
    assert cls == '1'
    assert [float(x) for x in (u, v, w, h)] == pytest.approx([0.11, 0.41, 0.02, 0.02])


def test_tiny_or_mostly_cut_objects_are_ignored_and_out_of_view_ones_skipped():
    tiny = {'x0': 10.0, 'y0': 10.0, 'x1': 11.5, 'y1': 11.0, 'in_image': 1.0}
    cut = {'x0': 0.0, 'y0': 10.0, 'x1': 10.0, 'y1': 20.0, 'in_image': 0.2}
    lines, ignored = yolo_labels.yolo_lines(truth(obj('d1', 'drone', tiny), obj('d2', 'drone', cut),
                                                  obj('d3', 'drone', None)))
    assert lines == []
    assert [i['name'] for i in ignored] == ['d1', 'd2']


def test_write_labels_makes_one_file_per_image_and_skips_old_json(tmp_path):
    box = {'x0': 100.0, 'y0': 200.0, 'x1': 120.0, 'y1': 210.0, 'in_image': 1.0}
    (tmp_path / 'a.json').write_text(json.dumps(truth(obj('d', 'drone', box))))
    (tmp_path / 'b.json').write_text(json.dumps(truth()))  # empty sky
    (tmp_path / 'old.json').write_text(json.dumps({'camera': {}, 'drones': []}))
    for name in ('a', 'b', 'old'):
        (tmp_path / f'{name}.png').write_bytes(b'')
    assert yolo_labels.write_labels(tmp_path) == (2, 1, 0)
    assert (tmp_path / 'labels' / 'a.txt').read_text().startswith('0 ')
    assert (tmp_path / 'labels' / 'b.txt').read_text() == ''
    assert not (tmp_path / 'labels' / 'old.txt').exists()
