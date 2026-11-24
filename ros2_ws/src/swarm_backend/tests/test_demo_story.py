from pathlib import Path

import pytest

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.story import (LAYOUTS, StoryError, load_story, parse_moment, pick_take,
                                      resolve)
from swarm_backend.demo.takes import find_takes

DEFAULT = Path(__file__).resolve().parents[4] / 'recording' / 'stories' / 'default.yaml'


def write(tmp_path, text):
    p = tmp_path / 'story.yaml'
    p.write_text(text)
    return p


def test_moment_expressions():
    assert parse_moment('hit-2.5') == ('hit', -2.5)
    assert parse_moment('takeoff +12') == ('takeoff', 12.0)
    assert parse_moment(' first_sighting - 5 ') == ('first_sighting', -5.0)
    assert parse_moment('hit') == ('hit', 0.0)
    assert parse_moment('release + 0.5') == ('release', 0.5)
    with pytest.raises(StoryError, match='first_sighting'):
        parse_moment('hti - 2')


def test_default_story_loads():
    story = load_story(DEFAULT)
    assert story.title and story.take == 'best'
    for s in story.scenes:
        assert s.card or len(s.show) == LAYOUTS[s.layout]


def test_wrong_source_count(tmp_path):
    p = write(tmp_path, 'scenes:\n- caption: X\n  from: hit\n  to: hit + 1\n  layout: grid\n  show: [map, chase, yolo]\n')
    with pytest.raises(StoryError, match='scene 1'):
        load_story(p)


def test_unknown_key_and_source(tmp_path):
    with pytest.raises(StoryError, match='layuot'):
        load_story(write(tmp_path, 'scenes:\n- caption: X\n  from: hit\n  to: hit\n  layuot: full\n  show: [map]\n'))
    with pytest.raises(StoryError, match='drone_cam'):
        load_story(write(tmp_path, 'scenes:\n- caption: X\n  from: hit\n  to: hit + 1\n  layout: full\n  show: [drone_cam]\n'))


STORY = '''take: best
scenes:
- caption: Drone {attacker} closes in
  from: first_sighting
  to: hit
  layout: side
  show: [yolo, chase]
- caption: The hit
  from: hit - 2.5
  to: hit + 0.3
  layout: full
  show: [map]
- card: Next
- caption: Wide
  from: release
  to: release + 5
  layout: full
  show: ["cam:base_tripod"]
'''


def test_resolve_ranges(tmp_path):
    make_run(tmp_path, takes=1, videos=False)
    takes = find_takes(tmp_path)
    scenes, problems = resolve(load_story(write(tmp_path, STORY)), takes)
    assert problems == []
    hit = takes[0].moments['hit']
    assert (scenes[1].start, scenes[1].end) == pytest.approx((hit - 2.5, hit + 0.3))
    assert scenes[0].sources == {'yolo': 'debug_d3', 'chase': 'cam_attacker_watch'}
    assert scenes[0].scene.caption == 'Drone 3 closes in'
    assert scenes[2].scene.card == 'Next' and scenes[2].take is None
    assert scenes[3].sources == {'cam:base_tripod': 'cam_base_tripod'}


def test_missing_moment_names_scene(tmp_path):
    make_run(tmp_path, takes=1, hit=[False], videos=False)
    _, problems = resolve(load_story(write(tmp_path, STORY)), find_takes(tmp_path))
    assert len(problems) == 2
    assert "scene 1 'Drone 3 closes in'" in problems[0] and "'hit'" in problems[0]
    assert "scene 2 'The hit'" in problems[1]


def test_best_take(tmp_path):
    make_run(tmp_path, takes=2, hit=[False, True], videos=False)
    takes = find_takes(tmp_path)
    assert pick_take('best', takes).number == 2
    assert pick_take('1', takes).number == 1
    with pytest.raises(StoryError, match='take 5'):
        pick_take('5', takes)
