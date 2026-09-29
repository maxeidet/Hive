import pytest

from swarm_backend.config import parse_config

DATA = {
    'swarm': {'takeoff_altitude': 7.0},
    'drones': [
        {'id': 0, 'spawn_pose': [0, 0, 0]},
        {'id': 1, 'spawn_pose': [5, 2, 0]},
        {'id': 2, 'spawn_pose': [0, 5, 0], 'namespace': '/custom', 'system_id': 9},
    ],
}


def test_defaults_follow_px4_instance_numbering():
    config = parse_config(DATA)
    assert [d.namespace for d in config.drones] == ['', 'px4_1', 'custom']
    assert [d.system_id for d in config.drones] == [1, 2, 9]
    assert config.takeoff_altitude == 7.0
    assert config.rate_hz == 10.0  # default


def test_spawn_pose_enu_converts_to_ned():
    # Gazebo x=5 (east), y=2 (north) -> NED north=2, east=5
    assert parse_config(DATA).drone(1).spawn_ned == (2.0, 5.0, -0.0)


def test_num_drones_keeps_first_n():
    assert [d.id for d in parse_config(DATA, num_drones=2).drones] == [0, 1]
    with pytest.raises(ValueError):
        parse_config(DATA, num_drones=4)


def test_duplicate_ids_rejected():
    with pytest.raises(ValueError):
        parse_config({'drones': [{'id': 1}, {'id': 1}]})


def test_unknown_drone_id():
    with pytest.raises(KeyError):
        parse_config(DATA).drone(5)


def test_shipped_config_parses():
    from pathlib import Path
    from swarm_backend.config import load_config
    config = load_config(Path(__file__).parent.parent / 'config' / 'swarm.yaml')
    assert [d.id for d in config.drones] == [0, 1, 2]
