import pytest

from swarm_backend.px4.drone import Drone
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm


class FakeDrone(Drone):
    """Records commands and 'flies' instantly, so swarm logic can be tested without PX4."""

    def __init__(self, drone_id, home=(0.0, 0.0, 0.0)):
        super().__init__(drone_id, home)
        self.position = home
        self.connected = self.preflight_ok = True
        self.commands = []

    def arm(self):
        self.commands.append('arm')
        self.armed = True

    def disarm(self):
        self.commands.append('disarm')
        self.armed = False

    def start_offboard(self):
        self.commands.append('offboard')
        self.offboard = True

    def land(self):
        self.commands.append('land')
        self.armed = False

    def set_position(self, x, y, z, yaw=None, velocity=None):
        self.target = self.position = (x, y, z)

    def set_velocity(self, vx, vy, vz):
        self.target = None


def make_manager():
    drones = [FakeDrone(0), FakeDrone(1, (0.0, 5.0, 0.0)), FakeDrone(2, (5.0, 0.0, 0.0))]
    return SwarmManager(Swarm(drones)), drones


def test_takeoff_arms_every_drone_and_climbs_above_home():
    manager, drones = make_manager()
    assert manager.ready()
    manager.takeoff(5.0)
    assert manager.all_armed() and manager.all_offboard() and manager.all_reached()
    assert drones[1].target == (0.0, 5.0, -5.0)
    assert drones[1].altitude == 5.0


def test_takeoff_selected_drones_only():
    manager, drones = make_manager()
    manager.takeoff(5.0, ids=[2])
    assert drones[2].armed and not drones[0].armed


def test_move_all_accepts_list_or_mapping():
    manager, drones = make_manager()
    manager.move_all([(1, 1, -5), (2, 2, -5), (3, 3, -5)])
    assert drones[2].target == (3, 3, -5)
    manager.move_all({0: (9, 9, -5)})
    assert drones[0].target == (9, 9, -5)
    with pytest.raises(ValueError):
        manager.move_all([(0, 0, 0)])


def test_move_to_formation_sends_each_drone_to_nearest_slot():
    manager, drones = make_manager()
    manager.move_to_formation([(5.0, 0.0, -5.0), (0.0, 0.0, -5.0), (0.0, 5.0, -5.0)])
    assert [d.target for d in drones] == [(0.0, 0.0, -5.0), (0.0, 5.0, -5.0), (5.0, 0.0, -5.0)]


def test_emergency_land_lands_everyone():
    manager, drones = make_manager()
    manager.takeoff(5.0)
    manager.emergency_land()
    assert manager.all_landed() and all(d.commands[-1] == 'land' for d in drones)
