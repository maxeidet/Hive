"""Swarm-level commands: what the swarm should do.

Like Drone, every command here is a non-blocking request; the predicates at the
bottom tell a caller when the swarm got there.
"""
from collections.abc import Iterable, Mapping, Sequence

from swarm_backend.swarm.formation import Point, assign_targets
from swarm_backend.swarm.swarm import Swarm


class SwarmManager:

    def __init__(self, swarm: Swarm, tolerance: float = 0.3):
        self.swarm = swarm
        self.tolerance = tolerance

    # --- commands ----------------------------------------------------------

    def takeoff(self, altitude: float, ids: Iterable[int] | None = None) -> None:
        """Arm, switch to offboard and climb straight up to `altitude` metres."""
        for drone in self.swarm.select(ids):
            drone.takeoff(altitude)  # set the setpoint first so offboard has one to stream
            drone.arm()
            drone.start_offboard()

    def move_all(self, targets: Mapping[int, Point] | Sequence[Point]) -> None:
        """Send each drone to its target: {drone_id: point}, or a list in swarm order."""
        if not isinstance(targets, Mapping):
            if len(targets) != len(self.swarm):
                raise ValueError(f'{len(self.swarm)} drones but {len(targets)} targets')
            targets = dict(zip(self.swarm.ids, targets))
        for drone_id, (x, y, z) in targets.items():
            self.swarm[drone_id].set_position(x, y, z)

    def move_to_formation(self, points: Sequence[Point]) -> None:
        """Fly into formation, choosing which drone takes which slot so the total
        distance flown is smallest."""
        drones = list(self.swarm)
        positions = [d.position or d.home for d in drones]
        for drone, (x, y, z) in zip(drones, assign_targets(positions, points)):
            drone.set_position(x, y, z)

    def hold(self, ids: Iterable[int] | None = None) -> None:
        self.swarm.hold_all(ids)

    def land(self, ids: Iterable[int] | None = None) -> None:
        self.swarm.land_all(ids)

    def emergency_land(self) -> None:
        """Every drone lands where it is, now, overriding any pending command."""
        self.swarm.land_all()

    # --- state -------------------------------------------------------------

    def connected(self) -> bool:
        return self.swarm.all(lambda d: d.connected and d.position is not None)

    def not_connected(self) -> list[int]:
        return [d.id for d in self.swarm if not (d.connected and d.position is not None)]

    def ready(self) -> bool:
        return self.swarm.all(lambda d: d.ready)

    def all_armed(self) -> bool:
        return self.swarm.all(lambda d: d.armed)

    def all_offboard(self) -> bool:
        return self.swarm.all(lambda d: d.offboard)

    def all_reached(self, ids: Iterable[int] | None = None) -> bool:
        return self.swarm.all(lambda d: d.reached(self.tolerance), ids)

    def all_landed(self) -> bool:
        """Landed drones auto-disarm, so disarmed means down."""
        return self.swarm.all(lambda d: not d.armed)

    def any_armed(self) -> bool:
        return any(d.armed for d in self.swarm)
