"""The collection of drones: which drones exist, and broadcasting to them."""
from collections.abc import Callable, Iterable, Iterator

from swarm_backend.px4.drone import Drone


class Swarm:

    def __init__(self, drones: Iterable[Drone]):
        self._drones = {drone.id: drone for drone in drones}

    def __iter__(self) -> Iterator[Drone]:
        return iter(self._drones.values())

    def __len__(self) -> int:
        return len(self._drones)

    def __getitem__(self, drone_id: int) -> Drone:
        return self._drones[drone_id]

    @property
    def ids(self) -> list[int]:
        return list(self._drones)

    def select(self, ids: Iterable[int] | None = None) -> list[Drone]:
        """The drones with these ids, or all of them."""
        return list(self) if ids is None else [self._drones[i] for i in ids]

    def all(self, predicate: Callable[[Drone], bool], ids: Iterable[int] | None = None) -> bool:
        return all(predicate(drone) for drone in self.select(ids))

    def arm_all(self, ids: Iterable[int] | None = None) -> None:
        for drone in self.select(ids):
            drone.arm()

    def start_offboard_all(self, ids: Iterable[int] | None = None) -> None:
        for drone in self.select(ids):
            drone.start_offboard()

    def hold_all(self, ids: Iterable[int] | None = None) -> None:
        for drone in self.select(ids):
            drone.hold()

    def land_all(self, ids: Iterable[int] | None = None) -> None:
        for drone in self.select(ids):
            drone.land()

    def disarm_all(self, ids: Iterable[int] | None = None) -> None:
        for drone in self.select(ids):
            drone.disarm()
