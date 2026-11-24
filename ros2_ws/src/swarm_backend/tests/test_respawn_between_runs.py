"""Between runs, crashed defenders are recorded in the run and respawned
(SurveillanceWorld.fly / note_crashes). The fast simulation has no crashes of its own, so
these worlds fake one; the Gazebo respawn itself needs a person and the simulation."""
from swarm_backend.config import load_config
from swarm_backend.evaluation.attack_scenario import Inbound, inbound_path
from swarm_backend.swarm.base_station import Mode
from swarm_backend.scenarios import common
from swarm_backend.scenarios.crash import Crash
from swarm_backend.scenarios.sim_surveillance import SimSurveillance
from swarm_backend.scenarios.surveillance import Options

SURVEILLANCE = load_config(common.PKG / common.SURVEILLANCE_CONFIG)


class CrashingWorld(SimSurveillance):
    """Fakes a crash: `crash_hitter` crashes the drone that hits, right after the hit;
    `crash_at` = (drone, seconds after the run starts) crashes one mid-run. A respawn just
    clears the crash. `fail_respawns` respawns fail first."""

    def __init__(self, crash_hitter=False, crash_at=None, fail_respawns=0):
        super().__init__(SURVEILLANCE, Options(detector='fake'), quiet=True)
        self.crash_hitter, self.crash_at, self.fail_respawns = crash_hitter, crash_at, fail_respawns
        self._crashes: dict[int, Crash] = {}
        self.respawned: list[list[int]] = []
        self._run_start = None

    def observe_truth(self, stamp, bodies, hostile, cameras=None):
        super().observe_truth(stamp, bodies, hostile, cameras)
        judges = self.judges
        if judges is None:
            return
        if self._run_start is None:
            self._run_start = stamp
        if self.crash_hitter:
            for d, j in judges.items():
                if j.hit and d not in self._crashes:
                    self._crashes[d] = Crash(d, stamp + 0.5, 'tilted 170 deg')
        if self.crash_at is not None:
            d, after = self.crash_at
            if stamp - self._run_start >= after and d not in self._crashes:
                self._crashes[d] = Crash(d, stamp, 'down (0.4 m up)')

    def crashed(self):
        return dict(self._crashes)

    def respawn_defenders(self, drone_ids):
        self.respawned.append(list(drone_ids))
        if self.fail_respawns:
            self.fail_respawns -= 1
            raise TimeoutError('drone did not connect')
        for d in drone_ids:
            self._crashes.pop(d)

    def head_on(self):
        self._run_start = None
        scenario = Inbound(speed=2.0, start_m=self.surv.arc_radius + 100.0)
        path, _ = inbound_path(scenario, self.surv.base_position,
                               self.patrols[self.ids[0]].patrol.slot_point[2], self.heading)
        return self.fly(path.points[0], path.points[1], scenario.speed)


def world(**kwargs):
    w = CrashingWorld(**kwargs)
    w.setup()
    return w


def test_no_crash_no_respawn():
    w = world()
    flight = w.head_on()
    assert flight.crashes == [] and w.respawned == []


def test_the_hitter_crashing_with_the_hit_is_still_a_hit():
    w = world(crash_hitter=True)
    flight = w.head_on()
    assert flight.killer is not None  # scored a hit, as before
    assert flight.crashes == [{'drone': flight.killer, 't': flight.crashes[0]['t'],
                               'reason': 'tilted 170 deg', 'cause': 'intercept'}]
    assert flight.return_s is None  # it did not fly back, it was respawned
    assert w.respawned == [[flight.killer]]
    assert w.crashed() == {}


def test_another_crash_is_other_and_the_next_run_starts_with_everyone():
    w = world(crash_at=(1, 1.0))  # drone 1 crashes 1 s into the run
    flight = w.head_on()
    assert [(c['drone'], c['cause']) for c in flight.crashes] == [(1, 'other')]
    assert w.respawned == [[1]]
    # The base station kept or took drone 1 back: next run all three are on patrol.
    assert w.station.station.drones[1].mode is not Mode.LOST
    w.crash_at = None
    second = w.head_on()
    assert second.crashes == [] and w.respawned == [[1]]
    assert set(second.judges) == set(w.ids)  # every defender took part


def test_a_failed_respawn_is_tried_again_and_the_crash_recorded_once():
    w = world(crash_at=(1, 1.0), fail_respawns=1)
    first = w.head_on()
    assert [c['drone'] for c in first.crashes] == [1]
    assert w.crashed() == {1: w.crashed()[1]}  # still crashed after the failed respawn
    w.crash_at = None
    second = w.head_on()  # flown with two; then the respawn is tried again
    assert second.crashes == []  # the same crash is not recorded twice
    assert w.respawned == [[1], [1]] and w.crashed() == {}


# --- restart_on_crash: Gazebo is restarted instead (baseline_eval --restart-on-crash) ----

def test_restart_on_crash_records_the_crash_and_does_not_respawn():
    w = world(crash_hitter=True)
    w.restart_on_crash = True
    flight = w.head_on()
    assert flight.killer is not None
    assert [c['drone'] for c in flight.crashes] == [flight.killer]  # still in the saved run
    assert w.respawned == []  # no teleport, no new PX4
    assert w.restart_needed


def test_without_a_crash_no_restart_is_asked_for():
    w = world()
    w.restart_on_crash = True
    w.head_on()
    assert not getattr(w, 'restart_needed', False)


class RecordingWorld(CrashingWorld):
    """Records how the scenario ended: a landing, or a stop for a Gazebo restart (which
    the fast simulation, having no Gazebo, carries out as a landing)."""

    def __init__(self):
        super().__init__()
        self.calls = []

    def stop_and_land(self):
        self.calls.append('land')
        super().stop_and_land()

    def stop_for_restart(self):
        self.calls.append('stop_for_restart')
        super().stop_for_restart()


def test_a_restart_ends_through_stop_for_restart_with_exit_75():
    w = RecordingWorld()

    def body(world):
        raise common.RestartGazebo('one run per Gazebo start; 2 runs left')

    assert common._fly(w, body) == common.EXIT_RESTART == 75
    assert w.calls[0] == 'stop_for_restart'


def test_a_normal_end_still_lands():
    w = RecordingWorld()
    assert common._fly(w, lambda world: None) == 0
    assert w.calls == ['land']
