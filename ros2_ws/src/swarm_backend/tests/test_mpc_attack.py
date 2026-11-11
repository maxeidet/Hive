"""The MPC behind the attack's face (intercept/mpc_attack.py), in a patrol and on its own,
and the scenarios' --setup (scenarios/common.py)."""
import argparse
from pathlib import Path

import pytest
from test_attack import SimDrone

from swarm_backend.config import AttackConfig, InterceptConfig, SurveillanceConfig
from swarm_backend.intercept.attack import DetectionInbox, State, attack_status, state_label
from swarm_backend.intercept.base_station import Mode, Order
from swarm_backend.intercept.mpc_attack import MpcAttack, make_attack
from swarm_backend.intercept.patrol import STALE_DETECTION_S, PatrolAction
from swarm_backend.scenarios import common
from swarm_backend.scenarios.surveillance import Options

STEP = 0.1


def run(step, drone, clock, seconds, inbox=None, message=None):
    for _ in range(round(seconds / STEP)):
        clock[0] = round(clock[0] + STEP, 6)
        if message is not None:
            inbox.put_position(message, clock[0], clock[0])
        drone.advance(STEP)
        step()


def mpc_patrol():
    clock = [0.0]
    drone = SimDrone(2, (0.0, 5.0, -30.0))
    inbox = DetectionInbox()
    attack = AttackConfig()

    def make(guard):
        return make_attack('mpc', drone, inbox, attack, InterceptConfig(), lambda: clock[0],
                           guard=guard, keep_recent_s=STALE_DETECTION_S)
    patrol = PatrolAction(drone, inbox, attack, SurveillanceConfig(), 1, lambda: clock[0],
                          make_attack=make)
    patrol.start()
    return patrol, drone, inbox, clock


def test_make_attack_gives_pursuit_or_mpc():
    drone, inbox = SimDrone(2, (0.0, 0.0, -30.0)), DetectionInbox()
    args = (drone, inbox, AttackConfig(), InterceptConfig(), lambda: 0.0)
    assert type(make_attack('pursuit', *args)).__name__ == 'AttackAction'
    assert isinstance(make_attack('mpc', *args), MpcAttack)
    with pytest.raises(ValueError):
        make_attack('ram', *args)


def test_a_patrol_flies_the_mpc_on_the_detection_that_made_it_engage():
    patrol, drone, inbox, clock = mpc_patrol()
    patrol.apply(Order(2, Mode.PATROL, sector=(-15.0, 15.0), may_start=True))
    run(patrol.step, drone, clock, 0.5, inbox, (20.0, 5.0, 0.0))
    assert patrol.mode is Mode.ENGAGE
    assert isinstance(patrol.attack, MpcAttack)
    assert patrol.attack.state is State.ATTACK
    assert drone.mode == 'velocity'


def test_an_mpc_that_never_sees_the_target_is_reported_lost():
    patrol, drone, inbox, clock = mpc_patrol()
    patrol.apply(Order(2, Mode.ENGAGE, 'T1'))
    run(patrol.step, drone, clock, 4.0)
    assert [e.kind for e in patrol.take_events()] == ['target_lost']


def test_a_lost_track_is_lost_and_the_ui_status_works():
    clock = [0.0]
    drone = SimDrone(2, (0.0, 0.0, -30.0))
    inbox = DetectionInbox()
    attack = MpcAttack(drone, inbox, AttackConfig(), InterceptConfig(), lambda: clock[0])
    attack.start()
    assert attack.state is State.SEARCH and state_label(attack) == 'search'
    run(attack.step, drone, clock, 1.0, inbox, (30.0, 0.0, 0.0))
    assert attack.state is State.ATTACK
    status = attack_status(attack, clock[0])
    assert status['state'] == 'attack' and status['range_m'] > 0
    assert status['closing_mps'] is not None and status['velocity'] is not None
    run(attack.step, drone, clock, InterceptConfig().track_timeout + 1.0)
    assert attack.state is State.LOST
    assert attack_status(attack, clock[0])['state'] == 'lost'
    attack.stop()
    assert state_label(attack) == 'idle'


def parse(*argv):
    p = argparse.ArgumentParser()
    common.add_setup_args(p)
    args = p.parse_args(argv)
    common.resolve_setup(args)
    return args


def test_the_default_setup_is_yolo_bytetrack_mpc():
    args = parse()
    assert (args.tracker, args.detect_args, args.guidance) == (
        'bytetrack', ('--detector', 'yolo', '--lock'), 'mpc')
    assert common.setup_tag(args) == 'yolo-bytetrack-mpc'
    options = Options(**common.setup_options(args))
    assert options.setup == 'real, bytetrack --detector yolo --lock, mpc'


def test_the_baseline_keeps_the_reports_file_names():
    args = parse('--setup', 'baseline')
    assert (args.tracker, args.detect_args, args.guidance) == ('confirmer', (), 'pursuit')
    assert common.setup_tag(args) == ''
    assert common.with_tag(Path('data/runs.csv'), '') == Path('data/runs.csv')


def test_one_changed_part_gets_its_own_name():
    args = parse('--guidance', 'pursuit')
    assert common.setup_tag(args) == 'bytetrack_detector_yolo_lock_pursuit'
    assert common.with_tag(Path('data/runs.csv'), 'x') == Path('data/runs_x.csv')
    assert common.with_tag(Path('data/baseline_eval'), 'x') == Path('data/baseline_eval_x')


def test_the_fake_detectors_only_count_the_attack():
    assert common.setup_tag(parse(), real_detector=False) == 'mpc'
    assert common.setup_tag(parse('--setup', 'baseline'), real_detector=False) == ''
