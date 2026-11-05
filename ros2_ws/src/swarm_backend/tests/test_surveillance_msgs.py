import pytest

from swarm_backend.config import parse_config
from swarm_backend.intercept import surveillance_msgs as m
from swarm_backend.intercept.base_station import Mode, Order
from swarm_backend.intercept.target_association import TargetAssociator


def test_messages_round_trip():
    s = m.SightingMsg(2, (1.0, -2.5, -20.0), True)
    assert m.decode_sighting(m.encode_sighting(s)) == s
    st = m.StatusMsg(3, Mode.ENGAGE, 41.5)
    assert m.decode_status(m.encode_status(st)) == st
    st = m.StatusMsg(3, Mode.PATROL, 60.0, (60.0, 0.0, -20.0))
    assert m.decode_status(m.encode_status(st)) == st
    for kind in m.EVENT_KINDS:
        e = m.EventMsg(1, kind)
        assert m.decode_event(m.encode_event(e)) == e
    with pytest.raises(ValueError):
        m.encode_event(m.EventMsg(1, 'boom'))


@pytest.mark.parametrize('order', [
    Order(1, Mode.PATROL, sector=(-45.0, 0.0), may_start=True),
    Order(2, Mode.PATROL),
    Order(3, Mode.ENGAGE, 'T4'),
    Order(1, Mode.RETURN)])
def test_orders_round_trip(order):
    assert m.decode_order(m.encode_order(order)) == order


def test_an_unknown_event_is_refused():
    with pytest.raises(ValueError):
        m.encode_event(m.EventMsg(1, 'boom'))


def test_a_nearby_sighting_is_the_same_target_and_a_far_one_is_new():
    a = TargetAssociator(gate_m=15.0, expiry_s=5.0)
    first = a.identify((100.0, 0.0, -20.0), 0.0)
    assert a.identify((105.0, 3.0, -20.0), 1.0) == first
    assert a.identify((200.0, 0.0, -20.0), 1.0) != first


def test_a_target_not_seen_for_a_while_gets_a_new_id():
    a = TargetAssociator(gate_m=15.0, expiry_s=5.0)
    first = a.identify((100.0, 0.0, -20.0), 0.0)
    assert a.identify((100.0, 0.0, -20.0), 10.0) != first


def test_surveillance_settings_are_parsed_and_checked():
    data = {'drones': [{'id': 1}], 'surveillance': {'drone_ids': [1, 2, 3], 'leash_m': 150}}
    s = parse_config(data).surveillance
    assert s.drone_ids == (1, 2, 3) and s.leash_m == 150.0
    with pytest.raises(ValueError):
        parse_config({'drones': [{'id': 1}], 'surveillance': {'oops': 1}})
    with pytest.raises(ValueError):
        parse_config({'drones': [{'id': 1}], 'surveillance': {'drone_ids': [1, 1]}})
