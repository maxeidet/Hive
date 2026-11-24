"""Several hostiles in one run: surveillance2.yaml, SurveillanceWorld.fly_many."""
from swarm_backend.config import load_config
from swarm_backend.scenarios import common
from swarm_backend.scenarios.sim_surveillance import SimSurveillance
from swarm_backend.scenarios.surveillance import Options

TWO = load_config(common.PKG / 'scenarios' / 'surveillance2.yaml')
ONE = load_config(common.PKG / common.SURVEILLANCE_CONFIG)


def test_hostile_ids():
    assert SimSurveillance(TWO, Options(detector='fake'), quiet=True).hostile_ids == (4, 5)
    assert SimSurveillance(ONE, Options(detector='fake'), quiet=True).hostile_ids == (4,)
