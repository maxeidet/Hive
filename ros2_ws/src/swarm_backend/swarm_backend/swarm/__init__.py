"""The swarm: how the defenders work together, and the base station that decides.

  swarm.py, manager.py, formation.py   the fleet of PX4 drones, take-off, formations
  arc.py                               the surveillance arc: each defender's slot and sector
  patrol.py                            one defender: PATROL, ENGAGE, RETURN
  base_station.py                      the rules: who attacks which target, the reserve, sectors
  dispatcher.py                        the base station's messages in, orders out
  messages.py                          the /surveillance/* topics between drones and base
  association.py                       which sightings are the same target
  friendlies.py                        the hivemind: where every friendly drone is

Pure logic, no ROS: nodes/ (patrol_node, base_station_node, hivemind_node) runs it on ROS.
"""
