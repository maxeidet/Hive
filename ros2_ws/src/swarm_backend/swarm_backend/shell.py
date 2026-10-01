"""Interactive Python shell for flying the swarm by hand while watching Gazebo.

    ros2 run swarm_backend swarm_shell [--num-drones N] [--config path/to/swarm.yaml]

ROS spins in the background, so drones keep streaming setpoints while you type.
The helpers only act on drones that are actually connected, so the full config
works even if only some PX4 instances are running.
"""
import argparse
import code
import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept import guidance
from swarm_backend.intercept.chase import ChaseAction
from swarm_backend.intercept.mock import HostileFlight
from swarm_backend.intercept.paths import make_path
from swarm_backend.intercept.radar import MockRadar, RadarTarget
from swarm_backend.px4.drone import wrap_deg
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.swarm import formation as formation_module
from swarm_backend.swarm.formation import Formation, centroid
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

CONNECT_WAIT_S = 10.0
MOVE_TIMEOUT_S = 180.0  # long climbs are slow, more so when Gazebo runs below real time
GIMBAL_TIMEOUT_S = 10.0
GIMBAL_PITCH_TOL = 1.0  # degrees
GIMBAL_YAW_TOL = 10.0

BANNER = """
Swarm shell. Positions: x = north, y = east (metres from the Gazebo origin); alt = metres up.

  status()                        table of every drone
  takeoff(alt=None, ids=None)     arm, offboard, climb (default alt from config)
  goto(id, x, y, alt=None)        fly one drone to a point
  move(dx, dy, dalt=0, ids=None)  shift drones relative to their current target
  form('grid'|'line'|'circle', spacing=None, alt=None)
  land(ids=None)                  land and wait for disarm
  gimbal(id, pitch=None, yaw=None)  point a gimbal camera (degrees; pitch -90 = straight
                                  down, yaw from the nose); gimbal(id) prints where it points.
                                  Only gimbal drones (./mount-backend.sh spawn gimbal); fixed
                                  cameras turn with the drone. Watch it: ./mount-backend.sh view id

Intercept (defaults from the `intercept:` section of swarm.yaml):
  takeoff(50)                     both drones up to the chase altitude first
  hostile(id=None, pattern=None, speed=None)  fly a pattern: 'line', 'circle', 'figure8',
                                  'random'; it heads away from the interceptor
  chase(id=None, target=None)     chase the target using the mock radar's returns:
                                  intercept, then shadow at standoff (further while the
                                  radar alone cannot place it that well); aims the gimbal
  stop_chase()                    both stop and hover; status() shows phase and range

Waiting helpers block until done; Ctrl-C stops waiting (drones keep their last command).
Pass wait=False to return immediately.

Objects: swarm, manager, formation, config, d0, d1, ... (the PX4Drone objects, NED z)
  e.g. d0.position, d1.set_velocity(1, 0, 0), manager.all_reached()

Ctrl-D exits; any drone still armed is landed first.
"""


class Pilot:
    """Blocking, forgiving wrappers around SwarmManager for use at a prompt."""

    def __init__(self, manager: SwarmManager, config, node: Node):
        self.manager = manager
        self.swarm = manager.swarm
        self.config = config
        self._node = node
        self._gimbal_targets: dict[int, tuple[float, float]] = {}  # last requested per drone
        # Intercept: run by a timer in the ROS thread, so they keep flying while you type.
        self._flight: HostileFlight | None = None
        self._chase: ChaseAction | None = None
        self._radar: MockRadar | None = None
        self._radar_target: RadarTarget | None = None
        node.create_timer(1.0 / config.rate_hz, self._step_intercept)

    def active(self, ids=None) -> list[int]:
        """ids of connected drones (restricted to `ids` if given)."""
        return [d.id for d in self.swarm.select(ids) if d.connected and d.position is not None]

    def status(self) -> None:
        print(f'{"id":>3} {"conn":>5} {"pre":>5} {"armed":>6} {"mode":>9} '
              f'{"x":>7} {"y":>7} {"alt":>6}   target')
        for d in self.swarm:
            mode = 'offboard' if d.offboard else 'landing' if d.landing else (
                f'nav {d.nav_state}' if d.nav_state is not None else '-')
            if d.position is None:
                pos = f'{"-":>7} {"-":>7} {"-":>6}'
            else:
                pos = f'{d.position[0]:7.2f} {d.position[1]:7.2f} {d.altitude:6.2f}'
            target = '-' if d.target is None else (
                f'({d.target[0]:.1f}, {d.target[1]:.1f}, alt {d.home[2] - d.target[2]:.1f})')
            print(f'{d.id:>3} {yes(d.connected):>5} {yes(d.preflight_ok):>5} {yes(d.armed):>6} '
                  f'{mode:>9} {pos}   {target}')
        if self._flight is not None:
            print(f'Drone {self._flight.drone.id} is flying a hostile pattern')
        if self._chase is not None:
            hunter, hostile = self._chase.drone, self._radar.hostile
            true_range = '-' if None in (hunter.position, hostile.position) else (
                f'{guidance.horizontal_distance(hunter.position, hostile.position):.1f}')
            chase = self._chase
            print(f'Drone {hunter.id} chasing drone {hostile.id}: {chase.phase.value}, '
                  f'range {true_range} m (standoff {chase.standoff:.1f} m, '
                  f'estimate +-{chase.estimate_std:.1f} m)')

    def takeoff(self, alt=None, ids=None, wait=True) -> bool:
        ids = self.active(ids)
        if not ids:
            return self._nothing_connected()
        not_ready = [d.id for d in self.swarm.select(ids) if not d.ready]
        if not_ready:
            print(f'Drones {not_ready} are not ready (pre-flight checks failing); '
                  'waiting for them...')
            if not self._wait(lambda: all(self.swarm[i].ready for i in ids), 60, 'pre-flight'):
                return False
        alt = self.config.takeoff_altitude if alt is None else alt
        self.manager.takeoff(alt, ids)
        print(f'Taking off drones {ids} to {alt} m')
        return not wait or self._wait(lambda: self.manager.all_reached(ids), MOVE_TIMEOUT_S, 'takeoff')

    def goto(self, drone_id, x, y, alt=None, wait=True) -> bool:
        drone = self.swarm[drone_id]
        if not drone.offboard:
            print(f'Drone {drone_id} is not flying in offboard mode; takeoff() first')
            return False
        if alt is not None:
            z = drone.home[2] - alt
        elif drone.target is not None:
            z = drone.target[2]  # keep the current altitude
        else:
            z = drone.home[2] - self.config.takeoff_altitude
        drone.set_position(x, y, z)
        return not wait or self._wait(lambda: drone.reached(self.manager.tolerance), MOVE_TIMEOUT_S, 'goto')

    def move(self, dx, dy, dalt=0.0, ids=None, wait=True) -> bool:
        ids = [i for i in self.active(ids) if self.swarm[i].offboard]
        if not ids:
            print('No drone is flying in offboard mode; takeoff() first')
            return False
        for i in ids:
            drone = self.swarm[i]
            x, y, z = drone.target or drone.position
            drone.set_position(x + dx, y + dy, z - dalt)
        return not wait or self._wait(lambda: self.manager.all_reached(ids), MOVE_TIMEOUT_S, 'move')

    def form(self, shape='grid', spacing=None, alt=None, wait=True) -> bool:
        ids = [i for i in self.active() if self.swarm[i].offboard]
        if not ids:
            print('No drone is flying in offboard mode; takeoff() first')
            return False
        drones = self.swarm.select(ids)
        spacing = self.config.formation_spacing if spacing is None else spacing
        cx, cy, cz = centroid([d.target or d.position for d in drones])
        if alt is not None:
            cz = centroid([d.home for d in drones])[2] - alt
        shapes = Formation(len(drones))
        if shape == 'grid':
            points = shapes.grid((cx, cy, cz), spacing)
        elif shape == 'line':
            points = shapes.line((cx, cy, cz), spacing)
        elif shape == 'circle':
            radius = formation_module.circle_radius_for_spacing(len(drones), spacing)
            points = shapes.circle((cx, cy, cz), radius)
        else:
            print("shape must be 'grid', 'line' or 'circle'")
            return False
        positions = [d.position for d in drones]
        for drone, (x, y, z) in zip(drones, formation_module.assign_targets(positions, points)):
            drone.set_position(x, y, z)
        print(f'Forming {shape} with drones {ids}')
        return not wait or self._wait(lambda: self.manager.all_reached(ids), MOVE_TIMEOUT_S, shape)

    def land(self, ids=None, wait=True) -> bool:
        ids = [d.id for d in self.swarm.select(ids) if d.armed]
        if self._flight is not None and self._flight.drone.id in ids:
            self._stop_hostile()
        if self._chase is not None and self._chase.drone.id in ids:
            self.stop_chase()
        if not ids:
            print('No drone is armed')
            return True
        self.manager.land(ids)
        print(f'Landing drones {ids}')
        return not wait or self._wait(
            lambda: not any(self.swarm[i].armed for i in ids), LAND_TIMEOUT_S, 'landing')

    def gimbal(self, drone_id, pitch=None, yaw=None, wait=True) -> bool:
        drone = self.swarm[drone_id]
        if pitch is None and yaw is None:
            if drone.gimbal is None:
                print(f'Drone {drone_id} has not reported a gimbal (only gimbal drones have one)')
                return False
            print(f'Drone {drone_id} gimbal: pitch {drone.gimbal[0]:.1f}, yaw {drone.gimbal[1]:.1f}')
            return True
        if not drone.connected:
            return self._nothing_connected()
        last = self._gimbal_targets.get(drone_id) or drone.gimbal or (0.0, 0.0)
        pitch = last[0] if pitch is None else pitch
        yaw = last[1] if yaw is None else yaw
        self._gimbal_targets[drone_id] = (pitch, yaw)
        drone.point_gimbal(pitch, yaw)
        print(f'Pointing drone {drone_id} gimbal to pitch {pitch:.1f}, yaw {yaw:.1f}')
        if not wait:
            return True
        # Yaw is read back against the drone's own heading estimate, which can be a few
        # degrees off the simulated truth the gimbal uses, so it gets a looser tolerance.
        return self._wait(
            lambda: drone.gimbal is not None and abs(drone.gimbal[0] - pitch) < GIMBAL_PITCH_TOL
            and abs(wrap_deg(drone.gimbal[1] - yaw)) < GIMBAL_YAW_TOL, GIMBAL_TIMEOUT_S, 'gimbal')

    def hostile(self, drone_id=None, pattern=None, speed=None) -> bool:
        settings = self.config.intercept
        drone = self.swarm[settings.hostile_id if drone_id is None else drone_id]
        if not drone.offboard:
            print(f'Drone {drone.id} is not flying in offboard mode; takeoff() first')
            return False
        pattern = pattern or settings.hostile_pattern
        speed = speed or settings.hostile_speed
        # Head away from the interceptor, so a chase starts with a gap to close.
        hunter = next((d for d in self.swarm if d.id == settings.interceptor_id), None)
        heading = 0.0 if hunter is None or hunter.position is None or hunter is drone else (
            guidance.bearing(hunter.position, drone.position))
        try:
            path = make_path(pattern, drone.position, heading, speed, settings.pattern_size,
                             settings.seed)
        except ValueError as e:
            print(e)
            return False
        self._stop_hostile()
        self._flight = HostileFlight(drone, path, lambda: drone.stamp)
        self._flight.start()
        print(f'Drone {drone.id} flying {pattern} at {speed} m/s ({path.length:.0f} m loop)')
        return True

    def chase(self, drone_id=None, target_id=None) -> bool:
        settings = self.config.intercept
        hunter = self.swarm[settings.interceptor_id if drone_id is None else drone_id]
        target = self.swarm[settings.hostile_id if target_id is None else target_id]
        if hunter is target:
            print('A drone cannot chase itself')
            return False
        if not hunter.offboard:
            print(f'Drone {hunter.id} is not flying in offboard mode; takeoff() first')
            return False
        self.stop_chase()
        clock = lambda: hunter.stamp  # noqa: E731
        self._radar = MockRadar(self._node, target, settings)
        self._radar_target = RadarTarget(self._node, settings, clock)
        self._chase = ChaseAction(hunter, [self._radar_target], settings, clock)
        self._chase.start()
        print(f'Drone {hunter.id} chasing drone {target.id} at {settings.altitude} m '
              f'(radar at {settings.radar_position}). status() shows how it goes.')
        return True

    def stop_chase(self) -> None:
        """Stop the chase and the hostile pattern; both drones hover where they are."""
        if self._chase is not None:
            chase, self._chase = self._chase, None
            chase.stop()
            self._radar.destroy()
            self._radar_target.destroy()
            self._radar = self._radar_target = None
        self._stop_hostile()

    def _stop_hostile(self) -> None:
        if self._flight is not None:
            flight, self._flight = self._flight, None
            flight.stop()

    def _step_intercept(self) -> None:
        flight, chase = self._flight, self._chase  # the prompt thread may clear them
        if flight is not None:
            flight.step()
        if chase is not None:
            chase.step()

    def _wait(self, predicate, timeout, what) -> bool:
        try:
            wait_until(predicate, timeout, what)
            print('done')
            return True
        except TimeoutError as e:
            print(f'{e}. Check status() and the PX4 terminal.')
        except KeyboardInterrupt:
            print('\nStopped waiting; drones keep their last command.')
        return False

    def _nothing_connected(self) -> bool:
        print('No drone is connected yet. Is PX4 running? See status().')
        return False


def yes(flag: bool) -> str:
    return 'yes' if flag else 'no'


def main(args=None):
    parser = argparse.ArgumentParser(description='Interactive swarm shell')
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    parser.add_argument('--num-drones', type=int, help='use only the first N drones in the config')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])

    config = load_config(cli.config or default_config_path(), cli.num_drones)

    # Ctrl-C should interrupt the current command, not shut ROS down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('swarm_shell')
    swarm = Swarm(PX4Drone(node, c, rate_hz=config.rate_hz) for c in config.drones)
    manager = SwarmManager(swarm, config.position_tolerance)
    pilot = Pilot(manager, config, node)

    namespace = {
        'swarm': swarm, 'manager': manager, 'config': config, 'pilot': pilot,
        'formation': Formation(len(swarm)),
        'status': pilot.status, 'takeoff': pilot.takeoff, 'goto': pilot.goto,
        'move': pilot.move, 'form': pilot.form, 'land': pilot.land, 'gimbal': pilot.gimbal,
        'hostile': pilot.hostile, 'chase': pilot.chase, 'stop_chase': pilot.stop_chase,
    }
    namespace.update({f'd{d.id}': d for d in swarm})

    try:
        import readline
        import rlcompleter
        readline.set_completer(rlcompleter.Completer(namespace).complete)
        readline.parse_and_bind('tab: complete')
    except ImportError:
        pass

    with spinning(node):
        # Let the connection log lines print before the prompt, so it is not buried under them.
        print('Waiting for drones to connect...')
        try:
            wait_until(manager.connected, CONNECT_WAIT_S, 'drones')
        except TimeoutError:
            print(f'No data yet from drones {manager.not_connected()}; '
                  'start them with ./mount-backend.sh spawn (they can join later)')
        except KeyboardInterrupt:
            pass
        try:
            code.interact(banner=BANNER, local=namespace, exitmsg='')
        finally:
            if manager.any_armed():
                print('Landing armed drones before exit (Ctrl-C to skip)...')
                pilot.land()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
