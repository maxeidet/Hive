"""Bridge between DAEP and one PX4 drone of the Hive swarm backend.

DAEP only plans. This node is the robot side it expects:

  goal (daep_msgs/Goal)             -> flies there (goal_follower.py), ENU -> NED
  /goal_reached (std_msgs/Bool)     <- True once per goal, when the drone is there
  simple_pose (PoseStamped)         <- the drone's pose in the world frame (ENU)
  /exploration_completed (Bool)     -> lands when DAEP says it is done

It takes off to --alt first and only then publishes the pose, so DAEP (which waits for a
pose) starts planning from the air. Ctrl-C lands too. `goal` and `simple_pose` are
relative names: run the node in DAEP's namespace.

    ros2 run swarm_backend daep_bridge --ros-args -r __ns:=/daep
    ros2 run swarm_backend daep_bridge --id 0 --alt 2 --ros-args -r __ns:=/daep

Needs daep_msgs (LiU's private DAEP messages, ros2_ws/src/daep_msgs) built in the same
workspace, and PX4 SITL, Gazebo and the DDS agent running. Started by
launch/daep_exploration.launch.py. Not `pose`: aeplanner adds 0.5 m to z on that topic (the M100's lidar).
"""
import argparse
import math
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool

from swarm_backend.config import default_config_path, load_config
from swarm_backend.explore.goal_follower import (
    DONE, NEW, GoalFollower, enu_quaternion, enu_to_ned, ned_to_enu)
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until

# DAEP stops repeating a goal within 0.1 s of our /goal_reached. If it still repeats the
# goal this long after, the message was lost and is sent again. Not sooner: a late True
# would count for DAEP's next goal.
REACK_S = 1.0


class Bridge:
    def __init__(self, node: Node, drone: PX4Drone, follower: GoalFollower, rate_hz: float,
                 world_frame: str):
        self.node, self.drone, self.follower = node, drone, follower
        self.log = node.get_logger()
        self.world_frame = world_frame
        self.flying = False  # set after take-off: publish the pose and take goals
        self.completed = threading.Event()
        self._acked = 0.0  # time.monotonic() of the last /goal_reached
        self._pose_pub = node.create_publisher(PoseStamped, 'simple_pose', 1)
        self._reached_pub = node.create_publisher(Bool, '/goal_reached', 1)
        from daep_msgs.msg import Goal  # only with DAEP built; Hive itself does not need it
        node.create_subscription(Goal, 'goal', self._on_goal, 1)
        node.create_subscription(Bool, '/exploration_completed', self._on_completed, 1)
        node.create_timer(1.0 / rate_hz, self._step)

    def _on_goal(self, msg) -> None:
        if not self.flying:
            return
        result = self.follower.on_goal(msg.uuid, (msg.x, msg.y, msg.z), msg.yaw,
                                       msg.initial_motion)
        if result == NEW:
            what = 'one turn on the spot' if msg.initial_motion else (
                f'({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}) m, yaw {math.degrees(msg.yaw):.0f} deg')
            self.log.info(f'Goal: {what}')
        elif result == DONE and time.monotonic() - self._acked > REACK_S:
            self._ack()

    def _on_completed(self, msg: Bool) -> None:
        if msg.data and self.flying and not self.completed.is_set():
            self.log.info('DAEP: exploration completed')
            self.completed.set()

    def _step(self) -> None:
        d = self.drone
        if not self.flying or d.position is None or d.attitude is None:
            return
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = self.world_frame
        p, q = pose.pose.position, pose.pose.orientation
        p.x, p.y, p.z = ned_to_enu(d.position)
        q.x, q.y, q.z, q.w = enu_quaternion(d.attitude)
        self._pose_pub.publish(pose)
        if self.follower.step():
            self.log.info('Goal reached')
            self._ack()

    def _ack(self) -> None:
        self._acked = time.monotonic()
        self._reached_pub.publish(Bool(data=True))

    def take_off(self, altitude: float, tolerance: float, timeout: float,
                 start: tuple[float, float] | None = None) -> None:
        """Take off to `altitude` and, with `start` (x, y in ENU), fly there at that altitude."""
        d = self.drone
        wait_until(lambda: d.ready, timeout, 'PX4 status, position and pre-flight checks')
        d.arm()
        wait_until(lambda: d.armed, timeout, 'arming')
        d.start_offboard()
        wait_until(lambda: d.offboard, timeout, 'offboard mode')
        self.log.info(f'Taking off to {altitude:g} m')
        d.takeoff(altitude)
        wait_until(lambda: d.reached(tolerance), timeout, 'take-off altitude')
        if start is not None:
            north, east, _ = enu_to_ned((start[0], start[1], 0.0))
            self.log.info(f'Flying to the area at ({start[0]:g}, {start[1]:g})')
            d.set_position(north, east, d.target[2])
            wait_until(lambda: d.reached(tolerance), timeout, 'the start of the area')
        self.flying = True
        self.log.info('In the air: publishing the pose and taking goals from DAEP')

    def land(self) -> None:
        d = self.drone
        self.flying = False
        if not d.armed:
            return
        self.log.info('Landing')
        d.land()
        try:
            wait_until(lambda: not d.armed, LAND_TIMEOUT_S, 'landing and auto-disarm')
        except TimeoutError as e:
            self.log.error(str(e))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    parser.add_argument('--id', type=int, default=0, help='PX4 instance number of the drone')
    parser.add_argument('--alt', type=float, default=2.0,
                        help='take-off altitude in metres, where DAEP starts (default 2)')
    parser.add_argument('--start', type=float, nargs=2, metavar=('X', 'Y'),
                        help='fly to this point (world frame, ENU) at --alt before DAEP starts')
    parser.add_argument('--lead', type=float, default=1.0,
                        help='metres the setpoint runs ahead of the drone; about its speed '
                             'in m/s (default 1)')
    parser.add_argument('--yaw-tolerance', type=float, default=10.0,
                        help='degrees from the goal yaw that count as reached (default 10)')
    parser.add_argument('--turn-step', type=float, default=30.0,
                        help='degrees per step of the first turn on the spot (default 30)')
    parser.add_argument('--world-frame', default='world', help='frame_id of the pose')
    parser.add_argument('--timeout', type=float, default=900.0,
                        help='seconds to wait for each take-off step; long, because Gazebo '
                             'with a depth camera can run at 0.1x real time (default 900)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    config = load_config(args.config or default_config_path())
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('px4_bridge')
    drone = PX4Drone(node, config.drone(args.id), rate_hz=config.rate_hz)
    follower = GoalFollower(drone, config.position_tolerance, math.radians(args.yaw_tolerance),
                            args.lead, math.radians(args.turn_step))
    bridge = Bridge(node, drone, follower, config.rate_hz, args.world_frame)

    stopping = threading.Event()

    def stop(*_):
        # Only the first signal: ros2 launch sends SIGTERM 5 s after SIGINT, and that must
        # not interrupt the landing.
        if not stopping.is_set():
            stopping.set()
            raise KeyboardInterrupt
    # SIGTERM lands like Ctrl-C. SIGINT is set too: Python leaves it ignored when the node
    # is started in the background of a script (`px4_bridge.py &`).
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    with spinning(node):
        try:
            bridge.take_off(args.alt, config.position_tolerance, args.timeout, args.start)
            while not bridge.completed.wait(0.5):
                pass
        except (KeyboardInterrupt, TimeoutError) as e:
            node.get_logger().warning(f'Stopping ({str(e) or "interrupted"})')
        finally:
            stopping.set()
            bridge.land()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
