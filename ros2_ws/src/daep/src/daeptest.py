#!/usr/bin/env python3

import os
import sys
import time
import uuid
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from threading import RLock
from typing import Any
from rclpy.task import Future

from daep_msgs.action import Aeplanner
from daep_msgs.msg import Goal
from std_msgs.msg import Bool

from geographic_msgs.msg import GeoPoint
from unique_identifier_msgs.msg import UUID

from optparse import OptionParser
parser = OptionParser()
parser.add_option ("", "--get-current-position", action="store_true", dest="get_current_position", help="GetCurrentPosition")
parser.add_option ("", "--make-plan", action="store_true", dest="make_plan", help="Make Plan")
parser.add_option ("", "--goal", action="store_true", dest="goal", help="Send goal")
parser.add_option ("", "--yaw", action="store", type="float", dest="yaw", default=0.0, help="Yaw in degree")
parser.add_option ("-x", "--pos-x", action="store", type="float", dest="x", default=30.0, help="x")
parser.add_option ("-y", "--pos-y", action="store", type="float", dest="y", default=20.0, help="y")
parser.add_option ("-z", "--pos-z", action="store", type="float", dest="z", default=10.0, help="z")
parser.add_option ("-s", "--speed", action="store", type="float", dest="speed", default=7.0, help="speed")
parser.add_option ("", "--uuid", action="store", type="string", dest="uuid", default="", help="uuid")
(options, args) = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv))


class PatchRclpyIssue1123(ActionClient):

    _lock: RLock = None  # type: ignore

    @property
    def _cpp_client_handle_lock(self) -> RLock:
        if self._lock is None:
            self._lock = RLock()
        return self._lock

    async def execute(self, *args: Any, **kwargs: Any) -> None:
        with self._cpp_client_handle_lock:
            return await super().execute(*args, **kwargs)  # type: ignore

    def send_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super().send_goal_async(*args, **kwargs)

    def _cancel_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._cancel_goal_async(*args, **kwargs)

    def _get_result_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._get_result_async(*args, **kwargs)


class MakePlanActionClient():

    def __init__(self, node, shutdown=False, callback_group=None):
        self.node = node
        self._action_client = PatchRclpyIssue1123(node, Aeplanner, 'make_plan', callback_group=callback_group)
        self.shutdown = shutdown
        self.result = None
        self.have_goal_response = False
        self.goal_accepted = False
        self.goal_publisher = node.create_publisher(Goal, 'goal', 1)
        self.goal_reached = False
        self.goal_reached_sub = node.create_subscription(Bool, "/goal_reached", self.goal_reached_callback, 10)
        self.goal_msg = None
        timer_period = 0.5  # seconds
        self.timer = node.create_timer(timer_period, self.timer_callback)

    def goal_reached_callback(self, msg):
        print("goal_reached_callback:", msg)
        if msg.data:
            self.goal_reached = True
        
    def send_goal(self):
        goal_msg = Aeplanner.Goal()
        print("Wait for server")
        self._action_client.wait_for_server()
        print("Send goal")        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            self.have_goal_response = True
            return

        self.goal_accepted = True
        self.node.get_logger().info('Goal accepted :)')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.have_goal_response = True

    def get_result_callback(self, future):
        self.result = future.result().result
        self.node.get_logger().info(f'Result: {self.result.is_clear} - {self.result.tree_size}')
        if self.shutdown:
            rclpy.shutdown()        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'Received feedback')

    def set_goal_msg(self, msg):
        self.goal_msg = msg

    def timer_callback(self):
        if self.goal_msg:
            # print("timer_callback:", self.goal_msg)
            if self.goal_reached:
                print("GOAL REACHED")
                rclpy.shutdown()
            else:
                self.goal_publisher.publish(self.goal_msg)
                
def main(args=None):
    rclpy.init(args=args)

    node = Node("daeptest")

    if options.make_plan:
        print("Make_Plan")
        action_client = MakePlanActionClient(node, shutdown=True)
        action_client.send_goal()

    if options.goal:
        msg = Goal()
        msg.x = options.x
        msg.y = options.y
        msg.z = options.z
        msg.yaw = math.radians(options.yaw)
        msg.linear_velocity = 0.0
        msg.angular_velocity = 0.0
        msg.is_last = True
        print("Goal:", msg)
        action_client = MakePlanActionClient(node, shutdown=True)
        action_client.set_goal_msg(msg)
#        for i in range(10000):
#            if action_client.goal_reached:
#                break
#            action_client.goal_publisher.publish(msg)
#            time.sleep(0.5)
#        rclpy.shutdown()                    

#    if options.signal_goal:
#        client = SignalGoalClient(node, shutdown=True)
#        # guuid = uuid.UUID('{00010203-0405-0607-0809-0a0b0c0d0e0f}')
#        guuid = uuid.UUID(options.uuid)
#        goal_uuid = UUID(uuid=list(guuid.bytes))
#        signal = options.signal
#        if signal in ["abort", "enough", "pause", "resume"]:
#            signal = f'${signal}'
#        print("SIGNAL:", signal)
#        client.send_request(goal_uuid, signal)

    rclpy.spin(node)        

if __name__ == '__main__':
    main()
        
