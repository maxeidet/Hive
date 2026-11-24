#!/usr/bin/env python3

import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import matplotlib.pyplot as plt

from daep_msgs.msg import HumanUpdate, HumanListUpdate

class Object():
    def __init__(self, name, speed, min_x, min_y, max_x, max_y, object_type, start_x=0.0, start_y=0.0, start_z=0.0, world_frame="world"):
        self.name = name
        self.uuid = str(uuid.uuid4())
        self.x = start_x
        self.y = start_y
        self.z = start_z
        self.speed = speed
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.object_type = object_type
        self.world_frame = world_frame

        if object_type == "dx":
            self.dx = speed
            self.dy = 0.0
        if object_type == "dy":
            self.dx = 0.0
            self.dy = speed

        self.wait_timer = 0  # Initialize wait timer
    """
    def tick(self, dt):
        # print("TICK:", dt, self.object_type, self.min_x, self.max_x)
        self.x += self.dx * dt
        self.y += self.dy * dt
        if self.x > self.max_x and self.object_type == "dx":
            self.dx = -self.speed
            self.x += self.dx * dt
        if self.x < self.min_x and self.object_type == "dx":
            self.dx = self.speed
            self.x += self.dx * dt
        if self.y > self.max_y and self.object_type == "dy":
            self.dy = -self.speed
            self.y += self.dy * dt         
        if self.y < self.min_y and self.object_type == "dy":
            self.dy = self.speed
            self.y += self.dy * dt
        #print(self.x, self.y)
    """

    def tick(self, dt):

        # Check if we're waiting at a border
        if self.wait_timer > 0:
            # Still waiting, decrease timer
            self.wait_timer -= dt
            
            # Check if we've finished waiting and need to switch direction
            if self.wait_timer <= 0:
                # Time to change direction
                if self.x >= self.max_x and self.object_type == "dx":
                    self.dx = -self.speed
                elif self.x <= self.min_x and self.object_type == "dx":
                    self.dx = self.speed
                
                if self.y >= self.max_y and self.object_type == "dy":
                    self.dy = -self.speed
                elif self.y <= self.min_y and self.object_type == "dy":
                    self.dy = self.speed
            
            return  # Don't move while waiting
        
        # Normal movement
        self.x += self.dx * dt
        self.y += self.dy * dt
        
        # Check x boundaries
        if self.x > self.max_x and self.object_type == "dx":
            self.x = self.max_x  # Set to exactly at boundary
            self.wait_timer = 5.0  # Wait for 2 seconds
        elif self.x < self.min_x and self.object_type == "dx":
            self.x = self.min_x  # Set to exactly at boundary
            self.wait_timer = 5.0  # Wait for 2 seconds
        
        # Check y boundaries
        if self.y > self.max_y and self.object_type == "dy":
            self.y = self.max_y  # Set to exactly at boundary
            self.wait_timer = 5.0  # Wait for 2 seconds
        elif self.y < self.min_y and self.object_type == "dy":
            self.y = self.min_y  # Set to exactly at boundary
            self.wait_timer = 5.0  # Wait for 2 seconds

    def get_msg(self, time):
        msg = HumanUpdate()
        msg.uuid = self.uuid
        msg.name = self.name
        msg.header.frame_id = self.world_frame
        msg.header.stamp = time.to_msg()
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.twist.linear.x = self.dx #m/s
        msg.twist.linear.y = self.dy #m/s
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        return msg
    
class AnimateObjects(Node):
    def __init__(self):
        super().__init__("animate_objects")

        self.group = ReentrantCallbackGroup()                

        self.declare_parameter("human_height", 1.8)
        self.declare_parameter("human_width", 1.0)
        self.declare_parameter("human_linear_velocity", 0.35)
        self.declare_parameter("human_angular_velocity", 1.0)
        self.declare_parameter("boundary/min", [-100.0, -100.0, 0.0])
        self.declare_parameter("boundary/max", [100.0, 100.0, 0.0])
        self.declare_parameter("num_obstacles", 5)  # Default to 5 obstacles if not specified
        self.declare_parameter("period_time", 0.1)
        self.declare_parameter("system/bbx/use_sphere_collision_checking", False) #Used to determine if dynamic objects should be walking or flying
        self.declare_parameter("flying_obstacle_linear_velocity", 0.5)
        self.declare_parameter("world_frame", "world")

        self.human_height = self.get_parameter('human_height').value
        self.human_width = self.get_parameter('human_width').value
        self.human_angular_velocity = self.get_parameter('human_angular_velocity').value
        self.min = self.get_parameter('boundary/min').value
        self.max = self.get_parameter('boundary/max').value
        self.num_obstacles = self.get_parameter('num_obstacles').value
        self.dt = self.get_parameter('period_time').value #seconds
        self.world_frame = self.get_parameter('world_frame').value
        self.flying_dynamic_obstacles = self.get_parameter('system/bbx/use_sphere_collision_checking').value
        print("WORLD FRAME PARAMETER:", self.world_frame)

        if(self.flying_dynamic_obstacles):
            self.dynamic_obstacle_speed = self.get_parameter('flying_obstacle_linear_velocity').value
        else:
            self.dynamic_obstacle_speed = self.get_parameter('human_linear_velocity').value

        self.min_x = self.min[0]
        self.max_x = self.max[0]
        self.min_y = self.min[1]
        self.max_y = self.max[1]
        self.min_z = self.min[2]
        self.max_z = self.max[2]

        print(f"World boundaries: min_x: {self.min_x}, max_x: {self.max_x}, min_y: {self.min_y}, max_y: {self.max_y}")
        print(f"Number of dynamic obstacles: {self.num_obstacles}")
        print(f"Dynamic obstacle speed: {self.dynamic_obstacle_speed} m/s")
        if(self.flying_dynamic_obstacles):
            print(f"Dynamic obstacle type: FLYING")
        else:
            print(f"Dynamic obstacle type: WALKING")

        self.objects = []
        self.initialize_objects(self.num_obstacles)
     
        self.pub = self.create_publisher(HumanListUpdate, "dynamic_objects", 10)

        self.timer = self.create_timer(self.dt, self.update_timer_callback, callback_group=self.group)        


    def initialize_objects(self, num_obstacles):
            """Initialize N objects with random positions and directions"""
            import random
            
            # Direction types
            direction_types = ["dx", "dy"]

            #spos = [(self.min_x + 4.0, self.min_y), (self.min_x, self.max_y-5.0) ]
            dir = ["dy", "dx"]
            
            for i in range(num_obstacles):
                # Generate random starting positions within the boundaries
                start_x = random.uniform(self.min_x, self.max_x)
                start_y = random.uniform(self.min_y, self.max_y)
                #start_x = spos[i][0]
                #start_y = spos[i][1]
                start_z = 0.0

                if(self.flying_dynamic_obstacles):
                    # Pick random start height in bounding range
                    start_z = random.uniform(self.min_z, self.max_z)
                
                # Randomly choose a direction type
                object_type = random.choice(direction_types)
                #object_type = dir[i]
                
                # Create a unique name for each object
                name = f"obj{i+1}"
                
                new_obj = Object(
                    name, 
                    self.dynamic_obstacle_speed, 
                    self.min_x, 
                    self.min_y, 
                    self.max_x, 
                    self.max_y, 
                    object_type,
                    start_x,
                    start_y,
                    start_z,
                    self.world_frame
                )
                
                self.objects.append(new_obj)

    # Update the grid with new observations from timer callback
    def update_timer_callback(self):
        humans = HumanListUpdate()
        for obj in self.objects:
            obj.tick(self.dt)
            humans.humans.append(obj.get_msg(self.get_clock().now()))
        self.pub.publish(humans)


def main(args=None):
    rclpy.init(args=args)

    node = AnimateObjects()

    executor = MultiThreadedExecutor(num_threads=8)
    # minimal_client.send_request()
    
    executor.add_node(node)

    #thread = threading.Thread(target=spin_thread, args=(node,))
    #thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    # rclpy.spin(node)
    # node.do_command()

    node.destroy_node()
    executor.shutdown()    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
            
