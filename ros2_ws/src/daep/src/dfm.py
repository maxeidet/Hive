#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import PointStamped
from daep_msgs.srv import QueryDFM
from matplotlib.animation import FuncAnimation
from daep_msgs.msg import HumanUpdate, HumanListUpdate

import threading


"""
Keep track of history of dynamic obstacles
"""
class DynamicFrequencyMap(Node):
    def __init__(self):
        super().__init__("dfm")

        self.group = ReentrantCallbackGroup()                

        self.declare_parameter("boundary/min", [-100.0, -100.0, 0.0])
        self.declare_parameter("boundary/max", [ 100.0,  100.0, 3.0])
        self.declare_parameter("visualizeDFM", False)
        self.declare_parameter("dfm/square_size", 1.0)

        self.min = self.get_parameter('boundary/min').value
        self.max = self.get_parameter('boundary/max').value
        self.visualizeDFM = self.get_parameter('visualizeDFM').value
        self.square_size = self.get_parameter('dfm/square_size').value
        
        self.dfm_query_srv = self.create_service(QueryDFM, "dfm_query_server", self.dfm_callback, callback_group=self.group)
        
        self.min_x = self.min[0]
        self.max_x = self.max[0]
        self.min_y = self.min[1]
        self.max_y = self.max[1]

        print(f"DFM Init: Raw boundary min_x = {self.min_x}")        
        print(f"DFM Init: Raw boundary max_x = {self.max_x}")        
        print(f"DFM Init: Raw boundary min_y = {self.min_y}")        
        print(f"DFM Init: Raw boundary max_y = {self.max_y}")        

        self.round_min_x = self.round(self.min_x)
        self.round_max_x = self.round(self.max_x)
        self.round_min_y = self.round(self.min_y)
        self.round_max_y = self.round(self.max_y)
        print(f"DFM Init: Rounded boundary min_x = {self.round_min_x}")        
        print(f"DFM Init: Rounded boundary max_x = {self.round_max_x}")        
        print(f"DFM Init: Rounded boundary min_y = {self.round_min_y}")        
        print(f"DFM Init: Rounded boundary max_y = {self.round_max_y}")        
        self.x_size = self.round_max_x - self.round_min_x
        self.y_size = self.round_max_y - self.round_min_y
        print(f"DFM Init: Grid size in X direction = {self.x_size} meters")        
        print(f"DFM Init: Grid size in Y direction = {self.y_size} meters")        
        self.array_size_x = int(self.x_size / self.square_size)
        self.array_size_y = int(self.y_size / self.square_size)
        print(f"DFM Init: Array size in X dimension = {self.array_size_x} cells")        
        print(f"DFM Init: Array size in Y dimension = {self.array_size_y} cells")        
        self.grid = np.zeros(shape=(self.array_size_x, self.array_size_y))
        self.dynamic_objects = {}

        #self.point_sub = self.create_subscription(PointStamped, "updateDFM", self.update_frequency_map_callback, 1, callback_group=self.group)
        
        self.dynamic_object_sub = self.create_subscription(HumanListUpdate, "dynamic_objects", self.update_human_positions, 10, callback_group=self.group)

        self.timer = self.create_timer(0.1, self.update_timer_callback, callback_group=self.group)        

    def visualize_dfm(self):
        # Visualize the grid
        if self.visualizeDFM:
            self.fig, self.ax = plt.subplots()
            self.im = self.ax.imshow(self.grid, cmap='Blues', vmin=0, vmax=1, origin='upper', extent=[0, self.y_size, 0, self.x_size])
            plt.colorbar(self.im, ax=self.ax, label='Value')
            self.ax.set_xlabel('X [m]')
            self.ax.set_ylabel('Y [m]')
            self.ax.set_title('Dynamic Frequency Map')
            self.ani = FuncAnimation(self.fig, self.update_plot)
            plt.show()

    # Keep track of human positions
    def update_human_positions(self, msg):
        #print(msg)
        for human in msg.humans:
            #print("Update human: ", human.name)
            self.dynamic_objects[human.name] = human.pose

    # Update animation               
    def update_plot(self, frame):
        normalized_grid = self.normalize_grid()
        self.im.set_array(normalized_grid)
        return self.im

    # Round map boundaries 
    def round(self, num):
        if num < 0:
            return int(np.floor(num))
        else:
            return int(np.ceil(num))

    # Normalize frequency of observed obstacles
    def normalize_grid(self):
        max_val = np.max(self.grid)
        if max_val != 0:
            normalized_grid = self.grid / max_val
        else:
            normalized_grid = self.grid
        return normalized_grid

    # Return the normalized score of a requested position
    def dfm_callback(self, request, response):
        self.get_logger().info(f'dfm_callback: {request.point.x} {request.point.y}')
        normalized_grid = self.normalize_grid()
        x = int((request.point.x - self.round_min_x) / self.square_size)
        y = int((request.point.y - self.round_min_y) / self.square_size)
        response.score =  normalized_grid[x, y]
        print("DFM score at (", request.point.x, ",", request.point.y, ") = ", response.score)
        return response

    # Update the grid with new observations from topic
    def update_frequency_map_callback(self, msg):
        observation = (msg.point.x, msg.point.y)
        x = int((observation[0] - self.round_min_x) / self.square_size)
        y = int((observation[1] - self.round_min_y) / self.square_size)        
        self.grid[x, y] += 1

    # Update the grid with new observations from timer callback
    def update_timer_callback(self):
        for person in self.dynamic_objects.values():
            observation = (person.position.x, person.position.y)
            x = int((observation[0] - self.round_min_x) / self.square_size)
            y = int((observation[1] - self.round_min_y) / self.square_size)
            #print(f"X: {x}  Y: {y}")
            self.grid[x-1, y-1] += 1



def main(args=None):
    rclpy.init(args=args)

    node = DynamicFrequencyMap()

    executor = MultiThreadedExecutor(num_threads=8)
    # minimal_client.send_request()
    
    executor.add_node(node)

    thread = threading.Thread(target=node.visualize_dfm)
    thread.start()
    
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
            
