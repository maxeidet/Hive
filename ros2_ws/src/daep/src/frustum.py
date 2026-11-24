#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion
from rclpy.callback_groups import ReentrantCallbackGroup


class FrustumVisualizer(Node):
    def __init__(self, publisher_name):
        # Initialize the node
        super().__init__('frustum_visualizer')
        self.group = ReentrantCallbackGroup()        


        # Create publisher for the frustum marker
        self.frustum_pub = self.create_publisher(Marker, publisher_name, 1000)

        # Subscribe to pose updates
        self.create_subscription(
            PoseStamped,
            'simple_pose',
            self.pose_callback,
            10, 
            callback_group=self.group
        )

        # Subscribe to pose updates
        self.create_subscription(
            PoseStamped,
            'pose',
            self.dji_pose_callback,
            10, 
            callback_group=self.group
        )
        
        # Declare parameters for ROS2
        self.declare_parameter("camera/vertical_fov", 45.0)
        self.declare_parameter("camera/horizontal_fov", 60.0)
        self.declare_parameter("aep/gain/r_min", 0.1)
        self.declare_parameter("aep/gain/r_max", 5.0)
        self.declare_parameter("world_frame", "world")
        
        # Get parameters
        self.vertical_fov = self.get_parameter("camera/vertical_fov").value
        self.horizontal_fov = self.get_parameter("camera/horizontal_fov").value
        self.r_min = self.get_parameter("aep/gain/r_min").value
        self.r_max = self.get_parameter("aep/gain/r_max").value
        self.frame_id = self.get_parameter("world_frame").value
        print("WORLD FRAME PARAMETER:", self.frame_id)

        # Store current state (x, y, z, yaw)
        self.current_state = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Set up the timer for continuous visualization
        self.timer = self.create_timer(0.1, self.visualize_callback)
        
        self.get_logger().info("Frustum visualizer initialized.")
    
    def pose_callback(self, pose_msg):
        # Extract position
        self.current_state[0] = pose_msg.pose.position.x
        self.current_state[1] = pose_msg.pose.position.y
        self.current_state[2] = pose_msg.pose.position.z 
        
        # Extract yaw from quaternion
        orientation_q = pose_msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_state[3] = yaw
    
    def dji_pose_callback(self, pose_msg):
        # Extract position
        self.current_state[0] = pose_msg.pose.position.x
        self.current_state[1] = pose_msg.pose.position.y
        self.current_state[2] = pose_msg.pose.position.z
        
        # Extract yaw from quaternion
        orientation_q = pose_msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_state[3] = yaw
    
    def visualize_callback(self):
        self.visualize_frustum(
            self.frustum_pub,
            self.horizontal_fov,
            self.vertical_fov,
            self.r_min,
            self.r_max,
            self.current_state
        )
    
    def visualize_frustum(self, frustum_pub, hfov, vfov, r_min, r_max, current_state):
        # Convert field of view from degrees to radians
        hfov_rad = hfov * np.pi / 180.0
        vfov_rad = vfov * np.pi / 180.0
        
        # Extract position and yaw from current state
        x = current_state[0]
        y = current_state[1]
        z = current_state[2]
        yaw = current_state[3]
        
        # Calculate half angles
        h_half_angle = hfov_rad / 2.0
        v_half_angle = vfov_rad / 2.0
        
        # Create a line list marker for the frustum
        frustum_marker = Marker()
        frustum_marker.header.frame_id = self.frame_id
        frustum_marker.header.stamp = self.get_clock().now().to_msg()
        frustum_marker.ns = "frustum"
        frustum_marker.id = 0
        frustum_marker.type = Marker.LINE_LIST
        frustum_marker.action = Marker.ADD
        
        # Set the pose origin (position of the agent)
        frustum_marker.pose.position.x = x
        frustum_marker.pose.position.y = y
        frustum_marker.pose.position.z = z
        
        # Set orientation to identity quaternion as we'll calculate points manually
        frustum_marker.pose.orientation.x = 0.0
        frustum_marker.pose.orientation.y = 0.0
        frustum_marker.pose.orientation.z = 0.0
        frustum_marker.pose.orientation.w = 1.0
        
        # Set scale - thickness of lines
        frustum_marker.scale.x = 0.05  # Line width
        
        # Set color - use green with some transparency
        frustum_marker.color.r = 0.0
        frustum_marker.color.g = 1.0
        frustum_marker.color.b = 0.0
        frustum_marker.color.a = 0.7
        
        # Set lifetime
        frustum_marker.lifetime.sec = 0  # 0 = forever
        
        # Calculate the eight corners of the frustum in local coordinates
        corners_near = []
        corners_far = []
        
        # Calculate corners of near plane
        corners_near.append(np.array([r_min * np.cos(-h_half_angle), r_min * np.sin(-h_half_angle), r_min * np.tan(-v_half_angle)]))
        corners_near.append(np.array([r_min * np.cos(-h_half_angle), r_min * np.sin(-h_half_angle), r_min * np.tan(v_half_angle)]))
        corners_near.append(np.array([r_min * np.cos(h_half_angle), r_min * np.sin(h_half_angle), r_min * np.tan(v_half_angle)]))
        corners_near.append(np.array([r_min * np.cos(h_half_angle), r_min * np.sin(h_half_angle), r_min * np.tan(-v_half_angle)]))
        
        # Calculate corners of far plane
        corners_far.append(np.array([r_max * np.cos(-h_half_angle), r_max * np.sin(-h_half_angle), r_max * np.tan(-v_half_angle)]))
        corners_far.append(np.array([r_max * np.cos(-h_half_angle), r_max * np.sin(-h_half_angle), r_max * np.tan(v_half_angle)]))
        corners_far.append(np.array([r_max * np.cos(h_half_angle), r_max * np.sin(h_half_angle), r_max * np.tan(v_half_angle)]))
        corners_far.append(np.array([r_max * np.cos(h_half_angle), r_max * np.sin(h_half_angle), r_max * np.tan(-v_half_angle)]))
        
        # Rotation matrix for yaw rotation
        rot = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Origin point (relative to the drone's position)
        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0
        
        # LINE_LIST needs pairs of points
        frustum_marker.points = []
        
        # Add lines for the near rectangle
        for i in range(4):
            # Rotate corners to world frame
            rotated_corner = np.dot(rot, corners_near[i])
            
            # Create a point for the corner
            p1 = Point()
            p1.x = rotated_corner[0]
            p1.y = rotated_corner[1]
            p1.z = rotated_corner[2]
            
            # Connect to next corner of near rectangle
            rotated_next = np.dot(rot, corners_near[(i + 1) % 4])
            p2 = Point()
            p2.x = rotated_next[0]
            p2.y = rotated_next[1]
            p2.z = rotated_next[2]
            
            # Add line segment
            frustum_marker.points.append(p1)
            frustum_marker.points.append(p2)
        
        # Add lines for the far rectangle
        for i in range(4):
            # Rotate corners to world frame
            rotated_corner = np.dot(rot, corners_far[i])
            
            # Create a point for the corner
            p1 = Point()
            p1.x = rotated_corner[0]
            p1.y = rotated_corner[1]
            p1.z = rotated_corner[2]
            
            # Connect to next corner of far rectangle
            rotated_next = np.dot(rot, corners_far[(i + 1) % 4])
            p2 = Point()
            p2.x = rotated_next[0]
            p2.y = rotated_next[1]
            p2.z = rotated_next[2]
            
            # Add line segment
            frustum_marker.points.append(p1)
            frustum_marker.points.append(p2)
        
        # Add lines connecting near and far rectangles
        for i in range(4):
            # Rotate corners to world frame
            rotated_near = np.dot(rot, corners_near[i])
            rotated_far = np.dot(rot, corners_far[i])
            
            # Create points
            p_near = Point()
            p_near.x = rotated_near[0]
            p_near.y = rotated_near[1]
            p_near.z = rotated_near[2]
            
            p_far = Point()
            p_far.x = rotated_far[0]
            p_far.y = rotated_far[1]
            p_far.z = rotated_far[2]
            
            # Add line segment
            frustum_marker.points.append(p_near)
            frustum_marker.points.append(p_far)
        
        # Add lines from origin to near corners to represent the viewing pyramid
        for i in range(4):
            # Rotate corner to world frame
            rotated_corner = np.dot(rot, corners_near[i])
            
            # Create point
            p = Point()
            p.x = rotated_corner[0]
            p.y = rotated_corner[1]
            p.z = rotated_corner[2]
            
            # Add line segment
            frustum_marker.points.append(origin)
            frustum_marker.points.append(p)
        
        # Publish the marker
        frustum_pub.publish(frustum_marker)


class BoundingBoxVisualizer(Node):
    def __init__(self, publisher_name):
        # Initialize the node   
        super().__init__('bounding_box_visualizer')
        self.group = ReentrantCallbackGroup()        

        # Create publisher for the bounding box marker
        self.bb_pub = self.create_publisher(Marker, publisher_name, 10)
        
        # Declare parameters for ROS2
        self.declare_parameter("boundary/min", [0.0, 0.0, 0.0])
        self.declare_parameter("boundary/max", [10.0, 10.0, 10.0])
        self.declare_parameter("world_frame", "world")
        
        # Get parameters
        self.boundary_min = self.get_parameter("boundary/min").value
        self.boundary_max = self.get_parameter("boundary/max").value
        self.frame_id = self.get_parameter("world_frame").value
        print(f'{self.boundary_min=}')
        print(f'{self.boundary_max=}')
        
        # Set up the timer for continuous visualization
        self.timer = self.create_timer(1.0, self.visualize_callback)
        
        self.get_logger().info("Bounding Box visualizer initialized.")
    
    def visualize_callback(self):
        self.visualize_world_bounding_box(
            self.bb_pub,
            self.boundary_min,
            self.boundary_max
        )
    
    def visualize_world_bounding_box(self, bb_pub, boundary_min, boundary_max):
        # Create a visualization marker
        marker = Marker()
        
        # Set the frame ID and timestamp
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Set namespace and ID for this marker
        marker.ns = "world_bounds"
        marker.id = 0
        
        # Set the marker type to CUBE
        marker.type = Marker.CUBE
        
        # Set the marker action to ADD
        marker.action = Marker.ADD
        
        # Set the pose of the marker
        # Position is at the center of the bounding box
        marker.pose.position.x = (boundary_max[0] + boundary_min[0]) / 2.0
        marker.pose.position.y = (boundary_max[1] + boundary_min[1]) / 2.0
        marker.pose.position.z = (boundary_max[2] + boundary_min[2]) / 2.0
        
        # Orientation - using default (identity quaternion)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker (dimensions of the box)
        marker.scale.x = boundary_max[0] - boundary_min[0]
        marker.scale.y = boundary_max[1] - boundary_min[1]
        marker.scale.z = boundary_max[2] - boundary_min[2]
        
        # Set the color -- red with transparency
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3  # Slightly transparent
        
        # Set the lifetime
        marker.lifetime.sec = 0  # 0 = forever
        
        # Publish the marker
        bb_pub.publish(marker)


class RobotBoundingBox(Node):
    def __init__(self, publisher_name):
        # Initialize the node
        super().__init__('robot_bounding_box_visualizer')
        self.group = ReentrantCallbackGroup()        
        # Create publisher for the robot bounding box marker
        self.robot_bb_pub = self.create_publisher(Marker, publisher_name, 10)
        
        # Declare parameters for ROS2
        self.declare_parameter("system/bbx/r", 0.5)  # Default radius of 0.5 if not specified
        self.declare_parameter("system/bbx/use_sphere_collision_checking", False)  # Default radius of 0.5 if not specified
        self.declare_parameter("system/bbx/x", 0.5)  # Default radius of 0.5 if not specified
        self.declare_parameter("system/bbx/y", 0.5)  # Default radius of 0.5 if not specified
        self.declare_parameter("system/bbx/z", 0.5)  # Default radius of 0.5 if not specified
        self.declare_parameter("world_frame", "world")
        
        # Get parameter
        self.robot_radius = self.get_parameter("system/bbx/r").value
        self.sphere_or_cube = self.get_parameter("system/bbx/use_sphere_collision_checking").value
        self.x_diameter = self.get_parameter("system/bbx/x").value
        self.y_diameter = self.get_parameter("system/bbx/y").value  
        self.z_diameter = self.get_parameter("system/bbx/z").value
        self.frame_id = self.get_parameter("world_frame").value

        # Store current position
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Subscribe to pose updates
        self.create_subscription(
            PoseStamped,
            'simple_pose',
            self.pose_callback,
            10,
            callback_group=self.group
        )

          # Subscribe to pose updates
        self.create_subscription(
            PoseStamped,
            'pose',
            self.dji_pose_callback,
            10, 
            callback_group=self.group
        )
        
        # Set up the timer for continuous visualization
        self.timer = self.create_timer(0.1, self.visualize_callback)
        
        if(self.sphere_or_cube):
            self.get_logger().info(f"Robot Bounding Sphere visualizer initialized with radius: {self.robot_radius}")
        else:
         self.get_logger().info(f"Robot Bounding Box visualizer initialized with diameter ({self.x_diameter}, {self.y_diameter}, {self.z_diameter})")    
    
    def pose_callback(self, pose_msg):
        # Extract position
        self.current_position[0] = pose_msg.pose.position.x
        self.current_position[1] = pose_msg.pose.position.y
        self.current_position[2] = pose_msg.pose.position.z
        self.current_orientation[0] = pose_msg.pose.orientation.x
        self.current_orientation[1] = pose_msg.pose.orientation.y
        self.current_orientation[2] = pose_msg.pose.orientation.z
        self.current_orientation[3] = pose_msg.pose.orientation.w


    def dji_pose_callback(self, pose_msg):
        self.current_position[0] = pose_msg.pose.position.x
        self.current_position[1] = pose_msg.pose.position.y
        self.current_position[2] = pose_msg.pose.position.z
        self.current_orientation[0] = pose_msg.pose.orientation.x
        self.current_orientation[1] = pose_msg.pose.orientation.y
        self.current_orientation[2] = pose_msg.pose.orientation.z
        self.current_orientation[3] = pose_msg.pose.orientation.w

    def visualize_callback(self):
        self.visualize_robot_bounding_box(
            self.robot_bb_pub,
            self.robot_radius,
            self.current_position,
            self.sphere_or_cube,
            self.x_diameter,
            self.y_diameter,
            self.z_diameter
        )

    def visualize_robot_bounding_box(self, robot_bb_pub, radius, position, sphere_or_cube, x_diameter, y_diameter, z_diameter):
        # Create a visualization marker
        marker = Marker()
        
        # Set the frame ID and timestamp
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Set namespace and ID for this marker
        marker.ns = "robot_bounds"
        marker.id = 0
        
        # Set the marker type to SPHERE
        if sphere_or_cube:
            marker.type = Marker.SPHERE
        else:
            marker.type = Marker.CUBE
        
        # Set the marker action to ADD
        marker.action = Marker.ADD
        
        # Set the pose of the marker
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        # Orientation - using default (identity quaternion)
        marker.pose.orientation.x = self.current_orientation[0]
        marker.pose.orientation.y = self.current_orientation[1]
        marker.pose.orientation.z = self.current_orientation[2]
        marker.pose.orientation.w = self.current_orientation[3]
        
        # Set the scale of the marker (sphere diameter in each dimension)
        if(sphere_or_cube):
            marker.scale.x = 2.0 * radius  # Diameter = 2 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = 2.0 * radius
        else:
            marker.scale.x = x_diameter
            marker.scale.y = y_diameter
            marker.scale.z = z_diameter
            
        # Set the color -- blue with transparency
        marker.color.r = 1.0
        marker.color.g = 0.714
        marker.color.b = 0.0
        marker.color.a = 0.9  # Semi-transparent
        
        # Set the lifetime
        marker.lifetime.sec = 0  # 0 = forever
        
        # Publish the marker
        robot_bb_pub.publish(marker)


class PathVisualizer(Node):
    def __init__(self, publisher_name):
        # Initialize the node
        super().__init__('path_visualizer')
        self.group = ReentrantCallbackGroup()
        
        # Create publisher for the path marker
        self.path_pub = self.create_publisher(Marker, publisher_name, 10)
        
        self.declare_parameter("world_frame", "world")
        self.frame_id = self.get_parameter("world_frame").value
        
        # Subscribe to pose updates
        self.create_subscription(
            PoseStamped,
            'simple_pose',
            self.pose_callback,
            10,
            callback_group=self.group
        )

        # Subscribe to pose updates
        self.create_subscription(
            PoseStamped,
            'pose',
            self.dji_pose_callback,
            10,
            callback_group=self.group
        )
        
        # Store the path as a list of points
        self.path_points = []

        self.update_interval = 1.0
        
        self.last_update_time = self.get_clock().now()
        
        # Set up the timer for continuous visualization
        self.timer = self.create_timer(0.1, self.visualize_callback)
        
        self.get_logger().info("Path visualizer initialized.")
    
    def pose_callback(self, pose_msg):
        current_time = self.get_clock().now()
        if (current_time - self.last_update_time).nanoseconds * 1e-9 < self.update_interval:
            return  # Skip updates if within delay period
        
        self.last_update_time = current_time
        
        # Extract position
        new_point = Point()
        new_point.x = pose_msg.pose.position.x
        new_point.y = pose_msg.pose.position.y
        new_point.z = pose_msg.pose.position.z
        
        # Append new position to the path
        self.path_points.append(new_point)

    def dji_pose_callback(self, pose_msg):
        current_time = self.get_clock().now()
        if (current_time - self.last_update_time).nanoseconds * 1e-9 < self.update_interval:
            return  # Skip updates if within delay period
        
        self.last_update_time = current_time
        
        # Extract position
        new_point = Point()
        new_point.x = pose_msg.pose.position.x
        new_point.y = pose_msg.pose.position.y
        new_point.z = pose_msg.pose.position.z
        
        # Append new position to the path
        self.path_points.append(new_point)
    
    def visualize_callback(self):
        self.visualize_path(self.path_pub, self.path_points)
    
    def visualize_path(self, path_pub, path_points):
        # Create a visualization marker
        path_marker = Marker()
        
        # Set the frame ID and timestamp
        path_marker.header.frame_id = self.frame_id
        path_marker.header.stamp = self.get_clock().now().to_msg()
        
        # Set namespace and ID for this marker
        path_marker.ns = "robot_path"
        path_marker.id = 0
        
        # Set the marker type to LINE_STRIP
        path_marker.type = Marker.LINE_STRIP
        
        # Set the marker action to ADD
        path_marker.action = Marker.ADD
        
        # Set the scale - line width
        path_marker.scale.x = 0.05  # Thin line
        
        # Set color - blue with transparency
        path_marker.color.r = 1.0
        path_marker.color.g = 0.714
        path_marker.color.b = 0.0
        path_marker.color.a = 0.8  # Semi-transparent
        
        # Set the lifetime
        path_marker.lifetime.sec = 0  # 0 = forever
        
        # Assign points to the marker
        path_marker.points = path_points
        
        # Publish the marker
        path_pub.publish(path_marker)


def main(args=None):
    rclpy.init(args=args)

    # Common publisher, different namespaces
    publisher_name = "bounds_visualization"

    # Create both visualizers
    frustum_visualizer = FrustumVisualizer(publisher_name)
    bounding_box_visualizer = BoundingBoxVisualizer(publisher_name)
    robot_bounding_box = RobotBoundingBox(publisher_name)
    path_visualizer = PathVisualizer(publisher_name)

    # Use a MultiThreadedExecutor to run both nodes
    import threading
    
    # Use MultiThreadedExecutor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(frustum_visualizer)
    executor.add_node(bounding_box_visualizer)
    executor.add_node(robot_bounding_box)
    executor.add_node(path_visualizer)

    try:
        executor.spin()  # This will manage both nodes in parallel
    except KeyboardInterrupt:
        pass
    finally:
        frustum_visualizer.destroy_node()
        bounding_box_visualizer.destroy_node()
        robot_bounding_box.destroy_node()
        path_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
