#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64

from daep_msgs.msg import PigainNode, Score, PigainNodeList
from daep_msgs.srv import Query, BestNode, Reevaluate

import numpy as np
from rtree import index
from time import sleep
import threading

#Visualization of dynamic_gain
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib as mpl
import gp
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler  # was lrs_util (LiU-internal); same [x, y, z, w] order

class PIGain(Node):
    def __init__(self):
        super().__init__("pigain")

        self.lock = threading.RLock()

        self.group = ReentrantCallbackGroup()        

        self.declare_parameter("visualize/mean", False)        
        self.declare_parameter("visualize/sigma", False)        
        self.declare_parameter("visualize/pts", True)        
        self.declare_parameter("visualize/resolution", 0.5)
        self.declare_parameter("daep/gain/cache_node_threshold", 30)
        self.declare_parameter("daep/gain/node_gain_threshold", 10)
        self.declare_parameter("boundary/min", [-100.0, -100.0, 0.0])
        self.declare_parameter("boundary/max", [ 100.0,  100.0, 3.0])
        self.declare_parameter("aep/gain/r_max", 8.0)
        self.declare_parameter("add_to_frontier_ratio", 0.6)
        self.declare_parameter("reeval_ratio", 0.7)

        self.declare_parameter("world_frame", "world")

        self.visualize_mean  = self.get_parameter('visualize/mean').value
        self.visualize_sigma = self.get_parameter('visualize/sigma').value
        self.visualize_pts   = self.get_parameter('visualize/pts').value
        self.resolution      = self.get_parameter('visualize/resolution').value
        self.reevaluate_node_threshold = self.get_parameter('daep/gain/cache_node_threshold').value
        self.add_node_to_frontier_threshold  = self.get_parameter('daep/gain/node_gain_threshold').value
        self.min = self.get_parameter('boundary/min').value
        self.max = self.get_parameter('boundary/max').value
        self.range = self.get_parameter('aep/gain/r_max').value
        self.add_to_frontier_ratio = self.get_parameter('add_to_frontier_ratio').value
        self.reeval_ratio = self.get_parameter('reeval_ratio').value
        self.world_frame = self.get_parameter("world_frame").value
        print("WORLD FRAME PARAMETER:", self.world_frame)


        self.s = self.create_service(Query, "gp_query_server", self.query_server, callback_group=self.group)
        self.nn_yaw_srv = self.create_service(Query, "nn_yaw_query_server", self.nn_yaw_angle_srv_callback, callback_group=self.group)
        self.best_node_srv = self.create_service(BestNode, "best_node_server", self.best_node_srv_callback, callback_group=self.group)
        
        
        self.reevaluate_client = self.create_client(Reevaluate, "reevaluate", callback_group=self.group)               

        self.gain_sub = self.create_subscription(PigainNodeList, "gain_node", self.gain_callback, 1, callback_group=self.group)
        self.pose_sub = self.create_subscription(PoseStamped, "simple_pose", self.pose_callback, 1, callback_group=self.group)
        self.pose_sub = self.create_subscription(PoseStamped, "pose", self.dji_pose_callback, 1, callback_group=self.group)
    
    
        self.lock = threading.RLock()
        self.msg_event = threading.Event()   # will be set once msg arrives
        self.latest_max_volume = None
    
        # Transient local like aeplanner's publisher, so the value also arrives when pig.py starts late.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(Float64,'max_volume_bound',self.max_volume_bound_callback, latched, callback_group=self.group)

        # Wait until the first message arrives
        self.get_logger().info("Waiting for max_volume_bound message...")
        while not self.msg_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)  # process events

        self.daep_marker_array_pub = self.create_publisher(MarkerArray, "daep_marker_array_visualization", 100)
        self.frontier_marker_array_pub = self.create_publisher(MarkerArray, "frontier_marker_array", 10)
        self.mean_pub = self.create_publisher(MarkerArray, "mean_markers", 10)
        self.sigma_pub = self.create_publisher(MarkerArray, "sigma_markers", 10)
        
        #if self.visualize_pts:
            #self.pts_timer = self.create_timer(1.0, self.rviz_callback, callback_group=self.group)
        
        #if self.visualize_mean or self.visualize_sigma:
            #self.evaluate_timer = self.create_timer(5.0, self.evaluate, callback_group=self.group)

        self.bbx = (self.min[0], self.min[1], self.min[2], self.max[0], self.max[1], self.max[2])

        self.x = None
        self.y = None
        self.z = None

        self.hyperparam = gp.HyperParam(l = 1, sigma_f = 1, sigma_n = 0.1)

        # Create r-tree
        p = index.Property()
        p.dimension = 3
        self.idx = index.Index(properties = p)
        self.id = 0

        self.reeval_time = 15.0
        self.reevaluate_timer = self.create_timer(self.reeval_time, self.reevaluate_timer_callback, callback_group=self.group)
        self.inside_reevaluate = False

        self.frontier_visualizer = FrontierVisualizer(self, self.reeval_time)


    def wait_future(self, future, print_waiting=False):
        while not future.done():
            if not rclpy.ok():
                return None
            sleep(0.01)
        resp = future.result()
        return resp
        

    def max_volume_bound_callback(self, msg):
        with self.lock:
            self.latest_max_volume = msg.data
            self.get_logger().info(f"max_volume_bound_callback: {msg.data}")
            self.add_node_to_frontier_threshold = msg.data * self.add_to_frontier_ratio
            self.reevaluate_node_threshold = msg.data * self.reeval_ratio
            self.get_logger().info(f"Set add_node_to_frontier_threshold to {self.add_node_to_frontier_threshold}")
            self.get_logger().info(f"Set reevaluate_node_threshold to {self.reevaluate_node_threshold}")

        self.msg_event.set()   # unblock waiting code


    """ Save current pose of agent """
    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

    """ Save current pose of agent when flying with M100"""
    def dji_pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
    
    """ Extract the nearest neighbour yaw angle from a 2x2x2 bounding box around the query point"""
    def nn_yaw_angle_srv_callback(self, request, response):
        #self.get_logger().info(f'nn_yaw_angle_srv_callback enter')
        bbx = (request.point.x-2, request.point.y-2, request.point.z-2,
               request.point.x+2, request.point.y+2, request.point.z+2)
        # Find the nearest neighbor
        with self.lock:
            nearest = self.idx.nearest(bbx, 1, objects=True)
        # yaw of nearest neighbor
        yaw = 0.0
        for item in nearest:
            yaw = item.object.yaw
            
        response.mu = 0.0
        response.sigma = 0.0
        response.yaw = yaw 
        #self.get_logger().info(f'nn_yaw_angle_srv_callback exit: {yaw}')        
        return response

    def reevaluate_call(self, points):
        try:
            req = Reevaluate.Request()
            req.points = points
            if not self.reevaluate_client.wait_for_service(timeout_sec=1.0):
                if not getattr(self, '_warned_no_reevaluate', False):
                    self._warned_no_reevaluate = True
                    self.get_logger().warn('reevaluate service not available (it is off in '
                                           'aeplanner); cached frontiers are not reevaluated')
                return None
            future = self.reevaluate_client.call_async(req)
            resp = self.wait_future(future, print_waiting=True)
            return resp
        except Exception as exc:
            self.get_logger().error(f'EXCEPTION reevaluate_call: {type(exc)} - {exc}')
            
       


    """ Reevaluate gain in all cached nodes that are closer to agent than self.range 
    Here self.range = aep/gain/r_max * 2.0 = 5 * 2.0 = 10 m """
    def reevaluate_timer_callback(self):
        if self.inside_reevaluate:
            return
        self.inside_reevaluate = True
        
        if self.x is None or self.y is None or self.z is None:
            self.get_logger().warn("No position received yet...")
            self.get_logger().warn("Make sure that 'pose' has been correctly mapped and that it is being published")
            self.inside_reevaluate = False
            return
        
        # Bounding box with self.range = aep/gain/r_max * 2(Sensor range)
        bbx = (self.x-self.range, self.y-self.range, self.z-self.range, 
               self.x+self.range, self.y+self.range, self.z+self.range)

        # Get all objects (frontiers) in this bounding box
        with self.lock:
            hits = self.idx.intersection(bbx, objects=True)

        # If frontier's gain > self.reevaluate_node_threshold, reevaluate it. Otherwise delete it
        # This is to only reevaluate the most promising frontiers and remove the garbage
        reevaluate_list = []
        reevaluate_position_list = []
        #self.get_logger().info(f"reevaluate_timer_callback: Pick nodes to reeval with threshold {self.reevaluate_node_threshold}")                
        for item in hits:
            if(item.object.gain > self.reevaluate_node_threshold):
                reevaluate_position_list.append(item.object.position)
                reevaluate_list.append(item)
            else:
                with self.lock:
                    self.idx.delete(item.id, (item.object.position.x, item.object.position.y, item.object.position.z))
            
        # Aquire new gain for each node in reevaluate list
        res = self.reevaluate_call(reevaluate_position_list)
        if not res:
            if not getattr(self, '_warned_no_reevaluate', False):  # else reevaluate_call warned once
                self.get_logger().error("Calling reevaluate service failed")
            self.inside_reevaluate = False
            return

        # For every object in the reevaluate list, delete the previous item
        count = 0
        for i, item in enumerate(reevaluate_list):
            item.object.gain = res.gain[i]
            item.object.yaw = res.yaw[i]

            with self.lock:
                self.idx.delete(item.id, (item.object.position.x, item.object.position.y, item.object.position.z))

            # And if the new gain is higher than the threshold, insert it again
            #self.get_logger().info(f"reevaluate_timer_callback: After reeval, gain is: {item.object.gain} - Threshold {self.add_node_to_frontier_threshold}")                
            if(item.object.gain >= self.add_node_to_frontier_threshold):
                count = count + 1
                #print("BEFORE INSERT:", item.object.position.x, item.object.position.y, item.object.position.z)
                with self.lock:
                    self.idx.insert(item.id, (item.object.position.x, item.object.position.y, item.object.position.z), obj=item.object)
                #print("AFTER INSERT")

        self.get_logger().info(f"reevaluate service: re-insert {count} frontiers in idx after reevaluate with threshold: {self.add_node_to_frontier_threshold}")

        #self.rviz_callback()                
        self.inside_reevaluate = False


    """ Insert node with estimated gain in rtree """
    def gain_callback(self, msg):
        count = 0
        for node in msg.nodes:        
            # Only add nodes to the idx data structure if the gain is higher than the add_node_to_frontier_threshold
            if(node.gain > self.add_node_to_frontier_threshold):
                with self.lock:
                    self.idx.insert(self.id, (node.position.x, node.position.y, node.position.z), obj=node)
                    count = count + 1
                self.id += 1
        
        self.get_logger().info(f"gain_callback: Inserted {count} frontiers in idx with threshold: {self.add_node_to_frontier_threshold}")
        
    """ Handle query to Gaussian Process """    
    def query_server(self, request, response):
        self.get_logger().info(f"Querying gain at point: {request.point.x}, {request.point.y}, {request.point.z} with GP")
        # The bounding box with offset 2 from the drone
        bbx = (request.point.x-2, request.point.y-2, request.point.z-2, 
               request.point.x+2, request.point.y+2, request.point.z+2)
        y = np.empty((0))
        x = np.empty((0,3))

        # Retrieve all objects that intersects with the bbox
        while self.inside_reevaluate:
            sleep(0.1)
        with self.lock:
            hits = self.idx.intersection(bbx, objects=True)

        # Get the gain and position of every goal
        for item in hits:
            # We havent evaluated this node in the last time_threshold seconds
            time_threshold = 15 # Fix me
            time_since = item.object.time_since
            if time_since < time_threshold:
                dynamic_gain = item.object.dynamic_gain
                time_of_arrival = item.object.time
                pos = [item.object.position.x, item.object.position.y, item.object.position.z]
                y = np.append(y, [item.object.gain], axis=0)
                x = np.append(x, [pos], axis = 0)

        # yaw of nearest neighbor
        while self.inside_reevaluate:
            sleep(0.1)
        with self.lock:
            nearest = self.idx.nearest(bbx, 1, objects=True)
        yaw = 0
        for item in nearest:
            yaw = item.object.yaw

        # No hits
        if y.shape[0] == 0:
            response = QueryResponse()
            response.mu = 0
            response.sigma = 1
            return response

        xstar = np.array([[req.point.x, req.point.y, req.point.z]])
        mean, sigma = gp.gp(y, x, xstar, self.hyperparam, gp.sqexpkernel)

        response.mu = mean
        response.sigma = sigma
        response.yaw = yaw

        return response

    """ Return all nodes with gain higher than req.threshold in world bounding box """
    def best_node_srv_callback(self, request, response):
        # Retrieve objects that intersects with the bbox

        self.get_logger().info(f"Retrives frontiers with gain higher than threshold: {request.threshold}")
        
        with self.lock:
            hits = self.idx.intersection(self.bbx, objects=True)

        best_gain = -1.0

        for item in hits:
            #self.get_logger().info(f"best_node_srv_callback hit gain: {item.object.gain}")
            if item.object.gain > request.threshold:
                response.best_node.append(item.object.position)
            if item.object.gain > best_gain:
                best_gain = item.object.gain

        response.gain = best_gain
        return response

    """ Evaluate potential information gain function over grid and publish it in rviz """
    def evaluate(self):
        y = np.empty((0))
        x = np.empty((0,3))
        xstar = np.empty((0,3))
        # Retrieve objects that intersects with the bbox
        with self.lock:
            hits = self.idx.intersection(self.bbx, objects=True)
        
        for item in hits:
            y = np.append(y, [item.object.gain], axis=0)
            x = np.append(x, [[item.object.position.x, item.object.position.y, item.object.position.z]], axis = 0)

        xt = np.arange(self.min[0], self.max[0], self.resolution)
        yt = np.arange(self.min[1], self.max[1], self.resolution)
        zt = [1]

        for xx in xt:
            for yy in yt:
                for zz in zt:
                    xstar = np.append(xstar, [[xx, yy, zz]], axis = 0)

        mean, sigma = gp.gp(y, x, xstar, self.hyperparam, gp.sqexpkernel)
        mean_markers = MarkerArray()
        sigma_markers = MarkerArray()
        for id, pts in enumerate(zip(xstar, mean, sigma)):
            mean_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[1], max(1-pts[2], 0)))
            # sigma_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[2] * 2))
        
        self.mean_pub.publish(mean_markers)
        self.sigma_pub.publish(sigma_markers)

    """ Publish all cached nodes in rviz. This translates to visualizing the frontiers
     when we search for frontiers we search in all cached nodes. """
    def rviz_callback(self):
        markers = MarkerArray()
        yaw_markers = MarkerArray()
        text_markers = MarkerArray()
        
        # Retrieve objects that intersects with the map's bounding box
        with self.lock:
            hits = self.idx.intersection(self.bbx, objects=True)
        
        for item in hits:
            markers.markers.append(self.node_to_marker(item.id, item.object))
            yaw_markers.markers.append(self.node_to_marker_yaw(item.id, item.object))
            text_markers.markers.append(self.node_to_text(item.id, item.object))

        self.daep_marker_array_pub.publish(markers)
        self.daep_marker_array_pub.publish(yaw_markers)
        self.daep_marker_array_pub.publish(text_markers)


    def node_to_text(self, id, node):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.ns = "gain_text"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.id = id
        marker.scale.z = 0.5  # Text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.text = str(np.round(node.gain, 1))
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = node.position.x
        marker.pose.position.y = node.position.y
        marker.pose.position.z = node.position.z + 0.5  # Offset text above the node
        marker.lifetime = Duration(seconds=self.reeval_time).to_msg()        
        return marker

    def np_array_to_marker(self, id, p, v=0, a=0):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.1
        marker.color.r = v / 72.0
        marker.color.g = 0 
        marker.color.b = 0.5
        marker.color.a = a
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = p[0]
        marker.pose.position.y = p[1]
        marker.pose.position.z = p[2]
        marker.lifetime = Duration(seconds=self.reeval_time).to_msg()

        return marker


    def node_to_marker(self, id, node):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.ns = "pig"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 1.0 # node.gain / 72.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.text = str(np.round(node.gain, 1))
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = node.position.x
        marker.pose.position.y = node.position.y
        marker.pose.position.z = node.position.z
        marker.lifetime = Duration(seconds=self.reeval_time).to_msg() #Show markers for 10 seconds (This is to remove old markes that no longer exists in idx)

        return marker

    def node_to_marker_yaw(self, id, node):
        yaw = node.yaw
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.ns = "yaw"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = node.gain / 72.0
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0
        
        q = quaternion_from_euler(0.0, 0.0, yaw)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.pose.position.x = node.position.x
        marker.pose.position.y = node.position.y
        marker.pose.position.z = node.position.z
        marker.lifetime = Duration(seconds=self.reeval_time).to_msg()  #Show markers for 10 seconds (This is to remove old markes that no longer exists in idx)

        return marker


import copy
import threading

class FrontierVisualizer:
    def __init__(self, pigain_node, interval=5.0):
        self.pigain_node = pigain_node
        self.idx = index.Index(properties=index.Property(dimension=3))
        self.lock = threading.RLock()
        self.running = True
        self.interval = interval  # seconds

        # Start the visualization thread
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def update_idx(self):
        # Deep copy the idx from PIGain in a thread-safe way
        with self.pigain_node.lock:
            # Create a new R-tree index
            p = index.Property()
            p.dimension = 3
            new_idx = index.Index(properties=p)
            # Iterate through all objects in the original idx and insert them into the new one
            for item in self.pigain_node.idx.intersection(self.pigain_node.bbx, objects=True):
                # Deepcopy the object to avoid shared references
                obj_copy = copy.deepcopy(item.object)
                new_idx.insert(item.id, (obj_copy.position.x, obj_copy.position.y, obj_copy.position.z), obj=obj_copy)
            self.idx = new_idx

    def visualize(self):
        # Visualize all nodes in self.idx
        self.pigain_node.get_logger().info("FrontierVisualizer: Updating visualization...")
        with self.lock:
            markers = MarkerArray()
            yaw_markers = MarkerArray()
            text_markers = MarkerArray()

            with self.pigain_node.lock:
                hits = list(self.idx.intersection(self.pigain_node.bbx, objects=True))
            
            for item in hits:
                markers.markers.append(self.pigain_node.node_to_marker(item.id, item.object))
                yaw_markers.markers.append(self.pigain_node.node_to_marker_yaw(item.id, item.object))
                text_markers.markers.append(self.pigain_node.node_to_text(item.id, item.object))

            self.pigain_node.frontier_marker_array_pub.publish(markers)
            self.pigain_node.frontier_marker_array_pub.publish(yaw_markers)
            self.pigain_node.frontier_marker_array_pub.publish(text_markers)

    def run(self):
        while self.running:
            self.update_idx()
            self.visualize()
            threading.Event().wait(self.interval)  # Sleep for interval seconds

    def stop(self):
        self.running = False
        self.thread.join()

def main(args=None):
    rclpy.init(args=args)

    node = PIGain()
    executor = MultiThreadedExecutor(num_threads=8)   
    executor.add_node(node)
 
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C, or ROS shut down by its own SIGINT handler (Jazzy)
    finally:
        node.frontier_visualizer.stop()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()  # ROS may already be shut down

if __name__ == '__main__':
    main()

