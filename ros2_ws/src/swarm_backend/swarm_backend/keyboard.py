#!/usr/bin/env python3

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

PX4_IN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

CONTROLS_MSG = """
Drone Keyboard Controller
---------------------------
Press SPACE to Arm & Takeoff

Movement (WASD):
  W / S : Move Forward / Backward
  A / D : Move Left / Right

Altitude & Yaw (IJKL):
  I / K : Fly Up / Fly Down
  J / L : Rotate Left / Rotate Right

CTRL-C to quit
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_IN_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_IN_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_IN_QOS)

        # 10Hz Timer for PX4 Heartbeat
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Velocity variables
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        
        # Save terminal settings for non-blocking input
        self.settings = termios.tcgetattr(sys.stdin)

    def timer_callback(self):
        # 1. Publish Heartbeat (Velocity Control = True)
        mode_msg = OffboardControlMode()
        mode_msg.position = False
        mode_msg.velocity = True
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(mode_msg)

        # 2. Publish Velocity Setpoints
        sp_msg = TrajectorySetpoint()
        # In PX4, setting position to NaN tells the drone to use velocity instead
        sp_msg.position = [float('nan'), float('nan'), float('nan')]
        sp_msg.velocity = [self.vx, self.vy, self.vz]
        sp_msg.yawspeed = self.yaw_rate
        sp_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(sp_msg)

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(CONTROLS_MSG)
        speed = 2.0  # meters per second
        
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x03':  # Ctrl-C
                    break
                
                if key == ' ':
                    self.send_command(400, param1=1.0)  # Arm
                    self.send_command(176, param1=1.0, param2=6.0)  # Offboard Mode
                    print("Armed & Offboard Mode Engaged!")
                
                # Reset velocities to 0 unless a key is actively pressed
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.yaw_rate = 0.0

                # Map keys to PX4 NED Coordinate System (North, East, Down)
                if key == 'w': self.vx = speed         # North
                elif key == 's': self.vx = -speed      # South
                elif key == 'd': self.vy = speed       # East
                elif key == 'a': self.vy = -speed      # West
                elif key == 'k': self.vz = speed       # Down (Positive Z)
                elif key == 'i': self.vz = -speed      # Up (Negative Z)
                elif key == 'j': self.yaw_rate = -1.0  # Yaw Left
                elif key == 'l': self.yaw_rate = 1.0   # Yaw Right

                rclpy.spin_once(self, timeout_sec=0)
                
        except Exception as e:
            print(f"Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()