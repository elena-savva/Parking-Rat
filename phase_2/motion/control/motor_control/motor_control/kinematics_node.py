#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
 
from .motorDriver import DaguWheelsDriver
 
 
class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
 
        # Init
        self.declare_parameter('baseline', 0.185)
        self.declare_parameter('wheel_radius', 0.0325)
 
        self.declare_parameter('gain', 0.60)
        self.declare_parameter('trim', -0.2400)
 
        self.declare_parameter('max_physical_velocity', 0.5)
 
        # Fetch (Only if using ros2 launch command)
        self.baseline     = self.get_parameter('baseline').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.gain         = self.get_parameter('gain').value
        self.trim         = self.get_parameter('trim').value
        self.max_vel      = self.get_parameter('max_physical_velocity').value
 
        self.get_logger().info(
            f"Kinematics initialized with shared gain/trim. "
            f"gain={self.gain}  trim={self.trim}"
        )
 
        # Motor driver initialization
        try:
            self.motor = DaguWheelsDriver()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motor driver: {e}")
            self.motor = None
 
        # Velocity command subscription
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
 
    def cmd_vel_callback(self, msg):
        v     = msg.linear.x
        omega = msg.angular.z
 
        # Differential drive kinematics
        v_l_ideal = v - (omega * self.baseline / 2.0)
        v_r_ideal = v + (omega * self.baseline / 2.0)
 
        # Calibrate with shared gain and trim
        v_l_calib = v_l_ideal * self.gain * (1.0 - self.trim)
        v_r_calib = v_r_ideal * self.gain * (1.0 + self.trim)
 
        # Convert to motor commands (normalized -1.0 to 1.0)
        left_motor_cmd  = v_l_calib / self.max_vel
        right_motor_cmd = v_r_calib / self.max_vel
 
        # Normalize to max range [-1.0, 1.0]
        left_motor_cmd  = max(min(left_motor_cmd,  1.0), -1.0)
        right_motor_cmd = max(min(right_motor_cmd, 1.0), -1.0)
 
        # Send to motor driver
        if self.motor is not None:
            self.motor.set_wheels_speed(left=left_motor_cmd, right=right_motor_cmd)
 
    def destroy_node(self):
        if self.motor is not None:
            self.motor.set_wheels_speed(left=0.0, right=0.0)
            self.motor.close()
        super().destroy_node()
 
 
def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()