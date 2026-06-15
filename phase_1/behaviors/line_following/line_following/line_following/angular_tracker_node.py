#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
import time

class AngularTrackerNode(Node):
    def __init__(self):
        super().__init__('angular_tracker')

        self._enabled = False  # disabled until navigator says go

        # Subscribe to the pixel error from our vision node
        self.subscription = self.create_subscription(
            Float32, '/vision_error_x', self.error_callback, 10)

        # Enable/disable from navigator
        self.create_subscription(
            Bool, '/linefollower/enabled', self._cb_enabled, 10)

        # Publish directly to the wheels
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- PID Tuning Parameters ----
        self.kp = 0.02
        self.ki = 0.0005
        self.kd = 0.001

        # Memory variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

        # Deadband: Don't jitter if we are extremely close to the center
        self.deadband = 15.0

        self.get_logger().info('Angular tracker ready! Waiting for enable...')

    def _cb_enabled(self, msg):
        self._enabled = msg.data
        if not self._enabled:
            # Reset PID state and stop immediately when disabled
            self.integral   = 0.0
            self.prev_error = 0.0
            self.publisher.publish(Twist())
            self.get_logger().info('Angular tracker disabled.')
        else:
            self.get_logger().info('Angular tracker enabled.')

    def error_callback(self, msg):
        if not self._enabled:
            return  # not our turn — stay silent

        error_x = msg.data
        cmd = Twist()

        # 1. Line is lost -> Stop completely
        if error_x == 0.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.integral = 0.0
            self.get_logger().debug('Track Lost - Stopped.')

        # 2. Line is perfectly centered -> Drive straight forward
        elif abs(error_x) < self.deadband:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            self.get_logger().debug('Track Centered - Driving straight.')

        # 3. Line is drifting -> Move forward while steering to correct
        else:
            curr_time = time.time()
            dt = curr_time - self.prev_time
            if dt <= 0.0: dt = 0.01

            p_out = self.kp * error_x
            self.integral += (error_x * dt)
            self.integral = max(min(self.integral, 1000.0), -1000.0)
            i_out = self.ki * self.integral
            derivative = (error_x - self.prev_error) / dt
            d_out = self.kd * derivative

            raw_angular = p_out + i_out + d_out

            cmd.linear.x = 0.12
            cmd.angular.z = max(min(raw_angular, 1.5), -1.5)

            self.prev_error = error_x
            self.prev_time = curr_time

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AngularTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()