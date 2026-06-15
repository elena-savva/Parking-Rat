#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import math

from .encoderDriver import WheelEncoderDriver


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # ── Publisher ─────────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ── Robot physical parameters ─────────────────────────────────────────
        self.declare_parameter('baseline', 0.185)
        # Declared to prevent launch file errors, but no longer used for math
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('ticks_per_rev', 137.0)
        self.declare_parameter('use_imu_heading', False)
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('imu_timeout_sec', 0.5)
        self.declare_parameter('log_imu_debug', True)
        self.declare_parameter('goal_reached_topic', '/goal_reached')
        self.declare_parameter('stop_logging_after_goal', True)
        self.declare_parameter('imu_debug_log_period_sec', 1.0)

        self.baseline = self.get_parameter('baseline').value
        self.use_imu_heading = bool(self.get_parameter('use_imu_heading').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        self.log_imu_debug = self.parameter_as_bool(
            self.get_parameter('log_imu_debug').value
        )
        self.goal_reached_topic = str(
            self.get_parameter('goal_reached_topic').value
        )
        self.stop_logging_after_goal = self.parameter_as_bool(
            self.get_parameter('stop_logging_after_goal').value
        )
        self.imu_debug_log_period_sec = max(
            float(self.get_parameter('imu_debug_log_period_sec').value),
            0.1,
        )

        # ── Empirical Odometry Calibration ────────────────────────────────────
        # Derived from physical 1-meter straight line test
        self.left_meters_per_tick  = 1.0 / 660.0  #660 713
        self.right_meters_per_tick = 1.0 / 713.0

        # ── Encoders ──────────────────────────────────────────────────────────
        self.left_encoder  = WheelEncoderDriver(12)
        self.right_encoder = WheelEncoderDriver(35)

        # ── Pose state ────────────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.imu_theta = 0.0

        self.prev_left_ticks  = 0
        self.prev_right_ticks = 0
        self.prev_imu_time = None
        self.last_imu_time = None
        self.last_imu_gyro_z = 0.0
        self.last_imu_debug_log_time = None
        self.have_imu = False
        self.goal_reached = False
        self.warned_about_imu = False

        # ── Per-wheel direction signs ─────────────────────────────────────────
        # Encoders only count UP so we need to know the sign of each wheel's
        # velocity to apply the correct direction to the tick delta.
        # We derive this from /cmd_vel using the differential drive kinematics.
        self.dir_left  = 1.0
        self.dir_right = 1.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(
            Bool,
            self.goal_reached_topic,
            self.goal_reached_callback,
            10,
        )
        if self.use_imu_heading:
            self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        # ── Run odometry at 100 Hz ─────────────────────────────────────────────
        self.create_timer(0.01, self.update_odometry)

        self.get_logger().info(
            f"Odometry node ready. "
            f"baseline={self.baseline}m  "
            f"left_m/tick={self.left_meters_per_tick:.6f}  "
            f"right_m/tick={self.right_meters_per_tick:.6f}  "
            f"use_imu_heading={self.use_imu_heading}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        """
        Derive the sign of each wheel's velocity from the commanded
        linear and angular velocity using differential drive kinematics.

        This is needed because the encoders are unidirectional (always count
        up) so we must infer direction from the commanded motion.
        """
        v     = msg.linear.x
        omega = msg.angular.z

        v_left  = v - (omega * self.baseline / 2.0)
        v_right = v + (omega * self.baseline / 2.0)

        # Only update direction if the wheel is actually commanded to move.
        # If velocity is near zero, keep the previous sign to avoid flipping
        # direction on noise.
        DEADBAND = 0.001
        if abs(v_left) > DEADBAND:
            self.dir_left = 1.0 if v_left >= 0 else -1.0
        if abs(v_right) > DEADBAND:
            self.dir_right = 1.0 if v_right >= 0 else -1.0

    def imu_callback(self, msg: Imu):
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()

        if self.prev_imu_time is None:
            self.prev_imu_time = stamp
            self.last_imu_time = stamp
            self.last_imu_gyro_z = msg.angular_velocity.z
            self.imu_theta = self.theta
            self.have_imu = True
            self.get_logger().info(
                'Initial IMU measurement | '
                f'gyro_z={math.degrees(msg.angular_velocity.z):.2f} deg/s | '
                f'heading={math.degrees(self.imu_theta):.1f} deg'
            )
            return

        dt = (stamp - self.prev_imu_time).nanoseconds / 1e9
        self.prev_imu_time = stamp
        self.last_imu_time = stamp
        self.last_imu_gyro_z = msg.angular_velocity.z

        if dt <= 0.0 or dt > 1.0:
            return

        self.imu_theta = self.wrap_angle(
            self.imu_theta + msg.angular_velocity.z * dt
        )
        self.have_imu = True

    def get_quaternion_from_yaw(self, yaw: float) -> Quaternion:
        """Convert a yaw angle (radians) to a ROS Quaternion message."""
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )

    def imu_heading_available(self) -> bool:
        if not self.use_imu_heading or not self.have_imu:
            return False

        if self.last_imu_time is None:
            return False

        age = (self.get_clock().now() - self.last_imu_time).nanoseconds / 1e9
        return age <= self.imu_timeout_sec

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def update_odometry(self):
        """Main odometry update — called at 100 Hz."""

        # ── Read encoder ticks ────────────────────────────────────────────────
        curr_l = self.left_encoder._ticks
        curr_r = self.right_encoder._ticks

        delta_l = curr_l - self.prev_left_ticks
        delta_r = curr_r - self.prev_right_ticks

        self.prev_left_ticks  = curr_l
        self.prev_right_ticks = curr_r

        # ── Convert ticks to signed distances ─────────────────────────────────
        d_left  = delta_l * self.left_meters_per_tick * self.dir_left
        d_right = delta_r * self.right_meters_per_tick * self.dir_right

        # ── Differential drive kinematics ─────────────────────────────────────
        d_center    = (d_left + d_right) / 2.0
        encoder_delta_theta = (d_right - d_left) / self.baseline

        prev_theta = self.theta
        if self.imu_heading_available():
            next_theta = self.imu_theta
            delta_theta = self.wrap_angle(next_theta - prev_theta)
        else:
            next_theta = self.wrap_angle(prev_theta + encoder_delta_theta)
            delta_theta = encoder_delta_theta
            if self.use_imu_heading and not self.warned_about_imu:
                self.get_logger().warn(
                    'IMU heading enabled, but no recent IMU data is available. '
                    'Using encoder heading fallback.'
                )
                self.warned_about_imu = True

        # Update pose using midpoint angle for better accuracy during turns
        self.x     += d_center * math.cos(prev_theta + delta_theta / 2.0)
        self.y     += d_center * math.sin(prev_theta + delta_theta / 2.0)
        self.theta = next_theta

        # ── Publish Odometry message ───────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation = self.get_quaternion_from_yaw(self.theta)

        self.odom_pub.publish(odom)
        self.maybe_log_imu_debug()

    def maybe_log_imu_debug(self):
        if not self.use_imu_heading or not self.log_imu_debug:
            return

        now = self.get_clock().now()
        if self.last_imu_debug_log_time is not None:
            elapsed = (
                now - self.last_imu_debug_log_time
            ).nanoseconds / 1e9
            if elapsed < self.imu_debug_log_period_sec:
                return

        self.last_imu_debug_log_time = now
        source = 'imu' if self.imu_heading_available() else 'encoder_fallback'
        self.get_logger().info(
            'IMU step | '
            f'heading={math.degrees(self.imu_theta):.1f} deg | '
            f'gyro_z={math.degrees(self.last_imu_gyro_z):.2f} deg/s | '
            f'odom_theta={math.degrees(self.theta):.1f} deg | '
            f'source={source}'
        )

    def goal_reached_callback(self, msg: Bool):
        if not msg.data or self.goal_reached:
            return

        self.goal_reached = True
        if self.stop_logging_after_goal:
            self.log_imu_debug = False

    @staticmethod
    def parameter_as_bool(value):
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
