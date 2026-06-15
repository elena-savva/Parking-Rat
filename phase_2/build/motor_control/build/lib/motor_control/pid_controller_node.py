#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub          = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ── Target ───────────────────────────────────────────────────────────
        self.target_x = None
        self.target_y = None

        # ── Constant forward speed ────────────────────────────────────────────
        self.V_CRUISE   = 0.2    # m/s while heading is good
        self.V_SLOW     = 0.1    # m/s when close to goal (< SLOW_DIST)
        self.SLOW_DIST  = 0.1    # m — start slowing down
        self.STOP_DIST  = 0.02   # m — stop within 2cm

        # ── Angular PID gains (heading only) ─────────────────────────────────
        self.kp_ang = 6.0
        self.ki_ang = 0.3
        self.kd_ang = 0.15

        # ── PID state ────────────────────────────────────────────────────────
        self.prev_time       = self.get_clock().now()
        self.head_integral   = 0.0
        self.prev_head_error = 0.0

        # ── Minimum angular power to overcome static friction ─────────────────
        self.MIN_ANGULAR = 0.4

        # ── Threshold: must be aligned within this to drive forward ──────────
        self.ALIGN_THRESHOLD = math.radians(13)

        # ── Two-speed turn: slow down when within this angle of target ────────
        self.TURN_SLOW_THRESHOLD = math.radians(30)

        self.get_logger().info("PID Controller ready. Waiting for goal on /goal_pose ...")

    # ── Helpers ──────────────────────────────────────────────────────────────

    def euler_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def reset_pid(self):
        self.head_integral   = 0.0
        self.prev_head_error = 0.0
        self.prev_time       = self.get_clock().now()

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def goal_callback(self, msg: PoseStamped):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.reset_pid()
        self.get_logger().info(
            f"New goal → x: {self.target_x:.2f}  y: {self.target_y:.2f}"
        )

    def odom_callback(self, msg: Odometry):
        if self.target_x is None or self.target_y is None:
            return

        # ── Current pose ──────────────────────────────────────────────────────
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        ct = self.euler_from_quaternion(msg.pose.pose.orientation)

        # ── Time delta ────────────────────────────────────────────────────────
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.prev_time = now

        # ── Errors ────────────────────────────────────────────────────────────
        ex = self.target_x - cx
        ey = self.target_y - cy

        distance_error  = math.sqrt(ex**2 + ey**2)
        desired_heading = math.atan2(ey, ex)

        heading_error = desired_heading - ct
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # ── Debug log ─────────────────────────────────────────────────────────
        self.get_logger().info(
            f"dist: {distance_error:.2f}  "
            f"desired: {math.degrees(desired_heading):.1f}°  "
            f"theta: {math.degrees(ct):.1f}°  "
            f"err: {math.degrees(heading_error):.1f}°"
        )

        cmd = Twist()

        # ── Goal reached ──────────────────────────────────────────────────────
        if distance_error < self.STOP_DIST:
            self.get_logger().info("Goal reached!")
            self.stop()
            self.target_x = None
            self.target_y = None
            self.goal_reached_pub.publish(Bool(data=True))
            return

        # ── Angular PID (heading only) ────────────────────────────────────────
        self.head_integral += heading_error * dt
        self.head_integral  = max(min(self.head_integral, 0.3), -0.3)  # anti-windup
        head_derivative     = (heading_error - self.prev_head_error) / dt

        omega = (
            self.kp_ang * heading_error
            + self.ki_ang * self.head_integral
            + self.kd_ang * head_derivative
        )
        self.prev_head_error = heading_error

        # ── Linear speed — constant, no PID ──────────────────────────────────
        if abs(heading_error) < self.ALIGN_THRESHOLD:
            # Aligned — drive forward at cruise or slow speed
            if distance_error < self.SLOW_DIST:
                v = self.V_SLOW
            else:
                v = self.V_CRUISE
        else:
            # Misaligned — spin in place to align first
            v = 0.0
            self.head_integral = 0.0  # reset integral during spin

        cmd.linear.x  = v
        cmd.angular.z = omega

        # ── Clamp outputs ─────────────────────────────────────────────────────
        cmd.linear.x = max(min(cmd.linear.x, 0.35), -0.35)

        if v == 0.0:
            # Two-speed turn: fast when far from target, slow when close
            if abs(heading_error) > self.TURN_SLOW_THRESHOLD:
                cmd.angular.z = max(min(cmd.angular.z,  2.0), -2.0)  # fast spin
            else:
                cmd.angular.z = max(min(cmd.angular.z,  0.8), -0.8)  # slow near target
        else:
            cmd.angular.z = max(min(cmd.angular.z,  2.0), -2.0)  # driving corrections

        # ── Minimum angular power to overcome static friction ─────────────────
        if cmd.linear.x == 0.0:
            if 0 < cmd.angular.z < self.MIN_ANGULAR:
                cmd.angular.z = self.MIN_ANGULAR
            elif -self.MIN_ANGULAR < cmd.angular.z < 0:
                cmd.angular.z = -self.MIN_ANGULAR

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()