#!/usr/bin/env python3
"""
Publish a single start-relative goal pose for coordinate navigation tests.

Coordinate convention:
  - frame_id "odom" starts at the robot's initial pose.
  - +x is forward from the start direction.
  - +y is left from the start direction.
  - theta is counter-clockwise positive.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_theta', 0.0)
        self.declare_parameter('use_goal_theta', False)
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('publish_count', 5)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_theta = float(self.get_parameter('goal_theta').value)
        self.use_goal_theta = bool(self.get_parameter('use_goal_theta').value)
        self.publish_period_sec = float(
            self.get_parameter('publish_period_sec').value
        )
        self.publish_count = int(self.get_parameter('publish_count').value)

        self._published = 0
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.get_logger().info(
            'Goal publisher ready | '
            f'frame={self.frame_id} | '
            f'goal=({self.goal_x:.3f}, {self.goal_y:.3f}) | '
            f'topic={self.goal_topic}'
        )
        self.get_logger().info(
            'Coordinate convention: start=(0,0,0), +x=forward, +y=left.'
        )

        period = max(self.publish_period_sec, 0.1)
        self.timer = self.create_timer(period, self.publish_goal)

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = 0.0

        yaw = self.goal_theta if self.use_goal_theta else 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(msg)
        self._published += 1

        self.get_logger().info(
            f'Published goal {self._published}: '
            f'({self.goal_x:.3f}, {self.goal_y:.3f}) in {self.frame_id}'
        )

        if self.publish_count > 0 and self._published >= self.publish_count:
            self.timer.cancel()
            self.get_logger().info('Goal publication complete.')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
