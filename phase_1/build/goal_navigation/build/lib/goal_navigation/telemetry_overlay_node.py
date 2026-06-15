#!/usr/bin/env python3
"""
Camera overlay for goal navigation telemetry.

Subscribes to the live camera, odometry, and front ToF topics. It draws the
current obstacle distance and robot coordinates on the camera image, then shows
the result in a window when a desktop display is available.
"""

import math
import os

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Range


class TelemetryOverlayNode(Node):
    def __init__(self):
        super().__init__('telemetry_overlay_node')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('debug_topic', '/goal_navigation/debug/compressed')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('tof_topic', '/tof/front')
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('obstacle_stop_distance', 0.25)
        self.declare_parameter('window_name', 'Goal Navigation')
        self.declare_parameter('show_window', True)
        self.declare_parameter('publish_debug_image', True)

        camera_topic = str(self.get_parameter('camera_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        tof_topic = str(self.get_parameter('tof_topic').value)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.obstacle_stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value
        )
        self.window_name = str(self.get_parameter('window_name').value)
        self.show_window = self.parameter_as_bool(
            self.get_parameter('show_window').value
        )
        self.publish_debug_image = self.parameter_as_bool(
            self.get_parameter('publish_debug_image').value
        )

        self.x = None
        self.y = None
        self.z = None
        self.theta = None
        self.latest_tof_range = None
        self.last_tof_time = None
        self.last_odom_time = None
        self._warned_window_disabled = False

        if self.show_window and not os.environ.get('DISPLAY'):
            self.show_window = False
            self.get_logger().warn(
                'No DISPLAY environment variable found. '
                f'Publishing overlay on {debug_topic} without opening a window.'
            )

        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(
                CompressedImage,
                debug_topic,
                qos_profile_sensor_data,
            )

        self.create_subscription(
            CompressedImage,
            camera_topic,
            self.camera_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Range, tof_topic, self.tof_callback, 10)

        self.get_logger().info(
            'Telemetry overlay ready | '
            f'camera={camera_topic} | odom={odom_topic} | tof={tof_topic} | '
            f'debug={debug_topic} | show_window={self.show_window}'
        )

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.theta = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_odom_time = self.get_clock().now()

    def tof_callback(self, msg: Range):
        if math.isnan(msg.range) or msg.range <= 0.0:
            return

        self.latest_tof_range = msg.range
        self.last_tof_time = self.get_clock().now()

    def camera_callback(self, msg: CompressedImage):
        frame = self.decode_image(msg)
        if frame is None:
            return

        self.draw_overlay(frame)

        if self.debug_pub is not None:
            self.publish_debug_frame(frame, msg.header)

        if self.show_window:
            self.show_frame(frame)

    def decode_image(self, msg: CompressedImage):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Could not decode compressed camera image.')
        return frame

    def publish_debug_frame(self, frame, header):
        ok, encoded = cv2.imencode('.jpg', frame)
        if not ok:
            self.get_logger().warn('Could not encode telemetry overlay image.')
            return

        out = CompressedImage()
        out.header = header
        out.format = 'jpeg'
        out.data = encoded.tobytes()
        self.debug_pub.publish(out)

    def show_frame(self, frame):
        try:
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.show_window = False
            if not self._warned_window_disabled:
                self._warned_window_disabled = True
                self.get_logger().warn(
                    'Could not open OpenCV display window. '
                    f'Continuing with debug image topic only. Error: {exc}'
                )

    def draw_overlay(self, frame):
        lines, warning = self.overlay_lines()
        h, w = frame.shape[:2]

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 1
        line_height = 24
        padding = 12

        text_width = 0
        for line in lines:
            size, _ = cv2.getTextSize(line, font, scale, thickness)
            text_width = max(text_width, size[0])

        box_w = min(w - 20, text_width + padding * 2)
        box_h = min(h - 20, line_height * len(lines) + padding)
        x0 = 10
        y0 = 10
        x1 = x0 + box_w
        y1 = y0 + box_h

        overlay = frame.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.62, frame, 0.38, 0, frame)

        text_color = (255, 255, 255)
        alert_color = (40, 40, 255)
        for index, line in enumerate(lines):
            color = alert_color if warning and index == 0 else text_color
            y = y0 + padding + 16 + index * line_height
            cv2.putText(
                frame,
                line,
                (x0 + padding, y),
                font,
                scale,
                color,
                thickness,
                cv2.LINE_AA,
            )

    def overlay_lines(self):
        tof_text, obstacle_warning = self.format_tof_line()
        pose_text = self.format_pose_line()
        heading_text = self.format_heading_line()
        goal_text = self.format_goal_line()
        return [tof_text, pose_text, heading_text, goal_text], obstacle_warning

    def format_tof_line(self):
        if self.latest_tof_range is None:
            return 'ToF: waiting for front sensor', False

        age = self.message_age(self.last_tof_time)
        if age is None or age > 1.0:
            return 'ToF: no recent reading', False

        if math.isinf(self.latest_tof_range):
            return 'ToF: out of range', False

        cm = self.latest_tof_range * 100.0
        warning = self.latest_tof_range <= self.obstacle_stop_distance
        status = ' OBSTACLE' if warning else ''
        return f'ToF: {cm:.0f} cm ({self.latest_tof_range:.2f} m){status}', warning

    def format_pose_line(self):
        if self.x is None:
            return 'Pose: waiting for odometry'
        return f'Pose: x={self.x:.2f} y={self.y:.2f} z={self.z:.2f} m'

    def format_heading_line(self):
        if self.theta is None:
            return 'Heading: waiting'
        return f'Heading: {math.degrees(self.theta):.1f} deg'

    def format_goal_line(self):
        if self.x is None:
            return f'Goal: x={self.goal_x:.2f} y={self.goal_y:.2f} m'

        distance_left = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        return (
            f'Goal: x={self.goal_x:.2f} y={self.goal_y:.2f} m | '
            f'left={distance_left:.2f} m'
        )

    def message_age(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def parameter_as_bool(value):
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
