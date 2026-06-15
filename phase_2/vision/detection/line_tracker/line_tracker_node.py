#!/usr/bin/env python3
#
# line_tracker_node.py
#
# Vision detection node for line following.
# Ported from asg3_individual_ale into the final_project structure.
#
# Subscribes : /camera/image_raw   (sensor_msgs/CompressedImage)
# Publishes  : /vision_error_x     (std_msgs/Float32)
#              /camera/tracking_debug/compressed  (sensor_msgs/CompressedImage)
#
# The error published on /vision_error_x is the pixel distance between the
# detected line centroid and the configured target column.
#   Positive  -> line is to the LEFT  of target (steer left)
#   Negative  -> line is to the RIGHT of target (steer right)
#   Zero      -> line not found (angular_tracker_node will stop the robot)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
import cv2
import numpy as np


class LineTrackerNode(Node):
    def __init__(self):
        super().__init__('line_tracker')

        # ── ROS parameters ────────────────────────────────────────────────────
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/tracking_debug/compressed')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        # ── Tracking configuration ────────────────────────────────────────────
        # Brightness threshold for dark-line detection (0-255).
        # Lower = more sensitive; raise if the floor is picking up noise.
        self.threshold_val = 75

        # Horizontal target position as a fraction of image width.
        #   0.0 = far left edge
        #   0.5 = center (default)
        #   1.0 = far right edge
        # Set to 0.25 to track the line along the left side of the camera.
        self.target_ratio = 0.5

        # Vertical scanline position as a fraction of image height (from top).
        #   0.80 = close to the robot (default, reacts late but stable)
        #   0.60 = further ahead    (reacts earlier, better for sharp turns)
        self.scanline_y_ratio = 0.80
        self.scanline_thickness = 20   # Height of the scanline slit in pixels

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.subscription = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.debug_pub = self.create_publisher(
            CompressedImage,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.error_pub = self.create_publisher(Float32, '/vision_error_x', 10)

        self.get_logger().info(
            f'Line tracker initialized | '
            f'input={self.input_topic} | '
            f'target_ratio={self.target_ratio} | '
            f'scanline_y={self.scanline_y_ratio}'
        )

    # ── Callback ──────────────────────────────────────────────────────────────

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        height, width = frame.shape[:2]

        # Pixel column the robot wants the line to be at
        target_x = int(width * self.target_ratio)

        # Compute scanline vertical bounds
        scanline_center_y = int(height * self.scanline_y_ratio)
        half_thick = self.scanline_thickness // 2
        search_top = max(0, scanline_center_y - half_thick)
        search_bot = min(height, scanline_center_y + half_thick)

        # Threshold to find dark pixels, then mask everything outside scanline
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.threshold_val, 255, cv2.THRESH_BINARY_INV)
        mask[0:search_top, :] = 0
        mask[search_bot:height, :] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_found = False

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            # Ignore tiny noise specks (must be > 50 pixels area)
            if cv2.contourArea(largest_contour) > 50:
                M = cv2.moments(largest_contour)

                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    error_x = float(target_x - cx)
                    self.error_pub.publish(Float32(data=error_x))

                    # Draw debug overlays
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.line(frame, (target_x, search_top), (target_x, search_bot), (255, 0, 0), 3)
                    cv2.rectangle(frame, (0, search_top), (width, search_bot), (0, 255, 255), 2)
                    cv2.putText(
                        frame,
                        f'Target: {self.target_ratio} | Error: {error_x:.1f}',
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )
                    line_found = True

        if not line_found:
            # Publish 0.0 as the "line lost" signal — angular_tracker will stop
            self.error_pub.publish(Float32(data=0.0))
            cv2.putText(
                frame,
                'LINE LOST',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        # Publish debug image
        success, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if success:
            debug_msg = CompressedImage()
            debug_msg.header = msg.header
            debug_msg.format = 'jpeg'
            debug_msg.data = encoded.tobytes()
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
