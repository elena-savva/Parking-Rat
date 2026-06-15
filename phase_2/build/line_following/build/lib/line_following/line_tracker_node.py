#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32
import cv2
import numpy as np

# Number of consecutive frames without a line before declaring it lost.
# Avoids false positives from a single noisy frame.
LINE_LOST_FRAMES = 5

class LineTrackerNode(Node):
    def __init__(self):
        super().__init__('line_tracker')

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/robot/debug/compressed')

        self.input_topic  = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self._enabled = False
        self._lost_frames = 0   # consecutive frames with no line found
        self._was_lost    = False  # track last published lost state

        # ---- THE TRACKING VARIABLES ----
        self.threshold_val = 80 #120 outside markt
        self.target_ratio  = 0.5

        self.subscription = self.create_subscription(
            CompressedImage, self.input_topic, self.image_callback,
            qos_profile_sensor_data)

        self.create_subscription(
            Bool, '/line_tracker/enabled', self._cb_enabled, 10)

        self.debug_pub = self.create_publisher(
            CompressedImage, self.output_topic, qos_profile_sensor_data)

        self.error_pub = self.create_publisher(Float32, '/vision_error_x', 10)
        self.lost_pub  = self.create_publisher(Bool, '/line_tracker/lost',  10)

        self.get_logger().info('Line tracker ready! Waiting for enable...')

    def _cb_enabled(self, msg):
        self._enabled = msg.data
        if not self._enabled:
            self._lost_frames = 0
            self._was_lost    = False
            self.error_pub.publish(Float32(data=0.0))
            self.lost_pub.publish(Bool(data=False))
            self.get_logger().info('Line tracker disabled.')
        else:
            self.get_logger().info('Line tracker enabled.')

    def image_callback(self, msg):
        if not self._enabled:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        height, width = frame.shape[:2]

        target_x = int(width * self.target_ratio)

        scanline_y_ratio   = 0.75
        scanline_thickness = 20
        scanline_center_y  = int(height * scanline_y_ratio)
        half_thick         = scanline_thickness // 2
        search_top         = max(0, scanline_center_y - half_thick)
        search_bot         = min(height, scanline_center_y + half_thick)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.threshold_val, 255, cv2.THRESH_BINARY_INV)

        mask[0:search_top, 0:width] = 0
        mask[search_bot:height, 0:width] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_found = False

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 50:
                M = cv2.moments(largest_contour)

                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])

                    error_x = float(target_x - cx)
                    self.error_pub.publish(Float32(data=error_x))

                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, int(M['m01'] / M['m00'])), 8, (0, 0, 255), -1)
                    cv2.line(frame, (target_x, search_top), (target_x, search_bot), (255, 0, 0), 3)
                    cv2.rectangle(frame, (0, search_top), (width, search_bot), (0, 255, 255), 2)
                    cv2.putText(frame, f"Target: {self.target_ratio} | Error: {error_x:.1f}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    line_found = True

        if line_found:
            self._lost_frames = 0
            if self._was_lost:
                self._was_lost = False
                self.lost_pub.publish(Bool(data=False))
        else:
            self.error_pub.publish(Float32(data=0.0))
            self._lost_frames += 1
            cv2.putText(frame, "LINE LOST", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Only declare lost after several consecutive missed frames
            if self._lost_frames >= LINE_LOST_FRAMES and not self._was_lost:
                self._was_lost = True
                self.lost_pub.publish(Bool(data=True))
                self.get_logger().info('Line lost — publishing /line_tracker/lost')

        success, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if success:
            debug_msg        = CompressedImage()
            debug_msg.header = msg.header
            debug_msg.format = 'jpeg'
            debug_msg.data   = encoded.tobytes()
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