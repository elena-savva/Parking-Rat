#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from jetson_camera_interfaces.msg import ProcessedImagePair
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


COLOR_RANGES = {
    'pink': ((140, 50, 100), (175, 255, 255)),
    'blue': ((100, 60, 60), (130, 255, 255)),
}

BANNER_COLORS = {
    'pink': (180, 20, 180),
    'blue': (20, 180, 20),
    None: (60, 60, 60),
}

STATUS_TEXT = {
    'pink': 'STOP: PINK DETECTED',
    'blue': 'MOVE FORWARD: BLUE DETECTED',
    None: 'NO TARGET COLOR: STOP',
}


class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')

        self.declare_parameter('image_pair_topic', '/camera/image_pair')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('result_topic', '/color_detection/result')
        self.declare_parameter('roi_top_frac', 0.30)
        self.declare_parameter('roi_bottom_frac', 0.70)
        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('show_display', True)
        self.declare_parameter('window_name', 'Raw vs Calibrated Color Detection')

        self.image_pair_topic = self.get_parameter('image_pair_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.result_topic = self.get_parameter('result_topic').value
        self.roi_top_frac = float(self.get_parameter('roi_top_frac').value)
        self.roi_bottom_frac = float(self.get_parameter('roi_bottom_frac').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.show_display = bool(self.get_parameter('show_display').value)
        self.window_name = self.get_parameter('window_name').value

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.create_subscription(
            ProcessedImagePair,
            self.image_pair_topic,
            self.image_pair_callback,
            qos_profile_sensor_data,
        )

        self.last_color = 'uninitialized'

        if self.show_display:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 520)

        self.get_logger().info(
            'Color detection mode ready. '
            f'Listening on {self.image_pair_topic} and publishing motion on '
            f'{self.cmd_vel_topic}.'
        )

    def image_pair_callback(self, msg):
        raw_frame = self.decode_compressed_image(msg.raw_image.data)
        calibrated_frame = self.decode_compressed_image(msg.undistorted_image.data)

        if raw_frame is None or calibrated_frame is None:
            return

        detected_color, mask = self.detect_dominant_color(calibrated_frame)
        self.publish_motion(detected_color)
        self.publish_result(detected_color)
        self.log_state_change(detected_color)

        if self.show_display:
            combined = self.build_display(raw_frame, calibrated_frame, mask, detected_color)
            cv2.imshow(self.window_name, combined)
            cv2.waitKey(1)

    def decode_compressed_image(self, image_bytes) -> Optional[np.ndarray]:
        np_arr = np.frombuffer(image_bytes, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warning('Failed to decode compressed image.')

        return frame

    def detect_dominant_color(
        self,
        frame: np.ndarray,
    ) -> Tuple[Optional[str], Optional[np.ndarray]]:
        height, width = frame.shape[:2]
        roi_top = int(height * self.roi_top_frac)
        roi_bottom = int(height * self.roi_bottom_frac)
        roi = frame[roi_top:roi_bottom, :]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        best_color = None
        best_area = self.min_area
        best_mask = None

        for color_name, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv_roi, np.array(lower), np.array(upper))
            area = cv2.countNonZero(mask)
            if area > best_area:
                best_area = area
                best_color = color_name
                best_mask = mask

        full_mask = None
        if best_mask is not None:
            full_mask = np.zeros((height, width), dtype=np.uint8)
            full_mask[roi_top:roi_bottom, :] = best_mask

        return best_color, full_mask

    def publish_motion(self, detected_color: Optional[str]):
        cmd = Twist()
        if detected_color == 'blue':
            cmd.linear.x = self.forward_speed

        self.cmd_pub.publish(cmd)

    def publish_result(self, detected_color: Optional[str]):
        result_text = detected_color if detected_color is not None else 'none'
        self.result_pub.publish(String(data=result_text))

    def log_state_change(self, detected_color: Optional[str]):
        if detected_color == self.last_color:
            return

        self.last_color = detected_color

        if detected_color == 'pink':
            self.get_logger().info('Pink detected: commanding STOP.')
        elif detected_color == 'blue':
            self.get_logger().info(
                f'Blue detected: commanding forward motion at '
                f'{self.forward_speed:.2f} m/s.'
            )
        else:
            self.get_logger().info('No target color detected: commanding STOP.')

    def build_display(
        self,
        raw_frame: np.ndarray,
        calibrated_frame: np.ndarray,
        mask: Optional[np.ndarray],
        detected_color: Optional[str],
    ) -> np.ndarray:
        if raw_frame.shape != calibrated_frame.shape:
            calibrated_frame = cv2.resize(
                calibrated_frame,
                (raw_frame.shape[1], raw_frame.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )
            if mask is not None:
                mask = cv2.resize(
                    mask,
                    (raw_frame.shape[1], raw_frame.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )

        height, width = calibrated_frame.shape[:2]
        roi_top = int(height * self.roi_top_frac)
        roi_bottom = int(height * self.roi_bottom_frac)

        left = raw_frame.copy()
        right = calibrated_frame.copy()

        cv2.rectangle(left, (0, roi_top), (width - 1, roi_bottom), (0, 255, 255), 2)
        cv2.putText(
            left,
            'RAW IMAGE',
            (16, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if mask is not None:
            overlay = right.copy()
            tint_color = (255, 0, 255) if detected_color == 'pink' else (255, 140, 0)
            overlay[mask > 0] = tint_color
            cv2.addWeighted(overlay, 0.40, right, 0.60, 0, right)

        cv2.rectangle(right, (0, roi_top), (width - 1, roi_bottom), (0, 255, 255), 2)
        banner_height = 54
        cv2.rectangle(
            right,
            (0, 0),
            (width, banner_height),
            BANNER_COLORS[detected_color],
            -1,
        )
        cv2.putText(
            right,
            STATUS_TEXT[detected_color],
            (16, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            right,
            'CALIBRATED IMAGE',
            (16, banner_height + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        separator = np.full((height, 12, 3), 30, dtype=np.uint8)
        return np.hstack((left, separator, right))


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
