#!/usr/bin/env python3
#
# go_stop_node.py
#
# Detects green (GO) and pink/magenta (STOP) from the camera feed.
# No movement commands are sent — this node is display only.
#
# For every frame it:
#   1. Converts to HSV and masks for green or pink in a configurable ROI
#   2. Finds the largest contour of the detected color
#   3. Draws a bounding rectangle around it with the detection confidence
#   4. Overlays a large GO or STOP label on the frame
#   5. Publishes the annotated frame to /robot/debug/compressed
#
# Subscribes : /camera/image_raw          (sensor_msgs/CompressedImage)
# Publishes  : /robot/debug/compressed    (sensor_msgs/CompressedImage)
#              /color_detection/result    (std_msgs/String)  "green"|"pink"|"none"
#
# Colors tuned for the reference swatches:
#   Green : pure lime  #00ff00  →  HSV ~(60, 255, 255)
#   Pink  : magenta    #ff00ff  →  HSV ~(150, 255, 255)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# ── HSV color ranges ──────────────────────────────────────────────────────────
# Tuned for the reference PDF swatches.
# Hue in OpenCV is 0-179 (half of 0-360).
# Widen the ranges slightly if lighting conditions vary.

COLOR_RANGES = {
    'green': [(35,  80, 80), (85,  255, 255)],   # lime green
    'pink':  [(150, 100, 100), (179, 255, 255)],   # magenta/pink
}

# ── Display settings per color ────────────────────────────────────────────────

MIN_CONFIDENCE = 1.5

DISPLAY = {
    'green': {
        'label':      'GO',
        'box_color':  (0,   220,  0),    # green rectangle
        'text_color': (0,   220,  0),
        'bg_color':   (0,    60,  0),    # dark green banner background
    },
    'pink': {
        'label':      'STOP',
        'box_color':  (220,  0,  220),   # magenta rectangle
        'text_color': (220,  0,  220),
        'bg_color':   (60,   0,   60),   # dark magenta banner background
    },
    None: {
        'label':      'SEARCHING...',
        'box_color':  (80,  80,   80),
        'text_color': (180, 180, 180),
        'bg_color':   (30,  30,   30),
    },
}


class GoStopNode(Node):
    def __init__(self):
        super().__init__('go_stop_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('input_topic',  '/camera/image_raw')
        self.declare_parameter('output_topic', '/robot/debug/compressed')
        self.declare_parameter('result_topic', '/color_detection/result')

        # ROI: fraction of image height to search in (ignore top and bottom)
        self.declare_parameter('roi_top_frac',    0.20)
        self.declare_parameter('roi_bottom_frac', 0.80)

        # Minimum pixel area to count as a real detection (filters noise)
        self.declare_parameter('min_area', 500)

        self.input_topic    = self.get_parameter('input_topic').value
        self.output_topic   = self.get_parameter('output_topic').value
        self.result_topic   = self.get_parameter('result_topic').value
        self.roi_top_frac   = float(self.get_parameter('roi_top_frac').value)
        self.roi_bot_frac   = float(self.get_parameter('roi_bottom_frac').value)
        self.min_area       = int(self.get_parameter('min_area').value)

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.sub = self.create_subscription(
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

        self.result_pub = self.create_publisher(
            String,
            self.result_topic,
            10,
        )

        self._last_color = 'uninitialized'

        self.get_logger().info(
            f'Go/Stop node ready | '
            f'input={self.input_topic} | '
            f'output={self.output_topic}'
        )

    # ── Main callback ─────────────────────────────────────────────────────────

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        height, width = frame.shape[:2]
        roi_top = int(height * self.roi_top_frac)
        roi_bot = int(height * self.roi_bot_frac)

        # Detect the dominant color in the ROI
        detected_color, contour, confidence = self.detect_color(
            frame, roi_top, roi_bot
        )

        # Publish result string
        result_str = detected_color if detected_color is not None else 'none'
        self.result_pub.publish(String(data=result_str))

        # Log only on state change
        if detected_color != self._last_color:
            self._last_color = detected_color
            self.get_logger().info(
                f'Color changed → {result_str.upper()} '
                f'(confidence: {confidence:.1f}%)'
            )

        # Build annotated frame and publish to debug topic
        annotated = self.draw_overlay(
            frame, detected_color, contour, confidence, roi_top, roi_bot
        )
        self.publish_debug(annotated, msg.header)

    # ── Detection ─────────────────────────────────────────────────────────────

    def detect_color(self, frame, roi_top, roi_bot):
        """
        Returns (color_name, largest_contour, confidence_pct) or
        (None, None, 0.0) if nothing found.
        confidence is the percentage of the ROI covered by the detected color.
        """
        roi = frame[roi_top:roi_bot, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        roi_area = roi.shape[0] * roi.shape[1]

        best_color    = None
        best_contour  = None
        best_area     = self.min_area
        best_confidence = 0.0

        for color_name, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Clean up noise with morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)

            if area > best_area:
                best_area       = area
                best_color      = color_name
                best_confidence = min((area / roi_area) * 100.0, 100.0)

                # Shift contour y-coordinates back to full-frame space
                shifted = largest.copy()
                shifted[:, :, 1] += roi_top
                best_contour = shifted

        if best_color is not None and best_confidence < MIN_CONFIDENCE:
            return None, None, 0.0
        return best_color, best_contour, best_confidence

    # ── Drawing ───────────────────────────────────────────────────────────────

    def draw_overlay(self, frame, color, contour, confidence, roi_top, roi_bot):
        out = frame.copy()
        height, width = out.shape[:2]
        d = DISPLAY[color]

        # ROI boundary line (subtle, always visible)
        cv2.rectangle(
            out,
            (0, roi_top),
            (width - 1, roi_bot),
            (60, 60, 60),
            1,
        )

        # Bounding rectangle around detected color blob
        if contour is not None:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(out, (x, y), (x + w, y + h), d['box_color'], 3)

            # Confidence label next to the box
            conf_label = f'{confidence:.1f}%'
            cv2.putText(
                out,
                conf_label,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                d['box_color'],
                2,
                cv2.LINE_AA,
            )

        # Banner at the top of the frame
        banner_h = 70
        cv2.rectangle(out, (0, 0), (width, banner_h), d['bg_color'], -1)

        # Large GO / STOP label
        label      = d['label']
        font_scale = 2.0
        thickness  = 4
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_DUPLEX, font_scale, thickness
        )
        text_x = (width - tw) // 2
        text_y = banner_h - (banner_h - th) // 2

        cv2.putText(
            out,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_DUPLEX,
            font_scale,
            d['text_color'],
            thickness,
            cv2.LINE_AA,
        )

        return out

    # ── Publish ───────────────────────────────────────────────────────────────

    def publish_debug(self, frame, header):
        success, encoded = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        )
        if success:
            debug_msg = CompressedImage()
            debug_msg.header = header
            debug_msg.format = 'jpeg'
            debug_msg.data   = encoded.tobytes()
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
