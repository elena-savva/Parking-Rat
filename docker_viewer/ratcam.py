#!/usr/bin/env python3
#
# ai_viewer.py
#
# Run on your laptop inside Docker to view debug frames from the robot.
# Any node on the robot that publishes to /robot/debug/compressed
# will automatically appear in this window.
#
# Start with:  ./view.sh

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class AIViewer(Node):
    def __init__(self):
        super().__init__('ai_viewer')

        self.sub = self.create_subscription(
            CompressedImage,
            '/robot/debug/compressed',
            self.image_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'AI Viewer ready — waiting for frames on /robot/debug/compressed ...'
        )

    def image_cb(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow('Robot Debug View', frame)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Image error: {e}')


def main():
    rclpy.init()
    node = AIViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
