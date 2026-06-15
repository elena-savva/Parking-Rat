#!/usr/bin/env python3


import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from jetson_camera_interfaces.msg import ProcessedImagePair


class ImagePairSubscriber(Node):
    def __init__(self):
        super().__init__('image_pair_subscriber')

        self.declare_parameter('image_pair_topic', '/camera/image_pair')
        self.declare_parameter('window_name', 'Raw vs Undistorted')
        # --- video recording parameters ---
        self.declare_parameter('output_video', 'camera_pair_output.avi')
        self.declare_parameter('video_fps', 30.0)
        # -----------------------------------

        self.image_pair_topic = self.get_parameter('image_pair_topic').value
        self.window_name = self.get_parameter('window_name').value
        self._output_video = self.get_parameter('output_video').value
        self._video_fps = self.get_parameter('video_fps').value
        self._video_writer = None

        self.subscription = self.create_subscription(
            ProcessedImagePair,
            self.image_pair_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.latest_frame = None
        self.get_logger().info(
            f'Subscribing to processed image pairs on {self.image_pair_topic}.'
        )

    def _init_video_writer(self, frame):

        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self._video_writer = cv2.VideoWriter(
            self._output_video, fourcc, self._video_fps, (w, h)
        )
        if self._video_writer.isOpened():
            self.get_logger().info(
                f'Recording video to: {os.path.abspath(self._output_video)}'
            )
        else:
            self.get_logger().error(
                f'Failed to open VideoWriter for: {self._output_video}'
            )
            self._video_writer = None

    def image_callback(self, msg):

        raw_frame = self.decode_compressed_image(msg.raw_image.data)
        undistorted_frame = self.decode_compressed_image(
            msg.undistorted_image.data
        )

        if raw_frame is None or undistorted_frame is None:
            return

        combined = self.combine_views(raw_frame, undistorted_frame)
        self.latest_frame = combined

        # --- write frame to video ---
        if self._video_writer is None:
            self._init_video_writer(combined)
        if self._video_writer is not None:
            self._video_writer.write(combined)
        # ----------------------------

    def decode_compressed_image(self, image_bytes):

        np_arr = np.frombuffer(image_bytes, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warning('Failed to decode one processed image.')

        return frame

    def combine_views(self, raw_frame, undistorted_frame):

        if raw_frame.shape != undistorted_frame.shape:
            undistorted_frame = cv2.resize(
                undistorted_frame,
                (raw_frame.shape[1], raw_frame.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )

        left = raw_frame.copy()
        right = undistorted_frame.copy()

        cv2.putText(
            left,
            'RAW IMAGE',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            right,
            'UNDISTORTED IMAGE',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        separator = np.full((left.shape[0], 12, 3), 30, dtype=np.uint8)
        return np.hstack((left, separator, right))


def main(args=None):
    rclpy.init(args=args)
    node = ImagePairSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if node.latest_frame is not None:
                cv2.imshow(node.window_name, node.latest_frame)
                if cv2.waitKey(1) == 27:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        # Release the video writer so the file is properly finalised
        if node._video_writer is not None:
            node._video_writer.release()
            node.get_logger().info('Video file saved successfully.')
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
