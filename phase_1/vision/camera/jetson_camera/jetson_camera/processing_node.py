#!/usr/bin/env python3
"""
Processing node for the group assignment pipeline.

The node subscribes to the raw compressed camera feed, loads the previously
computed camera calibration, undistorts each frame, and publishes both the
raw and corrected images in one custom ROS message.
"""

import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from jetson_camera_interfaces.msg import ProcessedImagePair


class ProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processor')

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_pair')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('jpeg_quality', 95)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.camera_matrix, self.dist_coeffs, self.calibration_path = (
            self.load_calibration()
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.publisher_ = self.create_publisher(
            ProcessedImagePair,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.map1 = None
        self.map2 = None
        self.map_image_size = None
        self.first_publish_logged = False

        self.get_logger().info(
            f'Loaded calibration from {self.calibration_path} and listening on '
            f'{self.input_topic}. Publishing raw + undistorted pairs on '
            f'{self.output_topic}.'
        )

    def default_calibration_path(self):
        """Return the installed config path that should contain calibration."""
        package_share_dir = get_package_share_directory('jetson_camera')
        return os.path.join(
            package_share_dir,
            'config',
            'camera_calibration.yaml',
        )

    def load_calibration(self):
        """
        Load the camera matrix and distortion coefficients from YAML.

        The calibration script writes this file after Zhang's method finishes.
        """
        configured_path = self.get_parameter('calibration_file').value
        calibration_path = (
            os.path.abspath(os.path.expanduser(configured_path))
            if configured_path
            else self.default_calibration_path()
        )

        if not os.path.exists(calibration_path):
            self.get_logger().fatal(
                'Calibration file not found. Expected: '
                f'{calibration_path}. Run calibrate_camera first.'
            )
            raise FileNotFoundError(calibration_path)

        with open(calibration_path, 'r', encoding='utf-8') as calibration_file:
            data = yaml.safe_load(calibration_file)

        camera_matrix = np.array(data['camera_matrix'], dtype=np.float64)
        dist_coeffs = np.array(data['dist_coeffs'], dtype=np.float64)
        return camera_matrix, dist_coeffs, calibration_path

    def image_callback(self, msg):
        """Decode the incoming frame, undistort it, and publish the pair."""
        raw_frame = self.decode_compressed_image(msg)
        if raw_frame is None:
            return

        height, width = raw_frame.shape[:2]
        self.ensure_undistort_maps(width, height)
        undistorted_frame = cv2.remap(
            raw_frame,
            self.map1,
            self.map2,
            cv2.INTER_LINEAR,
        )

        stamp = msg.header.stamp
        frame_id = msg.header.frame_id or self.frame_id

        output_msg = ProcessedImagePair()
        output_msg.header.stamp = stamp
        output_msg.header.frame_id = frame_id
        output_msg.raw_image = self.copy_raw_image_message(msg, frame_id)
        output_msg.undistorted_image = self.encode_image(
            undistorted_frame,
            stamp,
            frame_id,
        )

        self.publisher_.publish(output_msg)

        if not self.first_publish_logged:
            self.first_publish_logged = True
            self.get_logger().info(
                'Published first processed message containing raw and '
                'undistorted images.'
            )

    def decode_compressed_image(self, msg):
        """Convert a ROS CompressedImage message into an OpenCV frame."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warning('Failed to decode incoming compressed image.')

        return frame

    def ensure_undistort_maps(self, width, height):

        image_size = (width, height)
        if self.map1 is not None and self.map_image_size == image_size:
            return

        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix,
            self.dist_coeffs,
            image_size,
            0,
            image_size,
        )
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            new_camera_matrix,
            image_size,
            cv2.CV_16SC2,
        )
        self.map_image_size = image_size
        self.get_logger().info(
            f'Prepared undistortion maps for image size {width}x{height}.'
        )

    def copy_raw_image_message(self, msg, frame_id):

        raw_image = CompressedImage()
        raw_image.header = msg.header
        raw_image.header.frame_id = frame_id
        raw_image.format = msg.format
        raw_image.data = msg.data
        return raw_image

    def encode_image(self, frame, stamp, frame_id):
        """Encode an OpenCV frame as JPEG for the custom output message."""
        success, encoded = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not success:
            raise RuntimeError('Failed to encode undistorted frame as JPEG.')

        image_msg = CompressedImage()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = frame_id
        image_msg.format = 'jpeg'
        image_msg.data = encoded.tobytes()
        return image_msg


def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
