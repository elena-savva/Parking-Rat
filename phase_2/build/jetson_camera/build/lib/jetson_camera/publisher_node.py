#!/usr/bin/env python3

# =============================================================================
#  CALIBRATION PARAMETERS
#  EXPOSURE_TIME_US : shutter time in microseconds (CSI camera only).
#      Hardware minimum for IMX219: 13000 µs.
# =============================================================================
EXPOSURE_TIME_US = 13_000   # µs
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/camera/image_raw',
            qos_profile_sensor_data,
        )

        self.bridge = CvBridge()
        config = self.load_parameters()

        self.sensor_id = config["sensor_id"]
        self.width     = config["width"]
        self.height    = config["height"]
        self.fps       = config["fps"]

        self.gst_pipeline = self.define_gstreamer_pipeline()
        if self.gst_pipeline is None:
            self.get_logger().error('Invalid sensor id in GStreamer pipeline')

        self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open GStreamer pipeline')
            raise RuntimeError('Camera open failed')

        self.get_logger().info(
            f'Camera publisher initialised. '
            f'Raw frames only. exposure={EXPOSURE_TIME_US} µs'
        )

    def load_parameters(self):
        config = {}
        self.declare_parameter('sensor_id', None)
        self.declare_parameter('width', None)
        self.declare_parameter('height', None)
        self.declare_parameter('fps', None)
        config["sensor_id"] = self.get_parameter('sensor_id').value
        config["width"]     = self.get_parameter('width').value
        config["height"]    = self.get_parameter('height').value
        config["fps"]       = self.get_parameter('fps').value
        self.get_logger().info(f'Loaded config {config}')
        return config

    def define_gstreamer_pipeline(self):
        if self.sensor_id == 0:
            return (
                f"nvarguscamerasrc sensor-id=0 "
                f"exposuretimerange=\"{EXPOSURE_TIME_US} {EXPOSURE_TIME_US}\" "
                f"gainrange=\"1 1\" "
                f"ispdigitalgainrange=\"1 1\" "
                f"aelock=true wbmode=1 ! "
                f"video/x-raw(memory:NVMM),"
                f"width={self.width},height={self.height},"
                f"framerate={self.fps}/1,format=NV12 ! "
                f"nvvidconv ! video/x-raw,format=BGRx ! "
                f"videoconvert ! video/x-raw,format=BGR ! "
                f"appsink drop=true sync=false max-buffers=1"
            )
        elif self.sensor_id == 1:
            return (
                f"v4l2src device=/dev/video1 ! "
                f"video/x-raw,format=YUY2,"
                f"width={self.width},height={self.height},"
                f"framerate={self.fps}/1 ! "
                f"videoconvert ! video/x-raw,format=BGR ! "
                f"appsink sync=false max-buffers=1 drop=true"
            )
        else:
            return None

    def pub_frame(self, frame):
        msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ret, frame = node.cap.read()
            if not ret:
                continue
            node.pub_frame(frame)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
