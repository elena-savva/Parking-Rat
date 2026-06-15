#!/usr/bin/env python3  
import glob
import os

import rclpy  
from rclpy.node import Node  
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage  
import cv2
from cv_bridge import CvBridge  
from rclpy.time import Time
import numpy as np
  
  
class ImageSubscriber(Node):  
    def __init__(self):  
        super().__init__('image_subscriber')  

        # --- video recording parameters ---
        self.declare_parameter('output_video', 'camera_output.avi')
        self.declare_parameter('video_fps', 30.0)
        self._output_video = self.get_parameter('output_video').value
        self._video_fps = self.get_parameter('video_fps').value
        self._video_writer = None
        # -----------------------------------
  
        self.sub = self.create_subscription(  
            CompressedImage,  
            '/camera/image_raw',  
            self.image_cb,
            qos_profile_sensor_data
        )  
  
        self.bridge = CvBridge()  
        self.latest_frame = None  
        self.first_image_received = False
        self.initialized = True

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
  
    def image_cb(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            self.get_logger().info(
                "Camera subscriber captured first image from publisher."
            )

        try:
            # compute PubSub delay
            msg_time = Time.from_msg(data.header.stamp)
            now = self.get_clock().now()
            delay_sec = (now - msg_time).nanoseconds * 1e-9

            self.get_logger().info(f"PubSub delay: {delay_sec:.4f} s")

            # decode compressedimage, without CVBridge
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.latest_frame = cv_image

            # --- write frame to video ---
            if cv_image is not None:
                if self._video_writer is None:
                    self._init_video_writer(cv_image)
                if self._video_writer is not None:
                    self._video_writer.write(cv_image)
            # ----------------------------

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
 
  
def main(args=None):  
    rclpy.init(args=args)  
    node = ImageSubscriber()  
  
    try:  
        while rclpy.ok():  
            rclpy.spin_once(node, timeout_sec=0.01)  
  
            if node.latest_frame is not None:  
                cv2.imshow('Camera', node.latest_frame)  
                if cv2.waitKey(1) == 27:  # ESC  
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
