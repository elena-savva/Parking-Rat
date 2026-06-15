#!/usr/bin/env python3
"""
ArUco Detector Node
===================
Detects ArUco markers and publishes:
  /aruco/detected     (Bool)   — whether a tag is currently visible
  /aruco/id           (Int32)  — ID of the detected marker
  /aruco/angle_error  (Float32)— horizontal angle error in radians;
                                 positive = tag is to the RIGHT of image centre
  /aruco/distance     (Float32)— estimated distance to the tag in metres
  /aruco/heading      (Float32)— direction the tag's TOP edge points,
                                 in radians relative to the robot's forward axis.
                                 0 = top edge points straight ahead,
                                 +pi/2 = points right, -pi/2 = points left.
                                 The robot should turn by -heading to face the
                                 direction the tag's top edge points.

The heading is derived from solvePnP using the camera intrinsics, so the
marker_size_m parameter must match your physically printed tag size.
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('input_topic',   '/camera/image_raw')
        self.declare_parameter('debug_topic',   '/robot/debug/compressed')
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('dict_id',       0)
        self.declare_parameter('fx',            320.0)
        self.declare_parameter('fy',            320.0)
        self.declare_parameter('cx',            320.0)
        self.declare_parameter('cy',            240.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.fx          = float(self.get_parameter('fx').value)
        self.fy          = float(self.get_parameter('fy').value)
        self.cx_cam      = float(self.get_parameter('cx').value)
        self.cy_cam      = float(self.get_parameter('cy').value)

        # Camera matrix and zero distortion (undistorted frames from processing_node)
        self._K = np.array([[self.fx, 0,       self.cx_cam],
                            [0,       self.fy,  self.cy_cam],
                            [0,       0,        1.0        ]], dtype=np.float64)
        self._D = np.zeros((4, 1), dtype=np.float64)

        # 3D corners of the marker in its own local frame (metres)
        # Convention: marker lies in the XY plane, Z points out of the front face.
        # Corners ordered: top-left, top-right, bottom-right, bottom-left
        # so that the top edge goes from corner[0] to corner[1].
        half = self.marker_size / 2.0
        self._obj_pts = np.array([
            [-half,  half, 0],   # top-left
            [ half,  half, 0],   # top-right
            [ half, -half, 0],   # bottom-right
            [-half, -half, 0],   # bottom-left
        ], dtype=np.float64)

        dict_id = int(self.get_parameter('dict_id').value)
        self._init_aruco(dict_id)

        self.create_subscription(
            CompressedImage, self.input_topic, self.image_callback,
            qos_profile_sensor_data)

        self.detected_pub = self.create_publisher(Bool,           '/aruco/detected',    10)
        self.id_pub       = self.create_publisher(Int32,          '/aruco/id',          10)
        self.angle_pub    = self.create_publisher(Float32,        '/aruco/angle_error', 10)
        self.dist_pub     = self.create_publisher(Float32,        '/aruco/distance',    10)
        self.heading_pub  = self.create_publisher(Float32,        '/aruco/heading',     10)
        self.debug_pub    = self.create_publisher(CompressedImage, self.debug_topic,
                                                  qos_profile_sensor_data)

        self.get_logger().info('ArUco detector ready.')

    def _init_aruco(self, dict_id):
        try:
            self._aruco_dict   = cv2.aruco.getPredefinedDictionary(dict_id)
            self._aruco_params = cv2.aruco.DetectorParameters()
            self._detector     = cv2.aruco.ArucoDetector(
                self._aruco_dict, self._aruco_params)
            self._new_api = True
        except AttributeError:
            self._aruco_dict   = cv2.aruco.Dictionary_get(dict_id)
            self._aruco_params = cv2.aruco.DetectorParameters_create()
            self._detector     = None
            self._new_api      = False

    def _detect(self, gray):
        if self._new_api:
            return self._detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)

    def _tag_heading(self, corners_2d):
        """
        Use solvePnP to find the tag's pose, then extract the direction
        the tag's top edge points relative to the camera (and therefore
        relative to the robot's forward axis).

        Returns heading in radians:
          0        = top edge points straight ahead (same direction as robot)
          +pi/2    = top edge points to the right
          -pi/2    = top edge points to the left

        The robot should turn by -heading to face the direction the top
        edge points (i.e. to align itself with the line).
        """
        img_pts = corners_2d.astype(np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            self._obj_pts, img_pts, self._K, self._D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)

        if not ok:
            return None

        # Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)

        # The tag's local Y axis (pointing from bottom edge to top edge)
        # in camera coordinates is the second column of R.
        tag_y_in_cam = R[:, 1]   # shape (3,)

        # We only care about the component in the camera's XZ plane
        # (horizontal plane from the robot's perspective).
        # Camera convention: X = right, Y = down, Z = forward.
        # The horizontal heading is atan2(X_component, Z_component).
        heading = math.atan2(tag_y_in_cam[0], tag_y_in_cam[2])

        return heading

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)

        if ids is not None and len(ids) > 0:
            tag_id      = int(ids[0][0])
            tag_corners = corners[0][0]   # (4, 2)

            # ── Angle error (horizontal, from pixel position) ──────────────
            tag_cx      = float(np.mean(tag_corners[:, 0]))
            angle_error = math.atan2(tag_cx - self.cx_cam, self.fx)

            # ── Distance (from apparent marker size) ───────────────────────
            side_px  = float(np.linalg.norm(tag_corners[1] - tag_corners[0]))
            distance = (self.marker_size * self.fx / side_px) if side_px > 0 else 0.0

            # ── Heading (from full pose via solvePnP) ──────────────────────
            heading = self._tag_heading(tag_corners)

            self.detected_pub.publish(Bool(data=True))
            self.id_pub.publish(Int32(data=tag_id))
            self.angle_pub.publish(Float32(data=angle_error))
            self.dist_pub.publish(Float32(data=distance))
            if heading is not None:
                self.heading_pub.publish(Float32(data=heading))

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            heading_str = f'{math.degrees(heading):.1f}°' if heading is not None else 'N/A'
            cv2.putText(
                frame,
                f'ID:{tag_id}  ang:{math.degrees(angle_error):.1f}  '
                f'dist:{distance:.2f}m  hdg:{heading_str}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        else:
            self.detected_pub.publish(Bool(data=False))
            self.angle_pub.publish(Float32(data=0.0))
            self.dist_pub.publish(Float32(data=0.0))
            self.heading_pub.publish(Float32(data=0.0))
            cv2.putText(frame, 'No ArUco', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        success, encoded = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if success:
            out        = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data   = encoded.tobytes()
            self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
