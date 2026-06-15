#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import mediapipe as mp

class HandFollowerNode(Node):
    def __init__(self):
        super().__init__('hand_follower_node')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_pub = self.create_publisher(Image, '/camera/image_processed', 10)
        
        # Tools
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # CSI Ribbon Camera Setup via GStreamer
        gstreamer_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! appsink"
        )
        self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        # Visual Servoing Parameters
        self.target_size = 150.0  # How big the hand should be (pixels) when the robot stops
        self.kp_angular = 0.003   # Turning sensitivity
        self.kp_linear = 0.005    # Forward/Backward sensitivity
        
        # Run the camera loop at 20 Hz
        self.timer = self.create_timer(0.05, self.process_frame)

    def is_open_palm(self, hand_landmarks):
        """Simple heuristic: Are the fingertips higher than the lower finger joints?"""
        tips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]
        pips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP
        ]
        
        # MediaPipe Y-coordinates go DOWN. So smaller Y = higher on screen.
        for tip, pip in zip(tips, pips):
            if hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y:
                return False # A finger is curled down
        return True

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab camera frame!")
            return
            
        # Flip frame horizontally for a mirror effect, then convert to RGB for MediaPipe
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the AI Hand Tracking
        results = self.hands.process(rgb_frame)
        
        h, w, c = frame.shape
        center_x = w // 2
        
        cmd = Twist()
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 1. Draw the skeleton on the image
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # 2. Check for the specific sign (Open Palm)
                if self.is_open_palm(hand_landmarks):
                    cv2.putText(frame, "FOLLOWING", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # 3. Calculate bounding box to find the hand's size and center
                    x_coords = [lm.x * w for lm in hand_landmarks.landmark]
                    y_coords = [lm.y * h for lm in hand_landmarks.landmark]
                    
                    hand_center_x = (min(x_coords) + max(x_coords)) / 2.0
                    hand_size = max(y_coords) - min(y_coords) # Height of the hand in pixels
                    
                    # 4. Visual Servoing Math (P-Controller)
                    # Turn toward the hand
                    error_x = center_x - hand_center_x
                    cmd.angular.z = error_x * self.kp_angular
                    
                    # Drive toward the hand if it's too small (far away)
                    error_size = self.target_size - hand_size
                    
                    # Only drive forward/backward if the hand is roughly centered
                    if abs(error_x) < 100:
                        cmd.linear.x = error_size * self.kp_linear
                else:
                    cv2.putText(frame, "WAITING FOR SIGN", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Clamp speeds for safety
        cmd.linear.x = max(min(cmd.linear.x, 0.4), -0.4)
        cmd.angular.z = max(min(cmd.angular.z, 1.5), -1.5)
        
        # Publish the command to the wheels
        self.cmd_pub.publish(cmd)
        
        # Hacker Bypass: Manually pack the ROS 2 Image message!
        ros_image = Image()
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera"
        ros_image.height = frame.shape[0]
        ros_image.width = frame.shape[1]
        ros_image.encoding = "bgr8"
        ros_image.is_bigendian = 0
        ros_image.step = frame.shape[1] * 3  # 3 bytes per pixel (BGR)
        ros_image.data = frame.tobytes()     # Convert raw pixels to bytes
        
        self.img_pub.publish(ros_image)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()