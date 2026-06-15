#!/usr/bin/env python3
#
# aruco_parking.launch.py
#
# Launches the ArUco marker detection pipeline used for parking.
#
# Pipeline:
#   publisher_node       (jetson_camera)  →  /camera/image_raw
#   aruco_detector_node  (aruco_parking)  →  /aruco/detected
#                                         →  /aruco/id
#                                         →  /aruco/angle_error
#                                         →  /aruco/distance
#                                         →  /aruco/heading
#                                         →  /camera/aruco_debug/compressed

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_config = os.path.join(
        get_package_share_directory('jetson_camera'),
        'config',
        'parameters.yaml',
    )

    aruco_config = os.path.join(
        get_package_share_directory('aruco_parking'),
        'config',
        'aruco_parking.yaml',
    )

    return LaunchDescription([

        # Camera — publishes /camera/image_raw (compressed frames)
        Node(
            package='jetson_camera',
            executable='publisher_node',
            name='camera_publisher',
            parameters=[camera_config],
            output='screen',
        ),

        # ArUco detector — publishes angle, distance, heading
        Node(
            package='aruco_parking',
            executable='aruco_detector_node',
            name='aruco_detector',
            parameters=[aruco_config],
            output='screen',
        ),
    ])
