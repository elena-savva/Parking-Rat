#!/usr/bin/env python3
#
# go_stop.launch.py
#
# Launches the green/pink color detection pipeline.
# No movement — display and result topic only.
#
# Pipeline:
#   publisher_node  (jetson_camera)   →  /camera/image_raw
#   go_stop_node    (color_detection_mode)  →  /robot/debug/compressed
#                                             →  /color_detection/result

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

    go_stop_config = os.path.join(
        get_package_share_directory('color_detection_mode'),
        'config',
        'go_stop.yaml',
    )

    return LaunchDescription([

        # Camera — publishes /camera/image_raw
        Node(
            package='jetson_camera',
            executable='publisher_node',
            name='camera_publisher',
            parameters=[camera_config],
            output='screen',
        ),

        # Go/Stop detector — no movement, display only
        Node(
            package='color_detection_mode',
            executable='go_stop_node',
            name='go_stop_node',
            parameters=[go_stop_config],
            output='screen',
        ),
    ])
