#!/usr/bin/env python3
#
# line_following.launch.py
#
# Launches the complete line-following pipeline:
#
#   publisher_node  (jetson_camera)
#       |  /camera/image_raw
#   line_tracker    (line_following)
#       |  /vision_error_x
#   angular_tracker (line_following)
#       |  /cmd_vel
#   kinematics_node (motor_control)
#       |  motors

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

    motion_config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'kinematics.yaml',
    )

    line_config = os.path.join(
        get_package_share_directory('line_following'),
        'config',
        'line_following.yaml',
    )

    return LaunchDescription([

        # 1. Camera — captures frames and publishes /camera/image_raw
        Node(
            package='jetson_camera',
            executable='publisher_node',
            name='camera_publisher',
            parameters=[camera_config],
            output='screen',
        ),

        # 2. Line tracker — detects the line and publishes /vision_error_x
        Node(
            package='line_following',
            executable='line_tracker',
            name='line_tracker',
            parameters=[line_config],
            output='screen',
        ),

        # 3. Angular tracker — PID controller, publishes /cmd_vel
        Node(
            package='line_following',
            executable='angular_tracker',
            name='angular_tracker',
            output='screen',
        ),

        # 4. Kinematics — converts /cmd_vel Twist to left/right motor PWM
        Node(
            package='motor_control',
            executable='kinematics_node',
            name='kinematics_node',
            parameters=[motion_config],
            output='screen',
        ),
    ])
