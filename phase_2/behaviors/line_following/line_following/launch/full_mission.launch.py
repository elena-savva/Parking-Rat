#!/usr/bin/env python3
#
# full_mission.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_config = os.path.join(
        get_package_share_directory('jetson_camera'),
        'config', 'parameters.yaml',
    )
    motion_config = os.path.join(
        get_package_share_directory('motor_control'),
        'config', 'kinematics.yaml',
    )
    line_config = os.path.join(
        get_package_share_directory('line_following'),
        'config', 'line_following.yaml',
    )
    aruco_config = os.path.join(
        get_package_share_directory('aruco_parking'),
        'config', 'aruco_parking.yaml',
    )

    return LaunchDescription([

        # ── Camera pipeline ───────────────────────────────────────────────
        # publisher_node  → /camera/image_raw  (raw, fixed exposure)
        # processing_node → /camera/image_line        (undistorted + LUT, for line tracker)
        #                 → /camera/image_undistorted  (undistorted clean, for ArUco + color)
        #                 → /camera/image_pair         (pair msg, for color_detection_node)
        Node(
            package='jetson_camera',
            executable='publisher_node',
            name='camera_publisher',
            parameters=[camera_config],
            output='screen',
        ),
        Node(
            package='jetson_camera',
            executable='processing_node',
            name='image_processor',
            parameters=[camera_config],
            output='screen',
        ),

        # ── Line tracker: LUT-processed undistorted image ─────────────────
        Node(
            package='line_following',
            executable='line_tracker',
            name='line_tracker',
            parameters=[
                line_config,
                {'input_topic': '/camera/image_line'},
            ],
            output='screen',
        ),

        # ── ArUco detector: clean undistorted image ───────────────────────
        Node(
            package='aruco_parking',
            executable='aruco_detector_node',
            name='aruco_detector',
            parameters=[
                aruco_config,
                {'input_topic': '/camera/image_undistorted'},
            ],
            output='screen',
        ),

        # ── Color / go-stop: clean undistorted image ──────────────────────
        Node(
            package='color_detection_mode',
            executable='go_stop_node',
            name='go_stop_node',
            parameters=[
                {'input_topic': '/camera/image_undistorted'},
            ],
            output='screen',
        ),

        # ── Motion ────────────────────────────────────────────────────────
        Node(package='line_following',  executable='angular_tracker',     name='angular_tracker',                             output='screen'),
        Node(package='motor_control',   executable='kinematics_node',     name='kinematics_node',  parameters=[motion_config], output='screen'),
        Node(package='motor_control',   executable='odometry_node',       name='odometry_node',    parameters=[motion_config], output='screen'),
        Node(package='motor_control',   executable='pid_controller', name='pid_controller',                              output='screen'),
        Node(package='line_following',  executable='navigator',           name='navigator',                                   output='screen'),

    ])