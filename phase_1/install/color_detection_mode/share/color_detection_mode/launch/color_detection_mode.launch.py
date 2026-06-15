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
    color_config = os.path.join(
        get_package_share_directory('color_detection_mode'),
        'config',
        'color_detection.yaml',
    )

    return LaunchDescription([
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
        Node(
            package='motor_control',
            executable='kinematics_node',
            name='kinematics_node',
            parameters=[motion_config],
            output='screen',
        ),
        Node(
            package='color_detection_mode',
            executable='color_detection_node',
            name='color_detection_node',
            parameters=[color_config],
            output='screen',
        ),
    ])
