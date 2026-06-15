import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'kinematics.yaml'
    )

    kinematics_node = Node(
        package='motor_control',
        executable='kinematics_node',
        name='kinematics_node',
        parameters=[config_file_path],
        output='screen'
    )

    return LaunchDescription([
        kinematics_node,

        Node(
            package='motor_control',
            executable='odometry_node',
            name='odometry_node'
        ),
        
        Node(
            package='motor_control',
            executable='pid_controller',
            name='pid_controller_node',
            output='screen'
        ),
    ])
