import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('goal_navigation'),
        'config',
        'goal_navigation.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_x',
            default_value='0.50',
            description='Goal x coordinate in meters. +x is forward from start.',
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.00',
            description='Goal y coordinate in meters. +y is left from start.',
        ),
        DeclareLaunchArgument(
            'publish_count',
            default_value='5',
            description='Number of times to publish the goal. Use 0 for forever.',
        ),
        Node(
            package='goal_navigation',
            executable='goal_publisher_node',
            name='goal_publisher_node',
            parameters=[
                config_file,
                {
                    'goal_x': ParameterValue(
                        LaunchConfiguration('goal_x'),
                        value_type=float,
                    ),
                    'goal_y': ParameterValue(
                        LaunchConfiguration('goal_y'),
                        value_type=float,
                    ),
                    'publish_count': ParameterValue(
                        LaunchConfiguration('publish_count'),
                        value_type=int,
                    ),
                },
            ],
            output='screen',
        ),
    ])
