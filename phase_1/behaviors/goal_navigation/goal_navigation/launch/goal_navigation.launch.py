import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    goal_config = os.path.join(
        get_package_share_directory('goal_navigation'),
        'config',
        'goal_navigation.yaml',
    )
    motion_config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'kinematics.yaml',
    )
    camera_config = PathJoinSubstitution([
        FindPackageShare('jetson_camera'),
        'config',
        'parameters.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_x',
            default_value='1.00',
            description='Goal x coordinate in meters. +x is forward from start.',
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.00',
            description='Goal y coordinate in meters. +y is left from start.',
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.05',
            description='Distance from the goal at which the robot stops.',
        ),
        DeclareLaunchArgument(
            'use_imu_heading',
            default_value='false',
            description='Use IMU gyro integration for odometry heading.',
        ),
        DeclareLaunchArgument(
            'start_delay_sec',
            default_value='3.0',
            description='Delay before the goal controller sends motion commands.',
        ),
        DeclareLaunchArgument(
            'orthogonal_navigation_enabled',
            default_value='true',
            description='Use only 0/90/180/-90 degree path headings.',
        ),
        DeclareLaunchArgument(
            'orthogonal_axis_tolerance',
            default_value='0.04',
            description='Axis error in meters before switching to the other goal axis.',
        ),
        DeclareLaunchArgument(
            'tof_start_delay_sec',
            default_value='2.0',
            description='Delay ToF startup so optional IMU calibration has less I2C contention.',
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.12',
            description='Maximum goal-navigation speed while obstacle detection is active.',
        ),
        DeclareLaunchArgument(
            'obstacle_stop_distance',
            default_value='0.25',
            description='Front ToF distance in meters that triggers obstacle stop.',
        ),
        DeclareLaunchArgument(
            'obstacle_confirm_count',
            default_value='3',
            description='Consecutive close ToF readings required before obstacle stop.',
        ),
        DeclareLaunchArgument(
            'path_clear_distance',
            default_value='0.30',
            description='Side-check distance in meters considered clear.',
        ),
        DeclareLaunchArgument(
            'path_check_turn_angle_deg',
            default_value='90.0',
            description='Angle used to check right and left paths.',
        ),
        DeclareLaunchArgument(
            'avoid_forward_distance',
            default_value='0.25',
            description='Distance to drive along the selected side before resuming goal navigation.',
        ),
        DeclareLaunchArgument(
            'avoid_linear_speed',
            default_value='0.10',
            description='Forward speed used during the short avoidance move.',
        ),
        DeclareLaunchArgument(
            'avoid_stop_distance',
            default_value='0.15',
            description='Front ToF distance in meters that interrupts the avoidance move.',
        ),
        DeclareLaunchArgument(
            'avoid_max_time_sec',
            default_value='5.0',
            description='Safety timeout for the short avoidance move.',
        ),
        DeclareLaunchArgument(
            'print_telemetry',
            default_value='true',
            description='Print current x/y/z, goal, ToF, command, and distance left.',
        ),
        DeclareLaunchArgument(
            'print_telemetry_period_sec',
            default_value='1.0',
            description='Seconds between repeated telemetry print lines.',
        ),
        DeclareLaunchArgument(
            'enable_camera_overlay',
            default_value='true',
            description='Start the camera and draw ToF/odometry labels on the image.',
        ),
        DeclareLaunchArgument(
            'show_overlay_window',
            default_value='true',
            description='Open an OpenCV window for the labeled camera image when DISPLAY is available.',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image_raw',
            description='Compressed camera image topic used by the overlay.',
        ),
        DeclareLaunchArgument(
            'overlay_debug_topic',
            default_value='/goal_navigation/debug/compressed',
            description='Annotated compressed camera image topic.',
        ),

        Node(
            package='jetson_camera',
            executable='publisher_node',
            name='camera_publisher',
            parameters=[camera_config],
            condition=IfCondition(LaunchConfiguration('enable_camera_overlay')),
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
            package='motor_control',
            executable='imu_reader_node',
            name='imu_reader_node',
            parameters=[motion_config],
            condition=IfCondition(LaunchConfiguration('use_imu_heading')),
            output='screen',
        ),
        TimerAction(
            period=LaunchConfiguration('tof_start_delay_sec'),
            actions=[
                Node(
                    package='motor_control',
                    executable='tof_reader_node',
                    name='tof_reader_node',
                    parameters=[motion_config],
                    output='screen',
                ),
            ],
        ),
        Node(
            package='motor_control',
            executable='odometry_node',
            name='odometry_node',
            parameters=[
                motion_config,
                {
                    'use_imu_heading': ParameterValue(
                        LaunchConfiguration('use_imu_heading'),
                        value_type=bool,
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='goal_navigation',
            executable='go_to_goal_node',
            name='go_to_goal_node',
            parameters=[
                goal_config,
                {
                    'goal_x': LaunchConfiguration('goal_x'),
                    'goal_y': LaunchConfiguration('goal_y'),
                    'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                    'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                    'start_delay_sec': LaunchConfiguration('start_delay_sec'),
                    'orthogonal_navigation_enabled': ParameterValue(
                        LaunchConfiguration('orthogonal_navigation_enabled'),
                        value_type=bool,
                    ),
                    'orthogonal_axis_tolerance': LaunchConfiguration(
                        'orthogonal_axis_tolerance'
                    ),
                    'obstacle_detection_enabled': True,
                    'avoidance_enabled': True,
                    'obstacle_stop_distance': LaunchConfiguration(
                        'obstacle_stop_distance'
                    ),
                    'obstacle_confirm_count': ParameterValue(
                        LaunchConfiguration('obstacle_confirm_count'),
                        value_type=int,
                    ),
                    'path_clear_distance': LaunchConfiguration(
                        'path_clear_distance'
                    ),
                    'path_check_turn_angle_deg': LaunchConfiguration(
                        'path_check_turn_angle_deg'
                    ),
                    'avoid_forward_distance': LaunchConfiguration(
                        'avoid_forward_distance'
                    ),
                    'avoid_linear_speed': LaunchConfiguration(
                        'avoid_linear_speed'
                    ),
                    'avoid_stop_distance': LaunchConfiguration(
                        'avoid_stop_distance'
                    ),
                    'avoid_max_time_sec': LaunchConfiguration(
                        'avoid_max_time_sec'
                    ),
                    'print_telemetry': LaunchConfiguration('print_telemetry'),
                    'print_telemetry_period_sec': LaunchConfiguration(
                        'print_telemetry_period_sec'
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='goal_navigation',
            executable='telemetry_overlay_node',
            name='telemetry_overlay_node',
            parameters=[
                {
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'debug_topic': LaunchConfiguration('overlay_debug_topic'),
                    'odom_topic': '/odom',
                    'tof_topic': '/tof/front',
                    'goal_x': LaunchConfiguration('goal_x'),
                    'goal_y': LaunchConfiguration('goal_y'),
                    'obstacle_stop_distance': LaunchConfiguration(
                        'obstacle_stop_distance'
                    ),
                    'show_window': ParameterValue(
                        LaunchConfiguration('show_overlay_window'),
                        value_type=bool,
                    ),
                    'publish_debug_image': True,
                },
            ],
            condition=IfCondition(LaunchConfiguration('enable_camera_overlay')),
            output='screen',
        ),
    ])
