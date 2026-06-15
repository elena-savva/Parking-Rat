#!/usr/bin/env python3
"""
Full goal navigation with obstacle detection and avoidance.

The robot drives to the goal, stops for a front obstacle, checks right then
left, moves along the clearer side, and resumes goal navigation from updated
odometry.
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool


class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_reached_topic', '/goal_reached')
        self.declare_parameter('goal_x', 0.5)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('slow_radius', 0.20)
        self.declare_parameter('max_linear_speed', 0.20)
        self.declare_parameter('min_linear_speed', 0.15)
        self.declare_parameter('kp_heading', 2.8)
        self.declare_parameter('max_angular_speed', 2) #1.5
        self.declare_parameter('min_turn_speed', 1.5)  #was 0.35
        self.declare_parameter('alignment_tolerance_deg', 12.0)
        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('orthogonal_navigation_enabled', True)
        self.declare_parameter('orthogonal_axis_tolerance', 0.04)
        self.declare_parameter('obstacle_detection_enabled', True)
        self.declare_parameter('tof_topic', '/tof/front')
        self.declare_parameter('tof_timeout_sec', 0.5)
        self.declare_parameter('obstacle_stop_distance', 0.5)
        self.declare_parameter('obstacle_confirm_count', 3)
        self.declare_parameter('path_clear_distance', 0.40)
        self.declare_parameter('path_check_turn_angle_deg', 90.0)
        self.declare_parameter('path_check_turn_angle_deg_blocked', 180.0)
        self.declare_parameter('path_check_heading_tolerance_deg', 6.0)
        self.declare_parameter('path_check_turn_speed', 2.0)
        self.declare_parameter('path_check_pause_sec', 0.5)
        self.declare_parameter('path_check_timeout_sec', 4.0)
        self.declare_parameter('avoidance_enabled', True)
        self.declare_parameter('avoid_forward_distance', 0.55)
        self.declare_parameter('avoid_linear_speed', 0.70)
        self.declare_parameter('avoid_stop_distance', 0.80)
        self.declare_parameter('avoid_max_time_sec', 2.0)  #was5
        self.declare_parameter('avoid_resume_pause_sec', 0.5)
        self.declare_parameter('print_telemetry', True)
        self.declare_parameter('print_telemetry_period_sec', 1.0)

        odom_topic = str(self.get_parameter('odom_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        goal_reached_topic = str(self.get_parameter('goal_reached_topic').value)
        tof_topic = str(self.get_parameter('tof_topic').value)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.slow_radius = float(self.get_parameter('slow_radius').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.min_linear_speed = float(
            self.get_parameter('min_linear_speed').value
        )
        self.kp_heading = float(self.get_parameter('kp_heading').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.min_turn_speed = float(self.get_parameter('min_turn_speed').value)
        self.alignment_tolerance = math.radians(
            float(self.get_parameter('alignment_tolerance_deg').value)
        )
        self.control_period_sec = max(
            float(self.get_parameter('control_period_sec').value),
            0.01,
        )
        self.start_delay_sec = max(
            float(self.get_parameter('start_delay_sec').value),
            0.0,
        )
        self.orthogonal_navigation_enabled = self.parameter_as_bool(
            self.get_parameter('orthogonal_navigation_enabled').value
        )
        self.orthogonal_axis_tolerance = max(
            float(self.get_parameter('orthogonal_axis_tolerance').value),
            0.0,
        )
        self.obstacle_detection_enabled = self.parameter_as_bool(
            self.get_parameter('obstacle_detection_enabled').value
        )
        self.tof_timeout_sec = float(self.get_parameter('tof_timeout_sec').value)
        self.obstacle_stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value
        )
        self.obstacle_confirm_count = max(
            int(self.get_parameter('obstacle_confirm_count').value),
            1,
        )
        self.path_clear_distance = float(
            self.get_parameter('path_clear_distance').value
        )
        self.path_check_turn_angle = math.radians(
            float(self.get_parameter('path_check_turn_angle_deg').value)
        )
        self.path_check_heading_tolerance = math.radians(
            float(self.get_parameter('path_check_heading_tolerance_deg').value)
        )
        self.path_check_turn_speed = float(
            self.get_parameter('path_check_turn_speed').value
        )
        self.path_check_pause_sec = float(
            self.get_parameter('path_check_pause_sec').value
        )
        self.path_check_timeout_sec = float(
            self.get_parameter('path_check_timeout_sec').value
        )
        self.avoidance_enabled = self.parameter_as_bool(
            self.get_parameter('avoidance_enabled').value
        )
        self.avoid_forward_distance = float(
            self.get_parameter('avoid_forward_distance').value
        )
        self.avoid_linear_speed = float(
            self.get_parameter('avoid_linear_speed').value
        )
        self.avoid_stop_distance = float(
            self.get_parameter('avoid_stop_distance').value
        )
        self.avoid_max_time_sec = float(
            self.get_parameter('avoid_max_time_sec').value
        )
        self.avoid_resume_pause_sec = float(
            self.get_parameter('avoid_resume_pause_sec').value
        )
        self.print_telemetry = self.parameter_as_bool(
            self.get_parameter('print_telemetry').value
        )
        self.print_telemetry_period_sec = max(
            float(self.get_parameter('print_telemetry_period_sec').value),
            0.1,
        )

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.have_odom = False
        self.active = True
        self.goal_reached = False
        self.mode = 'GOAL'
        self.mode_start_time = self.get_clock().now()
        self.latest_tof_range = math.inf
        self.last_tof_time = None
        self.tof_history = deque(maxlen=self.obstacle_confirm_count)
        self.obstacle_heading = 0.0
        self.right_check_heading = 0.0
        self.left_check_heading = 0.0
        self.selected_path_heading = 0.0
        self.right_distance = None
        self.left_distance = None
        self.path_choice = 'unknown'
        self.avoid_start_x = 0.0
        self.avoid_start_y = 0.0
        self.warned_waiting_for_tof = False
        self.start_time = self.get_clock().now()
        self._waiting_for_start = self.start_delay_sec > 0.0
        self._log_counter = 0
        self._avoid_log_counter = 0
        self._last_print_time = None
        self.goal_reached_publish_count = 0
        self.goal_reached_republish_limit = 10
        self.first_goal_x = self.goal_x
        self.first_goal_y = self.goal_y
        self.final_goal_extension_started = False
        self.final_goal_extension_distance = 0.20
        self.start_goal_heading = math.atan2(self.goal_y, self.goal_x)
        # Active orthogonal axis to prevent heading oscillation
        if abs(self.goal_x) >= abs(self.goal_y):
          self.active_axis = 'x'
        else:
          self.active_axis = 'y'

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.goal_reached_pub = self.create_publisher(
            Bool,
            goal_reached_topic,
            10,
        )

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        if self.obstacle_detection_enabled:
            self.create_subscription(Range, tof_topic, self.tof_callback, 10)
        self.control_timer = self.create_timer(
            self.control_period_sec,
            self.control_loop,
        )

        self.get_logger().info(
            'Goal navigation ready | '
            f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}) m | '
            f'tolerance={self.goal_tolerance:.2f} m'
        )
        if self.obstacle_detection_enabled:
            self.get_logger().info(
                'Obstacle detection active | '
                f'tof_topic={tof_topic} | '
                f'stop_distance={self.obstacle_stop_distance:.2f} m | '
                f'confirm_count={self.obstacle_confirm_count}'
            )
            if self.avoidance_enabled:
                self.get_logger().info(
                    'Obstacle avoidance active | '
                    f'avoid_forward={self.avoid_forward_distance:.2f} m | '
                    f'avoid_speed={self.avoid_linear_speed:.2f} m/s | '
                    f'avoid_stop={self.avoid_stop_distance:.2f} m'
                )
        else:
            self.get_logger().info(
                'Obstacle detection is disabled.'
            )
        self.get_logger().info(
            f'Initial goal heading target: '
            f'{math.degrees(self.start_goal_heading):.1f} deg'
        )
        if self.orthogonal_navigation_enabled:
            self.get_logger().info(
                'Orthogonal navigation active. '
                'Goal travel uses 0/90/180/-90 degree headings.'
            )
        if self.start_delay_sec > 0.0:
            self.get_logger().info(
                f'Waiting {self.start_delay_sec:.1f}s before motion so sensors can settle.'
            )
        self.print_nav_status('GOAL_READ', force=True)

    def odom_callback(self, msg: Odometry):
        had_odom = self.have_odom
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.theta = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True
        if not had_odom:
            self.print_nav_status('ODOM_READY', force=True)

    def tof_callback(self, msg: Range):
        if msg.range <= 0.0 or math.isnan(msg.range):
            return

        self.latest_tof_range = msg.range
        self.last_tof_time = self.get_clock().now()
        self.tof_history.append((msg.range, self.last_tof_time))

    def control_loop(self):
        if not self.have_odom:
            return

        if self.goal_reached:
            self.publish_goal_reached_stop()
            if self.goal_reached_publish_count >= self.goal_reached_republish_limit:
                self.control_timer.cancel()
            return

        if not self.active:
            return

        if self.waiting_for_start_delay():
            self.publish_stop()
            return

        if self.obstacle_detection_enabled and self.current_tof_distance() is None:
            self.publish_stop()
            if not self.warned_waiting_for_tof:
                self.warned_waiting_for_tof = True
                self.get_logger().warn(
                    'Waiting for recent ToF reading before moving with obstacle detection.'
                )
                self.print_nav_status('WAITING_FOR_TOF', force=True)
            return
        self.warned_waiting_for_tof = False

        if self.mode != 'GOAL':
            self.handle_obstacle_mode()
            return

        if self.front_obstacle_detected():
            self.start_obstacle_check()
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            if not self.final_goal_extension_started:
                self.start_final_goal_extension()
            else:
                self.finish_goal()
            return

        desired_heading = self.goal_heading(dx, dy)
        travel_distance = self.travel_distance_for_heading(
            dx,
            dy,
            desired_heading,
        )
        heading_error = self.wrap_angle(desired_heading - self.theta)

        cmd = Twist()
        cmd.angular.z = self.clamp(
            self.kp_heading * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        if abs(heading_error) <= self.alignment_tolerance:
            cmd.linear.x = self.linear_speed(travel_distance)
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = self.apply_min_turn_speed(cmd.angular.z)

        self.cmd_pub.publish(cmd)
        self.log_status(distance, desired_heading, heading_error, cmd)
        self.print_nav_status('MOVING_TO_GOAL', cmd)

    def front_obstacle_detected(self, stop_distance=None) -> bool:
        if not self.obstacle_detection_enabled:
            return False

        if stop_distance is None:
            stop_distance = self.obstacle_stop_distance

        if self.current_tof_distance() is None:
            return False

        return self.close_tof_readings_confirmed(stop_distance)

    def start_obstacle_check(self):
        self.publish_stop()
        self.obstacle_heading = self.path_base_heading()
        self.right_check_heading = self.wrap_angle(
            self.obstacle_heading - self.path_check_turn_angle
        )
        self.left_check_heading = self.wrap_angle(
            self.obstacle_heading + self.path_check_turn_angle
        )
        self.right_distance = None
        self.left_distance = None
        self.path_choice = 'unknown'

        self.set_mode('OBSTACLE_STOP')
        self.get_logger().warn(
            'Obstacle detected. Stopping and checking right/left paths | '
            f'front={self.format_distance(self.current_tof_distance())}'
        )
        self.print_nav_status('OBSTACLE_DETECTED', force=True)

    def handle_obstacle_mode(self):
        if self.mode == 'OBSTACLE_STOP':
            self.publish_stop()
            if self.mode_elapsed() >= self.path_check_pause_sec:
                self.set_mode('CHECK_RIGHT')
                self.get_logger().info('Checking right side first.')
                self.print_nav_status('CHECK_RIGHT', force=True)
            return

        if self.mode == 'CHECK_RIGHT':
            done = self.turn_to_heading(self.right_check_heading)
            if done:                                                      # or self.mode_elapsed() >= self.path_check_timeout_sec:
                self.publish_stop()
                self.right_distance = self.current_tof_distance()
                self.get_logger().info(
                    'Right path check | '
                    f'distance={self.format_distance(self.right_distance)}'
                )
                self.set_mode('CHECK_LEFT')
                self.get_logger().info('Checking left side next.')
                self.print_nav_status('CHECK_LEFT', force=True)
            return

        if self.mode == 'CHECK_LEFT':
            done = self.turn_to_heading(self.left_check_heading)
            if done or self.mode_elapsed() >= self.path_check_timeout_sec:
                self.publish_stop()
                self.left_distance = self.current_tof_distance()
                self.get_logger().info(
                    'Left path check | '
                    f'distance={self.format_distance(self.left_distance)}'
                )
                self.choose_path()
            return


        if self.mode == 'TURN_TO_SELECTED_PATH':

            err = self.wrap_angle(self.selected_path_heading - self.theta)

            self.get_logger().info(f"TURN_TO_SELECTED_PATH | "f"current={math.degrees(self.theta):.1f} deg | "
            f"target={math.degrees(self.selected_path_heading):.1f} deg | "
            f"error={math.degrees(err):.1f} deg")

            done = self.turn_to_heading(self.selected_path_heading)

            if done or self.mode_elapsed() >= self.path_check_timeout_sec:
                self.publish_stop()
                self.finish_path_turn()      




       # if self.mode == 'TURN_TO_SELECTED_PATH':
        #    done = self.turn_to_heading(self.selected_path_heading)
         #   if done or self.mode_elapsed() >= self.path_check_timeout_sec:
          #      self.publish_stop()
           #     self.finish_path_turn()
           
          # return

        if self.mode == 'AVOID_FORWARD':
            self.handle_avoid_forward()
            return

        if self.mode == 'AVOID_RESUME_PAUSE':
            self.publish_stop()
            if self.mode_elapsed() >= self.avoid_resume_pause_sec:
                self.set_mode('GOAL')
                self.get_logger().info(
                    'Avoidance complete. Resuming goal navigation from updated odometry.'
                )
                self.print_nav_status('RESUME_GOAL', force=True)
            return

        if self.mode == 'PATH_CHECK_DONE':
            self.publish_stop()
            return

    def finish_path_turn(self):
        if self.path_choice == 'blocked':
            self.set_mode('PATH_CHECK_DONE')
            self.get_logger().warn(
                'No clear path found. Robot is stopped.'
            )
            self.print_nav_status('BLOCKED_STOP', force=True)
            return

        if not self.avoidance_enabled:
            self.set_mode('PATH_CHECK_DONE')
            self.get_logger().info(
                'Path check complete. '
                f'selected={self.path_choice}. '
                'Robot is stopped because avoidance is disabled.'
            )
            return

        self.start_avoid_forward()

    def choose_path(self):
        candidates = [
            {
                'name': 'right',
                'distance': self.right_distance,
                'heading': self.right_check_heading,
            },
            {
                'name': 'left',
                'distance': self.left_distance,
                'heading': self.left_check_heading,
            },
        ]
        usable_candidates = [
            candidate
            for candidate in candidates
            if self.distance_is_usable_for_avoid(candidate['distance'])
        ]

        if usable_candidates:
            selected = max(usable_candidates, key=self.path_sort_key)
            self.path_choice = selected['name']
            self.selected_path_heading = selected['heading']
        else:
            self.path_choice = 'blocked'
            self.selected_path_heading = self.obstacle_heading

        right_progress = self.path_progress_score(self.right_check_heading)
        left_progress = self.path_progress_score(self.left_check_heading)
        self.get_logger().info(
            'Path choice | '
            f'right={self.format_distance(self.right_distance)}, '
            f'progress={right_progress:.2f} m | '
            f'left={self.format_distance(self.left_distance)}, '
            f'progress={left_progress:.2f} m | '
            f'selected={self.path_choice}'
        )
        self.print_nav_status(f'PATH_CHOICE_{self.path_choice.upper()}', force=True)
        self.set_mode('TURN_TO_SELECTED_PATH')

    def start_avoid_forward(self):
        self.avoid_start_x = self.x
        self.avoid_start_y = self.y
        self._avoid_log_counter = 0
        self.set_mode('AVOID_FORWARD')
        self.get_logger().info(
            'Starting obstacle avoidance | '
            f'selected={self.path_choice} | '
            f'distance={self.avoid_forward_distance:.2f} m'
        )
        self.print_nav_status('AVOID_START', force=True)

    def handle_avoid_forward(self):
        if self.front_obstacle_detected(self.avoid_stop_distance):
            self.publish_stop()
            self.get_logger().warn(
                'Obstacle detected during avoidance. Rechecking paths | '
                f'front={self.format_distance(self.current_tof_distance())}'
            )
            self.print_nav_status('AVOID_RECHECK', force=True)
            self.start_obstacle_check()
            return

        traveled = math.hypot(
            self.x - self.avoid_start_x,
            self.y - self.avoid_start_y,
        )
        timed_out = self.mode_elapsed() >= self.avoid_max_time_sec
        if traveled >= self.avoid_forward_distance or timed_out:
            self.publish_stop()
            self.set_mode('AVOID_RESUME_PAUSE')
            reason = 'distance reached' if not timed_out else 'timeout'
            self.get_logger().info(
                'Avoidance forward motion complete | '
                f'traveled={traveled:.2f} m | reason={reason}'
            )
            self.print_nav_status('AVOID_DONE', force=True)
            return

        cmd = Twist()
        cmd.linear.x = self.avoid_linear_speed
        self.cmd_pub.publish(cmd)
        self.log_avoid_status(traveled, cmd)
        self.print_nav_status('AVOID_FORWARD', cmd)

    def turn_to_heading(self, target_heading: float) -> bool:
        heading_error = self.wrap_angle(target_heading - self.theta)
        if abs(heading_error) <= self.path_check_heading_tolerance:
            self.publish_stop()
            return True

        cmd = Twist()
        cmd.angular.z = self.clamp(
            1.8 * heading_error,
            -self.path_check_turn_speed,
            self.path_check_turn_speed,
        )
        cmd.angular.z = self.apply_min_turn_speed(cmd.angular.z)
        self.cmd_pub.publish(cmd)
        self.print_nav_status('TURNING', cmd)
        return False

    def current_tof_distance(self):
        if self.last_tof_time is None:
            return None

        age = (self.get_clock().now() - self.last_tof_time).nanoseconds / 1e9
        if age > self.tof_timeout_sec:
            return None

        return self.latest_tof_range

    def close_tof_readings_confirmed(self, stop_distance: float) -> bool:
        if len(self.tof_history) < self.obstacle_confirm_count:
            return False

        now = self.get_clock().now()
        recent_readings = list(self.tof_history)[-self.obstacle_confirm_count:]
        for distance, stamp in recent_readings:
            age = (now - stamp).nanoseconds / 1e9
            if age > self.tof_timeout_sec:
                return False
            if distance > stop_distance:
                return False

        return True

    def distance_is_clear(self, distance) -> bool:
        return distance is not None and distance >= self.path_clear_distance

    def distance_is_usable_for_avoid(self, distance) -> bool:
        return distance is not None and distance >= self.obstacle_stop_distance

    def path_base_heading(self) -> float:
        if self.orthogonal_navigation_enabled:
            return self.snap_to_cardinal(self.theta)
        return self.theta

    def goal_heading(self, dx: float, dy: float) -> float:
      if not self.orthogonal_navigation_enabled:
        return math.atan2(dy, dx)

    # X axis complete -> switch to Y
      if (
        self.active_axis == 'x'
        and abs(dx) <= self.orthogonal_axis_tolerance
      ):
        self.active_axis = 'y'

    # Y axis complete -> switch to X
      elif (
        self.active_axis == 'y'
        and abs(dy) <= self.orthogonal_axis_tolerance
      ):
        self.active_axis = 'x'

    # Follow currently selected axis
      if self.active_axis == 'x':
        return 0.0 if dx >= 0.0 else math.pi

      return math.pi / 2.0 if dy >= 0.0 else -math.pi / 2.0

    def travel_distance_for_heading(
        self,
        dx: float,
        dy: float,
        heading: float,
    ) -> float:
        if not self.orthogonal_navigation_enabled:
            return math.hypot(dx, dy)

        distance_along_heading = dx * math.cos(heading) + dy * math.sin(heading)
        return max(abs(distance_along_heading), self.goal_tolerance)

    def path_sort_key(self, candidate):
        progress = self.path_progress_score(candidate['heading'])
        clearance = self.clearance_score(candidate['distance'])
        progress_rank = self.progress_rank(progress)
        clear_rank = 1 if self.distance_is_clear(candidate['distance']) else 0
        right_preference = 1 if candidate['name'] == 'right' else 0
        return (
            progress_rank,
            progress,
            clear_rank,
            clearance,
            right_preference,
        )

    def path_progress_score(self, heading: float) -> float:
        distance_now = self.distance_left_to_goal()
        next_x = self.x + self.avoid_forward_distance * math.cos(heading)
        next_y = self.y + self.avoid_forward_distance * math.sin(heading)
        distance_after_move = math.hypot(
            self.goal_x - next_x,
            self.goal_y - next_y,
        )
        return distance_now - distance_after_move

    def clearance_score(self, distance) -> float:
        if distance is None:
            return -math.inf
        return distance

    @staticmethod
    def progress_rank(progress: float) -> int:
        if progress > 0.01:
            return 2
        if progress >= -0.01:
            return 1
        return 0

    def snap_to_cardinal(self, heading: float) -> float:
        quarter_turn = math.pi / 2.0
        return self.wrap_angle(round(heading / quarter_turn) * quarter_turn)

    def set_mode(self, mode: str):
        self.mode = mode
        self.mode_start_time = self.get_clock().now()

    def mode_elapsed(self) -> float:
        return (self.get_clock().now() - self.mode_start_time).nanoseconds / 1e9

    @staticmethod
    def format_distance(distance) -> str:
        if distance is None:
            return 'no recent reading'
        if math.isinf(distance):
            return 'out of range'
        return f'{distance:.2f} m'

    def waiting_for_start_delay(self) -> bool:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.start_delay_sec:
            return True

        if self._waiting_for_start:
            self._waiting_for_start = False
            self.get_logger().info('Start delay complete. Beginning goal navigation.')
            self.print_nav_status('START_MOVING', force=True)

        return False

    def distance_left_to_goal(self) -> float:
        return math.sqrt(
            (self.goal_x - self.x) ** 2
            + (self.goal_y - self.y) ** 2
            + self.z ** 2
        )

    def linear_speed(self, distance: float) -> float:
        if self.slow_radius <= 0.0 or distance >= self.slow_radius:
            return self.max_linear_speed

        scaled_speed = self.max_linear_speed * (distance / self.slow_radius)
        return self.clamp(
            scaled_speed,
            self.min_linear_speed,
            self.max_linear_speed,
        )

    def apply_min_turn_speed(self, angular_speed: float) -> float:
        if angular_speed == 0.0:
            return 0.0
        if abs(angular_speed) >= self.min_turn_speed:
            return angular_speed
        return math.copysign(self.min_turn_speed, angular_speed)

    def start_final_goal_extension(self):
        self.publish_stop()
        self.final_goal_extension_started = True
        self.goal_x = self.first_goal_x + self.final_goal_extension_distance
        self.goal_y = self.first_goal_y
        self.active_axis = 'x'
        self.get_logger().info(
            'First goal reached. Continuing to final alignment goal | '
            f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}) m'
        )
        self.print_nav_status('FINAL_GOAL_EXTENSION', force=True)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_goal_reached_stop(self):
        self.publish_stop()
        self.goal_reached_pub.publish(Bool(data=True))
        self.goal_reached_publish_count += 1

    def finish_goal(self):
        self.active = False
        self.goal_reached = True
        self.mode = 'DONE'
        self.goal_reached_publish_count = 0

        self.get_logger().info(
            f'Goal reached at x={self.x:.2f}, y={self.y:.2f}, '
            f'final_heading={math.degrees(self.theta):.1f} deg, '
            f'goal_heading=0.0 deg. '
            'Stopping robot and telemetry.'
        )
        self.print_nav_status('GOAL_REACHED', force=True)
        self.print_telemetry = False
        self.publish_goal_reached_stop()

    def log_status(
        self,
        distance: float,
        desired_heading: float,
        heading_error: float,
        cmd: Twist,
    ):
        self._log_counter += 1
        log_every = max(1, int(1.0 / self.control_period_sec))
        if self._log_counter % log_every != 0:
            return

        self.get_logger().info(
            f'pose=({self.x:.2f}, {self.y:.2f}, '
            f'{math.degrees(self.theta):.1f} deg) | '
            f'dist={distance:.2f} m | '
            f'target={math.degrees(desired_heading):.1f} deg | '
            f'err={math.degrees(heading_error):.1f} deg | '
            f'cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})'
        )

    def log_avoid_status(self, traveled: float, cmd: Twist):
        self._avoid_log_counter += 1
        log_every = max(1, int(1.0 / self.control_period_sec))
        if self._avoid_log_counter % log_every != 0:
            return

        self.get_logger().info(
            'avoid_forward | '
            f'selected={self.path_choice} | '
            f'traveled={traveled:.2f}/{self.avoid_forward_distance:.2f} m | '
            f'front={self.format_distance(self.current_tof_distance())} | '
            f'avoid_stop={self.avoid_stop_distance:.2f} m | '
            f'cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})'
        )

    def print_nav_status(self, event: str, cmd: Twist = None, force: bool = False):
        if not self.print_telemetry:
            return

        now = self.get_clock().now()
        if not force and self._last_print_time is not None:
            elapsed = (now - self._last_print_time).nanoseconds / 1e9
            if elapsed < self.print_telemetry_period_sec:
                return

        self._last_print_time = now
        linear_cmd = cmd.linear.x if cmd is not None else 0.0
        angular_cmd = cmd.angular.z if cmd is not None else 0.0
        print(
            '[GOAL_NAV] '
            f'event={event} | '
            f'mode={self.mode} | '
            f'current=(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}) m | '
            f'heading={math.degrees(self.theta):.1f} deg | '
            f'goal=(x={self.goal_x:.2f}, y={self.goal_y:.2f}, z=0.00) m | '
            f'distance_left={self.distance_left_to_goal():.2f} m | '
            f'tof_front={self.format_distance(self.current_tof_distance())} | '
            f'cmd=(linear={linear_cmd:.2f}, angular={angular_cmd:.2f})',
            flush=True,
        )

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value: float, lower: float, upper: float) -> float:
        return max(min(value, upper), lower)

    @staticmethod
    def parameter_as_bool(value):
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def destroy_node(self):
        self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
