#!/usr/bin/env python3
"""
navigator_node.py — mission state machine

STRATEGY for every ArUco target (ID 0/1 and ID 3/4)
────────────────────────────────────────────────────
  1. CENTER   — robot stationary. Internal PID on aruco/angle_error drives
                angular.z until |angle_error| < ANGLE_CENTER_THRESH (~2%).
                Camera re-checked after each correction; loops until settled.

  2. APPROACH — robot drives forward. Internal PID on aruco/angle_error keeps
                angular.z corrected every camera frame. Linear speed scales
                proportionally from MAX_SPEED (far) down to MIN_SPEED as
                distance approaches LATCH_DIST_M.

  3. LATCH    — at LATCH_DIST_M the camera-based approach stops. We snapshot
                distance and angle from the camera (last known values before
                the marker disappears at close range), then:
                  a) Drive forward snapshot_distance via waypoint → pid_controller
                     (converts distance to absolute odom coords, PID navigates)
                  b) Rotate snapshot_angle via odometry spin loop inside navigator
                     (reads /odom wheel ticks, proportional controller, no waypoint)

DECISION MARKERS (ID 2 & ID 5) — special case
──────────────────────────────────────────────
  CENTER only (same PID loop). Approach until within DECISION_STOP_DIST.
  Stop. Read the detected marker ID to decide path:
    - If ID 2: Go Left (Target ID 3)
    - If ID 5: Go Right (Target ID 4)
  No drive/rotate at the end — just stop and decide.

  LOST-MARKER RECOVERY (SEEK_DECISION only)
  ──────────────────────────────────────────
  If the decision marker is lost for SEEK_LOST_TICKS consecutive ticks,
  the robot backs up SEEK_BACKUP_M and returns to CENTER_DECISION to
  re-acquire from a better angle/distance. This repeats up to
  SEEK_MAX_RETRIES times before giving up and stopping.

SEQUENCE
────────
 1. CENTER_ARUCO      Center ID 0/1.
 2. SEEK_ARUCO        Approach ID 0/1. At latch → drive + odom-spin → WAIT_FOR_COLOR.
 3. WAIT_FOR_COLOR    Wait for green.
 4. LINE_FOLLOW_1     Follow line until lost (15-frame rule in line_tracker).
 5. CENTER_DECISION   Center ID 2 or 5.
 6. SEEK_DECISION     Approach ID 2 or 5 until DECISION_STOP_DIST. Decide left/right.
 7. CENTER_TARGET     Center ID 3 or 4.
 8. APPROACH_TARGET   Approach ID 3/4. At latch → drive + odom-spin → LINE_FOLLOW_2.
 9. LINE_FOLLOW_2     Follow line until lost.
10. DONE              Stop forever.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32, String

# ── Approach tuning ───────────────────────────────────────────────────────────
MAX_SPEED            = 0.20
MIN_SPEED            = 0.06
LATCH_DIST_M         = 0.30   # upper bound — start latching below this
LATCH_DIST_MIN       = 0.25   # lower bound — ignore glitchy close readings
DECISION_STOP_DIST   = 0.30   # upper bound
DECISION_STOP_MIN    = 0.25   # lower bound

# ── Approach PID (angle correction while driving) ─────────────────────────────
APPROACH_KP          = 1.2
APPROACH_KI          = 0.05
APPROACH_KD          = 0.08
APPROACH_INTEGRAL_MAX = 0.3

# ── Centering PID (angle correction while stationary) ────────────────────────
CENTER_KP            = 0.8 #was 1.5
CENTER_KI            = 0.08
CENTER_KD            = 0.15 #was 0.05
CENTER_INTEGRAL_MAX  = 0.3
ANGLE_CENTER_THRESH  = math.radians(3.6)
CENTER_MAX_OMEGA     = 1.2

# ── Final odom spin (after drive, no camera) ──────────────────────────────────
SPIN_KP              = 3.0
SPIN_MIN_OMEGA       = 0.25
SPIN_MAX_OMEGA       = 1.5
SPIN_THRESH          = math.radians(2.0)

# ── Decision turn (odom spin after decision, before centering target) ─────────
DECISION_TURN_DEG    = 20.0
FIRST_MARKER_IDS     = (0, 1)
LEFT_DECISION_ID     = 2
RIGHT_DECISION_ID    = 5
DECISION_MARKER_IDS  = (LEFT_DECISION_ID, RIGHT_DECISION_ID)

LEFT_TARGET_ID       = 3
RIGHT_TARGET_ID      = 4

MIN_VALID_DIST       = 0.05

# ── Lost-marker recovery (SEEK_DECISION only) ─────────────────────────────────
SEEK_LOST_TICKS  = 10    # consecutive ticks without marker before backing up
SEEK_BACKUP_M    = 0.25  # metres to reverse
SEEK_BACKUP_SPEED = 0.08 # m/s reverse speed
SEEK_MAX_RETRIES = 5     # give up after this many backup attempts


# ── State names ───────────────────────────────────────────────────────────────
class S:
    CENTER_ARUCO      = 'CENTER_ARUCO'
    CENTER_DECISION   = 'CENTER_DECISION'
    CENTER_TARGET     = 'CENTER_TARGET'

    SEEK_ARUCO        = 'SEEK_ARUCO'
    SEEK_DECISION     = 'SEEK_DECISION'
    APPROACH_TARGET   = 'APPROACH_TARGET'

    PID_DRIVE         = 'PID_DRIVE'
    ODOM_SPIN         = 'ODOM_SPIN'
    BACKUP            = 'BACKUP'

    WAIT_FOR_COLOR    = 'WAIT_FOR_COLOR'
    LINE_FOLLOW_1     = 'LINE_FOLLOW_1'
    LINE_FOLLOW_2     = 'LINE_FOLLOW_2'
    DONE              = 'DONE'


CENTER_STATES        = (S.CENTER_ARUCO, S.CENTER_DECISION, S.CENTER_TARGET)
APPROACH_STATES      = (S.SEEK_ARUCO, S.SEEK_DECISION, S.APPROACH_TARGET)


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator')

        # ── Sensor state ──────────────────────────────────────────────────
        self._aruco_detected  = False
        self._aruco_id        = -1
        self._aruco_angle     = 0.0
        self._aruco_heading   = 0.0
        self._aruco_distance  = 999.0
        self._color           = 'none'

        # ── Odometry state ────────────────────────────────────────────────
        self._odom_x          = 0.0
        self._odom_y          = 0.0
        self._odom_theta      = 0.0

        # ── Approach PID state ────────────────────────────────────────────
        self._app_integral    = 0.0
        self._app_prev_error  = 0.0
        self._app_prev_time   = None

        # ── Centering PID state ───────────────────────────────────────────
        self._ctr_integral    = 0.0
        self._ctr_prev_error  = 0.0
        self._ctr_prev_time   = None

        # ── Odom spin state ───────────────────────────────────────────────
        self._spin_target_angle = 0.0
        self._spin_start_theta  = 0.0
        self._spin_after        = S.WAIT_FOR_COLOR

        # ── Latch / mission state ─────────────────────────────────────────
        self._latched_angle   = 0.0
        self._latched_heading = 0.0
        self._latched_dist    = 0.0
        self._after_drive     = S.WAIT_FOR_COLOR
        self._used_ids        = set()
        self._after_center    = S.SEEK_ARUCO
        self._target_id       = -1

        # ── Lost-marker backup state ──────────────────────────────────────
        self._seek_lost_ticks   = 0    # ticks since marker was last seen
        self._seek_retries      = 0    # how many times we've backed up
        self._backup_start_x    = 0.0
        self._backup_start_y    = 0.0
        self._backup_after      = S.CENTER_DECISION

        # ── Mission state ─────────────────────────────────────────────────
        self.state            = S.CENTER_ARUCO

        # ── Debug throttle ────────────────────────────────────────────────
        self._dbg_approach_n  = 0
        self._dbg_centering_n = 0
        self._dbg_spin_n      = 0
        self._dbg_every       = 10

        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_pub             = self.create_publisher(Twist,       '/cmd_vel',              10)
        self.goal_pub            = self.create_publisher(PoseStamped, '/goal_pose',            10)
        self.angular_enabled_pub = self.create_publisher(Bool,        '/linefollower/enabled', 10)
        self.line_tracker_pub    = self.create_publisher(Bool,        '/line_tracker/enabled', 10)
        self.state_pub           = self.create_publisher(String,      '/navigator/state',      10)
        self.target_id_pub       = self.create_publisher(Int32,       '/aruco/target_id',      10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Bool,     '/aruco/detected',         self._cb_aruco_detected, 10)
        self.create_subscription(Int32,    '/aruco/id',               self._cb_aruco_id,       10)
        self.create_subscription(Float32,  '/aruco/angle_error',      self._cb_aruco_angle,    10)
        self.create_subscription(Float32,  '/aruco/heading',          self._cb_aruco_heading,  10)
        self.create_subscription(Float32,  '/aruco/distance',         self._cb_aruco_distance, 10)
        self.create_subscription(String,   '/color_detection/result', self._cb_color,          10)
        self.create_subscription(Bool,     '/line_tracker/lost',      self._cb_line_lost,      10)
        self.create_subscription(Bool,     '/goal_reached',           self._cb_goal_reached,   10)
        self.create_subscription(Odometry, '/odom',                   self._cb_odom,           10)

        self.create_timer(0.1, self._tick)

        self._set_linefollower(False)
        self.get_logger().info('Navigator ready — CENTER_ARUCO')

    # =========================================================================
    # General helpers
    # =========================================================================

    def _set_linefollower(self, enabled: bool):
        self.angular_enabled_pub.publish(Bool(data=enabled))
        self.line_tracker_pub.publish(Bool(data=enabled))
        if not enabled:
            self._stop()

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _transition(self, new_state: str):
        self.get_logger().info(f'{self.state} → {new_state}')
        self.state = new_state

    def _proportional_speed(self, distance: float) -> float:
        far = 1.0
        t = max(0.0, min(1.0, (distance - LATCH_DIST_M) / (far - LATCH_DIST_M)))
        return MIN_SPEED + t * (MAX_SPEED - MIN_SPEED)

    def _expected_id(self) -> int:
        if self.state in (S.CENTER_ARUCO, S.SEEK_ARUCO):
            if self._aruco_id in FIRST_MARKER_IDS:
                return self._aruco_id
            return -1
        if self.state in (S.CENTER_DECISION, S.SEEK_DECISION):
            if self._aruco_id in DECISION_MARKER_IDS:
                return self._aruco_id
            return -2
        if self.state in (S.CENTER_TARGET, S.APPROACH_TARGET):
            return self._target_id
        return -1

    def _is_wanted_id(self) -> bool:
        exp = self._expected_id()
        if exp == -1:
            if self.state in (S.CENTER_ARUCO, S.SEEK_ARUCO):
                return self._aruco_id in FIRST_MARKER_IDS
            return True
        if exp == -2:
            return self._aruco_id in DECISION_MARKER_IDS
        return self._aruco_id == exp

    # =========================================================================
    # Centering PID
    # =========================================================================

    def _reset_center_pid(self):
        self._ctr_integral   = 0.0
        self._ctr_prev_error = 0.0
        self._ctr_prev_time  = None

    def _center_pid_output(self, error: float) -> float:
        now = self.get_clock().now()
        if self._ctr_prev_time is None:
            dt = 0.0
        else:
            dt = (now - self._ctr_prev_time).nanoseconds / 1e9
        self._ctr_prev_time = now

        self._ctr_integral += error * dt
        self._ctr_integral  = max(-CENTER_INTEGRAL_MAX, min(CENTER_INTEGRAL_MAX, self._ctr_integral))
        derivative = (error - self._ctr_prev_error) / dt if dt > 0 else 0.0
        self._ctr_prev_error = error

        omega = (CENTER_KP * error + CENTER_KI * self._ctr_integral + CENTER_KD * derivative)
        return max(-CENTER_MAX_OMEGA, min(CENTER_MAX_OMEGA, -omega))

    def _run_centering(self):
        if not self._aruco_detected or not self._is_wanted_id():
            self._dbg_centering_n += 1
            if self._dbg_centering_n % self._dbg_every == 0:
                self.get_logger().info(
                    f'[{self.state}] waiting for marker '
                    f'(detected={self._aruco_detected}, id={self._aruco_id})')
            self._stop()
            return

        error = self._aruco_angle
        self._dbg_centering_n += 1
        if self._dbg_centering_n % self._dbg_every == 0:
            self.get_logger().info(
                f'[{self.state}] centering ID {self._aruco_id} — '
                f'angle_error={math.degrees(error):.1f}°  '
                f'threshold={math.degrees(ANGLE_CENTER_THRESH):.1f}°')

        if abs(error) < ANGLE_CENTER_THRESH:
            self._stop()
            self._reset_center_pid()
            self.get_logger().info(
                f'[{self.state}] ✓ centred  angle={math.degrees(error):.1f}° '
                f'→ {self._after_center}')
            self._transition(self._after_center)
        else:
            omega = self._center_pid_output(error)
            cmd = Twist()
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)

    # =========================================================================
    # Approach PID
    # =========================================================================

    def _reset_approach_pid(self):
        self._app_integral   = 0.0
        self._app_prev_error = 0.0
        self._app_prev_time  = None

    def _approach_pid_omega(self, error: float) -> float:
        now = self.get_clock().now()
        if self._app_prev_time is None:
            dt = 0.0
        else:
            dt = (now - self._app_prev_time).nanoseconds / 1e9
        self._app_prev_time = now

        self._app_integral += error * dt
        self._app_integral  = max(-APPROACH_INTEGRAL_MAX, min(APPROACH_INTEGRAL_MAX, self._app_integral))
        derivative = (error - self._app_prev_error) / dt if dt > 0 else 0.0
        self._app_prev_error = error

        omega = (APPROACH_KP * error + APPROACH_KI * self._app_integral + APPROACH_KD * derivative)
        return max(-2.0, min(2.0, -omega))

    def _is_approach_target(self) -> bool:
        return self._is_wanted_id()

    # =========================================================================
    # Post-latch drive
    # =========================================================================

    def _send_drive_waypoint(self, distance: float):
        gx = self._odom_x + distance * math.cos(self._odom_theta)
        gy = self._odom_y + distance * math.sin(self._odom_theta)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f'Drive waypoint → ({gx:.3f}, {gy:.3f})  ({distance:.3f}m forward)')

    # =========================================================================
    # Post-latch spin
    # =========================================================================

    def _start_odom_spin(self, angle_rad: float, after: str):
        if abs(angle_rad) < SPIN_THRESH:
            self.get_logger().info(
                f'Odom spin: {math.degrees(angle_rad):.1f}° too small to execute — skipping')
            self._stop()
            if after in (S.LINE_FOLLOW_1, S.LINE_FOLLOW_2):
                self._set_linefollower(True)
            self._transition(after)
            return

        self._spin_target_angle = angle_rad
        self._spin_start_theta  = self._odom_theta
        self._spin_after        = after
        self.get_logger().info(
            f'Odom spin: {math.degrees(angle_rad):.1f}° from θ={math.degrees(self._odom_theta):.1f}°')
        self._transition(S.ODOM_SPIN)

    def _run_odom_spin(self):
        turned    = angle_diff(self._odom_theta, self._spin_start_theta)
        remaining = angle_diff(self._spin_target_angle, turned)

        self._dbg_spin_n += 1
        if self._dbg_spin_n % self._dbg_every == 0:
            self.get_logger().info(
                f'[ODOM_SPIN] target={math.degrees(self._spin_target_angle):.1f}°  '
                f'turned={math.degrees(turned):.1f}°  '
                f'remaining={math.degrees(remaining):.1f}°')

        if abs(remaining) < SPIN_THRESH:
            self._stop()
            self.get_logger().info(f'[ODOM_SPIN] ✓ done → {self._spin_after}')
            after = self._spin_after
            if after in (S.LINE_FOLLOW_1, S.LINE_FOLLOW_2):
                self._set_linefollower(True)
            self._transition(after)
            return

        omega = SPIN_KP * remaining
        if omega > 0:
            omega = max(SPIN_MIN_OMEGA, min(SPIN_MAX_OMEGA, omega))
        else:
            omega = min(-SPIN_MIN_OMEGA, max(-SPIN_MAX_OMEGA, omega))

        cmd = Twist()
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    # =========================================================================
    # Latch logic
    # =========================================================================

    def _latch(self, drive_after: str):
        self._latched_dist    = self._aruco_distance
        self._latched_angle   = self._aruco_angle
        self._latched_heading = self._aruco_heading
        self._after_drive     = drive_after
        self._stop()
        self.get_logger().info(
            f'Latch: dist={self._latched_dist:.3f}m  '
            f'angle={math.degrees(self._latched_angle):.1f}°')
        self._send_drive_waypoint(self._latched_dist)
        self._transition(S.PID_DRIVE)

    # =========================================================================
    # Lost-marker backup — crawl back until marker reacquired
    # =========================================================================

    def _start_backup(self, after=None):
        """Crawl backwards slowly until marker is reacquired, then centre and latch."""
        if after is None:
            after = S.CENTER_DECISION
        self._seek_lost_ticks = 0
        self._backup_after    = after
        self.get_logger().info(
            f'[{self.state}] Marker lost — crawling back slowly until reacquired')
        self._transition(S.BACKUP)

    def _run_backup(self):
        """Crawl backwards until marker is reacquired, then stop and centre."""
        if self._aruco_detected and self._is_wanted_id():
            self._stop()
            self.get_logger().info(
                f'[BACKUP] Marker reacquired (id={self._aruco_id}) — stopping and centring')
            self._seek_lost_ticks = 0
            self._reset_center_pid()
            self._after_center = self._backup_after
            self._transition(self._backup_after)
            return

        cmd = Twist()
        cmd.linear.x = -SEEK_BACKUP_SPEED
        self.cmd_pub.publish(cmd)

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _cb_odom(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_theta = math.atan2(siny, cosy)

    def _cb_aruco_detected(self, msg):
        prev = self._aruco_detected
        self._aruco_detected = msg.data
        if not prev and msg.data and self._is_wanted_id():
            self.get_logger().info(f'[{self.state}] ArUco marker acquired (id={self._aruco_id})')
            if self.state == S.SEEK_DECISION:
                self._seek_lost_ticks = 0   # reset counter on re-acquisition
        elif prev and not msg.data:
            self.get_logger().info(f'[{self.state}] ArUco marker lost')

    def _cb_aruco_id(self, msg):
        prev_id = self._aruco_id
        self._aruco_id = msg.data
        if self._aruco_id != prev_id and self._is_wanted_id():
            self.get_logger().info(f'[{self.state}] ArUco ID changed: {prev_id} → {self._aruco_id}')

    def _cb_aruco_heading(self, msg):
        self._aruco_heading = msg.data

    def _cb_aruco_angle(self, msg):
        self._aruco_angle = msg.data

        if self.state not in APPROACH_STATES:
            return
        if not self._aruco_detected or self._aruco_id in self._used_ids:
            return
        if not self._is_approach_target():
            return
        if (LATCH_DIST_MIN <= self._aruco_distance <= LATCH_DIST_M) or self._aruco_distance < MIN_VALID_DIST:
            return

        speed = self._proportional_speed(self._aruco_distance)
        omega = self._approach_pid_omega(self._aruco_angle)
        cmd = Twist()
        cmd.linear.x  = speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        self._dbg_approach_n += 1
        if self._dbg_approach_n % self._dbg_every == 0:
            self.get_logger().info(
                f'[{self.state}] approaching ID {self._aruco_id} — '
                f'dist={self._aruco_distance:.3f}m  '
                f'speed={speed:.3f}  omega={omega:.3f}')

    def _cb_aruco_distance(self, msg):
        self._aruco_distance = msg.data

        if self.state not in APPROACH_STATES:
            return
        if not self._aruco_detected or self._aruco_distance < MIN_VALID_DIST:
            return
        if self._aruco_id in self._used_ids:
            return
        if not self._is_approach_target():
            return

        if self.state == S.SEEK_DECISION:
            if DECISION_STOP_MIN <= self._aruco_distance <= DECISION_STOP_DIST:
                self._stop()
                self._reset_approach_pid()
                self.get_logger().info(
                    f'Decision marker (ID {self._aruco_id}) reached at {self._aruco_distance:.3f}m — evaluating path.')
                self._make_decision()
            return

        if LATCH_DIST_MIN <= self._aruco_distance <= LATCH_DIST_M:
            self._used_ids.add(self._aruco_id)
            self._reset_approach_pid()

            if self.state == S.SEEK_ARUCO:
                self._latch(drive_after=S.WAIT_FOR_COLOR)
            elif self.state == S.APPROACH_TARGET:
                self._latch(drive_after=S.LINE_FOLLOW_2)

    def _cb_goal_reached(self, msg: Bool):
        if not msg.data or self.state != S.PID_DRIVE:
            return

        self.get_logger().info(
            f'Drive done — starting odom spin {math.degrees(-self._latched_heading):.1f}°')
        self._start_odom_spin(
            angle_rad = -self._latched_heading,
            after     = self._after_drive)

    def _cb_color(self, msg):
        prev = self._color
        self._color = msg.data
        if self._color != prev:
            self.get_logger().info(f'[{self.state}] color detection: {self._color}')
        if self.state == S.WAIT_FOR_COLOR and self._color == 'green':
            self.get_logger().info('[WAIT_FOR_COLOR] GREEN detected — starting line follower')
            self._set_linefollower(True)
            self._transition(S.LINE_FOLLOW_1)

    def _cb_line_lost(self, msg):
        if not msg.data:
            return
        if self.state == S.LINE_FOLLOW_1:
            self.get_logger().info('Line lost — tracking decision markers')
            self._set_linefollower(False)
            self._seek_lost_ticks = 0
            self._seek_retries    = 0
            self._after_center = S.SEEK_DECISION
            self._reset_center_pid()
            self._transition(S.CENTER_DECISION)
        elif self.state == S.LINE_FOLLOW_2:
            self.get_logger().info('Line lost — mission complete')
            self._set_linefollower(False)
            self._transition(S.DONE)

    # =========================================================================
    # Explicit ID Decision Routing (ID 2 vs ID 5)
    # =========================================================================

    def _make_decision(self):
        chosen_marker = self._aruco_id
        self._used_ids.add(chosen_marker)

        if chosen_marker == LEFT_DECISION_ID:
            self._target_id = LEFT_TARGET_ID
            direction       = f'LEFT (Target ID {LEFT_TARGET_ID})'
            turn_rad        = math.radians(DECISION_TURN_DEG)
        elif chosen_marker == RIGHT_DECISION_ID:
            self._target_id = RIGHT_TARGET_ID
            direction       = f'RIGHT (Target ID {RIGHT_TARGET_ID})'
            turn_rad        = math.radians(-DECISION_TURN_DEG)
        else:
            self.get_logger().warn(f'Unexpected ID {chosen_marker} inside decision handler! Defaulting LEFT.')
            self._target_id = LEFT_TARGET_ID
            direction       = f'LEFT (Target ID {LEFT_TARGET_ID}) [fallback]'
            turn_rad        = math.radians(DECISION_TURN_DEG)

        self.get_logger().info(
            f'Decision: Detected ID {chosen_marker} → Routing {direction} '
            f'— execution turning {abs(DECISION_TURN_DEG):.0f}° via Odometry.')

        self._after_center = S.APPROACH_TARGET
        self._reset_center_pid()
        self._start_odom_spin(angle_rad=turn_rad, after=S.CENTER_TARGET)

    # =========================================================================
    # Tick
    # =========================================================================

    def _tick(self):
        self.state_pub.publish(String(data=self.state))

        exp = self._expected_id()
        if exp == -1 and self.state in (S.CENTER_ARUCO, S.SEEK_ARUCO):
            self.target_id_pub.publish(Int32(data=-1))
        elif exp == -2:
            self.target_id_pub.publish(Int32(data=-2))
        else:
            self.target_id_pub.publish(Int32(data=exp))

        if self.state == S.CENTER_ARUCO:
            self._after_center = S.SEEK_ARUCO
            self._seek_lost_ticks = 0
            self._seek_retries    = 0
            self._run_centering()

        elif self.state == S.CENTER_DECISION:
            self._after_center = S.SEEK_DECISION
            self._run_centering()

        elif self.state == S.CENTER_TARGET:
            self._seek_lost_ticks = 0
            self._seek_retries    = 0
            self._run_centering()

        elif self.state == S.ODOM_SPIN:
            self._run_odom_spin()

        elif self.state == S.BACKUP:
            self._run_backup()

        elif self.state == S.SEEK_DECISION:
            if not self._aruco_detected:
                self._seek_lost_ticks += 1
                self._stop()
                if self._seek_lost_ticks >= SEEK_LOST_TICKS:
                    self._start_backup()
                else:
                    self.get_logger().info(
                        f'[{self.state}] marker not detected — stopped '
                        f'({self._seek_lost_ticks}/{SEEK_LOST_TICKS} ticks)')
            else:
                self._seek_lost_ticks = 0   # marker visible, reset counter

        elif self.state == S.SEEK_ARUCO and not self._aruco_detected:
            self._seek_lost_ticks += 1
            self._stop()
            if self._seek_lost_ticks >= SEEK_LOST_TICKS:
                self._start_backup(after=S.CENTER_ARUCO)
            else:
                self.get_logger().info(
                    f'[SEEK_ARUCO] marker not detected — stopped '
                    f'({self._seek_lost_ticks}/{SEEK_LOST_TICKS} ticks)')

        elif self.state == S.APPROACH_TARGET and not self._aruco_detected:
            self._seek_lost_ticks += 1
            self._stop()
            if self._seek_lost_ticks >= SEEK_LOST_TICKS:
                self._start_backup(after=S.CENTER_TARGET)
            else:
                self.get_logger().info(
                    f'[APPROACH_TARGET] marker not detected — stopped '
                    f'({self._seek_lost_ticks}/{SEEK_LOST_TICKS} ticks)')

        elif self.state in APPROACH_STATES and not self._aruco_detected:
            self._stop()
            self.get_logger().info(f'[{self.state}] marker not detected — stopped')

        elif self.state == S.DONE:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()