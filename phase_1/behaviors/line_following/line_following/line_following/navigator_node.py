#!/usr/bin/env python3
"""
navigator_node.py — mission state machine

SEQUENCE
────────
 1. SEEK_ARUCO      Drive toward ArUco ID 1. Stop when close enough.
 2. WAIT_FOR_COLOR  Wait for green or pink on /color_detection/result.
 3. LINE_FOLLOW_1   Follow the line. Exit only when ArUco ID 2 is seen.
 4. SEEK_DECISION   Stopped. Read ID 2's angle to decide left or right.
                    -45° to +45° → left  (ID 3)
                    outside that → right (ID 4)
 5. APPROACH_TARGET Drive toward chosen marker (ID 3 or 4). Stop when close.
 6. LINE_FOLLOW_2   Follow the line until DONE (timer, or wire your own exit).
 7. DONE            Stop forever.

TUNE THESE
──────────
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32, String

# ── Phase 1 – approach ArUco ID 1 ────────────────────────────────────────────
SEEK_ARUCO_ID        = 1
SEEK_FORWARD_SPEED   = 0.10   # m/s
SEEK_ANGULAR_GAIN    = 1.2    # angle_error (rad) → angular.z scaling
SEEK_STOP_DIST_M     = 0.30   # stop this close to the marker
SEEK_LOST_TIMEOUT_S  = 0.5    # seconds without ID-1 before stopping in place

# ── Phase 3 – line follow until ID 2 is seen ─────────────────────────────────
DECISION_MARKER_ID   = 2

# ── Phase 4 – decision ───────────────────────────────────────────────────────
LEFT_TARGET_ID       = 3
RIGHT_TARGET_ID      = 4      # change if your right marker has a different ID
DECISION_ANGLE_DEG   = 45.0   # ±45° from centre → left, outside → right
DECISION_SCAN_TIMEOUT = 5.0   # give up waiting for ID 2 after this many seconds

# ── Phase 5 – approach chosen marker ─────────────────────────────────────────
APPROACH_FORWARD_SPEED = 0.10
APPROACH_ANGULAR_GAIN  = 1.2
APPROACH_STOP_DIST_M   = 0.20

# ── Phase 6 – second line follow ─────────────────────────────────────────────
PHASE6_FOLLOW_DURATION = 10.0   # seconds — replace with a real exit condition later


class S:
    SEEK_ARUCO      = 'SEEK_ARUCO'
    WAIT_FOR_COLOR  = 'WAIT_FOR_COLOR'
    LINE_FOLLOW_1   = 'LINE_FOLLOW_1'
    SEEK_DECISION   = 'SEEK_DECISION'
    APPROACH_TARGET = 'APPROACH_TARGET'
    LINE_FOLLOW_2   = 'LINE_FOLLOW_2'
    DONE            = 'DONE'


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator')

        # ── Latest sensor readings (updated by callbacks) ──────────────────
        # ArUco — separate flag+value per ID so a "not detected" message for
        # one ID doesn't wipe out a just-received reading for another.
        self._aruco_detected  = False   # True only if something is visible right now
        self._aruco_id        = -1
        self._aruco_angle     = 0.0
        self._aruco_distance  = 999.0
        self._last_seen_time  = 0.0     # wall time of last detected=True message

        self._color           = 'none'

        # ── Mission state ─────────────────────────────────────────────────
        self.state          = S.SEEK_ARUCO
        self._state_start   = time.time()
        self._target_id     = -1
        self._decision_seen  = False
        self._decision_angle = 0.0

        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_pub             = self.create_publisher(Twist,  '/cmd_vel',              10)
        self.angular_enabled_pub = self.create_publisher(Bool,   '/linefollower/enabled', 10)
        self.line_tracker_pub    = self.create_publisher(Bool,   '/line_tracker/enabled', 10)
        self.state_pub           = self.create_publisher(String, '/navigator/state',      10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Bool,    '/aruco/detected',         self._cb_detected, 10)
        self.create_subscription(Int32,   '/aruco/id',               self._cb_id,       10)
        self.create_subscription(Float32, '/aruco/angle_error',      self._cb_angle,    10)
        self.create_subscription(Float32, '/aruco/distance',         self._cb_distance, 10)
        self.create_subscription(String,  '/color_detection/result', self._cb_color,    10)

        # ── 20 Hz tick ────────────────────────────────────────────────────
        self.create_timer(0.05, self._step)

        self._set_linefollower(False)
        self.get_logger().info('Navigator ready — SEEK_ARUCO (looking for ID 1)')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_detected(self, msg):
        self._aruco_detected = msg.data
        if msg.data:
            self._last_seen_time = time.time()

    def _cb_id(self, msg):
        self._aruco_id = msg.data

    def _cb_angle(self, msg):
        self._aruco_angle = msg.data

    def _cb_distance(self, msg):
        self._aruco_distance = msg.data

    def _cb_color(self, msg):
        self._color = msg.data

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_linefollower(self, enabled: bool):
        self.angular_enabled_pub.publish(Bool(data=enabled))
        self.line_tracker_pub.publish(Bool(data=enabled))
        if not enabled:
            self._stop()

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _drive_toward_aruco(self, forward_speed: float, angular_gain: float):
        cmd = Twist()
        cmd.linear.x  = forward_speed
        # angle_error positive = marker to the right → turn right (negative angular.z)
        cmd.angular.z = max(-2.0, min(2.0, -angular_gain * self._aruco_angle))
        self.cmd_pub.publish(cmd)

    def _transition(self, new_state: str):
        self.get_logger().info(f'{self.state} → {new_state}')
        self.state        = new_state
        self._state_start = time.time()

    def _elapsed(self) -> float:
        return time.time() - self._state_start

    def _seeing(self, marker_id: int) -> bool:
        """True if the aruco detector is currently reporting this specific ID."""
        return self._aruco_detected and self._aruco_id == marker_id

    # ── State machine ─────────────────────────────────────────────────────────

    def _step(self):
        self.state_pub.publish(String(data=self.state))

        # ── SEEK_ARUCO ────────────────────────────────────────────────────
        if self.state == S.SEEK_ARUCO:
            if self._seeing(SEEK_ARUCO_ID):
                if self._aruco_distance <= SEEK_STOP_DIST_M:
                    self._stop()
                    self.get_logger().info(
                        f'ID {SEEK_ARUCO_ID} reached at '
                        f'{self._aruco_distance:.2f} m — waiting for colour')
                    self._transition(S.WAIT_FOR_COLOR)
                else:
                    self._drive_toward_aruco(SEEK_FORWARD_SPEED, SEEK_ANGULAR_GAIN)
            else:
                # ID 1 not visible — stop and wait for it to reappear
                if time.time() - self._last_seen_time > SEEK_LOST_TIMEOUT_S:
                    self._stop()

        # ── WAIT_FOR_COLOR ────────────────────────────────────────────────
        elif self.state == S.WAIT_FOR_COLOR:
            self._stop()
            if self._color in ('green', 'pink'):
                self.get_logger().info(
                    f'{self._color.upper()} — starting line follower')
                self._set_linefollower(True)
                self._transition(S.LINE_FOLLOW_1)

        # ── LINE_FOLLOW_1 ─────────────────────────────────────────────────
        # Keep following until ArUco ID 2 comes into view.
        elif self.state == S.LINE_FOLLOW_1:
            if self._seeing(DECISION_MARKER_ID):
                self._set_linefollower(False)
                self.get_logger().info(
                    f'ID {DECISION_MARKER_ID} spotted — making direction decision')
                self._decision_seen  = False
                self._decision_angle = 0.0
                self._transition(S.SEEK_DECISION)

        # ── SEEK_DECISION ─────────────────────────────────────────────────
        # Robot is stopped. Latch ID-2's angle, then decide.
        elif self.state == S.SEEK_DECISION:
            self._stop()

            if self._seeing(DECISION_MARKER_ID):
                self._decision_seen  = True
                self._decision_angle = self._aruco_angle

            timed_out = self._elapsed() >= DECISION_SCAN_TIMEOUT

            if self._decision_seen or timed_out:
                angle_deg = math.degrees(self._decision_angle)

                if abs(angle_deg) <= DECISION_ANGLE_DEG:
                    self._target_id = LEFT_TARGET_ID
                    direction = 'LEFT'
                else:
                    self._target_id = RIGHT_TARGET_ID
                    direction = 'RIGHT'

                if timed_out and not self._decision_seen:
                    self.get_logger().warn(
                        f'ID {DECISION_MARKER_ID} never seen — '
                        f'defaulting {direction} to ID {self._target_id}')
                else:
                    self.get_logger().info(
                        f'ID {DECISION_MARKER_ID} angle={angle_deg:.1f}° → '
                        f'{direction} to ID {self._target_id}')

                self._transition(S.APPROACH_TARGET)

        # ── APPROACH_TARGET ───────────────────────────────────────────────
        elif self.state == S.APPROACH_TARGET:
            if self._seeing(self._target_id):
                if self._aruco_distance <= APPROACH_STOP_DIST_M:
                    self._stop()
                    self.get_logger().info(
                        f'Reached ID {self._target_id} at '
                        f'{self._aruco_distance:.2f} m — starting LINE_FOLLOW_2')
                    self._set_linefollower(True)
                    self._transition(S.LINE_FOLLOW_2)
                else:
                    self._drive_toward_aruco(
                        APPROACH_FORWARD_SPEED, APPROACH_ANGULAR_GAIN)
            else:
                # Target not visible — hold still
                self._stop()

        # ── LINE_FOLLOW_2 ─────────────────────────────────────────────────
        elif self.state == S.LINE_FOLLOW_2:
            if self._elapsed() >= PHASE6_FOLLOW_DURATION:
                self._set_linefollower(False)
                self.get_logger().info('Mission complete.')
                self._transition(S.DONE)

        # ── DONE ──────────────────────────────────────────────────────────
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