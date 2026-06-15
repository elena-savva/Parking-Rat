#!/usr/bin/env python3
"""
Movement Selector Node
======================
Presents a menu and sends sequential goal poses to the PID controller.
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        # ── Publisher ─────────────────────────────────────────────────────────
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ── Subscriber — listens for PID to signal "goal reached" ─────────────
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)

        # ── Internal state ────────────────────────────────────────────────────
        self.waypoints     = []     # list of (x, y) tuples for current movement
        self.current_wp    = 0      # index of the next waypoint to send
        self.waiting       = False  # True while waiting for PID to finish

        # ── Show the menu once the node is spinning ───────────────────────────
        # We use a one-shot timer so the node is fully initialized first
        self.create_timer(1.0, self.show_menu_once)
        self._menu_shown = False

    # ── Menu ──────────────────────────────────────────────────────────────────

    def show_menu_once(self):
        """Show the menu exactly once after startup."""
        if self._menu_shown:
            return
        self._menu_shown = True
        self.show_menu()

    def show_menu(self):
        print("\n" + "=" * 50)
        print("  MOVEMENT SELECTOR")
        print("=" * 50)
        print("  1 — Square  (4 × 1 m sides, CCW)")
        print("  2 — Circle  (radius 1 m, back to start)")
        print("  3 — Next stage (voice command node info)")
        print("  4 — Small Square Test")
        print("  5 — Test Report")
        print("  6 — Test Straight (1m)")
        print("  7 — Test Left")
        print("  8 — Test Right")
        print("=" * 50)
        choice = input("  Enter choice [1-8]: ").strip()

        if choice == "1":
            self.start_square()
        elif choice == "2":
            self.start_circle()
        elif choice == "3":
            self.next_stage_info()
        elif choice == '4':
            self.start_test()
        elif choice == '5':
            self.start_test_report()
        elif choice == '6':
            self.start_test_straight()
        elif choice == '7':
            self.start_test_left()
        elif choice == '8':
            self.start_test_right()
        elif choice == '9':
            self.start_spin_left_90()
        elif choice == '10':
            self.start_spin_right_90()
        else:
            print("  Invalid choice. Please try again.")
            self.show_menu()

    # ── Movements ─────────────────────────────────────────────────────────────

    def start_square(self):
        """
        1 m square, turning left (counter-clockwise).
        Starting at (0,0) facing +x:
          (0,0) → (1,0) → (1,1) → (0,1) → (0,0)
        """
        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]
        print("\n  Starting SQUARE. Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.1f}, {y:.1f})")
        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()

    def start_test(self):
        """
        Small 0.5m square test.
        """
        self.waypoints = [
            (0.5, 0.0),
            (0.5, 0.5),
            (1.0, 0.5),
            (0.5, 0.0),
        ]
        print("\n  Starting Small Square Test. Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.1f}, {y:.1f})")
        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()

    def start_test_report(self):
        """
        Test Report sequence.
        """
        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (2.0, 1.0)
        ]
        print("\n  Starting Test Report. Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.1f}, {y:.1f})")
        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()

    def start_test_straight(self):
        """
        Drive straight 1m.
        """
        self.waypoints = [
            (1.0, 0.0)
        ]
        print("\n  Starting Straight Test. Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.1f}, {y:.1f})")
        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()

    def start_test_left(self):
        """
        Drive left 1m (requires a 90-degree left turn first).
        """
        self.waypoints = [
            (0.0, 1.0)
        ]
        print("\n  Starting Left Test. Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.1f}, {y:.1f})")
        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()

    def start_test_right(self):
        """
        Drive right 1m (requires a 90-degree right turn first).
        """
        self.waypoints = [
            (0.0, -1.0)
        ]
        print("\n  Starting Right Test. Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.1f}, {y:.1f})")
        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()

    
    def start_spin_left_90(self):
        """
        Spin 90° left (CCW) in place.
        Sends a goal at a tiny distance in the +y direction so the robot
        turns 90° left and stops almost immediately after aligning.
        """
        self.waypoints = [
            (0.0, 0.03)   # 3 cm in +y → forces a 90° left turn
        ]
        print("\n  Starting Spin Left 90°. Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.3f}, {y:.3f})")
        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()
 
    def start_spin_right_90(self):
        """
        Spin 90° right (CW) in place.
        Sends a goal at a tiny distance in the -y direction so the robot
        turns 90° right and stops almost immediately after aligning.
        """
        self.waypoints = [
            (0.0, -0.03)  # 3 cm in -y → forces a 90° right turn
        ]
        print("\n  Starting Spin Right 90°. Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.3f}, {y:.3f})")
        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()


    def start_circle(self):
        """
        Circle of radius 1 m, 12 evenly-spaced waypoints, back to start.
        Centre at (0, 1) so the robot starts at (0,0) and goes CCW.
        """
        R     = 1.0
        steps = 12
        self.waypoints = []
        for i in range(1, steps + 1):
            theta = 2 * math.pi * i / steps
            x = R * math.sin(theta)
            y = R * (1.0 - math.cos(theta))
            self.waypoints.append((round(x, 3), round(y, 3)))

        print("\n  Starting CIRCLE (radius 1 m). Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"    {i+1}. ({x:.3f}, {y:.3f})")

        self.current_wp = 0
        self.waiting    = False
        self.send_next_waypoint()

    def next_stage_info(self):
        """Print information about the voice/text command node."""
        print("\n" + "=" * 50)
        print("  NEXT STAGE — Voice / Text Command Node")
        print("=" * 50)
        print("  This node listens on the ROS topic /text_command")
        print("  and maps words to movements:")
        print()
        print("    'run'  → drive forward away from operator")
        print("    'come' → drive toward operator")
        print("    'spin' → rotate 360°  in place")
        print()
        print("  To run it (once implemented):")
        print("    ros2 run motor_control text_command_node")
        print()
        print("  To send a command manually for testing:")
        print("    ros2 topic pub --once /text_command std_msgs/msg/String")
        print('    "{\\"data\\": \\"spin\\"}"')
        print("=" * 50)
        input("\nPress ENTER to return to the menu...")
        self.show_menu()

    # ── Waypoint sequencing ───────────────────────────────────────────────────

    def send_next_waypoint(self):
        """Publish the next waypoint in the list to /goal_pose."""
        if self.current_wp >= len(self.waypoints):
            print("\n  All waypoints reached! Movement complete.")
            input("  Press ENTER to return to the menu...")
            self.show_menu()
            return

        x, y = self.waypoints[self.current_wp]
        print(f"\n  → Sending waypoint {self.current_wp + 1}/{len(self.waypoints)}"
              f"  ({x:.3f}, {y:.3f})")

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        # Orientation doesn't matter for the PID — it only uses position
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        self.waiting = True   # wait for /goal_reached before sending the next one

    def goal_reached_cb(self, msg: Bool):
        """Called by the PID node when it has reached the current goal."""
        if not msg.data or not self.waiting:
            return

        self.waiting     = False
        self.current_wp += 1
        self.send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n  Movement node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()