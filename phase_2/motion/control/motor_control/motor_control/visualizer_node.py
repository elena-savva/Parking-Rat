#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Data storage
        self.x_data = []
        self.y_data = []
        self.target_x = None
        self.target_y = None

        # Setup Matplotlib in Interactive Mode
        plt.ion()
        self.fig, self.ax = plt.subplots()
        
        # Plot elements
        self.path_line, = self.ax.plot([], [], 'b-', label='Robot Path')
        self.robot_marker, = self.ax.plot([], [], 'ro', markersize=8, label='Robot Position')
        self.target_marker, = self.ax.plot([], [], 'gX', markersize=12, label='Target')

        self.ax.set_title("Real-Time Robot Trajectory")
        self.ax.set_xlabel("X Position (meters)")
        self.ax.set_ylabel("Y Position (meters)")
        self.ax.grid(True)
        self.ax.legend()

        # Timer to refresh the screen at 10 FPS
        self.timer = self.create_timer(0.1, self.update_plot)

    def odom_callback(self, msg):
        # Save the new coordinates
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

    def goal_callback(self, msg):
        # Save the new target
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

    def update_plot(self):
        if not self.x_data: return

        # Update the path line
        self.path_line.set_xdata(self.x_data)
        self.path_line.set_ydata(self.y_data)

        # Update the red dot (current position is the last item in the list)
        self.robot_marker.set_xdata([self.x_data[-1]])
        self.robot_marker.set_ydata([self.y_data[-1]])

        # Update the green X if a target exists
        if self.target_x is not None:
            self.target_marker.set_xdata([self.target_x])
            self.target_marker.set_ydata([self.target_y])

        # Auto-scale the graph so the robot never drives off-screen
        self.ax.relim()
        self.ax.autoscale_view()

        # Redraw the canvas
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()