#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUPrinterNode(Node):
    def __init__(self, node_name="imu_printer_node"):
        super().__init__(node_name)
        
        self.initialized = False
        self.get_logger().info("Initializing IMU printer node...")

        self.sub_imu = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_cb,
            10
        )

        self.first_imu_received = False
        self.latest_imu_data = None
        self.initialized = True
        self.get_logger().info("IMU printer node initialized!")

    def imu_cb(self, data):
        if not self.initialized:
            return

        if not self.first_imu_received:
            self.first_imu_received = True
            self.get_logger().info("IMU printer node captured first IMU measurement from publisher.")
            
        self.get_logger().info(
            f"IMU Data - Orientation: ({data.orientation.x:.2f}, {data.orientation.y:.2f}, {data.orientation.z:.2f}), "
            f"Angular Velocity: ({data.angular_velocity.x:.2f}, {data.angular_velocity.y:.2f}, {data.angular_velocity.z:.2f}), "
            f"Linear Acceleration: ({data.linear_acceleration.x:.2f}, {data.linear_acceleration.y:.2f}, {data.linear_acceleration.z:.2f})"
        )
        
        self.latest_imu_data = data

def main(args=None):
    rclpy.init(args=args)
    
    node = IMUPrinterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()