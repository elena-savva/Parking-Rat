#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from simple_pubsub.imuDriver import mpu6050

class IMUReaderNode(Node):
    def __init__(self, node_name, freq_divider, ang_vel_offset, accel_offset, i2c_bus, device_address):
        super().__init__(node_name)
        
        self.initialized = False
        self.get_logger().info("Initializing IMU reader node...")
        
        self.DMP_freq = 40
        self.g = 9.80665
        self._freq_divider = freq_divider
        self._gyro_offset = ang_vel_offset
        self._accel_offset = accel_offset
        self._bus = i2c_bus
        self._address = device_address
        self._offset_init = False
        
        op_rate = float(self.DMP_freq) / float(self._freq_divider)
        self.get_logger().info("===============IMU Node Init Val===============")
        self.get_logger().info(f"Op Rate: {op_rate:.2f} Hz")
        self.get_logger().info(f"Acceleration Offset: X:{self._accel_offset[0]:.2f}, Y: {self._accel_offset[1]:.2f}, Z: {self._accel_offset[2]:.2f} m/s^2")
        self.get_logger().info(f"Gyro Offset X:{self._gyro_offset[0]:.2f}, Y: {self._gyro_offset[1]:.2f}, Z: {self._gyro_offset[2]:.2f} degrees/s")
        self.get_logger().info("===============END of IMU Init Val===============")

        self._sensor = self._find_sensor()
        if not self._sensor:
            self.get_logger().error("No MPU6050 device found")
            exit(1)
            
        self.get_logger().info("===============Performing Initial Testing!===============")
        accel_dmp = self._sensor.get_accel_data()
        self.get_logger().info(f"Acceleration: X: {accel_dmp['x']:.2f}, Y: {accel_dmp['y']:.2f}, Z: {accel_dmp['z']:.2f} m/s^2")
        gyro_data = self._sensor.get_gyro_data()
        self.get_logger().info(f"Gyro X: {gyro_data['x']:.2f}, Y: {gyro_data['y']:.2f}, Z: {gyro_data['z']:.2f} degrees/s")
        self.get_logger().info("===============IMU Initialization Complete===============")

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        if not self._offset_init:
            self.zero_sensor()
            self._offset_init = True

        if self._freq_divider > 0:
            timer_period = float(self._freq_divider) / float(self.DMP_freq)
            self.timer = self.create_timer(timer_period, self.publish_data)
        else:
            self.get_logger().warn("Frequency divider is zero or invalid. Defaulting to 1 Hz.")
            self.timer = self.create_timer(1.0, self.publish_data)
            
        self.get_logger().info("IMU reader node initialized!")

    def _find_sensor(self):
        conn = f"[bus:{self._bus}](0x{self._address:02X})"
        self.get_logger().info(f"Trying to open device on connector {conn}")

        try:
            sensor = mpu6050(self._address, bus=self._bus)
            self.get_logger().info(f"Device found on connector {conn}")
            return sensor
        except Exception as e:
            self.get_logger().warn(f"No devices found on connector {conn}, but the bus exists. Error: {e}")
            return None

    def publish_data(self):
        msg = Imu()

        try:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link' # Added frame_id to prevent TF tree errors

            acc_data_dmp = self._sensor.get_accel_data()
            gyro_data = self._sensor.get_gyro_data()

            msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0.0
            
            msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            msg.angular_velocity.x = gyro_data['x'] * 250 / 2**15 - self._gyro_offset[0]
            msg.angular_velocity.y = gyro_data['y'] * 250 / 2**15 - self._gyro_offset[1]
            msg.angular_velocity.z = gyro_data['z'] * 250 / 2**15 - self._gyro_offset[2]
            msg.angular_velocity_covariance = [0.0] * 9

            msg.linear_acceleration.x = acc_data_dmp['x'] * 2 * self.g / 2**15 - self._accel_offset[0]
            msg.linear_acceleration.y = acc_data_dmp['y'] * 2 * self.g / 2**15 - self._accel_offset[1]
            msg.linear_acceleration.z = acc_data_dmp['z'] * 2 * self.g / 2**15 - self._accel_offset[2]
            msg.linear_acceleration_covariance = [0.0] * 9

            self.pub.publish(msg)

        except Exception as IMUCommLoss:
            self.get_logger().warn(f"IMU Comm Loss: {IMUCommLoss}")

    def zero_sensor(self):
        self.get_logger().info("zero_sensor service called.")
        acc_data = self._sensor.get_accel_data()
        self._accel_offset = [acc_data['x'] * 2 * self.g / 2**15,
                              acc_data['y'] * 2 * self.g / 2**15,
                              acc_data['z'] * 2 * self.g / 2**15]

        gyro_data = self._sensor.get_gyro_data()
        self._gyro_offset = [gyro_data['x'] * 250 / 2**15,
                             gyro_data['y'] * 250 / 2**15,
                             gyro_data['z'] * 250 / 2**15]

        self.get_logger().info(f"IMU zeroed with ACC: X:{self._accel_offset[0]:.2f}, Y: {self._accel_offset[1]:.2f}, Z: {self._accel_offset[2]:.2f} m/s^2")
        self.get_logger().info(f"IMU zeroed with Gyro X:{self._gyro_offset[0]:.2f}, Y: {self._gyro_offset[1]:.2f}, Z: {self._gyro_offset[2]:.2f} degrees/s")
        return None

def main(args=None):
    rclpy.init(args=args)
    
    node = IMUReaderNode(
        node_name="imu_node", 
        freq_divider=8, 
        ang_vel_offset=[0, 0, 0],
        accel_offset=[0, 0, 0], 
        i2c_bus=1, 
        device_address=0x68
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
