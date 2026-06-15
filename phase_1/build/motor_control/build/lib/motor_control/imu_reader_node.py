#!/usr/bin/env python3
"""
Read an MPU-6050 IMU and publish sensor_msgs/Imu on /imu/data_raw.

Only angular velocity is needed for Part 1 heading integration, but linear
acceleration is published too for debugging.
"""

import math
import time
from contextlib import contextmanager
import fcntl

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

try:
    import smbus2
except ImportError:
    smbus2 = None


class MPU6050:
    GRAVITY_MS2 = 9.80665

    PWR_MGMT_1 = 0x6B
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    MPU_CONFIG = 0x1A

    ACCEL_XOUT0 = 0x3B

    def __init__(self, address: int, bus: int):
        if smbus2 is None:
            raise RuntimeError('Python package smbus2 is not installed')

        self.address = address
        self.bus = smbus2.SMBus(bus)
        self.lock_file = open(f'/tmp/evc_i2c_{bus}.lock', 'a+')

        with self.locked_bus():
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
            self.bus.write_byte_data(self.address, self.MPU_CONFIG, 0x03)

    @contextmanager
    def locked_bus(self):
        # Share one I2C bus politely across separate ROS processes.
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)

    @staticmethod
    def to_signed_word(high: int, low: int) -> int:
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        return value

    def read_sensor_data(self):
        with self.locked_bus():
            data = self.bus.read_i2c_block_data(
                self.address,
                self.ACCEL_XOUT0,
                14,
            )

        accel_scale = 16384.0
        gyro_scale = 131.0
        accel = {
            'x': (
                self.to_signed_word(data[0], data[1])
                / accel_scale * self.GRAVITY_MS2
            ),
            'y': (
                self.to_signed_word(data[2], data[3])
                / accel_scale * self.GRAVITY_MS2
            ),
            'z': (
                self.to_signed_word(data[4], data[5])
                / accel_scale * self.GRAVITY_MS2
            ),
        }
        gyro = {
            'x': self.to_signed_word(data[8], data[9]) / gyro_scale,
            'y': self.to_signed_word(data[10], data[11]) / gyro_scale,
            'z': self.to_signed_word(data[12], data[13]) / gyro_scale,
        }
        return accel, gyro

    def get_accel_data(self):
        accel, _ = self.read_sensor_data()
        return accel

    def get_gyro_data_dps(self):
        _, gyro = self.read_sensor_data()
        return gyro

    def close(self):
        self.bus.close()
        self.lock_file.close()


class IMUReaderNode(Node):
    def __init__(self):
        super().__init__('imu_reader_node')

        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x68)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('calibrate_on_start', True)
        self.declare_parameter('calibration_samples', 100)
        self.declare_parameter('log_xyz', True)
        self.declare_parameter('goal_reached_topic', '/goal_reached')
        self.declare_parameter('stop_logging_after_goal', True)
        self.declare_parameter('xyz_log_period_sec', 1.0)
        self.declare_parameter('error_log_period_sec', 1.0)

        imu_topic = str(self.get_parameter('imu_topic').value)
        goal_reached_topic = str(self.get_parameter('goal_reached_topic').value)
        i2c_bus = int(self.get_parameter('i2c_bus').value)
        device_address = int(self.get_parameter('device_address').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        calibrate_on_start = bool(
            self.get_parameter('calibrate_on_start').value
        )
        calibration_samples = int(
            self.get_parameter('calibration_samples').value
        )
        self.log_xyz = bool(self.get_parameter('log_xyz').value)
        self.stop_logging_after_goal = self.parameter_as_bool(
            self.get_parameter('stop_logging_after_goal').value
        )
        self.xyz_log_period_sec = max(
            float(self.get_parameter('xyz_log_period_sec').value),
            0.1,
        )
        self.error_log_period_sec = max(
            float(self.get_parameter('error_log_period_sec').value),
            0.1,
        )

        self.sensor = None
        self.goal_reached = False
        self.gyro_offset_dps = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_xyz_log_time = None
        self.last_error_log_time = None
        self.pub = self.create_publisher(Imu, imu_topic, 10)
        self.create_subscription(
            Bool,
            goal_reached_topic,
            self.goal_reached_callback,
            10,
        )

        try:
            self.sensor = MPU6050(device_address, i2c_bus)
        except Exception as exc:
            self.get_logger().error(f'Failed to initialize IMU: {exc}')
            self.get_logger().warn(
                'Odometry will fall back to encoder heading until IMU data arrives.'
            )

        if self.sensor is not None and calibrate_on_start:
            self.calibrate_gyro(max(calibration_samples, 1))

        period = 1.0 / max(publish_rate_hz, 1.0)
        self.create_timer(period, self.publish_data)
        self.get_logger().info(
            f'IMU reader ready | topic={imu_topic} | rate={1.0 / period:.1f} Hz'
        )

    def calibrate_gyro(self, samples: int):
        self.get_logger().info(
            f'Calibrating gyro with {samples} samples. Keep the robot still.'
        )

        sums = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        valid_samples = 0

        for _ in range(samples):
            try:
                _, gyro = self.sensor.read_sensor_data()
            except Exception as exc:
                self.warn_throttled(f'Gyro calibration read failed: {exc}')
                continue

            sums['x'] += gyro['x']
            sums['y'] += gyro['y']
            sums['z'] += gyro['z']
            valid_samples += 1
            time.sleep(0.01)

        if valid_samples == 0:
            self.get_logger().warn('Gyro calibration failed; using zero offsets.')
            return

        self.gyro_offset_dps = {
            axis: sums[axis] / valid_samples
            for axis in ('x', 'y', 'z')
        }
        self.get_logger().info(
            'Gyro offsets dps | '
            f"x={self.gyro_offset_dps['x']:.3f}, "
            f"y={self.gyro_offset_dps['y']:.3f}, "
            f"z={self.gyro_offset_dps['z']:.3f}"
        )

    def publish_data(self):
        if self.sensor is None:
            return

        try:
            accel, gyro = self.sensor.read_sensor_data()
        except Exception as exc:
            self.warn_throttled(f'IMU read failed: {exc}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        msg.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]

        msg.angular_velocity.x = math.radians(
            gyro['x'] - self.gyro_offset_dps['x']
        )
        msg.angular_velocity.y = math.radians(
            gyro['y'] - self.gyro_offset_dps['y']
        )
        msg.angular_velocity.z = math.radians(
            gyro['z'] - self.gyro_offset_dps['z']
        )
        msg.angular_velocity_covariance = [0.0] * 9

        msg.linear_acceleration.x = accel['x']
        msg.linear_acceleration.y = accel['y']
        msg.linear_acceleration.z = accel['z']
        msg.linear_acceleration_covariance = [0.0] * 9

        self.pub.publish(msg)
        self.maybe_log_xyz(accel, gyro)

    def warn_throttled(self, message: str):
        if self.goal_reached and self.stop_logging_after_goal:
            return

        now = self.get_clock().now()
        if self.last_error_log_time is not None:
            elapsed = (now - self.last_error_log_time).nanoseconds / 1e9
            if elapsed < self.error_log_period_sec:
                return

        self.last_error_log_time = now
        self.get_logger().warn(message)

    def maybe_log_xyz(self, accel, gyro):
        if not self.log_xyz:
            return

        now = self.get_clock().now()
        if self.last_xyz_log_time is not None:
            elapsed = (now - self.last_xyz_log_time).nanoseconds / 1e9
            if elapsed < self.xyz_log_period_sec:
                return

        self.last_xyz_log_time = now
        self.get_logger().info(
            'IMU xyz | '
            f"accel x={accel['x']:.2f}, y={accel['y']:.2f}, z={accel['z']:.2f} m/s^2 | "
            f"gyro x={gyro['x'] - self.gyro_offset_dps['x']:.2f}, "
            f"y={gyro['y'] - self.gyro_offset_dps['y']:.2f}, "
            f"z={gyro['z'] - self.gyro_offset_dps['z']:.2f} deg/s"
        )

    def goal_reached_callback(self, msg: Bool):
        if not msg.data or self.goal_reached:
            return

        self.goal_reached = True
        if self.stop_logging_after_goal:
            self.log_xyz = False

    @staticmethod
    def parameter_as_bool(value):
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = IMUReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sensor is not None:
            node.sensor.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
