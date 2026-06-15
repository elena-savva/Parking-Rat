#!/usr/bin/env python3
"""
Read a VL53L0X time-of-flight sensor and publish front range.
"""

import math
import time
from contextlib import contextmanager
import fcntl

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

try:
    import smbus2
except ImportError:
    smbus2 = None


class VL53L0X:
    def __init__(self, bus=1, address=0x29, sample_time=0.02):
        if smbus2 is None:
            raise RuntimeError('Python package smbus2 is not installed')

        self.address = address
        self.sample_time = sample_time
        self.bus = smbus2.SMBus(bus)
        self.lock_file = open(f'/tmp/evc_i2c_{bus}.lock', 'a+')
        self.init_sensor()

    @contextmanager
    def locked_bus(self):
        # Share one I2C bus politely across separate ROS processes.
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)

    def init_sensor(self):
        with self.locked_bus():
            self.write_byte(0x88, 0x00)
            self.write_byte(0x80, 0x01)
            self.write_byte(0xFF, 0x01)
            self.write_byte(0x00, 0x00)
            self.stop_variable = self.read_byte(0x91)
            self.write_byte(0x00, 0x01)
            self.write_byte(0xFF, 0x00)
            self.write_byte(0x80, 0x00)

            self.write_byte(0x60, 0x00)
            self.write_byte(0x01, 0xFF)
            self.write_byte(0x02, 0x00)
            self.write_byte(0x16, 0x00)
            self.write_byte(0x17, 0x00)
            self.write_byte(0x31, 0x04)
            self.write_byte(0x40, 0x83)
            self.write_byte(0xFF, 0x01)
            self.write_byte(0x00, 0x00)
            self.write_byte(0x91, self.stop_variable)
            self.write_byte(0x00, 0x01)
            self.write_byte(0xFF, 0x00)
            self.write_byte(0x80, 0x00)
            self.write_byte(0x00, 0x04)

    def read_distance_mm(self):
        with self.locked_bus():
            self.write_byte(0x00, 0x01)
            time.sleep(self.sample_time)
            data = self.read_bytes(0x14, 12)
        return (data[10] << 8) | data[11]

    def write_byte(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    def read_byte(self, register):
        return self.bus.read_byte_data(self.address, register)

    def read_bytes(self, register, length):
        return self.bus.read_i2c_block_data(self.address, register, length)

    def close(self):
        self.bus.close()
        self.lock_file.close()


class TOFReaderNode(Node):
    def __init__(self):
        super().__init__('tof_reader_node')

        self.declare_parameter('tof_topic', '/tof/front')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x29)
        self.declare_parameter('publish_rate_hz', 8.0)
        self.declare_parameter('measurement_delay_sec', 0.02)
        self.declare_parameter('min_range_m', 0.03)
        self.declare_parameter('max_range_m', 25.0)
        self.declare_parameter('log_distance', True)
        self.declare_parameter('goal_reached_topic', '/goal_reached')
        self.declare_parameter('stop_logging_after_goal', True)
        self.declare_parameter('distance_log_period_sec', 1.0)
        self.declare_parameter('error_log_period_sec', 1.0)

        self.tof_topic = str(self.get_parameter('tof_topic').value)
        goal_reached_topic = str(self.get_parameter('goal_reached_topic').value)
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.log_distance = self.parameter_as_bool(
            self.get_parameter('log_distance').value
        )
        self.stop_logging_after_goal = self.parameter_as_bool(
            self.get_parameter('stop_logging_after_goal').value
        )
        self.distance_log_period_sec = max(
            float(self.get_parameter('distance_log_period_sec').value),
            0.1,
        )
        self.error_log_period_sec = max(
            float(self.get_parameter('error_log_period_sec').value),
            0.1,
        )
        i2c_bus = int(self.get_parameter('i2c_bus').value)
        device_address = int(self.get_parameter('device_address').value)
        measurement_delay_sec = max(
            float(self.get_parameter('measurement_delay_sec').value),
            0.005,
        )
        publish_rate_hz = max(
            float(self.get_parameter('publish_rate_hz').value),
            1.0,
        )

        self.sensor = None
        self.goal_reached = False
        self.last_log_time = None
        self.last_error_log_time = None
        self.pub = self.create_publisher(Range, self.tof_topic, 10)
        self.create_subscription(
            Bool,
            goal_reached_topic,
            self.goal_reached_callback,
            10,
        )

        try:
            self.sensor = VL53L0X(
                bus=i2c_bus,
                address=device_address,
                sample_time=measurement_delay_sec,
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to initialize ToF sensor: {exc}')

        self.create_timer(1.0 / publish_rate_hz, self.publish_range)
        self.get_logger().info(
            f'ToF reader ready | topic={self.tof_topic} | '
            f'rate={publish_rate_hz:.1f} Hz'
        )

    def publish_range(self):
        if self.sensor is None:
            return

        try:
            distance_mm = self.sensor.read_distance_mm()
        except Exception as exc:
            self.warn_throttled(f'ToF read failed: {exc}')
            return

        distance_m = distance_mm / 1000.0
        if distance_m <= 0.0 or distance_m > self.max_range_m:
            distance_m = math.inf

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tof_front'
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = math.radians(25.0)
        msg.min_range = self.min_range_m
        msg.max_range = self.max_range_m
        msg.range = distance_m

        self.pub.publish(msg)
        self.maybe_log_distance(distance_m)

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

    def maybe_log_distance(self, distance_m):
        if not self.log_distance:
            return

        now = self.get_clock().now()
        if self.last_log_time is not None:
            elapsed = (now - self.last_log_time).nanoseconds / 1e9
            if elapsed < self.distance_log_period_sec:
                return

        self.last_log_time = now
        if math.isfinite(distance_m):
            self.get_logger().info(f'ToF front distance: {distance_m:.2f} m')
        else:
            self.get_logger().info('ToF front distance: out of range')

    def goal_reached_callback(self, msg: Bool):
        if not msg.data or self.goal_reached:
            return

        self.goal_reached = True
        if self.stop_logging_after_goal:
            self.log_distance = False

    def destroy_node(self):
        if self.sensor is not None:
            self.sensor.close()
        super().destroy_node()

    @staticmethod
    def parameter_as_bool(value):
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = TOFReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
