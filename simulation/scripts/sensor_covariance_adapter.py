#!/usr/bin/env python3
"""Set realistic measurement covariances on Gazebo wheel odometry and IMU data."""

from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class SensorCovarianceAdapter(Node):
    def __init__(self):
        super().__init__('sensor_covariance_adapter')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('wheel_odom_output', '/wheel/odometry')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('imu_output', '/imu/data_fusion')
        self.declare_parameter('wheel_linear_velocity_variance', 0.04)
        self.declare_parameter('wheel_angular_velocity_variance', 0.09)
        self.declare_parameter('imu_yaw_rate_variance', 0.01)

        self._wheel_linear_variance = float(
            self.get_parameter('wheel_linear_velocity_variance').value)
        self._wheel_angular_variance = float(
            self.get_parameter('wheel_angular_velocity_variance').value)
        self._imu_yaw_rate_variance = float(
            self.get_parameter('imu_yaw_rate_variance').value)

        self._wheel_pub = self.create_publisher(
            Odometry, self.get_parameter('wheel_odom_output').value, 10)
        self._imu_pub = self.create_publisher(
            Imu, self.get_parameter('imu_output').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('wheel_odom_topic').value,
            self._wheel_callback, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._imu_callback, 10)

    def _wheel_callback(self, message):
        output = deepcopy(message)
        # robot_localization fuses vx and wz only from this source.
        output.twist.covariance[0] = self._wheel_linear_variance
        output.twist.covariance[35] = self._wheel_angular_variance
        self._wheel_pub.publish(output)

    def _imu_callback(self, message):
        output = deepcopy(message)
        output.angular_velocity_covariance[8] = self._imu_yaw_rate_variance
        self._imu_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SensorCovarianceAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
