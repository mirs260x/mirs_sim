#!/usr/bin/env python3
"""Gazeboの車輪オドメトリとIMUに、計測のばらつきを表す共分散を設定する。"""

from copy import deepcopy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class SensorCovarianceAdapter(Node):
    """Gazeboの観測値に正の分散を付け、EKF用の車輪・IMUトピックへ配信する。

    速度・加速度・姿勢の値と計測時刻は保持する。分散は起動時に読み取り、
    有限かつ正でない場合はValueErrorとする。無効な観測は警告して配信しない。
    """

    def __init__(self):
        """共分散設定を検証し、車輪・IMUメッセージの購読と配信を開始する。"""
        super().__init__('sensor_covariance_adapter')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('wheel_odom_output', '/wheel/odometry')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('imu_output', '/imu/data_fusion')
        self.declare_parameter('wheel_linear_velocity_variance', 0.04)
        self.declare_parameter('wheel_angular_velocity_variance', 0.09)
        self.declare_parameter('imu_yaw_rate_variance', 0.01)
        self.declare_parameter('wheel_lateral_velocity_variance', 0.01)
        self.declare_parameter('wheel_vertical_velocity_variance', 0.01)
        self.declare_parameter('imu_orientation_variance', 0.01)
        self.declare_parameter('imu_angular_velocity_variance', 0.01)
        self.declare_parameter('imu_linear_acceleration_variance', 0.25)

        self._wheel_linear_variance = float(
            self.get_parameter('wheel_linear_velocity_variance').value)
        self._wheel_angular_variance = float(
            self.get_parameter('wheel_angular_velocity_variance').value)
        self._imu_yaw_rate_variance = float(
            self.get_parameter('imu_yaw_rate_variance').value)

        self._wheel_lateral_variance = self._positive_variance('wheel_lateral_velocity_variance')
        self._wheel_vertical_variance = self._positive_variance('wheel_vertical_velocity_variance')
        self._orientation_variance = self._positive_variance('imu_orientation_variance')
        self._angular_variance = self._positive_variance('imu_angular_velocity_variance')
        self._acceleration_variance = self._positive_variance('imu_linear_acceleration_variance')
        for name in ('wheel_linear_velocity_variance', 'wheel_angular_velocity_variance',
                     'imu_yaw_rate_variance'):
            self._positive_variance(name)

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

    def _positive_variance(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _wheel_callback(self, message):
        v = message.twist.twist
        if not all(math.isfinite(x) for x in (v.linear.x, v.linear.y, v.linear.z, v.angular.z)):
            self.get_logger().warning('Skipping non-finite wheel velocity', throttle_duration_sec=5.0)
            return
        output = deepcopy(message)
        # 機体座標の前進・横・上下速度[m/s]とヨー角速度[rad/s]を使う。
        output.twist.covariance[0] = self._wheel_linear_variance
        output.twist.covariance[7] = self._wheel_lateral_variance
        output.twist.covariance[14] = self._wheel_vertical_variance
        output.twist.covariance[35] = self._wheel_angular_variance
        self._wheel_pub.publish(output)

    def _imu_callback(self, message):
        q, w, a = message.orientation, message.angular_velocity, message.linear_acceleration
        values = (q.x, q.y, q.z, q.w, w.x, w.y, w.z, a.x, a.y, a.z)
        if (not all(math.isfinite(x) for x in values) or
                abs(sum(x * x for x in (q.x, q.y, q.z, q.w)) - 1.0) > 0.01 or
                any(c[0] == -1.0 for c in (message.orientation_covariance,
                    message.angular_velocity_covariance, message.linear_acceleration_covariance))):
            self.get_logger().warning('Skipping unavailable or invalid IMU observation',
                                      throttle_duration_sec=5.0)
            return
        output = deepcopy(message)
        # Gazeboは共分散を0で出すため、独立な各軸の観測分散を明示する。
        output.orientation_covariance = [self._orientation_variance, 0.0, 0.0,
                                         0.0, self._orientation_variance, 0.0,
                                         0.0, 0.0, self._orientation_variance]
        output.angular_velocity_covariance = [self._angular_variance, 0.0, 0.0,
                                              0.0, self._angular_variance, 0.0,
                                              0.0, 0.0, self._imu_yaw_rate_variance]
        output.linear_acceleration_covariance = [self._acceleration_variance, 0.0, 0.0,
                                                 0.0, self._acceleration_variance, 0.0,
                                                 0.0, 0.0, self._acceleration_variance]
        self._imu_pub.publish(output)


def main(args=None):
    """ROS 2引数を受け取り、共分散を付けた観測の配信を終了要求まで実行する。"""
    rclpy.init(args=args)
    node = SensorCovarianceAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
