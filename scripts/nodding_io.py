#!/usr/bin/env python3
"""Gazeboの首振り関節を動かし、計測した関節の状態をnodding_lidarへ渡す。"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64


class NoddingIO(Node):
    def __init__(self):
        super().__init__('nodding_io')
        self.declare_parameter('amplitude_deg', 10.0)
        self.declare_parameter('period_sec', 2.0)
        self._cmd = self.create_publisher(Float64, '/nodding_cmd', 10)
        self._servo = self.create_publisher(JointState, '/servo_state', 30)
        self._scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.02, self._command)

    def _command(self):
        stamp = self.get_clock().now()
        if stamp.nanoseconds == 0:
            return
        amplitude = math.radians(self.get_parameter('amplitude_deg').value)
        period = self.get_parameter('period_sec').value
        if period <= 0.0:
            return
        phase = 2.0 * math.pi * (stamp.nanoseconds * 1e-9 / period)
        msg = Float64()
        msg.data = -amplitude * math.cos(phase)
        self._cmd.publish(msg)

    def _on_joints(self, source):
        for index, name in enumerate(source.name):
            if name.split('::')[-1] != 'nodding_joint':
                continue
            if index >= len(source.position) or index >= len(source.velocity):
                return
            state = JointState()
            state.header = source.header
            state.name = ['nodding_joint']
            state.position = [source.position[index]]
            state.velocity = [source.velocity[index]]
            self._servo.publish(state)
            return

    def _on_scan(self, scan):
        # Gazebo samples the entire scan at one simulation instant. Its rays
        # have no individual measurement times, unlike the rotating real S1.
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        self._scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = NoddingIO()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
