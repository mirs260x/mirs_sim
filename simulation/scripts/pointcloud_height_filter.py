#!/usr/bin/env python3
"""推定した地面から、設定した高さ以上にある点だけを取り出す。"""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudHeightFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_height_filter')
        self.declare_parameter('input_topic', '/points')
        self.declare_parameter('output_topic', '/points_icp_filtered')
        self.declare_parameter('min_height_above_ground', 0.5)
        self.declare_parameter('base_link_height_above_ground', 0.242)

        min_height = float(self.get_parameter('min_height_above_ground').value)
        base_height = float(
            self.get_parameter('base_link_height_above_ground').value)
        self._min_z = min_height - base_height

        self._publisher = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self._callback, 10)
        self.get_logger().info(
            f'Keeping points with base_link z >= {self._min_z:.3f} m '
            f'({min_height:.2f} m above the ground estimate)')

    def _callback(self, cloud):
        if cloud.header.frame_id != 'base_link':
            self.get_logger().error(
                f'Expected base_link cloud, got {cloud.header.frame_id!r}')
            return
        x_field = next((field for field in cloud.fields if field.name == 'x'), None)
        y_field = next((field for field in cloud.fields if field.name == 'y'), None)
        z_field = next((field for field in cloud.fields if field.name == 'z'), None)
        if (x_field is None or y_field is None or z_field is None or
                x_field.datatype != PointField.FLOAT32 or
                y_field.datatype != PointField.FLOAT32 or
                z_field.datatype != PointField.FLOAT32):
            self.get_logger().error('Input cloud must contain float32 x, y, z fields')
            return

        endian = '>' if cloud.is_bigendian else '<'
        xyz_format = struct.Struct(endian + 'f')
        filtered = bytearray()
        data = memoryview(cloud.data)
        for row in range(cloud.height):
            row_start = row * cloud.row_step
            for column in range(cloud.width):
                point_start = row_start + column * cloud.point_step
                x = xyz_format.unpack_from(data, point_start + x_field.offset)[0]
                y = xyz_format.unpack_from(data, point_start + y_field.offset)[0]
                z = xyz_format.unpack_from(data, point_start + z_field.offset)[0]
                if (math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and
                        z >= self._min_z):
                    filtered.extend(data[point_start:point_start + cloud.point_step])

        # Some servo angles see no structure above the height threshold.
        # Do not advance the ICP assembler with an empty scan.
        if not filtered:
            return

        output = PointCloud2()
        output.header = cloud.header
        output.height = 1
        output.width = len(filtered) // cloud.point_step
        output.fields = cloud.fields
        output.is_bigendian = cloud.is_bigendian
        output.point_step = cloud.point_step
        output.row_step = output.width * output.point_step
        output.data = bytes(filtered)
        output.is_dense = True
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudHeightFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
