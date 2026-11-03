#!/usr/bin/env python3
"""シミュレーションの位置推定が安定してから、機体の初期位置と向きを保存する。

清掃走行のBehavior Treeは、この保存処理が完了してから走行を開始する。
ほかの走行目標や手動操作も、保存が完了するまで待つ必要がある。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class HomePoseSaver(Node):
    """機体が静止し、地図上の位置が安定した時点の姿勢を一度だけ保存する。後から接続した購読側にも配信する。"""

    MAX_TF_AGE_SEC = 0.5
    FUTURE_TF_TOLERANCE_SEC = 0.5  # Allow localization TF publication tolerance.
    MAX_SPAWN_DRIFT_METERS = 0.05
    MAX_SPAWN_DRIFT_RADIANS = 0.05
    STABLE_POSE_DRIFT_METERS = 0.02
    STABLE_POSE_DRIFT_RADIANS = 0.02
    STABLE_FOR_SEC = 2.0

    def __init__(self) -> None:
        super().__init__('home_pose_saver')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('home_pose_topic', '/simulation/home_pose')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('check_period_sec', 0.1)

        self._global_frame = str(self.get_parameter('global_frame').value).strip('/')
        self._robot_frame = str(self.get_parameter('robot_frame').value).strip('/')
        self._odom_frame = str(self.get_parameter('odom_frame').value).strip('/')
        topic = str(self.get_parameter('home_pose_topic').value)
        publish_period = float(self.get_parameter('publish_period_sec').value)
        check_period = float(self.get_parameter('check_period_sec').value)
        if not self._global_frame or not self._robot_frame or not self._odom_frame or not topic:
            raise ValueError('frame names and home_pose_topic must not be empty')
        if not math.isfinite(publish_period) or publish_period <= 0.0:
            raise ValueError('publish_period_sec must be positive and finite')
        if not math.isfinite(check_period) or check_period <= 0.0:
            raise ValueError('check_period_sec must be positive and finite')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(PoseStamped, topic, qos)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._home_pose: PoseStamped | None = None
        self._odom_anchor: tuple[float, float, float] | None = None
        self._stable_pose: tuple[float, float, float] | None = None
        self._stable_since_ns: int | None = None
        self._failed = False
        self._check_timer = self.create_timer(check_period, self._check_pose)
        self._publish_timer = self.create_timer(publish_period, self._publish_home_pose)
        self.get_logger().info(
            f'Waiting for spawn anchor TF {self._odom_frame} -> {self._robot_frame} '
            f'and stable localization TF {self._global_frame} -> {self._odom_frame}; '
            f'home pose topic is {topic}. Wait for capture before sending external motion commands.')

    def _lookup_fresh(self, target: str, source: str, now_ns: int):
        """コールバック内で待機せずに、現在取得できる座標変換を返す。"""
        try:
            transform = self._tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None

        stamp_ns = (
            transform.header.stamp.sec * 1_000_000_000 +
            transform.header.stamp.nanosec)
        age_ns = now_ns - stamp_ns
        if (stamp_ns <= 0 or
                age_ns > self.MAX_TF_AGE_SEC * 1_000_000_000 or
                age_ns < -self.FUTURE_TF_TOLERANCE_SEC * 1_000_000_000):
            return None
        if transform.header.frame_id.lstrip('/') != target or \
                transform.child_frame_id.lstrip('/') != source:
            return None
        if not self._valid_transform(transform):
            return None
        return transform

    @staticmethod
    def _valid_transform(transform) -> bool:
        t = transform.transform.translation
        q = transform.transform.rotation
        values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        return all(math.isfinite(value) for value in values) and abs(norm - 1.0) < 1e-3

    @staticmethod
    def _yaw(rotation) -> float:
        sin_yaw = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cos_yaw = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _angle_distance(a: float, b: float) -> float:
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    @staticmethod
    def _planar_pose(transform) -> tuple[float, float, float]:
        translation = transform.transform.translation
        return (translation.x, translation.y, HomePoseSaver._yaw(transform.transform.rotation))

    @classmethod
    def _pose_distance(cls, a: tuple[float, float, float],
                       b: tuple[float, float, float]) -> tuple[float, float]:
        return (math.hypot(a[0] - b[0], a[1] - b[1]),
                cls._angle_distance(a[2], b[2]))

    def _fail_closed(self, reason: str) -> None:
        self._failed = True
        self.get_logger().error(
            f'Home pose capture failed closed: {reason}; no pose will be published')

    def _reset_stability(self) -> None:
        self._stable_pose = None
        self._stable_since_ns = None

    def _check_pose(self) -> None:
        if self._failed or self._home_pose is not None:
            return

        now_ns = self.get_clock().now().nanoseconds
        # Simulation time must be running; freshness and the two-second hold
        # are measured on the same clock as Gazebo.
        if now_ns <= 0:
            self._reset_stability()
            return

        odom_base = self._lookup_fresh(self._odom_frame, self._robot_frame, now_ns)
        if odom_base is None:
            self._reset_stability()
            return
        current_odom_pose = self._planar_pose(odom_base)
        if self._odom_anchor is None:
            self._odom_anchor = current_odom_pose
            self.get_logger().info(
                f'Established spawn odometry anchor in {self._odom_frame}: '
                f'x={current_odom_pose[0]:.3f}, y={current_odom_pose[1]:.3f}, '
                f'yaw={current_odom_pose[2]:.3f}')
        else:
            drift_m, drift_rad = self._pose_distance(self._odom_anchor, current_odom_pose)
            if (drift_m > self.MAX_SPAWN_DRIFT_METERS or
                    drift_rad > self.MAX_SPAWN_DRIFT_RADIANS):
                self._fail_closed(
                    f'robot moved {drift_m:.3f}m / {drift_rad:.3f}rad from its initial '
                    f'odometry anchor before localization capture')
                return

        map_odom = self._lookup_fresh(self._global_frame, self._odom_frame, now_ns)
        map_base = self._lookup_fresh(self._global_frame, self._robot_frame, now_ns)
        if map_odom is None or map_base is None:
            self._reset_stability()
            return

        current_map_pose = self._planar_pose(map_base)
        if self._stable_pose is None or self._stable_since_ns is None:
            self._stable_pose = current_map_pose
            self._stable_since_ns = now_ns
            return
        if now_ns < self._stable_since_ns:
            self._stable_pose = current_map_pose
            self._stable_since_ns = now_ns
            return

        stable_drift_m, stable_drift_rad = self._pose_distance(
            self._stable_pose, current_map_pose)
        if (stable_drift_m > self.STABLE_POSE_DRIFT_METERS or
                stable_drift_rad > self.STABLE_POSE_DRIFT_RADIANS):
            self._stable_pose = current_map_pose
            self._stable_since_ns = now_ns
            return

        if now_ns - self._stable_since_ns < self.STABLE_FOR_SEC * 1_000_000_000:
            return

        pose = PoseStamped()
        pose.header = map_base.header
        pose.pose.position.x = map_base.transform.translation.x
        pose.pose.position.y = map_base.transform.translation.y
        pose.pose.position.z = map_base.transform.translation.z
        pose.pose.orientation = map_base.transform.rotation
        self._home_pose = pose
        self.get_logger().info(
            f'Captured stable stationary spawn pose in {self._global_frame}: '
            f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, '
            f'yaw={self._yaw(pose.pose.orientation):.3f}; '
            f'stationary and localized for {self.STABLE_FOR_SEC:.1f}s')

    def _publish_home_pose(self) -> None:
        if self._home_pose is None or self._failed:
            return
        # Re-publish the frozen value so the transient-local sample remains
        # available to BT subscribers that start later.
        self._publisher.publish(self._home_pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HomePoseSaver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
