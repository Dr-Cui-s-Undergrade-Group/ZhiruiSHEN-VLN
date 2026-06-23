from __future__ import annotations

import math
from collections import deque
from typing import Deque, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _rotation_matrix(transform: TransformStamped) -> np.ndarray:
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _translation_vector(transform: TransformStamped) -> np.ndarray:
    t = transform.transform.translation
    return np.array([t.x, t.y, t.z], dtype=np.float32)


class RollingScanAccumulator(Node):
    """Accumulate sparse Isaac PointCloud2 frames into a short-horizon LaserScan."""

    def __init__(self) -> None:
        super().__init__("node8_scan_accumulator")
        self.declare_parameter("cloud_topic", "/front_3d_lidar/lidar_points")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("fixed_frame", "odom")
        self.declare_parameter("accumulation_time_sec", 1.0)
        self.declare_parameter("publish_frequency_hz", 5.0)
        self.declare_parameter("min_height", -0.5)
        self.declare_parameter("max_height", 0.2)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", 0.0087)
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 30.0)
        self.declare_parameter("transform_timeout_sec", 0.2)
        self.declare_parameter("max_points_per_cloud", 20000)

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.accumulation_time_sec = float(self.get_parameter("accumulation_time_sec").value)
        publish_frequency_hz = float(self.get_parameter("publish_frequency_hz").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.transform_timeout = float(self.get_parameter("transform_timeout_sec").value)
        self.max_points_per_cloud = int(self.get_parameter("max_points_per_cloud").value)

        self.scan_size = int(math.floor((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        if self.scan_size <= 0:
            raise ValueError("angle_max must be greater than angle_min")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.clouds: Deque[Tuple[float, np.ndarray]] = deque()

        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, cloud_qos)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, scan_qos)
        self.create_timer(1.0 / publish_frequency_hz, self._publish_scan)
        self.get_logger().info(
            "Rolling scan accumulator: "
            f"{self.cloud_topic} -> {self.scan_topic}, fixed_frame={self.fixed_frame}, "
            f"target_frame={self.target_frame}, "
            f"window={self.accumulation_time_sec:.2f}s"
        )

    def _lookup_transform(self, msg: PointCloud2) -> TransformStamped | None:
        return self._lookup_transform_between(self.fixed_frame, msg.header.frame_id, msg.header.stamp)

    def _lookup_transform_between(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time | None = None,
    ) -> TransformStamped | None:
        lookup_time = rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time,
                timeout=Duration(seconds=self.transform_timeout),
            )
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.transform_timeout),
                )
            except TransformException as exc:
                self.get_logger().warn(f"Skipping transform, no TF {source_frame}->{target_frame}: {exc}")
                return None

    def _on_cloud(self, msg: PointCloud2) -> None:
        transform = self._lookup_transform(msg)
        if transform is None:
            return
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append((float(point[0]), float(point[1]), float(point[2])))
            if len(points) >= self.max_points_per_cloud:
                break
        if not points:
            return

        xyz = np.asarray(points, dtype=np.float32)
        xyz = xyz @ _rotation_matrix(transform).T + _translation_vector(transform)

        stamp_sec = _stamp_to_sec(msg.header.stamp)
        self.clouds.append((stamp_sec, xyz.copy()))
        self._prune(stamp_sec)

    def _prune(self, now_sec: float) -> None:
        cutoff = now_sec - self.accumulation_time_sec
        while self.clouds and self.clouds[0][0] < cutoff:
            self.clouds.popleft()

    def _publish_scan(self) -> None:
        if not self.clouds:
            return
        now_msg = self.get_clock().now().to_msg()
        now_sec = _stamp_to_sec(now_msg)
        self._prune(now_sec)
        if not self.clouds:
            return

        transform = self._lookup_transform_between(self.target_frame, self.fixed_frame)
        if transform is None:
            return

        ranges = np.full(self.scan_size, math.inf, dtype=np.float32)
        xyz = np.concatenate([cloud for _, cloud in self.clouds], axis=0)
        xyz = xyz @ _rotation_matrix(transform).T + _translation_vector(transform)
        height_mask = (xyz[:, 2] >= self.min_height) & (xyz[:, 2] <= self.max_height)
        xyz = xyz[height_mask]
        if xyz.size == 0:
            return
        xy = xyz[:, :2]
        distances = np.linalg.norm(xy, axis=1)
        angles = np.arctan2(xy[:, 1], xy[:, 0])
        mask = (
            (distances >= self.range_min)
            & (distances <= self.range_max)
            & (angles >= self.angle_min)
            & (angles <= self.angle_max)
        )
        if np.any(mask):
            indices = np.floor((angles[mask] - self.angle_min) / self.angle_increment).astype(np.int32)
            indices = np.clip(indices, 0, self.scan_size - 1)
            np.minimum.at(ranges, indices, distances[mask].astype(np.float32))

        msg = LaserScan()
        msg.header.stamp = now_msg
        msg.header.frame_id = self.target_frame
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / float(self.get_parameter("publish_frequency_hz").value)
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges.tolist()
        self.scan_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RollingScanAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
