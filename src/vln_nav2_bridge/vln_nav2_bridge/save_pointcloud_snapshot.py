from __future__ import annotations

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class SavePointCloudSnapshot(Node):
    def __init__(self) -> None:
        super().__init__("save_pointcloud_snapshot")
        self.declare_parameter("cloud_topic", "/front_3d_lidar/lidar_points")
        self.declare_parameter("output_path", "data/pointcloud_snapshot.xyz")
        self.declare_parameter("frames", 1)
        self.declare_parameter("max_points_per_frame", 0)

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        self.frames = max(1, int(self.get_parameter("frames").value))
        self.max_points_per_frame = max(0, int(self.get_parameter("max_points_per_frame").value))
        self.received = 0
        self.points: list[tuple[float, float, float]] = []

        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f"Saving {self.frames} frame(s) from {self.cloud_topic} to {self.output_path}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        frame_points = 0
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            self.points.append((x, y, z))
            frame_points += 1
            if self.max_points_per_frame > 0 and frame_points >= self.max_points_per_frame:
                break

        self.received += 1
        self.get_logger().info(
            f"Captured frame {self.received}/{self.frames}: {frame_points} points, total {len(self.points)}"
        )
        if self.received >= self.frames:
            self._write_xyz()
            rclpy.shutdown()

    def _write_xyz(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="utf-8") as f:
            for x, y, z in self.points:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
        self.get_logger().info(f"Wrote {len(self.points)} points to {self.output_path}")


def main() -> None:
    rclpy.init()
    node = SavePointCloudSnapshot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
