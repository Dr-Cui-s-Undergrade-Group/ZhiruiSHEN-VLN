from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudToMapCloud(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_to_map_cloud")
        self.declare_parameter("cloud_topic", "/front_3d_lidar/lidar_points")
        self.declare_parameter("output_topic", "/front_3d_lidar/map_points")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("min_period_sec", 0.15)
        self.declare_parameter("transform_timeout_sec", 0.05)

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.min_period_sec = max(0.0, float(self.get_parameter("min_period_sec").value))
        self.transform_timeout_sec = float(self.get_parameter("transform_timeout_sec").value)
        self.last_publish_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f"Publishing full lidar cloud {self.cloud_topic} -> {self.output_topic} in {self.target_frame}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        now = self.get_clock().now()
        if self.last_publish_time is not None:
            if (now - self.last_publish_time).nanoseconds * 1e-9 < self.min_period_sec:
                return

        source_frame = msg.header.frame_id
        if not source_frame:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().debug(f"Waiting for {self.target_frame} <- {source_frame} TF: {exc}")
            return

        cloud = do_transform_cloud(msg, transform)
        cloud.header.stamp = now.to_msg()
        cloud.header.frame_id = self.target_frame
        self.publisher.publish(cloud)
        self.last_publish_time = now


def main() -> None:
    rclpy.init()
    node = PointCloudToMapCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
