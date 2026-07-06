#!/usr/bin/env python3
from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudToMapCloud(Node):
    def __init__(self, cloud_topic: str, output_topic: str, target_frame: str, min_period_sec: float) -> None:
        super().__init__("pointcloud_to_map_cloud")
        self.target_frame = target_frame
        self.min_period_sec = max(0.0, min_period_sec)
        self.last_publish_time = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, output_topic, 5)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg: PointCloud2) -> None:
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
                timeout=rclpy.duration.Duration(seconds=0.05),
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
    parser = argparse.ArgumentParser(description="Transform a full PointCloud2 topic into a target frame for RViz.")
    parser.add_argument("--cloud-topic", default="/front_3d_lidar/lidar_points")
    parser.add_argument("--output-topic", default="/front_3d_lidar/map_points")
    parser.add_argument("--target-frame", default="map")
    parser.add_argument("--min-period-sec", type=float, default=0.15)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = PointCloudToMapCloud(args.cloud_topic, args.output_topic, args.target_frame, args.min_period_sec)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
