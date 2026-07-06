#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import struct
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ScanToMapCloud(Node):
    def __init__(self, scan_topic: str, cloud_topic: str, target_frame: str) -> None:
        super().__init__("scan_to_map_cloud")
        self.target_frame = target_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, cloud_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)

    def lookup_2d_transform(self, source_frame: str) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def on_scan(self, msg: LaserScan) -> None:
        transform = self.lookup_2d_transform(msg.header.frame_id)
        if transform is None:
            return
        tx, ty, yaw = transform
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        points = bytearray()
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                local_x = float(distance) * math.cos(angle)
                local_y = float(distance) * math.sin(angle)
                map_x = tx + cos_yaw * local_x - sin_yaw * local_y
                map_y = ty + sin_yaw * local_x + cos_yaw * local_y
                points.extend(struct.pack("<fff", map_x, map_y, 0.05))
            angle += msg.angle_increment

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self.target_frame
        cloud.height = 1
        cloud.width = len(points) // 12
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = bytes(points)
        self.publisher.publish(cloud)


def main() -> None:
    parser = argparse.ArgumentParser(description="Transform LaserScan points into a map-frame PointCloud2 for RViz.")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--cloud-topic", default="/scan_map_cloud")
    parser.add_argument("--target-frame", default="map")
    args = parser.parse_args()

    rclpy.init()
    node = ScanToMapCloud(args.scan_topic, args.cloud_topic, args.target_frame)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
