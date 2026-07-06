#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


class OdomToPath(Node):
    def __init__(
        self,
        odom_topic: str,
        path_topic: str,
        min_delta_m: float,
        max_poses: int,
        target_frame: str,
        path_z: float,
    ) -> None:
        super().__init__("odom_to_path")
        self.min_delta_m = min_delta_m
        self.max_poses = max_poses
        self.target_frame = target_frame
        self.path_z = path_z
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path = Path()
        self.path.header.frame_id = target_frame
        self.last_xy: Optional[Tuple[float, float]] = None
        self.publisher = self.create_publisher(Path, path_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.5, self.publish_path)

    def transform_pose(self, msg: Odometry) -> Optional[PoseStamped]:
        source_frame = msg.header.frame_id or "odom"
        source = PoseStamped()
        source.header = msg.header
        source.header.frame_id = source_frame
        source.pose = msg.pose.pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().debug(f"Waiting for {self.target_frame} <- {source_frame} TF: {exc}")
            return None
        stamped = do_transform_pose_stamped(source, transform)
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.target_frame
        stamped.pose.position.z = self.path_z
        return stamped

    def on_odom(self, msg: Odometry) -> None:
        stamped = self.transform_pose(msg)
        if stamped is None:
            return

        position = stamped.pose.position
        xy = (float(position.x), float(position.y))
        if self.last_xy is not None:
            if math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1]) < self.min_delta_m:
                return
        self.last_xy = xy

        self.path.header.stamp = stamped.header.stamp
        self.path.header.frame_id = self.target_frame
        self.path.poses.append(stamped)
        if self.max_poses > 0 and len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses :]
        self.publish_path()

    def publish_path(self) -> None:
        if not self.path.poses:
            return
        self.publisher.publish(self.path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish an executed nav_msgs/Path from an Odometry topic.")
    parser.add_argument("--odom-topic", default="/chassis/odom")
    parser.add_argument("--path-topic", default="/executed_path")
    parser.add_argument("--min-delta-m", type=float, default=0.02)
    parser.add_argument("--max-poses", type=int, default=5000)
    parser.add_argument("--target-frame", default="map")
    parser.add_argument("--path-z", type=float, default=0.02)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = OdomToPath(
        args.odom_topic,
        args.path_topic,
        args.min_delta_m,
        args.max_poses,
        args.target_frame,
        args.path_z,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
