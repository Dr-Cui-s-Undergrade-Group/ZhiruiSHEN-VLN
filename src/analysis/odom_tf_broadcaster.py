#!/usr/bin/env python3
from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self, odom_topic: str) -> None:
        super().__init__("odom_tf_broadcaster")
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id or "base_link"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    parser = argparse.ArgumentParser(description="Broadcast odom->base_link TF from an Odometry topic.")
    parser.add_argument("--odom-topic", default="/chassis/odom")
    args = parser.parse_args()

    rclpy.init()
    node = OdomTfBroadcaster(args.odom_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
