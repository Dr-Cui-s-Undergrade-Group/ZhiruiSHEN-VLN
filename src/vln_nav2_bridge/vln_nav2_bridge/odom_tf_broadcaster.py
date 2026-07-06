from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data)
        self.get_logger().info(f"Broadcasting odom TF from {self.odom_topic}")

    def _on_odom(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id or "base_link"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
