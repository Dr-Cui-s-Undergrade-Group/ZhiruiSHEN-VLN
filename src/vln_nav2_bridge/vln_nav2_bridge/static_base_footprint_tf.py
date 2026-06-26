#!/usr/bin/env python3
"""Static identity TF: base_link -> base_footprint.

slam_toolbox mapper_params_offline.yaml uses base_frame: base_footprint, but
Isaac Sim publishes odom -> base_link only. Publish this static identity so the
TF chain map -> odom -> base_link -> base_footprint resolves for SLAM and Nav2.

Run with use_sim_time so the stamped transform aligns with Isaac's /clock.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class BaseFootprintTF(Node):
    def __init__(self) -> None:
        super().__init__("static_base_footprint_tf")
        self.broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "base_footprint"
        # Identity: base_footprint is base_link projected on the ground plane.
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(tf)
        self.get_logger().info("Static TF: base_link -> base_footprint (identity)")


def main() -> None:
    rclpy.init()
    node = BaseFootprintTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
