from __future__ import annotations

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
    def __init__(self) -> None:
        super().__init__("odom_to_path")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("path_topic", "/executed_path")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("min_delta_m", 0.02)
        self.declare_parameter("max_poses", 5000)
        self.declare_parameter("path_z", 0.02)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.min_delta_m = float(self.get_parameter("min_delta_m").value)
        self.max_poses = int(self.get_parameter("max_poses").value)
        self.path_z = float(self.get_parameter("path_z").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path = Path()
        self.path.header.frame_id = self.target_frame
        self.last_xy: Optional[Tuple[float, float]] = None

        self.publisher = self.create_publisher(Path, self.path_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_timer(0.5, self._publish_path)
        self.get_logger().info(
            f"Publishing executed path {self.odom_topic} -> {self.path_topic} in {self.target_frame}"
        )

    def _transform_pose(self, msg: Odometry) -> Optional[PoseStamped]:
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

    def _on_odom(self, msg: Odometry) -> None:
        stamped = self._transform_pose(msg)
        if stamped is None:
            return

        xy = (float(stamped.pose.position.x), float(stamped.pose.position.y))
        if self.last_xy is not None:
            if math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1]) < self.min_delta_m:
                return
        self.last_xy = xy

        self.path.header.stamp = stamped.header.stamp
        self.path.header.frame_id = self.target_frame
        self.path.poses.append(stamped)
        if self.max_poses > 0 and len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses :]
        self._publish_path()

    def _publish_path(self) -> None:
        if self.path.poses:
            self.publisher.publish(self.path)


def main() -> None:
    rclpy.init()
    node = OdomToPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
