#!/usr/bin/env python3
"""Safe waypoint driver for Phase 2 mapping coverage.

Drives Carter toward a sequence of waypoints using closed-loop cmd_vel control,
with a /scan-based safety stop (reverse + abort if an obstacle is within
safe_distance_m). Outputs progress so the mapping rosbag captures coverage of
the origin -> shelf corridor and back.

Carter's actual speed is ~0.045 m/s for 0.4 m/s commanded, so this is SLOW;
budget enough time. Waypoints are in odom frame (matches Isaac /chassis/odom).

Usage:
  ros2 run vln_nav2_bridge node8_waypoint_driver --ros-args -p use_sim_time:=true \
    -p waypoints:="[-6.3,10.8, 0.0,0.0]" -p max_linear:=0.4 -p max_angular:=0.5 \
    -p goal_tolerance:=0.4 -p safe_distance:=0.45
"""
from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class WaypointDriver(Node):
    def __init__(self) -> None:
        super().__init__("node8_waypoint_driver")
        self.declare_parameter("waypoints", [-6.3, 10.8, 0.0, 0.0])
        self.declare_parameter("max_linear", 0.4)
        self.declare_parameter("max_angular", 0.5)
        self.declare_parameter("goal_tolerance", 0.4)
        self.declare_parameter("safe_distance", 0.45)
        self.declare_parameter("reached_timeout_sec", 180.0)
        self.declare_parameter("cmd_rate_hz", 10.0)

        raw_value = self.get_parameter("waypoints").value
        if isinstance(raw_value, (list, tuple)):
            nums = [float(x) for x in raw_value]
        else:
            nums = [float(x) for x in str(raw_value).strip("[]").replace(",", " ").split()]
        if len(nums) % 2 != 0 or not nums:
            raise ValueError("waypoints must be an even-length list of x,y pairs")
        self.waypoints = [(nums[i], nums[i + 1]) for i in range(0, len(nums), 2)]
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.safe_dist = float(self.get_parameter("safe_distance").value)
        self.reached_timeout = float(self.get_parameter("reached_timeout_sec").value)
        cmd_rate = float(self.get_parameter("cmd_rate_hz").value)

        self.odom = None  # (x,y,yaw)
        self.min_scan = float("inf")

        qpub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
        qsub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
        qscan = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Twist, "/cmd_vel", qpub)
        self.create_subscription(Odometry, "/chassis/odom", self._odom_cb, qsub)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qscan)
        self.create_timer(1.0 / cmd_rate, self._tick)

        self.idx = 0
        self.wp_start_time = None
        self.aborted = False
        self.last_progress_pos = None
        self.last_progress_time = None
        self.recovery_active = False
        self.recovery_phase = ""
        self.recovery_start = 0.0
        self.recovery_start_pos = (0.0, 0.0)
        self.recovery_count = 0
        self.get_logger().info(
            f"Waypoints: {self.waypoints}  max_lin={self.max_linear} "
            f"goal_tol={self.goal_tol} safe={self.safe_dist}"
        )

    def _odom_cb(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom = (p.x, p.y, math.atan2(siny, cosy))

    def _scan_cb(self, msg) -> None:
        ranges = msg.ranges
        finite = [r for r in ranges if math.isfinite(r) and r >= msg.range_min]
        self.min_scan = min(finite) if finite else float("inf")

    def _tick(self) -> None:
        if self.aborted or self.odom is None:
            return
        if self.idx >= len(self.waypoints):
            self._stop()
            if not self.aborted:
                self.get_logger().info("All waypoints reached. Done.")
                self.aborted = True
            return

        # Handle recovery state (backup + turn to unstick)
        if self.recovery_active:
            self._recovery_step()
            return

        # Safety: obstacle too close -> trigger recovery (backup+turn) not skip
        if self.min_scan < self.safe_dist:
            self.get_logger().warn(
                f"Obstacle at {self.min_scan:.2f}m; starting recovery backup+turn."
            )
            self._start_recovery()
            return

        gx, gy = self.waypoints[self.idx]
        x, y, yaw = self.odom
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if self.wp_start_time is None:
            self.wp_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.last_progress_pos = (x, y)
            self.last_progress_time = self.wp_start_time
            self.get_logger().info(f"-> waypoint {self.idx} ({gx:.1f},{gy:.1f}), dist={dist:.2f}m")

        if dist < self.goal_tol:
            self.get_logger().info(f"reached waypoint {self.idx} (dist={dist:.2f}m)")
            self._stop()
            self.idx += 1
            self.wp_start_time = None
            return

        elapsed = (self.get_clock().now().nanoseconds * 1e-9) - self.wp_start_time
        if elapsed > self.reached_timeout:
            self.get_logger().warn(
                f"waypoint {self.idx} timeout after {elapsed:.0f}s (dist={dist:.2f}m); skipping"
            )
            self._stop()
            self.idx += 1
            self.wp_start_time = None
            return

        # Stuck detection: if moved < 0.05m in last 10s, trigger recovery
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.last_progress_pos is not None:
            moved = math.hypot(x - self.last_progress_pos[0], y - self.last_progress_pos[1])
            if moved > 0.15:
                self.last_progress_pos = (x, y)
                self.last_progress_time = now_sec
            elif (now_sec - self.last_progress_time) > 10.0:
                self.get_logger().warn(
                    f"Stuck (moved {moved:.2f}m in 10s); starting recovery backup+turn."
                )
                self._start_recovery()
                return

        # Pure-pursuit-ish steering: point at the goal with arc steering.
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        tw = Twist()
        lin = self.max_linear * max(0.4, 1.0 - abs(yaw_err) / (math.pi / 1.5))
        tw.linear.x = max(0.05, lin)
        tw.angular.z = max(-self.max_angular, min(self.max_angular, 1.8 * yaw_err))
        self.pub.publish(tw)

    def _start_recovery(self) -> None:
        """Begin backup-then-turn recovery sequence."""
        self.recovery_active = True
        self.recovery_phase = "backup"
        self.recovery_start = self.get_clock().now().nanoseconds * 1e-9
        self.recovery_start_pos = (self.odom[0], self.odom[1]) if self.odom else (0, 0)
        self.recovery_count += 1
        if self.recovery_count > 5:
            # Too many recoveries on this waypoint, skip it
            self.get_logger().warn(f"Too many recoveries ({self.recovery_count}); skipping waypoint {self.idx}")
            self.recovery_active = False
            self.recovery_count = 0
            self.idx += 1
            self.wp_start_time = None

    def _recovery_step(self) -> None:
        """Execute backup (1.5s reverse) then turn (3s) to unstick."""
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.recovery_start
        tw = Twist()
        if self.recovery_phase == "backup":
            if elapsed < 1.5:
                tw.linear.x = -0.25  # reverse at 0.25 m/s
                tw.angular.z = 0.0
            else:
                self.recovery_phase = "turn"
                self.recovery_start = now
                elapsed = 0.0
        if self.recovery_phase == "turn":
            if elapsed < 3.0:
                tw.linear.x = 0.0
                tw.angular.z = 1.0  # hard turn (alternate direction each recovery)
                if self.recovery_count % 2 == 0:
                    tw.angular.z = -1.0
            else:
                # Recovery done, resume normal driving
                self.recovery_active = False
                self.last_progress_pos = (self.odom[0], self.odom[1]) if self.odom else (0, 0)
                self.last_progress_time = now
                self.get_logger().info("Recovery complete, resuming navigation.")
                return
        self.pub.publish(tw)

    def _stop(self) -> None:
        tw = Twist()
        for _ in range(3):
            self.pub.publish(tw)


def main() -> None:
    rclpy.init()
    node = WaypointDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
