#!/usr/bin/env python3
import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

ROOT = Path(__file__).resolve().parents[2]
BRIDGE_SRC = ROOT / "src" / "vln_nav2_bridge"
sys.path.insert(0, str(BRIDGE_SRC))

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from vln_nav2_bridge.node6_map_preflight import DEFAULT_MAP_YAML, SimpleOccupancyMap


SHELF_EDGE_START = (-6.76, 10.78, 0.0)
CHAIR_GOAL = (-0.54, -0.69, 1.57)
RECORDED_UNSAFE_SHELF_EDGE = (-6.76, 10.78, 0.0)


class OdomWatcher(Node):
    def __init__(self, odom_topic: str) -> None:
        super().__init__("node7_safe_recovery_odom_watcher")
        self.pose: Optional[Tuple[float, float, float]] = None
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(yaw),
        )


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def wait_for_odom(watcher: OdomWatcher, timeout_sec: float) -> Optional[Tuple[float, float, float]]:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(watcher, timeout_sec=0.1)
        if watcher.pose is not None:
            return watcher.pose
    return watcher.pose


def navigate_to(
    navigator: BasicNavigator,
    watcher: OdomWatcher,
    name: str,
    goal: Tuple[float, float, float],
    timeout_sec: float,
) -> Dict[str, object]:
    start = time.monotonic()
    navigator.goToPose(make_pose(navigator, *goal))
    while not navigator.isTaskComplete():
        rclpy.spin_once(watcher, timeout_sec=0.05)
        if time.monotonic() - start > timeout_sec:
            navigator.cancelTask()
            return {
                "step": name,
                "result": "timeout",
                "duration_sec": round(time.monotonic() - start, 3),
            }

    result_code = navigator.getResult()
    if result_code == TaskResult.SUCCEEDED:
        result = "success"
    elif result_code == TaskResult.CANCELED:
        result = "canceled"
    else:
        result = "failed"

    final = wait_for_odom(watcher, 1.0)
    row = {
        "step": name,
        "result": result,
        "duration_sec": round(time.monotonic() - start, 3),
    }
    if final is not None:
        row.update(
            {
                "final_x": round(final[0], 3),
                "final_y": round(final[1], 3),
                "final_yaw": round(final[2], 3),
                "final_error_m": round(math.hypot(final[0] - goal[0], final[1] - goal[1]), 3),
            }
        )
    return row


def check_pose(
    occupancy: SimpleOccupancyMap,
    pose: Tuple[float, float, float],
    radius_m: float,
    min_free_ratio: float,
    search_m: float,
) -> Dict[str, object]:
    x, y, yaw = pose
    summary = occupancy.check_pose("safe_start", x, y, yaw, radius_m)
    nearest = occupancy.find_nearest_free_pose(
        x,
        y,
        yaw,
        radius_m=radius_m,
        min_free_ratio=min_free_ratio,
        max_search_m=search_m,
    )
    ok = summary.free_ratio >= min_free_ratio and summary.center_state == "free"
    row = {
        "safe_check_x": round(x, 3),
        "safe_check_y": round(y, 3),
        "center_state": summary.center_state,
        "free_ratio": round(summary.free_ratio, 3),
        "safe": ok,
        "recovery_used": False,
        "recovery_x": "",
        "recovery_y": "",
        "recovery_free_ratio": "",
    }
    if not ok and nearest is not None:
        row.update(
            {
                "recovery_used": True,
                "recovery_x": round(nearest.x, 3),
                "recovery_y": round(nearest.y, 3),
                "recovery_free_ratio": round(nearest.free_ratio, 3),
            }
        )
    return row


def write_rows(path: Path, rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "timestamp",
        "phase",
        "step",
        "result",
        "duration_sec",
        "safe_check_x",
        "safe_check_y",
        "center_state",
        "free_ratio",
        "safe",
        "recovery_used",
        "recovery_x",
        "recovery_y",
        "recovery_free_ratio",
        "final_x",
        "final_y",
        "final_yaw",
        "final_error_m",
        "notes",
    ]
    with path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=str(ROOT / "data" / "node7_safe_recovery_trials_2026-06-15.csv"))
    parser.add_argument("--odom-topic", default="/chassis/odom")
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    parser.add_argument("--safe-radius-m", type=float, default=0.30)
    parser.add_argument("--min-free-ratio", type=float, default=0.95)
    parser.add_argument("--nearest-search-m", type=float, default=0.75)
    parser.add_argument("--skip-shelf-positioning", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    watcher = OdomWatcher(args.odom_topic)
    occupancy = SimpleOccupancyMap.from_yaml(DEFAULT_MAP_YAML)

    rows = []
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        current = wait_for_odom(watcher, 10.0)
        if current is not None:
            row = check_pose(occupancy, current, args.safe_radius_m, args.min_free_ratio, args.nearest_search_m)
            row.update({"timestamp": timestamp, "phase": "initial_safe_check", "notes": "Current odom before targeted trial."})
            rows.append(row)

        if not args.skip_shelf_positioning:
            row = navigate_to(navigator, watcher, "position_to_shelf_edge", SHELF_EDGE_START, args.timeout_sec)
            row.update({"timestamp": timestamp, "phase": "setup", "notes": "Move to observed shelf-edge start pose."})
            rows.append(row)

        current = wait_for_odom(watcher, 3.0) or RECORDED_UNSAFE_SHELF_EDGE
        safe_row = check_pose(occupancy, current, args.safe_radius_m, args.min_free_ratio, args.nearest_search_m)
        safe_row.update({"timestamp": timestamp, "phase": "safe_start_check", "notes": "Node 7 safe-start check before chair navigation."})
        rows.append(safe_row)

        if safe_row.get("recovery_used"):
            recovery_goal = (float(safe_row["recovery_x"]), float(safe_row["recovery_y"]), current[2])
            row = navigate_to(navigator, watcher, "safe_start_recovery", recovery_goal, min(240.0, args.timeout_sec))
            row.update({"timestamp": timestamp, "phase": "recovery", "notes": "Navigate to nearest free candidate before target goal."})
            rows.append(row)

        row = navigate_to(navigator, watcher, "navigate_to_chair_after_safe_check", CHAIR_GOAL, args.timeout_sec)
        row.update({"timestamp": timestamp, "phase": "target_navigation", "notes": "Final target after applying safe-start policy."})
        rows.append(row)
    finally:
        write_rows(Path(args.output), rows)
        print(args.output)
        watcher.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
