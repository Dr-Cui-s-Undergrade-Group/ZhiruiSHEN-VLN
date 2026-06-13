#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import time
from typing import Dict, List, Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


DEFAULT_TRIALS = [
    "Go to the plant.",
    "Move to the potted plant on the floor.",
    "Navigate to the green plant near the chair.",
    "Go to the black office chair.",
    "Move to the chair near the robot.",
    "Navigate to the chair beside the plant.",
    "Go to the purple boxes.",
    "Move to the right shelf with purple boxes.",
    "Navigate to the shelf area containing purple packages.",
    "Go to the shelf.",
    "Move to the right shelf.",
    "Navigate to the warehouse rack near the boxes.",
    "Go to the object near the wall.",
    "Move to the package area.",
    "Navigate to the target object.",
]
DEFAULT_OUTPUT_FILENAME = "node6_auto_trials.csv"


class Node6AutoTrials(Node):
    """Publish Node 6 trial instructions and collect bridge results."""

    def __init__(
        self,
        instructions: List[str],
        output_path: str,
        instruction_topic: str,
        result_topic: str,
        pose_topic: str,
        odom_topic: str,
        timeout_sec: float,
        settle_sec: float,
        success_radius_m: float,
        trial_offset: int = 0,
        total_trial_count: Optional[int] = None,
    ) -> None:
        super().__init__("node6_auto_trials")
        self.instructions = instructions
        self.output_path = self._normalize_output_path(output_path)
        self.timeout_sec = timeout_sec
        self.settle_sec = settle_sec
        self.success_radius_m = success_radius_m
        self.trial_offset = trial_offset
        self.total_trial_count = total_trial_count or len(instructions)
        self.pending_instruction: Optional[str] = None
        self.latest_result: Optional[Dict[str, object]] = None
        self.latest_pose: Optional[Dict[str, float]] = None
        self.latest_odom_pose: Optional[Dict[str, float]] = None

        pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(String, instruction_topic, 10)
        self.subscription = self.create_subscription(
            String,
            result_topic,
            self._on_result,
            10,
        )
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._on_pose,
            pose_qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            10,
        )
        self.get_logger().info(
            f"Auto trials ready: {len(instructions)} instructions, "
            f"instruction_topic={instruction_topic}, result_topic={result_topic}, "
            f"pose_topic={pose_topic}, odom_topic={odom_topic}, "
            f"output={self.output_path}"
        )

    def run(self) -> List[Dict[str, object]]:
        results: List[Dict[str, object]] = []
        self._wait_for_subscription()

        for run_index, instruction in enumerate(self.instructions, start=1):
            trial_id = self.trial_offset + run_index
            self.pending_instruction = instruction
            self.latest_result = None
            self.get_logger().info(
                f"Trial {trial_id}/{self.total_trial_count}: {instruction}"
            )
            self.publisher.publish(String(data=instruction))

            result = self._wait_for_result(instruction=instruction, timeout_sec=self.timeout_sec)
            if result is None:
                result = {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "duration_sec": self.timeout_sec,
                    "instruction": instruction,
                    "image_path": "",
                    "model_output": "",
                    "model_target": "",
                    "visible": "",
                    "confidence": "",
                    "parse_method": "timeout",
                    "target_x": "",
                    "target_y": "",
                    "target_yaw": "",
                    "nav_result": "timeout",
                    "failure_reason": f"No /vln_trial_result within {self.timeout_sec}s.",
                }
                self.get_logger().error(f"Trial {trial_id} timed out.")
            else:
                self.get_logger().info(
                    "Bridge result: "
                    f"nav_result={result.get('nav_result')}, "
                    f"parse_method={result.get('parse_method')}, "
                    f"target=({result.get('target_x')}, {result.get('target_y')})"
                )

            self._spin_for_pose_update(0.5)
            result = self._append_final_pose_metrics(result)
            result["trial_id"] = trial_id
            self.get_logger().info(
                "Recorded trial: "
                f"nav_result={result.get('nav_result')}, "
                f"parse_method={result.get('parse_method')}, "
                f"target=({result.get('target_x')}, {result.get('target_y')}), "
                f"final=({result.get('final_x')}, {result.get('final_y')}), "
                f"error_m={result.get('final_error_m')}"
            )

            results.append(result)
            self._write_results(results)
            self._spin_for_pose_update(self.settle_sec)

        self._print_summary(results)
        return results

    def _wait_for_subscription(self) -> None:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and rclpy.ok():
            if self.publisher.get_subscription_count() > 0:
                return
            self.get_logger().info("Waiting for /vln_instruction subscriber...")
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.publisher.get_subscription_count() == 0:
            self.get_logger().warning(
                "No /vln_instruction subscriber detected. Publishing anyway."
            )

    def _wait_for_result(self, instruction: str, timeout_sec: float) -> Optional[Dict[str, object]]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_result and self.latest_result.get("instruction") == instruction:
                return self.latest_result
        return None

    def _on_result(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Ignoring malformed result JSON: {exc}")
            return
        self.latest_result = result

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.latest_pose = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": self._yaw_from_quaternion(
                x=float(pose.orientation.x),
                y=float(pose.orientation.y),
                z=float(pose.orientation.z),
                w=float(pose.orientation.w),
            ),
        }

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.latest_odom_pose = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": self._yaw_from_quaternion(
                x=float(pose.orientation.x),
                y=float(pose.orientation.y),
                z=float(pose.orientation.z),
                w=float(pose.orientation.w),
            ),
        }

    def _append_final_pose_metrics(self, result: Dict[str, object]) -> Dict[str, object]:
        enriched = dict(result)
        pose_source = "amcl_pose" if self.latest_pose is not None else "odom"
        pose = self.latest_pose or self.latest_odom_pose
        if pose is None:
            enriched.update(
                {
                    "final_pose_source": "",
                    "final_x": "",
                    "final_y": "",
                    "final_yaw": "",
                    "final_error_m": "",
                    "within_success_radius": "",
                }
            )
            return enriched

        final_x = pose["x"]
        final_y = pose["y"]
        final_yaw = pose["yaw"]
        enriched["final_pose_source"] = pose_source
        enriched["final_x"] = round(final_x, 3)
        enriched["final_y"] = round(final_y, 3)
        enriched["final_yaw"] = round(final_yaw, 3)

        try:
            target_x = float(result.get("target_x", ""))
            target_y = float(result.get("target_y", ""))
        except (TypeError, ValueError):
            enriched["final_error_m"] = ""
            enriched["within_success_radius"] = ""
            return enriched

        error_m = math.hypot(final_x - target_x, final_y - target_y)
        enriched["final_error_m"] = round(error_m, 3)
        enriched["within_success_radius"] = error_m <= self.success_radius_m
        return enriched

    def _spin_for_pose_update(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _write_results(self, results: List[Dict[str, object]]) -> None:
        output_dir = os.path.dirname(self.output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        fieldnames = [
            "trial_id",
            "timestamp",
            "duration_sec",
            "instruction",
            "image_path",
            "model_output",
            "model_target",
            "visible",
            "confidence",
            "parse_method",
            "target_x",
            "target_y",
            "target_yaw",
            "final_pose_source",
            "final_x",
            "final_y",
            "final_yaw",
            "final_error_m",
            "within_success_radius",
            "nav_result",
            "failure_reason",
        ]
        with open(self.output_path, "w", newline="", encoding="utf-8") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
            writer.writeheader()
            for index, result in enumerate(results, start=1):
                row = {key: result.get(key, "") for key in fieldnames}
                row["trial_id"] = result.get("trial_id", index)
                writer.writerow(row)

    @staticmethod
    def _normalize_output_path(output_path: str) -> str:
        path = os.path.expanduser(output_path.strip())
        if not path:
            return DEFAULT_OUTPUT_FILENAME

        if path.endswith(os.sep) or os.path.isdir(path):
            return os.path.join(path, DEFAULT_OUTPUT_FILENAME)

        return path

    def _print_summary(self, results: List[Dict[str, object]]) -> None:
        total = len(results)
        success = sum(1 for item in results if item.get("nav_result") == "success")
        json_count = sum(1 for item in results if item.get("parse_method") == "json")
        fallback_count = sum(
            1
            for item in results
            if item.get("parse_method") in ("instruction_fallback", "fallback")
        )
        within_radius = sum(1 for item in results if item.get("within_success_radius") is True)
        self.get_logger().info("Node 6 auto trials complete.")
        self.get_logger().info(f"Output CSV: {self.output_path}")
        self.get_logger().info(f"Navigation success: {success}/{total}")
        self.get_logger().info(
            f"Final pose within {self.success_radius_m:.2f} m: {within_radius}/{total}"
        )
        self.get_logger().info(f"JSON parse rate: {json_count}/{total}")
        self.get_logger().info(f"Fallback rate: {fallback_count}/{total}")

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def _load_instructions(path: Optional[str]) -> List[str]:
    if not path:
        return DEFAULT_TRIALS

    with open(path, "r", encoding="utf-8") as file_obj:
        if path.endswith(".json"):
            data = json.load(file_obj)
            if not isinstance(data, list):
                raise ValueError("Instruction JSON must be a list of strings.")
            return [str(item) for item in data if str(item).strip()]

        return [line.strip() for line in file_obj if line.strip() and not line.startswith("#")]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--instructions-file", default="")
    parser.add_argument(
        "--output",
        default="/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/node6_auto_trials.csv",
    )
    parser.add_argument("--instruction-topic", default="/vln_instruction")
    parser.add_argument("--result-topic", default="/vln_trial_result")
    parser.add_argument("--pose-topic", default="/amcl_pose")
    parser.add_argument("--odom-topic", default="/chassis/odom")
    parser.add_argument("--timeout-sec", type=float, default=240.0)
    parser.add_argument("--settle-sec", type=float, default=2.0)
    parser.add_argument("--success-radius-m", type=float, default=0.8)
    parser.add_argument(
        "--start-index",
        type=int,
        default=1,
        help="1-based default trial index to start from.",
    )
    args = parser.parse_args()

    instructions = _load_instructions(args.instructions_file)
    if args.start_index < 1 or args.start_index > len(instructions):
        raise ValueError(
            f"--start-index must be between 1 and {len(instructions)}, got {args.start_index}."
        )
    selected_instructions = instructions[args.start_index - 1:]

    rclpy.init()
    node = Node6AutoTrials(
        instructions=selected_instructions,
        output_path=args.output,
        instruction_topic=args.instruction_topic,
        result_topic=args.result_topic,
        pose_topic=args.pose_topic,
        odom_topic=args.odom_topic,
        timeout_sec=args.timeout_sec,
        settle_sec=args.settle_sec,
        success_radius_m=args.success_radius_m,
        trial_offset=args.start_index - 1,
        total_trial_count=len(instructions),
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
