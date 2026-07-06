#!/usr/bin/env python3
import csv
import io
import json
import math
import os
import re
import shutil
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.srv import SetInitialPose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from .qwen_model_wrapper import QwenVLWrapper
from .text_to_pose_converter import TextToPoseConverter


def _yaw_from_quaternion(q) -> float:
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def _now_cst() -> str:
    return datetime.now(timezone(timedelta(hours=8))).strftime("%Y-%m-%dT%H:%M:%S%z")


class Node8ActiveVisualRelocalization(Node):
    """Run a Node 8 AMCL test with active 360-degree visual landmark sampling."""

    def __init__(self) -> None:
        super().__init__("node8_active_visual_relocalization")
        self.declare_parameter("image_topic", "/front_stereo_camera/left/image_raw")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("target_x", -6.3)
        self.declare_parameter("target_y", 10.8)
        self.declare_parameter("target_yaw", math.pi)
        self.declare_parameter("output_csv", "data/node8_active_visual_relocalization_2026-06-24.csv")
        self.declare_parameter(
            "snapshot_dir",
            "data/runtime/trial_images/node8_active_visual_relocalization",
        )
        self.declare_parameter("spin_steps", 8)
        self.declare_parameter("spin_angular_speed_rad_s", 0.25)
        self.declare_parameter("spin_settle_sec", 1.0)
        self.declare_parameter("require_origin_start", True)
        self.declare_parameter("max_origin_start_distance_m", 0.35)
        self.declare_parameter("max_origin_start_yaw_abs_rad", 0.35)
        self.declare_parameter("nav_timeout_sec", 900.0)
        self.declare_parameter("divergence_cancel_m", 3.0)
        self.declare_parameter("divergence_cancel_hold_sec", 40.0)
        self.declare_parameter("enable_visual_reanchor", True)
        self.declare_parameter("visual_anchor_target", "purple boxes")
        self.declare_parameter("reanchor_disagreement_m", 1.2)
        self.declare_parameter("reanchor_cooldown_sec", 25.0)
        self.declare_parameter("arrival_radius_m", 0.8)
        self.declare_parameter("enable_final_visual_search", True)
        self.declare_parameter("final_visual_steps", 8)
        self.declare_parameter("final_visual_confidence_threshold", 0.65)
        self.declare_parameter("model_path", "/home/bluepoisons/Desktop/FURP/VLN/models/Qwen3-VL-2B-Instruct")
        self.declare_parameter("inference_mode", "api")
        self.declare_parameter("max_new_tokens", 256)
        self.declare_parameter("model_startup_timeout_sec", 300.0)
        self.declare_parameter("inference_request_timeout_sec", 180.0)
        self.declare_parameter("force_cpu", False)
        self.declare_parameter("gpu_device", "0")
        self.declare_parameter(
            "model_python_executable",
            "/home/bluepoisons/miniconda3/envs/isaaclab/bin/python",
        )
        self.declare_parameter(
            "inference_script_path",
            "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/src/vln_inference/run_inference_cli.py",
        )
        self.declare_parameter(
            "server_script_path",
            "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/src/vln_inference/run_inference_server.py",
        )

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.target_yaw = float(self.get_parameter("target_yaw").value)
        self.output_csv = Path(str(self.get_parameter("output_csv").value))
        self.snapshot_dir = Path(str(self.get_parameter("snapshot_dir").value))
        self.spin_steps = max(1, int(self.get_parameter("spin_steps").value))
        self.spin_angular_speed_rad_s = abs(float(self.get_parameter("spin_angular_speed_rad_s").value))
        self.spin_settle_sec = float(self.get_parameter("spin_settle_sec").value)
        self.require_origin_start = self._as_bool(self.get_parameter("require_origin_start").value)
        self.max_origin_start_distance_m = float(
            self.get_parameter("max_origin_start_distance_m").value
        )
        self.max_origin_start_yaw_abs_rad = float(
            self.get_parameter("max_origin_start_yaw_abs_rad").value
        )
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.divergence_cancel_m = float(self.get_parameter("divergence_cancel_m").value)
        self.divergence_cancel_hold_sec = float(
            self.get_parameter("divergence_cancel_hold_sec").value
        )
        self.enable_visual_reanchor = self._as_bool(
            self.get_parameter("enable_visual_reanchor").value
        )
        self.visual_anchor_target = str(self.get_parameter("visual_anchor_target").value).strip().lower()
        self.reanchor_disagreement_m = float(self.get_parameter("reanchor_disagreement_m").value)
        self.reanchor_cooldown_sec = float(self.get_parameter("reanchor_cooldown_sec").value)
        self.arrival_radius_m = float(self.get_parameter("arrival_radius_m").value)
        self.enable_final_visual_search = self._as_bool(
            self.get_parameter("enable_final_visual_search").value
        )
        self.final_visual_steps = max(1, int(self.get_parameter("final_visual_steps").value))
        self.final_visual_confidence_threshold = float(
            self.get_parameter("final_visual_confidence_threshold").value
        )

        self.latest_image = None
        self.latest_image_stamp = 0.0
        self.latest_raw_pose: Optional[Tuple[float, float, float]] = None
        self.latest_amcl_pose: Optional[Tuple[float, float, float]] = None
        self.latest_scan_quality: Dict[str, object] = {}

        raw_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, self.image_topic, self._on_image, 1)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, raw_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self._on_amcl, amcl_qos)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, raw_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.initial_pose_client = self.create_client(SetInitialPose, "/set_initial_pose")

        self.converter = TextToPoseConverter()
        self.model = QwenVLWrapper(
            model_path=str(self.get_parameter("model_path").value),
            max_new_tokens=int(self.get_parameter("max_new_tokens").value),
            mode=str(self.get_parameter("inference_mode").value),
            python_executable=str(self.get_parameter("model_python_executable").value),
            inference_script_path=str(self.get_parameter("inference_script_path").value),
            server_script_path=str(self.get_parameter("server_script_path").value),
            force_cpu=self._as_bool(self.get_parameter("force_cpu").value),
            gpu_device=str(self.get_parameter("gpu_device").value),
            request_timeout_sec=float(self.get_parameter("inference_request_timeout_sec").value),
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _on_image(self, msg: Image) -> None:
        self.latest_image = msg
        self.latest_image_stamp = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.latest_raw_pose = (float(p.x), float(p.y), _yaw_from_quaternion(msg.pose.pose.orientation))

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self.latest_amcl_pose = (float(p.x), float(p.y), _yaw_from_quaternion(msg.pose.pose.orientation))

    def _on_scan(self, msg: LaserScan) -> None:
        finite = [v for v in msg.ranges if math.isfinite(v)]
        self.latest_scan_quality = {
            "frame": msg.header.frame_id,
            "count": len(msg.ranges),
            "finite": len(finite),
            "finite_ratio": (len(finite) / len(msg.ranges)) if msg.ranges else 0.0,
            "min_range": min(finite) if finite else "",
            "max_range": max(finite) if finite else "",
        }

    def run(self) -> None:
        self.snapshot_dir.mkdir(parents=True, exist_ok=True)
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info("Starting Qwen VLM worker...")
        self.model.start(timeout_sec=float(self.get_parameter("model_startup_timeout_sec").value))
        try:
            self._wait_for_inputs()
            self._validate_clean_origin_start()
            visual_observations = self._run_active_visual_spin()
            visual_anchor_seen = self._visual_anchor_seen(visual_observations)
            pre_nav_reanchored = False
            if self.enable_visual_reanchor and visual_anchor_seen:
                pre_nav_reanchored = self._reanchor_amcl_to_raw("pre_nav_visual_anchor")
            nav_row = self._run_nav2_goal(visual_anchor_seen)
            nav_row["visual_anchor_seen"] = visual_anchor_seen
            nav_row["pre_nav_reanchored"] = pre_nav_reanchored
            navigation_arrived = self._target_error(self.latest_raw_pose) <= self.arrival_radius_m
            final_visual_observations = []
            final_visual_confirmed = False
            if self.enable_final_visual_search and navigation_arrived:
                final_visual_observations, final_visual_confirmed = self._run_final_visual_search()
            nav_row["navigation_arrived"] = navigation_arrived
            nav_row["final_visual_confirmed"] = final_visual_confirmed
            nav_row["task_success"] = navigation_arrived and final_visual_confirmed
            row = self._build_output_row(visual_observations, nav_row, final_visual_observations)
            self._append_csv(row)
            self.get_logger().info(f"Wrote {self.output_csv}")
        finally:
            self._publish_zero_cmd_vel()
            self.model.shutdown()

    def _wait_for_inputs(self) -> None:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image and self.latest_raw_pose and self.latest_amcl_pose:
                return
        raise RuntimeError(
            f"Timed out waiting for inputs: image={self.latest_image is not None}, "
            f"raw={self.latest_raw_pose}, amcl={self.latest_amcl_pose}"
        )

    def _validate_clean_origin_start(self) -> None:
        if not self.require_origin_start or self.latest_raw_pose is None:
            return
        x, y, yaw = self.latest_raw_pose
        distance = math.hypot(x, y)
        yaw_abs = abs(self._normalize_angle(yaw))
        if distance <= self.max_origin_start_distance_m and yaw_abs <= self.max_origin_start_yaw_abs_rad:
            return
        raise RuntimeError(
            "Refusing Node8 origin trial because Isaac raw odom is not at a clean start: "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, distance_from_origin={distance:.3f} m. "
            "Reset Carter/world to origin before running this evidence trial."
        )

    def _run_active_visual_spin(self) -> List[Dict[str, object]]:
        observations = []
        step_angle = (2.0 * math.pi) / float(self.spin_steps)
        instruction = (
            "Look for known warehouse relocalization landmarks in this camera view: purple boxes, "
            "right shelf, package area, black chair, green plant, wall corner, or rack. "
            "Prefer distinctive objects over generic walls. Estimate whether the best landmark is "
            "near, medium, or far, and whether it is left, center, or right. Return one compact JSON "
            "object with keys target, visible, confidence, horizontal_position, distance_estimate, "
            "evidence. Use confidence below 0.45 for a generic wall corner unless it has a unique "
            "warehouse layout cue."
        )
        for step in range(self.spin_steps):
            self._spin_for_duration(0.0, self.spin_settle_sec)
            image_path = self._snapshot_image(step, prefix="relocalization")
            raw_before = self.latest_raw_pose
            amcl_before = self.latest_amcl_pose
            output = self.model.infer_goal_text(instruction=instruction, image_path=str(image_path))
            model_json = self.converter.parse_model_json(output) or {}
            observations.append(
                {
                    "step": step,
                    "image_path": str(image_path),
                    "raw_x": raw_before[0] if raw_before else "",
                    "raw_y": raw_before[1] if raw_before else "",
                    "raw_yaw": raw_before[2] if raw_before else "",
                    "amcl_x": amcl_before[0] if amcl_before else "",
                    "amcl_y": amcl_before[1] if amcl_before else "",
                    "amcl_yaw": amcl_before[2] if amcl_before else "",
                    "model_target": model_json.get("target", ""),
                    "visible": model_json.get("visible", ""),
                    "confidence": model_json.get("confidence", ""),
                    "horizontal_position": model_json.get("horizontal_position", ""),
                    "distance_estimate": model_json.get("distance_estimate", ""),
                    "evidence": model_json.get("evidence", ""),
                    "model_output": output,
                }
            )
            self.get_logger().info(
                f"Visual relocalization step {step + 1}/{self.spin_steps}: "
                f"target={model_json.get('target', '')}, visible={model_json.get('visible', '')}, "
                f"confidence={model_json.get('confidence', '')}"
            )
            self._spin_relative(step_angle)
        self._publish_zero_cmd_vel()
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        return observations

    def _spin_relative(self, delta_yaw: float) -> None:
        previous_yaw = self.latest_raw_pose[2] if self.latest_raw_pose else 0.0
        signed_progress = 0.0
        direction = 1.0 if delta_yaw >= 0.0 else -1.0
        twist = Twist()
        twist.angular.z = direction * self.spin_angular_speed_rad_s
        timeout_sec = abs(delta_yaw) / max(1.0e-6, self.spin_angular_speed_rad_s) * 3.0 + 2.0
        deadline = time.monotonic() + timeout_sec
        target = abs(delta_yaw)
        while time.monotonic() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.latest_raw_pose:
                continue
            current_yaw = self.latest_raw_pose[2]
            step = self._normalize_angle(current_yaw - previous_yaw)
            signed_progress += direction * step
            previous_yaw = current_yaw
            if signed_progress >= target * 0.98:
                break
        self._publish_zero_cmd_vel()

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _run_final_visual_search(self) -> Tuple[List[Dict[str, object]], bool]:
        observations = []
        step_angle = (2.0 * math.pi) / float(self.final_visual_steps)
        instruction = (
            "Confirm whether purple boxes are visibly present in this current robot camera image. "
            "Return one compact JSON object with keys target, visible, confidence, "
            "horizontal_position, distance_estimate, evidence. Do not use map memory; only use "
            "the current image."
        )
        confirmed = False
        for step in range(self.final_visual_steps):
            self._spin_for_duration(0.0, self.spin_settle_sec)
            image_path = self._snapshot_image(step, prefix="final_search")
            raw_before = self.latest_raw_pose
            amcl_before = self.latest_amcl_pose
            output = self.model.infer_goal_text(instruction=instruction, image_path=str(image_path))
            model_json = self.converter.parse_model_json(output) or {}
            obs = {
                "step": step,
                "image_path": str(image_path),
                "raw_x": raw_before[0] if raw_before else "",
                "raw_y": raw_before[1] if raw_before else "",
                "raw_yaw": raw_before[2] if raw_before else "",
                "amcl_x": amcl_before[0] if amcl_before else "",
                "amcl_y": amcl_before[1] if amcl_before else "",
                "amcl_yaw": amcl_before[2] if amcl_before else "",
                "model_target": model_json.get("target", ""),
                "visible": model_json.get("visible", ""),
                "confidence": model_json.get("confidence", ""),
                "horizontal_position": model_json.get("horizontal_position", ""),
                "distance_estimate": model_json.get("distance_estimate", ""),
                "evidence": model_json.get("evidence", ""),
                "model_output": output,
            }
            observations.append(obs)
            confirmed = self._is_target_confirmed(obs)
            self.get_logger().info(
                f"Final visual search step {step + 1}/{self.final_visual_steps}: "
                f"target={model_json.get('target', '')}, visible={model_json.get('visible', '')}, "
                f"confidence={model_json.get('confidence', '')}, confirmed={confirmed}"
            )
            if confirmed:
                break
            if step < self.final_visual_steps - 1:
                self._spin_relative(step_angle)
        self._publish_zero_cmd_vel()
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        return observations, confirmed

    def _snapshot_image(self, step: int, prefix: str = "step") -> Path:
        if self.latest_image is None:
            raise RuntimeError("No image available for snapshot")
        image = self._pil_from_image_msg(self.latest_image)
        filename = f"{_now_cst()}_{prefix}_{step + 1:02d}.png".replace(":", "")
        path = self.snapshot_dir / filename
        tmp = path.with_suffix(".png.tmp")
        image.save(tmp, format="PNG")
        os.replace(tmp, path)
        return path

    def _spin_for_duration(self, angular_z: float, duration_sec: float) -> None:
        twist = Twist()
        twist.angular.z = float(angular_z)
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        if angular_z != 0.0:
            self._publish_zero_cmd_vel()

    def _publish_zero_cmd_vel(self) -> None:
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.02)

    def _run_nav2_goal(self, visual_anchor_seen: bool) -> Dict[str, object]:
        navigator = BasicNavigator()
        navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        navigator.waitUntilNav2Active(localizer="amcl")
        q = quaternion_from_euler(0.0, 0.0, self.target_yaw)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = self.target_x
        goal.pose.position.y = self.target_y
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        start = time.monotonic()
        last_feedback = ""
        feedback_count = 0
        best_raw_error = self._target_error(self.latest_raw_pose)
        best_amcl_error = self._target_error(self.latest_amcl_pose)
        max_disagreement = 0.0
        divergence_start = None
        reanchor_count = 0
        last_reanchor_time = 0.0
        result = "unknown"
        failure_reason = ""
        navigator.goToPose(goal)
        while not navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.05)
            fb = navigator.getFeedback()
            if fb is not None:
                feedback_count += 1
                last_feedback = getattr(fb, "distance_remaining", "")
            best_raw_error = min(best_raw_error, self._target_error(self.latest_raw_pose))
            best_amcl_error = min(best_amcl_error, self._target_error(self.latest_amcl_pose))
            disagreement = self._pose_disagreement()
            max_disagreement = max(max_disagreement, disagreement)
            if (
                self.enable_visual_reanchor
                and visual_anchor_seen
                and disagreement >= self.reanchor_disagreement_m
                and time.monotonic() - last_reanchor_time >= self.reanchor_cooldown_sec
            ):
                if self._reanchor_amcl_to_raw("nav_disagreement"):
                    reanchor_count += 1
                    last_reanchor_time = time.monotonic()
                    divergence_start = None
            if disagreement >= self.divergence_cancel_m:
                divergence_start = divergence_start or time.monotonic()
            else:
                divergence_start = None
            elapsed = time.monotonic() - start
            if divergence_start and time.monotonic() - divergence_start > self.divergence_cancel_hold_sec:
                result = "canceled_diverged"
                failure_reason = (
                    f"AMCL/raw disagreement exceeded {self.divergence_cancel_m}m "
                    f"for {self.divergence_cancel_hold_sec}s"
                )
                navigator.cancelTask()
                break
            if elapsed > self.nav_timeout_sec:
                result = "timeout"
                failure_reason = f"exceeded timeout {self.nav_timeout_sec}s"
                navigator.cancelTask()
                break
        if result == "unknown":
            code = navigator.getResult()
            result = {
                TaskResult.SUCCEEDED: "success",
                TaskResult.CANCELED: "canceled",
                TaskResult.FAILED: "failed",
            }.get(code, str(code))
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        return {
            "nav_result": result,
            "failure_reason": failure_reason,
            "duration_sec": time.monotonic() - start,
            "feedback_distance_m": last_feedback,
            "feedback_count": feedback_count,
            "best_raw_error_m": best_raw_error,
            "best_amcl_error_m": best_amcl_error,
            "max_amcl_raw_disagreement_m": max_disagreement,
            "reanchor_count": reanchor_count,
        }

    def _visual_anchor_seen(self, observations: List[Dict[str, object]]) -> bool:
        return any(self._is_target_confirmed(obs) for obs in observations)

    def _is_target_confirmed(self, obs: Dict[str, object]) -> bool:
        target_tokens = [token for token in re.split(r"[^a-z0-9]+", self.visual_anchor_target) if token]
        if not target_tokens:
            return False
        target = str(obs.get("model_target", "")).lower()
        confidence = self._float_or_zero(obs.get("confidence"))
        visible = obs.get("visible")
        is_visible = visible is True or str(visible).strip().lower() in ("true", "yes", "1")
        return (
            is_visible
            and confidence >= self.final_visual_confidence_threshold
            and all(token in target for token in target_tokens)
        )

    def _reanchor_amcl_to_raw(self, reason: str) -> bool:
        if self.latest_raw_pose is None:
            return False
        if not self.initial_pose_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Cannot reanchor AMCL: /set_initial_pose unavailable")
            return False
        x, y, yaw = self.latest_raw_pose
        q = quaternion_from_euler(0.0, 0.0, yaw)
        req = SetInitialPose.Request()
        req.pose.header.frame_id = "map"
        req.pose.header.stamp = self.get_clock().now().to_msg()
        req.pose.pose.pose.position.x = float(x)
        req.pose.pose.pose.position.y = float(y)
        req.pose.pose.pose.position.z = 0.0
        req.pose.pose.pose.orientation.x = q[0]
        req.pose.pose.pose.orientation.y = q[1]
        req.pose.pose.pose.orientation.z = q[2]
        req.pose.pose.pose.orientation.w = q[3]
        req.pose.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02,
        ]
        future = self.initial_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        ok = future.done() and future.exception() is None
        if ok:
            self.get_logger().info(
                f"Reanchored AMCL to raw odom for {reason}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
            )
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
        else:
            self.get_logger().warn(f"AMCL reanchor failed for {reason}")
        return ok

    def _build_output_row(
        self,
        observations: List[Dict[str, object]],
        nav: Dict[str, object],
        final_visual_observations: List[Dict[str, object]],
    ) -> Dict[str, object]:
        raw = self.latest_raw_pose or (math.nan, math.nan, math.nan)
        amcl = self.latest_amcl_pose or (math.nan, math.nan, math.nan)
        yaw_coverage = self._visual_yaw_coverage(observations)
        seen = [
            obs for obs in observations
            if str(obs.get("visible", "")).lower() == "true"
            or self._float_or_zero(obs.get("confidence")) >= 0.5
        ]
        return {
            "timestamp": _now_cst(),
            "target_x": self.target_x,
            "target_y": self.target_y,
            "target_yaw": self.target_yaw,
            **nav,
            "final_raw_x": raw[0],
            "final_raw_y": raw[1],
            "final_raw_yaw": raw[2],
            "final_amcl_x": amcl[0],
            "final_amcl_y": amcl[1],
            "final_amcl_yaw": amcl[2],
            "raw_error_m": self._target_error(raw),
            "amcl_error_m": self._target_error(amcl),
            "amcl_raw_disagreement_m": self._pose_disagreement(),
            "scan_finite_ratio": self.latest_scan_quality.get("finite_ratio", ""),
            "scan_finite": self.latest_scan_quality.get("finite", ""),
            "visual_steps": len(observations),
            "visual_yaw_coverage_rad": yaw_coverage,
            "visual_visible_count": len(seen),
            "visual_targets_seen": ";".join(str(obs.get("model_target", "")) for obs in seen),
            "visual_confidences": ";".join(str(obs.get("confidence", "")) for obs in seen),
            "visual_image_paths": ";".join(str(obs.get("image_path", "")) for obs in observations),
            "visual_observations_json": json.dumps(observations, ensure_ascii=False),
            "final_visual_steps": len(final_visual_observations),
            "final_visual_targets_seen": ";".join(
                str(obs.get("model_target", "")) for obs in final_visual_observations
            ),
            "final_visual_confidences": ";".join(
                str(obs.get("confidence", "")) for obs in final_visual_observations
            ),
            "final_visual_image_paths": ";".join(
                str(obs.get("image_path", "")) for obs in final_visual_observations
            ),
            "final_visual_observations_json": json.dumps(
                final_visual_observations, ensure_ascii=False
            ),
        }

    def _append_csv(self, row: Dict[str, object]) -> None:
        exists = self.output_csv.exists()
        with self.output_csv.open("a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            if not exists:
                writer.writeheader()
            writer.writerow(row)

    def _target_error(self, pose: Optional[Tuple[float, float, float]]) -> float:
        if pose is None:
            return math.inf
        return math.hypot(pose[0] - self.target_x, pose[1] - self.target_y)

    def _pose_disagreement(self) -> float:
        if self.latest_raw_pose is None or self.latest_amcl_pose is None:
            return 0.0
        return math.hypot(
            self.latest_raw_pose[0] - self.latest_amcl_pose[0],
            self.latest_raw_pose[1] - self.latest_amcl_pose[1],
        )

    def _visual_yaw_coverage(self, observations: List[Dict[str, object]]) -> float:
        yaws = []
        for obs in observations:
            try:
                yaws.append(float(obs.get("raw_yaw", "")))
            except (TypeError, ValueError):
                pass
        if len(yaws) < 2:
            return 0.0
        coverage = 0.0
        previous = yaws[0]
        for yaw in yaws[1:]:
            coverage += abs(self._normalize_angle(yaw - previous))
            previous = yaw
        if len(yaws) >= 3:
            closing_gap = abs(self._normalize_angle(yaws[0] - yaws[-1]))
            coverage += closing_gap
        return coverage

    @staticmethod
    def _float_or_zero(value) -> float:
        try:
            return float(value)
        except Exception:
            return 0.0

    @staticmethod
    def _pil_from_image_msg(msg: Image):
        from PIL import Image as PILImage

        encoding = msg.encoding.lower()
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        if encoding not in channels_by_encoding:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")
        width = int(msg.width)
        height = int(msg.height)
        channels = channels_by_encoding[encoding]
        row_bytes = width * channels
        data = bytes(msg.data)
        if int(msg.step) != row_bytes:
            rows = [data[i * int(msg.step): i * int(msg.step) + row_bytes] for i in range(height)]
            data = b"".join(rows)
        mode = "L" if channels == 1 else ("RGBA" if channels == 4 else "RGB")
        image = PILImage.frombytes(mode, (width, height), data)
        if encoding.startswith("bgr"):
            image = image.convert("RGB")
            r, g, b = image.split()
            image = PILImage.merge("RGB", (b, g, r))
        elif mode != "RGB":
            image = image.convert("RGB")
        return image


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node8ActiveVisualRelocalization()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
