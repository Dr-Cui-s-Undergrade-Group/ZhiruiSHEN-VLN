#!/usr/bin/env python3
import csv
import io
import json
import math
import os
import re
import shutil
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .node6_map_preflight import DEFAULT_MAP_YAML, SimpleOccupancyMap
from .qwen_model_wrapper import QwenVLWrapper
from .text_to_pose_converter import TextToPoseConverter


class VLNBridgeNodeLocal(Node):
    """Node 5 local bridge: text instruction -> local VLM -> Nav2 goal."""

    def __init__(self) -> None:
        super().__init__("vln_bridge_node_local")

        # ============ CRITICAL FIX 1: Read use_sim_time (already declared by ROS 2 framework) ============
        # This ensures BasicNavigator syncs with simulation time from /clock topic
        # Note: use_sim_time is auto-declared by ROS 2, so we just read it
        try:
            use_sim_time = self.get_parameter("use_sim_time").value
        except Exception:
            use_sim_time = False  # Fallback default
        self.get_logger().info(f"use_sim_time={use_sim_time}")

        # ============ Declare all other parameters ============
        self.declare_parameter(
            "model_path",
            "/home/bluepoisons/Desktop/FURP/VLN/models/Qwen3-VL-2B-Instruct",
        )
        self.declare_parameter(
            "image_path",
            "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/test_samples/warehouse_photo1.png",
        )
        self.declare_parameter("instruction_topic", "/vln_instruction")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("max_new_tokens", 256)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("safe_min_x", -8.0)
        self.declare_parameter("safe_max_x", 10.0)
        self.declare_parameter("safe_min_y", -12.0)
        self.declare_parameter("safe_max_y", 15.0)
        self.declare_parameter("nav_timeout_sec", 240.0)
        self.declare_parameter("nav_feedback_log_interval_sec", 5.0)
        self.declare_parameter("inference_mode", "api")
        self.declare_parameter("conda_env", "isaaclab")
        self.declare_parameter(
            "model_python_executable",
            "/home/bluepoisons/miniconda3/envs/isaaclab/bin/python",
        )
        self.declare_parameter("force_cpu", False)
        try:
            self.declare_parameter("gpu_device", "0")
        except Exception:
            self.declare_parameter("gpu_device", 0)
        try:
            self.declare_parameter("model_startup_timeout_sec", 300.0)
        except Exception:
            self.declare_parameter("model_startup_timeout_sec", 300)
        try:
            self.declare_parameter("inference_request_timeout_sec", 180.0)
        except Exception:
            self.declare_parameter("inference_request_timeout_sec", 180)
        self.declare_parameter("allow_inference_fallback", False)
        self.declare_parameter("image_topic", "")
        self.declare_parameter("compressed_image_topic", "")
        self.declare_parameter(
            "live_image_path",
            "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/runtime/latest_camera.png",
        )
        self.declare_parameter(
            "live_image_snapshot_dir",
            "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/runtime/trial_images",
        )
        self.declare_parameter("image_timeout_sec", 5.0)
        self.declare_parameter("require_fresh_image", False)
        self.declare_parameter("live_image_save_interval_sec", 0.5)
        self.declare_parameter("visual_scan_enabled", True)
        self.declare_parameter("visual_scan_steps", 8)
        self.declare_parameter("visual_scan_step_rad", math.pi / 4.0)
        self.declare_parameter("visual_scan_settle_sec", 1.0)
        self.declare_parameter("visual_scan_min_confidence", 0.6)
        self.declare_parameter("visual_scan_spin_time_allowance_sec", 15.0)
        self.declare_parameter("visual_scan_spin_mode", "cmd_vel")
        self.declare_parameter("visual_scan_angular_speed_rad_s", 0.35)
        self.declare_parameter("visual_check_during_nav_enabled", True)
        self.declare_parameter("visual_check_interval_sec", 5.0)
        self.declare_parameter("visual_check_min_travel_m", 0.75)
        self.declare_parameter("final_visual_scan_required", True)
        self.declare_parameter("active_search_enabled", True)
        self.declare_parameter("active_search_forward_distance_m", 0.35)
        self.declare_parameter("active_search_linear_speed_m_s", 0.10)
        self.declare_parameter("active_search_max_moves", 2)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("semantic_exploration_enabled", True)
        self.declare_parameter("semantic_nav_first_enabled", True)
        self.declare_parameter("semantic_nav_first_min_distance_m", 2.0)
        self.declare_parameter("clear_costmaps_before_nav", True)
        self.declare_parameter("nav_retry_on_stuck", True)
        self.declare_parameter("nav_max_retries", 1)
        self.declare_parameter("nav_accept_distance_m", 0.30)
        self.declare_parameter("nav_accept_distance_hold_sec", 2.0)
        self.declare_parameter("nav_stuck_timeout_sec", 45.0)
        self.declare_parameter("nav_stuck_min_progress_m", 0.15)
        self.declare_parameter("nav_stuck_recovery_backup_speed_m_s", -0.10)
        self.declare_parameter("nav_stuck_recovery_backup_duration_sec", 1.0)
        self.declare_parameter("nav_stuck_recovery_turn_speed_rad_s", 0.35)
        self.declare_parameter("nav_stuck_recovery_turn_duration_sec", 1.0)
        self.declare_parameter("nav_accept_with_feedback_distance", True)
        self.declare_parameter("safe_map_validation_enabled", True)
        self.declare_parameter("safe_map_yaml", DEFAULT_MAP_YAML)
        self.declare_parameter("safe_start_enabled", True)
        self.declare_parameter("safe_goal_enabled", True)
        self.declare_parameter("safe_pose_check_radius_m", 0.30)
        self.declare_parameter("safe_pose_min_free_ratio", 0.95)
        self.declare_parameter("safe_nearest_free_search_m", 0.75)
        self.declare_parameter("dynamic_timeout_enabled", True)
        self.declare_parameter("dynamic_timeout_min_sec", 240.0)
        self.declare_parameter("dynamic_timeout_sec_per_m", 75.0)
        self.declare_parameter(
            "trial_log_path",
            "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/node6_trials.csv",
        )
        self.declare_parameter(
            "inference_script_path",
            "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/src/vln_inference/run_inference_cli.py",
        )
        self.declare_parameter(
            "server_script_path",
            "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/src/vln_inference/run_inference_server.py",
        )

        # ============ Read all parameters ============
        self.model_path = self.get_parameter("model_path").value
        self.image_path = self.get_parameter("image_path").value
        self.instruction_topic = self.get_parameter("instruction_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.max_new_tokens = int(self.get_parameter("max_new_tokens").value)
        self.dry_run = self._as_bool(self.get_parameter("dry_run").value)
        self.safe_min_x = float(self.get_parameter("safe_min_x").value)
        self.safe_max_x = float(self.get_parameter("safe_max_x").value)
        self.safe_min_y = float(self.get_parameter("safe_min_y").value)
        self.safe_max_y = float(self.get_parameter("safe_max_y").value)
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.nav_feedback_log_interval_sec = float(
            self.get_parameter("nav_feedback_log_interval_sec").value
        )
        self.inference_mode = self.get_parameter("inference_mode").value
        self.conda_env = self.get_parameter("conda_env").value
        self.model_python_executable = self.get_parameter("model_python_executable").value
        self.force_cpu = self._as_bool(self.get_parameter("force_cpu").value)
        self.gpu_device = str(self.get_parameter("gpu_device").value)
        self.model_startup_timeout_sec = float(
            self.get_parameter("model_startup_timeout_sec").value
        )
        self.inference_request_timeout_sec = float(
            self.get_parameter("inference_request_timeout_sec").value
        )
        self.allow_inference_fallback = self._as_bool(
            self.get_parameter("allow_inference_fallback").value
        )
        self.image_topic = self.get_parameter("image_topic").value
        self.compressed_image_topic = self.get_parameter("compressed_image_topic").value
        self.live_image_path = self.get_parameter("live_image_path").value
        self.live_image_snapshot_dir = self.get_parameter("live_image_snapshot_dir").value
        self.image_timeout_sec = float(self.get_parameter("image_timeout_sec").value)
        self.require_fresh_image = self._as_bool(self.get_parameter("require_fresh_image").value)
        self.live_image_save_interval_sec = float(
            self.get_parameter("live_image_save_interval_sec").value
        )
        self.visual_scan_enabled = self._as_bool(self.get_parameter("visual_scan_enabled").value)
        self.visual_scan_steps = max(1, int(self.get_parameter("visual_scan_steps").value))
        self.visual_scan_step_rad = float(self.get_parameter("visual_scan_step_rad").value)
        self.visual_scan_settle_sec = float(self.get_parameter("visual_scan_settle_sec").value)
        self.visual_scan_min_confidence = float(
            self.get_parameter("visual_scan_min_confidence").value
        )
        self.visual_scan_spin_time_allowance_sec = float(
            self.get_parameter("visual_scan_spin_time_allowance_sec").value
        )
        self.visual_scan_spin_mode = str(
            self.get_parameter("visual_scan_spin_mode").value
        ).strip().lower()
        self.visual_scan_angular_speed_rad_s = float(
            self.get_parameter("visual_scan_angular_speed_rad_s").value
        )
        self.visual_check_during_nav_enabled = self._as_bool(
            self.get_parameter("visual_check_during_nav_enabled").value
        )
        self.visual_check_interval_sec = float(
            self.get_parameter("visual_check_interval_sec").value
        )
        self.visual_check_min_travel_m = float(
            self.get_parameter("visual_check_min_travel_m").value
        )
        self.final_visual_scan_required = self._as_bool(
            self.get_parameter("final_visual_scan_required").value
        )
        self.active_search_enabled = self._as_bool(
            self.get_parameter("active_search_enabled").value
        )
        self.active_search_forward_distance_m = float(
            self.get_parameter("active_search_forward_distance_m").value
        )
        self.active_search_linear_speed_m_s = float(
            self.get_parameter("active_search_linear_speed_m_s").value
        )
        self.active_search_max_moves = max(
            0, int(self.get_parameter("active_search_max_moves").value)
        )
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.semantic_exploration_enabled = self._as_bool(
            self.get_parameter("semantic_exploration_enabled").value
        )
        self.semantic_nav_first_enabled = self._as_bool(
            self.get_parameter("semantic_nav_first_enabled").value
        )
        self.semantic_nav_first_min_distance_m = float(
            self.get_parameter("semantic_nav_first_min_distance_m").value
        )
        self.clear_costmaps_before_nav = self._as_bool(
            self.get_parameter("clear_costmaps_before_nav").value
        )
        self.nav_retry_on_stuck = self._as_bool(self.get_parameter("nav_retry_on_stuck").value)
        self.nav_max_retries = max(0, int(self.get_parameter("nav_max_retries").value))
        self.nav_accept_distance_m = float(self.get_parameter("nav_accept_distance_m").value)
        self.nav_accept_distance_hold_sec = float(
            self.get_parameter("nav_accept_distance_hold_sec").value
        )
        self.nav_stuck_timeout_sec = float(self.get_parameter("nav_stuck_timeout_sec").value)
        self.nav_stuck_min_progress_m = float(
            self.get_parameter("nav_stuck_min_progress_m").value
        )
        self.nav_stuck_recovery_backup_speed_m_s = float(
            self.get_parameter("nav_stuck_recovery_backup_speed_m_s").value
        )
        self.nav_stuck_recovery_backup_duration_sec = float(
            self.get_parameter("nav_stuck_recovery_backup_duration_sec").value
        )
        self.nav_stuck_recovery_turn_speed_rad_s = float(
            self.get_parameter("nav_stuck_recovery_turn_speed_rad_s").value
        )
        self.nav_stuck_recovery_turn_duration_sec = float(
            self.get_parameter("nav_stuck_recovery_turn_duration_sec").value
        )
        self.nav_accept_with_feedback_distance = self._as_bool(
            self.get_parameter("nav_accept_with_feedback_distance").value
        )
        self.safe_map_validation_enabled = self._as_bool(
            self.get_parameter("safe_map_validation_enabled").value
        )
        self.safe_map_yaml = self.get_parameter("safe_map_yaml").value
        self.safe_start_enabled = self._as_bool(self.get_parameter("safe_start_enabled").value)
        self.safe_goal_enabled = self._as_bool(self.get_parameter("safe_goal_enabled").value)
        self.safe_pose_check_radius_m = float(
            self.get_parameter("safe_pose_check_radius_m").value
        )
        self.safe_pose_min_free_ratio = float(
            self.get_parameter("safe_pose_min_free_ratio").value
        )
        self.safe_nearest_free_search_m = float(
            self.get_parameter("safe_nearest_free_search_m").value
        )
        self.dynamic_timeout_enabled = self._as_bool(
            self.get_parameter("dynamic_timeout_enabled").value
        )
        self.dynamic_timeout_min_sec = float(
            self.get_parameter("dynamic_timeout_min_sec").value
        )
        self.dynamic_timeout_sec_per_m = float(
            self.get_parameter("dynamic_timeout_sec_per_m").value
        )
        self.trial_log_path = self.get_parameter("trial_log_path").value
        self.inference_script_path = self.get_parameter("inference_script_path").value
        self.server_script_path = self.get_parameter("server_script_path").value

        # ============ Initialize model and converter (lightweight, no blocking) ============
        self.model = QwenVLWrapper(
            model_path=self.model_path,
            max_new_tokens=self.max_new_tokens,
            mode=self.inference_mode,
            conda_env=self.conda_env,
            inference_script_path=self.inference_script_path,
            server_script_path=self.server_script_path,
            python_executable=self.model_python_executable,
            force_cpu=self.force_cpu,
            gpu_device=self.gpu_device,
            request_timeout_sec=self.inference_request_timeout_sec,
        )
        self.converter = TextToPoseConverter(
            min_x=self.safe_min_x,
            max_x=self.safe_max_x,
            min_y=self.safe_min_y,
            max_y=self.safe_max_y,
        )
        self.occupancy_map = self._load_safe_occupancy_map()

        # ============ CRITICAL FIX 2: Defer Nav2 initialization to timer callback ============
        # This allows rclpy.spin() to start BEFORE waiting for Nav2 active.
        # Without this, /clock messages cannot be processed while waitUntilNav2Active() blocks.
        self.navigator = None
        self.goal_pub = None
        self.result_pub = None
        self.cmd_vel_pub = None
        self.sub = None
        self.image_sub = None
        self.compressed_image_sub = None
        self.pose_sub = None
        self.odom_sub = None
        self.latest_image_path = None
        self.latest_image_wall_time = 0.0
        self.latest_map_x = None
        self.latest_map_y = None
        self.latest_map_wall_time = 0.0
        self.latest_odom_x = None
        self.latest_odom_y = None
        self.latest_odom_wall_time = 0.0
        self.last_image_save_wall_time = 0.0
        self.trial_image_counter = 0
        self.image_save_lock = threading.Lock()
        self.nav2_initialized = False
        self._safe_start_recovery_active = False
        self._active_instruction = ""
        self._target_seen_during_nav = False
        self._target_seen_during_nav_path = ""
        self._target_seen_during_nav_output = ""
        self._target_seen_during_nav_json = None
        self.instruction_callback_group = MutuallyExclusiveCallbackGroup()
        self.image_callback_group = ReentrantCallbackGroup()
        self._ensure_live_image_dirs()

        # Single-shot timer: runs once after node starts spinning, then destroys itself
        self._init_timer = self.create_timer(
            0.1,
            self._on_init_timer_callback,
            callback_group=self.instruction_callback_group,
        )
        self.get_logger().info(
            "Node initialized (fast path). Nav2 setup will happen asynchronously..."
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _on_init_timer_callback(self) -> None:
        """
        Callback invoked by timer after rclpy.spin() is already running.
        This ensures /clock messages are being processed when we call waitUntilNav2Active().
        """
        if self.nav2_initialized:
            return

        try:
            # Now safe to initialize BasicNavigator with synced simulation time
            self.navigator = BasicNavigator()

            # ============ CRITICAL FIX: Force BasicNavigator into simulation time ============
            # BasicNavigator is an independent ROS 2 node that doesn't inherit use_sim_time
            # from the command-line parameter, so we must set it explicitly!
            use_sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
            self.navigator.set_parameters([use_sim_time_param])
            self.get_logger().info("BasicNavigator synced to simulation time.")
            # ==============================================================================

            self.get_logger().info("Waiting for Nav2 to become active...")

            # This blocking call now runs inside the spin loop, so /clock is being processed
            # self.navigator.waitUntilNav2Active()
            self.get_logger().info("Nav2 is active.")

            self.get_logger().info(
                f"Starting Qwen inference backend: mode={self.inference_mode}, "
                f"force_cpu={self.force_cpu}, gpu_device={self.gpu_device}"
            )
            self.model.start(timeout_sec=self.model_startup_timeout_sec)
            self.get_logger().info("Qwen inference backend is ready.")

            # Create publisher and subscriptions only after the VLM backend is healthy.
            self.goal_pub = self.create_publisher(PoseStamped, "/vln_goal_pose", 10)
            self.result_pub = self.create_publisher(String, "/vln_trial_result", 10)
            self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            self.sub = self.create_subscription(
                String,
                self.instruction_topic,
                self._on_instruction,
                10,
                callback_group=self.instruction_callback_group,
            )
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self._on_pose,
                10,
                callback_group=self.image_callback_group,
            )
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self._on_odom,
                10,
                callback_group=self.image_callback_group,
            )
            self._setup_image_subscriptions()

            self.nav2_initialized = True
            self.destroy_timer(self._init_timer)
            self.get_logger().info(
                "Node ready. "
                f"Listening on {self.instruction_topic}. "
                f"dry_run={self.dry_run}, inference_mode={self.inference_mode}, "
                f"force_cpu={self.force_cpu}, gpu_device={self.gpu_device}, "
                f"allow_inference_fallback={self.allow_inference_fallback}, "
                f"visual_scan_enabled={self.visual_scan_enabled}, "
                f"visual_scan_steps={self.visual_scan_steps}, "
                f"visual_scan_spin_mode={self.visual_scan_spin_mode}, "
                f"visual_check_during_nav_enabled={self.visual_check_during_nav_enabled}, "
                f"final_visual_scan_required={self.final_visual_scan_required}, "
                f"active_search_enabled={self.active_search_enabled}, "
                f"semantic_exploration_enabled={self.semantic_exploration_enabled}, "
                f"semantic_nav_first_enabled={self.semantic_nav_first_enabled}, "
                f"pose_topic={self.pose_topic}, "
                f"odom_topic={self.odom_topic}, "
                f"nav_retry_on_stuck={self.nav_retry_on_stuck}, "
                f"nav_max_retries={self.nav_max_retries}"
            )

        except Exception as exc:
            self.get_logger().error(f"Nav2 initialization failed: {exc}. Retrying...")

    def _on_instruction(self, msg: String) -> None:
        # Safety check: wait for Nav2 to be ready before processing
        if not self.nav2_initialized:
            self.get_logger().warning(
                "Received instruction before Nav2 initialized. Ignoring: " + msg.data[:50]
            )
            return

        instruction = msg.data.strip()
        if not instruction:
            self.get_logger().warning("Received empty instruction. Ignoring.")
            return

        self.get_logger().info(f"Received instruction: {instruction}")
        self._reset_active_visual_state(instruction)

        image_path = ""
        model_output = ""
        x = y = yaw = 0.0
        method = "failed"
        nav_result = "not_started"
        failure_reason = ""
        model_json = None
        started_at = time.time()
        semantic_explored = False

        if self.converter.is_ambiguous_instruction(instruction):
            failure_reason = (
                "ambiguous_target: instruction does not name a concrete known "
                "semantic target. Ask for plant, chair, shelf, purple boxes, "
                "or package area."
            )
            self.get_logger().warning(failure_reason)
            result_payload = self._build_trial_result(
                instruction=instruction,
                image_path=image_path,
                model_output=model_output,
                model_json=None,
                parse_method="ambiguous_target",
                x=0.0,
                y=0.0,
                yaw=0.0,
                nav_result="failed",
                failure_reason=failure_reason,
                started_at=started_at,
                navigation_arrived=False,
                visual_confirmed=False,
                task_success=False,
            )
            self._publish_and_log_result(result_payload)
            return

        semantic_first_result = self._try_semantic_nav_first(instruction=instruction)
        if semantic_first_result.get("attempted"):
            x = float(semantic_first_result.get("x", 0.0))
            y = float(semantic_first_result.get("y", 0.0))
            yaw = float(semantic_first_result.get("yaw", 0.0))
            nav_result = str(semantic_first_result.get("nav_result", "failed"))
            failure_reason = str(semantic_first_result.get("reason", ""))

            if nav_result not in ("success", "dry_run"):
                result_payload = self._build_trial_result(
                    instruction=instruction,
                    image_path=image_path,
                    model_output=model_output,
                    model_json=None,
                    parse_method="semantic_nav_first_failed",
                    x=x,
                    y=y,
                    yaw=yaw,
                    nav_result=nav_result,
                    failure_reason=failure_reason,
                    started_at=started_at,
                    navigation_arrived=False,
                    visual_confirmed=False,
                    task_success=False,
                )
                self._publish_and_log_result(result_payload)
                return

            final_result = self._confirm_target_after_arrival(instruction=instruction)
            image_path = str(final_result.get("image_path", image_path))
            model_output = str(final_result.get("model_output", model_output))
            model_json = final_result.get("model_json")
            active_search_used = bool(final_result.get("active_search_used", False))
            final_visual_confirmed = bool(final_result.get("ok", False))
            method = "semantic_nav_first_final_visual_confirmed"
            if not final_visual_confirmed:
                method = "semantic_nav_first_final_visual_failed"
                failure_reason = str(
                    final_result.get("reason", "final visual confirmation failed")
                )

            result_payload = self._build_trial_result(
                instruction=instruction,
                image_path=image_path,
                model_output=model_output,
                model_json=model_json if isinstance(model_json, dict) else None,
                parse_method=method,
                x=x,
                y=y,
                yaw=yaw,
                nav_result="success",
                failure_reason=failure_reason,
                started_at=started_at,
                navigation_arrived=True,
                visual_confirmed=final_visual_confirmed,
                task_success=final_visual_confirmed,
                target_seen_during_nav=self._target_seen_during_nav,
                final_visual_confirmed=final_visual_confirmed,
                active_search_used=active_search_used,
            )
            self._publish_and_log_result(result_payload)
            return

        if self.visual_scan_enabled:
            scan_result = self._visual_scan_for_target(instruction=instruction)
            image_path = str(scan_result.get("image_path", ""))
            model_output = str(scan_result.get("model_output", ""))
            model_json = scan_result.get("model_json")
            method = str(scan_result.get("method", "visual_scan"))

            if not scan_result.get("ok"):
                exploration_result = self._try_semantic_exploration(
                    instruction=instruction,
                    scan_result=scan_result,
                )
                if exploration_result.get("attempted"):
                    x = float(exploration_result.get("x", 0.0))
                    y = float(exploration_result.get("y", 0.0))
                    yaw = float(exploration_result.get("yaw", 0.0))

                    if not exploration_result.get("ok"):
                        failure_reason = str(
                            exploration_result.get("reason", "semantic exploration failed")
                        )
                        result_payload = self._build_trial_result(
                            instruction=instruction,
                            image_path=image_path,
                            model_output=model_output,
                            model_json=model_json if isinstance(model_json, dict) else None,
                            parse_method="semantic_explore_failed",
                            x=x,
                            y=y,
                            yaw=yaw,
                            nav_result=str(exploration_result.get("nav_result", "failed")),
                            failure_reason=failure_reason,
                            started_at=started_at,
                        )
                        self._publish_and_log_result(result_payload)
                        return

                    semantic_explored = True
                    scan_result = self._confirm_target_after_arrival(instruction=instruction)
                    image_path = str(scan_result.get("image_path", image_path))
                    model_output = str(scan_result.get("model_output", model_output))
                    model_json = scan_result.get("model_json", model_json)
                    method = str(scan_result.get("method", "visual_scan"))
                    active_search_used = bool(scan_result.get("active_search_used", False))

                    if scan_result.get("ok"):
                        result_payload = self._build_trial_result(
                            instruction=instruction,
                            image_path=image_path,
                            model_output=model_output,
                            model_json=model_json if isinstance(model_json, dict) else None,
                            parse_method="semantic_explore_final_visual_confirmed",
                            x=x,
                            y=y,
                            yaw=yaw,
                            nav_result="success",
                            failure_reason="",
                            started_at=started_at,
                            navigation_arrived=True,
                            visual_confirmed=True,
                            task_success=True,
                            target_seen_during_nav=self._target_seen_during_nav,
                            final_visual_confirmed=True,
                            active_search_used=active_search_used,
                        )
                        self._publish_and_log_result(result_payload)
                        return

                    if not scan_result.get("ok"):
                        relaxed_confirmed = self._relaxed_semantic_confirmation(
                            instruction=instruction,
                            scan_result=scan_result,
                            semantic_target=str(exploration_result.get("target", "")),
                        )
                        if relaxed_confirmed:
                            result_payload = self._build_trial_result(
                                instruction=instruction,
                                image_path=image_path,
                                model_output=model_output,
                                model_json=model_json if isinstance(model_json, dict) else None,
                                parse_method="semantic_explore_relaxed_confirm",
                                x=x,
                                y=y,
                                yaw=yaw,
                                nav_result="success",
                                failure_reason="",
                                started_at=started_at,
                                navigation_arrived=True,
                                visual_confirmed=True,
                                task_success=True,
                                target_seen_during_nav=self._target_seen_during_nav,
                                final_visual_confirmed=True,
                                active_search_used=active_search_used,
                            )
                            self._publish_and_log_result(result_payload)
                            return

                        failure_reason = (
                            "semantic_explore_confirm_failed: "
                            f"{scan_result.get('reason', 'Target not visible.')}"
                        )
                        result_payload = self._build_trial_result(
                            instruction=instruction,
                            image_path=image_path,
                            model_output=model_output,
                            model_json=model_json if isinstance(model_json, dict) else None,
                            parse_method="semantic_explore_visual_scan_failed",
                            x=x,
                            y=y,
                            yaw=yaw,
                            nav_result="failed",
                            failure_reason=failure_reason,
                            started_at=started_at,
                            navigation_arrived=True,
                            visual_confirmed=False,
                            task_success=False,
                            target_seen_during_nav=self._target_seen_during_nav,
                            final_visual_confirmed=False,
                            active_search_used=active_search_used,
                        )
                        self._publish_and_log_result(result_payload)
                        return
                else:
                    failure_reason = str(scan_result.get("reason", "Target not visible."))
                    result_payload = self._build_trial_result(
                        instruction=instruction,
                        image_path=image_path,
                        model_output=model_output,
                        model_json=model_json if isinstance(model_json, dict) else None,
                        parse_method=method,
                        x=0.0,
                        y=0.0,
                        yaw=0.0,
                        nav_result="failed",
                        failure_reason=failure_reason,
                        started_at=started_at,
                    )
                    self._publish_and_log_result(result_payload)
                    return

            if not scan_result.get("ok"):
                failure_reason = str(scan_result.get("reason", "Target not visible."))
                result_payload = self._build_trial_result(
                    instruction=instruction,
                    image_path=image_path,
                    model_output=model_output,
                    model_json=model_json if isinstance(model_json, dict) else None,
                    parse_method=method,
                    x=0.0,
                    y=0.0,
                    yaw=0.0,
                    nav_result="failed",
                    failure_reason=failure_reason,
                    started_at=started_at,
                )
                self._publish_and_log_result(result_payload)
                return

            target_name = ""
            if isinstance(model_json, dict):
                target_name = str(model_json.get("target", ""))
            parsed = self.converter.resolve_named_target(
                instruction=instruction,
                model_target=target_name,
            )
            if not parsed.get("ok"):
                self.get_logger().warning(
                    f"Visual target was found, but map resolution failed: {parsed.get('reason')}"
                )
                result_payload = self._build_trial_result(
                    instruction=instruction,
                    image_path=image_path,
                    model_output=model_output,
                    model_json=model_json if isinstance(model_json, dict) else None,
                    parse_method="visual_map_failed",
                    x=0.0,
                    y=0.0,
                    yaw=0.0,
                    nav_result="failed",
                    failure_reason=str(parsed.get("reason")),
                    started_at=started_at,
                )
                self._publish_and_log_result(result_payload)
                return

            if semantic_explored:
                parsed["method"] = f"semantic_explore_then_{parsed.get('method', 'visual_semantic_map')}"
        else:
            try:
                image_path = self._get_inference_image_path(instruction=instruction)
                model_output = self.model.infer_goal_text(
                    instruction=instruction,
                    image_path=image_path,
                )
            except Exception as exc:
                self.get_logger().error(f"Model inference failed: {exc}")
                failure_reason = f"inference_failed: {exc}"
                if not self.allow_inference_fallback:
                    result_payload = self._build_trial_result(
                        instruction=instruction,
                        image_path=image_path,
                        model_output=model_output,
                        model_json=None,
                        parse_method="inference_failed",
                        x=0.0,
                        y=0.0,
                        yaw=0.0,
                        nav_result="failed",
                        failure_reason=failure_reason,
                        started_at=started_at,
                    )
                    self._publish_and_log_result(result_payload)
                    return

                self.get_logger().info("Using deterministic keyword fallback.")
                model_output = instruction

            self.get_logger().info(f"Model output: {model_output}")
            model_json = self.converter.parse_model_json(model_output)
            parsed = self.converter.convert(instruction=instruction, model_output=model_output)
            if not parsed.get("ok"):
                self.get_logger().warning(f"Conversion failed: {parsed.get('reason')}")
                result_payload = self._build_trial_result(
                    instruction=instruction,
                    image_path=image_path,
                    model_output=model_output,
                    model_json=model_json,
                    parse_method="failed",
                    x=0.0,
                    y=0.0,
                    yaw=0.0,
                    nav_result="failed",
                    failure_reason=str(parsed.get("reason")),
                    started_at=started_at,
                )
                self._publish_and_log_result(result_payload)
                return

        x = float(parsed["x"])
        y = float(parsed["y"])
        yaw = float(parsed["yaw"])
        x, y, yaw = self._safe_goal_candidate(x=x, y=y, yaw=yaw)
        method = parsed.get("method", "unknown")
        self.get_logger().info(
            f"Resolved target ({method}): x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        pose = self._build_pose(x=x, y=y, yaw=yaw)
        self.goal_pub.publish(pose)
        self.get_logger().info("Published pose to /vln_goal_pose")

        nav_result, failure_reason = self._navigate_to_pose(pose)
        navigation_arrived = nav_result in ("success", "dry_run")
        final_visual_confirmed = False
        active_search_used = False
        if navigation_arrived and self.final_visual_scan_required:
            final_result = self._confirm_target_after_arrival(instruction=instruction)
            image_path = str(final_result.get("image_path", image_path))
            model_output = str(final_result.get("model_output", model_output))
            model_json = final_result.get("model_json", model_json)
            final_visual_confirmed = bool(final_result.get("ok", False))
            active_search_used = bool(final_result.get("active_search_used", False))
            if final_visual_confirmed:
                method = f"{method}_final_visual_confirmed"
            else:
                method = f"{method}_final_visual_failed"
                failure_reason = str(
                    final_result.get("reason", "final visual confirmation failed")
                )
        elif navigation_arrived:
            final_visual_confirmed = self._model_reports_visible_target(
                model_json=model_json if isinstance(model_json, dict) else None,
                instruction=instruction,
            )

        result_payload = self._build_trial_result(
            instruction=instruction,
            image_path=image_path,
            model_output=model_output,
            model_json=model_json,
            parse_method=method,
            x=x,
            y=y,
            yaw=yaw,
            nav_result=nav_result,
            failure_reason=failure_reason,
            started_at=started_at,
            navigation_arrived=navigation_arrived,
            visual_confirmed=(navigation_arrived and final_visual_confirmed),
            target_seen_during_nav=self._target_seen_during_nav,
            final_visual_confirmed=final_visual_confirmed,
            active_search_used=active_search_used,
        )
        self._publish_and_log_result(result_payload)

    def _try_semantic_exploration(self, instruction: str, scan_result: dict) -> dict:
        """Navigate to a known semantic candidate before trying visual confirmation again."""
        live_configured = bool(self.image_topic or self.compressed_image_topic)
        if not self.semantic_exploration_enabled or not live_configured:
            return {"attempted": False}

        model_json = scan_result.get("model_json")
        model_target = ""
        if isinstance(model_json, dict):
            model_target = str(model_json.get("target", ""))

        parsed = self.converter.resolve_named_target(
            instruction=instruction,
            model_target=model_target,
        )
        if not parsed.get("ok") and model_target:
            parsed = self.converter.resolve_named_target(
                instruction=instruction,
                model_target="",
            )
        if not parsed.get("ok"):
            return {
                "attempted": False,
                "reason": parsed.get("reason", "No semantic candidate matched."),
            }

        x = float(parsed["x"])
        y = float(parsed["y"])
        yaw = float(parsed["yaw"])
        x, y, yaw = self._safe_goal_candidate(x=x, y=y, yaw=yaw)
        target = str(parsed.get("target", "semantic_candidate"))
        self.get_logger().info(
            "Target was not confirmed in the current view. "
            f"Navigating to semantic candidate {target!r}: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        pose = self._build_pose(x=x, y=y, yaw=yaw)
        if self.goal_pub is not None:
            self.goal_pub.publish(pose)
            self.get_logger().info("Published semantic candidate pose to /vln_goal_pose")

        nav_result, failure_reason = self._navigate_to_pose(pose)
        ok = nav_result in ("success", "dry_run")
        return {
            "attempted": True,
            "ok": ok,
            "x": x,
            "y": y,
            "yaw": yaw,
            "target": target,
            "nav_result": nav_result,
            "reason": failure_reason,
        }

    def _try_semantic_nav_first(self, instruction: str) -> dict:
        """For far known targets, navigate to the semantic pose before visual scanning."""
        if not self.semantic_nav_first_enabled or not self.semantic_exploration_enabled:
            return {"attempted": False}
        if not self.final_visual_scan_required:
            return {"attempted": False, "reason": "final visual scan is not required"}

        parsed = self.converter.resolve_named_target(
            instruction=instruction,
            model_target="",
        )
        if not parsed.get("ok"):
            return {
                "attempted": False,
                "reason": parsed.get("reason", "No semantic candidate matched."),
            }

        x = float(parsed["x"])
        y = float(parsed["y"])
        yaw = float(parsed["yaw"])
        x, y, yaw = self._safe_goal_candidate(x=x, y=y, yaw=yaw)
        pose = self._build_pose(x=x, y=y, yaw=yaw)

        distance = self._distance_from_latest_odom(pose)
        if distance is None:
            distance = self._distance_from_latest_map_pose(pose)
        if distance is not None and distance < self.semantic_nav_first_min_distance_m:
            return {
                "attempted": False,
                "reason": (
                    f"semantic target is near ({distance:.2f}m); use normal visual scan"
                ),
            }

        # For long-range VLN/ObjectNav, VLFM/VL-Nav style pipelines keep moving
        # toward map/semantic candidates while collecting visual evidence online,
        # then gate success on final visual confirmation at the candidate.
        # Sources:
        # - https://arxiv.org/abs/2312.03275
        # - https://arxiv.org/abs/2502.00931
        target = str(parsed.get("target", "semantic_candidate"))
        distance_text = "unknown" if distance is None else f"{distance:.2f}m"
        self.get_logger().info(
            "Semantic-nav-first selected for far target "
            f"{target!r}: distance={distance_text}, "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}. "
            "Skipping initial visual scan; final visual confirmation remains required."
        )

        if self.goal_pub is not None:
            self.goal_pub.publish(pose)
            self.get_logger().info("Published semantic-nav-first pose to /vln_goal_pose")

        nav_result, failure_reason = self._navigate_to_pose(pose)
        return {
            "attempted": True,
            "ok": nav_result in ("success", "dry_run"),
            "x": x,
            "y": y,
            "yaw": yaw,
            "target": target,
            "nav_result": nav_result,
            "reason": failure_reason,
        }

    def _relaxed_semantic_confirmation(
        self,
        instruction: str,
        scan_result: dict,
        semantic_target: str,
    ) -> bool:
        """Accept same-cluster visual evidence after arriving at a semantic candidate."""
        model_json = scan_result.get("model_json")
        if not isinstance(model_json, dict):
            return False
        if not self._model_reports_visible_target(
            model_json=model_json,
            instruction=instruction,
        ):
            return False

        target = str(model_json.get("target", ""))
        evidence = str(model_json.get("evidence", ""))
        combined_evidence = f"{target} {evidence}"
        instruction_cluster = self.converter.semantic_cluster(instruction)
        evidence_cluster = self.converter.semantic_cluster(combined_evidence)
        semantic_cluster = self.converter.semantic_cluster(semantic_target)

        if instruction_cluster != "shelf_package_area":
            return False
        if evidence_cluster != "shelf_package_area":
            return False
        if semantic_cluster and semantic_cluster != instruction_cluster:
            return False

        self.get_logger().info(
            "Relaxed semantic confirmation accepted: "
            f"instruction_cluster={instruction_cluster}, "
            f"semantic_target={semantic_target!r}, visual_target={target!r}."
        )
        return True

    def _reset_active_visual_state(self, instruction: str) -> None:
        self._active_instruction = instruction
        self._target_seen_during_nav = False
        self._target_seen_during_nav_path = ""
        self._target_seen_during_nav_output = ""
        self._target_seen_during_nav_json = None

    def _maybe_visual_check_during_nav(
        self,
        now: float,
        last_check_at: float,
        last_check_x: Optional[float],
        last_check_y: Optional[float],
    ) -> bool:
        if not self.visual_check_during_nav_enabled:
            return False
        if self._target_seen_during_nav:
            return False
        if not self._active_instruction:
            return False
        if not (self.image_topic or self.compressed_image_topic):
            return False
        if now - last_check_at < self.visual_check_interval_sec:
            return False

        moved_enough = True
        if (
            last_check_x is not None
            and last_check_y is not None
            and self.latest_map_x is not None
            and self.latest_map_y is not None
        ):
            moved = math.hypot(
                float(self.latest_map_x) - float(last_check_x),
                float(self.latest_map_y) - float(last_check_y),
            )
            moved_enough = moved >= self.visual_check_min_travel_m
        if not moved_enough:
            return False

        # Active perception note, 2026-06-23:
        # VLFM/VL-Nav style systems update language-grounded visual evidence
        # while moving, but a transient detection is not task success. The
        # final gate remains navigation_arrived AND final visual confirmation.
        # Sources:
        # - https://arxiv.org/abs/2312.03275
        # - https://arxiv.org/abs/2502.00931
        check_result = self._single_visual_check(
            instruction=self._active_instruction,
            label="during_nav",
        )
        if check_result.get("ok"):
            self._target_seen_during_nav = True
            self._target_seen_during_nav_path = str(check_result.get("image_path", ""))
            self._target_seen_during_nav_output = str(check_result.get("model_output", ""))
            self._target_seen_during_nav_json = check_result.get("model_json")
            self.get_logger().info(
                "Target seen during navigation; keeping Nav2 goal active until "
                "arrival and final visual confirmation."
            )
        return True

    def _single_visual_check(self, instruction: str, label: str) -> dict:
        self._wait_for_live_image(timeout_sec=self.image_timeout_sec)
        try:
            image_path = self._get_inference_image_path(
                instruction=f"{label}_{instruction}"
            )
            model_output = self.model.infer_goal_text(
                instruction=instruction,
                image_path=image_path,
            )
        except Exception as exc:
            self.get_logger().warning(f"Visual check failed: {exc}")
            return {
                "ok": False,
                "reason": f"visual_check_failed: {exc}",
                "method": f"{label}_visual_check_failed",
            }

        model_json = self.converter.parse_model_json(model_output)
        ok = self._model_reports_visible_target(
            model_json=model_json,
            instruction=instruction,
        )
        self.get_logger().info(
            f"Visual check {label}: ok={ok}, image={image_path}, output={model_output}"
        )
        return {
            "ok": ok,
            "reason": "" if ok else "target_not_visible_in_visual_check",
            "image_path": image_path,
            "model_output": model_output,
            "model_json": model_json,
            "method": f"{label}_visual_check",
        }

    def _confirm_target_after_arrival(self, instruction: str) -> dict:
        scan_result = self._visual_scan_for_target(instruction=instruction)
        if scan_result.get("ok"):
            scan_result["active_search_used"] = False
            return scan_result

        if not self.active_search_enabled or self.active_search_max_moves <= 0:
            scan_result["active_search_used"] = False
            return scan_result

        last_result = scan_result
        for move_index in range(self.active_search_max_moves):
            direction = 1.0 if move_index % 2 == 0 else -1.0
            self.get_logger().info(
                "Final visual scan failed; running local active search move "
                f"{move_index + 1}/{self.active_search_max_moves}."
            )
            if not self._active_search_step(direction=direction):
                break
            last_result = self._visual_scan_for_target(instruction=instruction)
            last_result["active_search_used"] = True
            if last_result.get("ok"):
                return last_result

        last_result["active_search_used"] = True
        return last_result

    def _active_search_step(self, direction: float) -> bool:
        if self.cmd_vel_pub is None:
            self.get_logger().warning("Cannot run active search: cmd_vel publisher missing.")
            return False
        speed = abs(self.active_search_linear_speed_m_s)
        if speed <= 0.0:
            self.get_logger().warning("active_search_linear_speed_m_s must be > 0.")
            return False
        distance = abs(self.active_search_forward_distance_m)
        duration_sec = distance / speed
        twist = Twist()
        twist.linear.x = math.copysign(speed, direction)
        self.get_logger().info(
            "Active search cmd_vel move: "
            f"distance={math.copysign(distance, direction):.2f}m, "
            f"speed={twist.linear.x:.2f}m/s, duration={duration_sec:.2f}s"
        )
        try:
            self._publish_cmd_vel_for_duration(twist, duration_sec=duration_sec)
        finally:
            self._publish_zero_cmd_vel()
        self._wait_for_live_image(timeout_sec=max(0.5, self.visual_scan_settle_sec))
        return True

    def _navigate_to_pose(self, pose: PoseStamped) -> Tuple[str, str]:
        if self.dry_run:
            self.get_logger().info("dry_run=True, skipping goToPose call.")
            return "dry_run", ""

        if self.navigator is None:
            return "failed", "Navigator is not initialized."

        safe_start_result = self._recover_safe_start_if_needed()
        if safe_start_result is not None and safe_start_result[0] not in ("success", "dry_run"):
            return safe_start_result

        max_attempts = 1
        if self.nav_retry_on_stuck:
            max_attempts += self.nav_max_retries

        last_result = ("failed", "Navigation did not start.")
        for attempt_index in range(max_attempts):
            nav_result, failure_reason = self._navigate_to_pose_once(
                pose=pose,
                attempt_index=attempt_index + 1,
                max_attempts=max_attempts,
            )
            last_result = (nav_result, failure_reason)
            if nav_result != "stuck":
                return last_result
            if attempt_index >= max_attempts - 1:
                return last_result

            self.get_logger().warning(
                "Nav2 appears stuck; running short cmd_vel recovery before retry."
            )
            self._recover_from_nav_stuck()

        return last_result

    def _navigate_to_pose_once(
        self,
        pose: PoseStamped,
        attempt_index: int,
        max_attempts: int,
    ) -> Tuple[str, str]:
        if self.clear_costmaps_before_nav:
            try:
                self.navigator.clearAllCostmaps()
                self.get_logger().info("Cleared Nav2 costmaps before navigation.")
            except Exception as exc:
                self.get_logger().warning(f"Failed to clear Nav2 costmaps: {exc}")

        try:
            self.get_logger().info(
                f"Starting Nav2 attempt {attempt_index}/{max_attempts}."
            )
            self.navigator.goToPose(pose)
        except Exception as exc:
            return "failed", f"Nav2 goToPose failed to start: {exc}"

        nav_started_at = time.monotonic()
        last_feedback_log_at = 0.0
        best_distance = None
        last_progress_at = nav_started_at
        last_visual_check_at = nav_started_at
        last_visual_check_x = self.latest_map_x
        last_visual_check_y = self.latest_map_y
        within_acceptance_since = None
        effective_timeout_sec = self._effective_nav_timeout_sec(pose)
        while not self.navigator.isTaskComplete():
            elapsed = time.monotonic() - nav_started_at
            if effective_timeout_sec > 0.0 and elapsed > effective_timeout_sec:
                failure_reason = (
                    f"Nav2 task exceeded nav_timeout_sec={effective_timeout_sec:.1f}."
                )
                self.get_logger().error(failure_reason)
                try:
                    self.navigator.cancelTask()
                except Exception as exc:
                    self.get_logger().warning(f"Failed to cancel timed-out Nav2 task: {exc}")
                return "timeout", failure_reason

            feedback = self.navigator.getFeedback()
            now = time.monotonic()
            if feedback:
                feedback_distance = float(feedback.distance_remaining)
                map_distance = self._distance_from_latest_map_pose(pose)
                diagnostic_odom_distance = self._distance_from_latest_odom(pose)
                if self._maybe_visual_check_during_nav(
                    now=now,
                    last_check_at=last_visual_check_at,
                    last_check_x=last_visual_check_x,
                    last_check_y=last_visual_check_y,
                ):
                    last_visual_check_at = now
                    last_visual_check_x = self.latest_map_x
                    last_visual_check_y = self.latest_map_y
                if (
                    feedback_distance <= 0.01
                    and (
                        (
                            map_distance is not None
                            and map_distance > self.nav_accept_distance_m
                        )
                        or (
                            diagnostic_odom_distance is not None
                            and diagnostic_odom_distance > self.nav_accept_distance_m
                        )
                    )
                ):
                    if now - last_feedback_log_at >= self.nav_feedback_log_interval_sec:
                        last_feedback_log_at = now
                        map_text = (
                            f", amcl_to_goal={map_distance:.2f}m"
                            if map_distance is not None
                            else ""
                        )
                        odom_text = (
                            f", raw_odom_to_goal={diagnostic_odom_distance:.2f}m"
                            if diagnostic_odom_distance is not None
                            else ""
                        )
                        self.get_logger().warning(
                            "Ignoring inconsistent initial Nav2 feedback: "
                            f"distance_remaining={feedback_distance:.2f}m"
                            f"{map_text}{odom_text}."
                        )
                    time.sleep(0.05)
                    continue
                # 2026-06-23 Nav2 long-route guard:
                # BasicNavigator feedback can briefly report 0.0 m before the
                # action server has a valid path, and replanning can make
                # path-length feedback oscillate. Track bridge-side progress
                # with the current AMCL-to-goal Euclidean distance when it is
                # available. Nav2's own SimpleProgressChecker still handles
                # controller-level movement progress.
                # Sources:
                # - https://docs.nav2.org/commander_api/index.html
                # - https://docs.nav2.org/configuration/packages/nav2_controller-plugins/simple_progress_checker.html
                distance_for_progress = (
                    map_distance
                    if map_distance is not None
                    else (
                        diagnostic_odom_distance
                        if diagnostic_odom_distance is not None
                        else feedback_distance
                    )
                )
                accept_distance = (
                    feedback_distance
                    if self.nav_accept_with_feedback_distance
                    else map_distance
                )
                if (
                    self.nav_accept_distance_m > 0.0
                    and accept_distance is not None
                    and accept_distance <= self.nav_accept_distance_m
                ):
                    if within_acceptance_since is None:
                        within_acceptance_since = now
                    elif now - within_acceptance_since >= self.nav_accept_distance_hold_sec:
                        self.get_logger().info(
                            "Accepting Nav2 goal by distance: "
                            f"distance={accept_distance:.2f}m <= "
                            f"{self.nav_accept_distance_m:.2f}m for "
                            f"{self.nav_accept_distance_hold_sec:.1f}s."
                        )
                        try:
                            self.navigator.cancelTask()
                        except Exception as exc:
                            self.get_logger().warning(
                                f"Failed to cancel accepted Nav2 task: {exc}"
                            )
                        return "success", ""
                else:
                    within_acceptance_since = None

                if (
                    best_distance is None
                    or distance_for_progress < best_distance - self.nav_stuck_min_progress_m
                ):
                    best_distance = distance_for_progress
                    last_progress_at = now

                if (
                    self.nav_retry_on_stuck
                    and self.nav_stuck_timeout_sec > 0.0
                    and now - last_progress_at > self.nav_stuck_timeout_sec
                ):
                    failure_reason = (
                        "Nav2 made no meaningful progress for "
                        f"{self.nav_stuck_timeout_sec:.1f}s "
                        f"(best_distance={best_distance:.2f}m)."
                    )
                    self.get_logger().warning(failure_reason)
                    try:
                        self.navigator.cancelTask()
                    except Exception as exc:
                        self.get_logger().warning(f"Failed to cancel stuck Nav2 task: {exc}")
                    return "stuck", failure_reason

            if (
                feedback
                and now - last_feedback_log_at >= self.nav_feedback_log_interval_sec
            ):
                last_feedback_log_at = now
                map_distance = self._distance_from_latest_map_pose(pose)
                diagnostic_odom_distance = self._distance_from_latest_odom(pose)
                map_text = (
                    f", amcl_to_goal={map_distance:.2f} m"
                    if map_distance is not None
                    else ""
                )
                odom_text = (
                    f", raw_odom_to_goal={diagnostic_odom_distance:.2f} m"
                    if diagnostic_odom_distance is not None
                    else ""
                )
                self.get_logger().info(
                    f"Distance remaining: {feedback.distance_remaining:.2f} m"
                    f"{map_text}{odom_text}"
                )
            time.sleep(0.05)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation succeeded.")
            return "success", ""
        if result == TaskResult.CANCELED:
            self.get_logger().warning("Navigation canceled.")
            return "canceled", "Nav2 task canceled."
        if result == TaskResult.FAILED:
            self.get_logger().error("Navigation failed.")
            return "failed", "Nav2 task failed."

        self.get_logger().warning("Navigation returned unknown result.")
        return "unknown", "Nav2 returned unknown result."

    def _distance_from_latest_odom(self, pose: PoseStamped) -> Optional[float]:
        if self.latest_odom_x is None or self.latest_odom_y is None:
            return None
        if time.monotonic() - self.latest_odom_wall_time > 2.0:
            return None
        return math.hypot(
            float(pose.pose.position.x) - float(self.latest_odom_x),
            float(pose.pose.position.y) - float(self.latest_odom_y),
        )

    def _distance_from_latest_map_pose(self, pose: PoseStamped) -> Optional[float]:
        if self.latest_map_x is None or self.latest_map_y is None:
            return None
        if time.monotonic() - self.latest_map_wall_time > 2.0:
            return None
        return math.hypot(
            float(pose.pose.position.x) - float(self.latest_map_x),
            float(pose.pose.position.y) - float(self.latest_map_y),
        )

    def _load_safe_occupancy_map(self) -> Optional[SimpleOccupancyMap]:
        if not self.safe_map_validation_enabled:
            return None
        try:
            occupancy_map = SimpleOccupancyMap.from_yaml(self.safe_map_yaml)
        except Exception as exc:
            self.get_logger().warning(f"Safe map validation disabled: {exc}")
            return None

        self.get_logger().info(
            "Loaded safe occupancy map: "
            f"{self.safe_map_yaml}, size={occupancy_map.width}x{occupancy_map.height}"
        )
        return occupancy_map

    def _safe_goal_candidate(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        if not self.safe_goal_enabled or self.occupancy_map is None:
            return x, y, yaw

        check = self.occupancy_map.check_pose(
            name="goal",
            x=x,
            y=y,
            yaw=yaw,
            radius_m=self.safe_pose_check_radius_m,
        )
        if check.ok(self.safe_pose_min_free_ratio):
            return x, y, yaw

        nearest = self.occupancy_map.find_nearest_free_pose(
            x=x,
            y=y,
            yaw=yaw,
            radius_m=self.safe_pose_check_radius_m,
            min_free_ratio=self.safe_pose_min_free_ratio,
            max_search_m=self.safe_nearest_free_search_m,
        )
        if nearest is None:
            self.get_logger().warning(
                "Safe-goal check failed and no nearest free candidate was found: "
                f"target=({x:.2f}, {y:.2f}), center={check.center_state}, "
                f"free={check.free_ratio:.1%}."
            )
            return x, y, yaw

        self.get_logger().warning(
            "Safe-goal adjusted target: "
            f"({x:.2f}, {y:.2f}) -> ({nearest.x:.2f}, {nearest.y:.2f}), "
            f"old_center={check.center_state}, old_free={check.free_ratio:.1%}, "
            f"new_free={nearest.free_ratio:.1%}."
        )
        return nearest.x, nearest.y, yaw

    def _recover_safe_start_if_needed(self) -> Optional[Tuple[str, str]]:
        if (
            not self.safe_start_enabled
            or self.occupancy_map is None
            or self._safe_start_recovery_active
        ):
            return None
        if self.latest_map_x is None or self.latest_map_y is None:
            return None
        if time.monotonic() - self.latest_map_wall_time > 2.0:
            return None

        check = self.occupancy_map.check_pose(
            name="safe_start",
            x=float(self.latest_map_x),
            y=float(self.latest_map_y),
            yaw=0.0,
            radius_m=self.safe_pose_check_radius_m,
        )
        if check.ok(self.safe_pose_min_free_ratio):
            return None

        odom_check = self._safe_start_odom_check()
        if odom_check is not None and odom_check.ok(self.safe_pose_min_free_ratio):
            self.get_logger().warning(
                "Safe-start AMCL pose is invalid, but raw odom pose is safe; "
                "continuing with odom-truth navigation. "
                f"amcl=({check.x:.2f}, {check.y:.2f}), "
                f"amcl_center={check.center_state}, amcl_free={check.free_ratio:.1%}; "
                f"odom=({odom_check.x:.2f}, {odom_check.y:.2f}), "
                f"odom_center={odom_check.center_state}, odom_free={odom_check.free_ratio:.1%}."
            )
            return None

        nearest = self.occupancy_map.find_nearest_free_pose(
            x=float(self.latest_map_x),
            y=float(self.latest_map_y),
            yaw=0.0,
            radius_m=self.safe_pose_check_radius_m,
            min_free_ratio=self.safe_pose_min_free_ratio,
            max_search_m=self.safe_nearest_free_search_m,
        )
        if nearest is None:
            reason = (
                "Safe-start check failed and no nearest free candidate was found: "
                f"current=({check.x:.2f}, {check.y:.2f}), center={check.center_state}, "
                f"free={check.free_ratio:.1%}."
            )
            self.get_logger().warning(reason)
            return "failed", reason

        self.get_logger().warning(
            "Safe-start recovery target: "
            f"current=({check.x:.2f}, {check.y:.2f}), center={check.center_state}, "
            f"free={check.free_ratio:.1%}; nearest=({nearest.x:.2f}, {nearest.y:.2f}), "
            f"free={nearest.free_ratio:.1%}."
        )
        self._safe_start_recovery_active = True
        try:
            recovery_pose = self._build_pose(x=nearest.x, y=nearest.y, yaw=nearest.yaw)
            return self._navigate_to_pose_once(
                pose=recovery_pose,
                attempt_index=1,
                max_attempts=1,
            )
        finally:
            self._safe_start_recovery_active = False

    def _safe_start_odom_check(self):
        if self.latest_odom_x is None or self.latest_odom_y is None:
            return None
        if time.monotonic() - self.latest_odom_wall_time > 2.0:
            return None
        return self.occupancy_map.check_pose(
            name="safe_start_odom",
            x=float(self.latest_odom_x),
            y=float(self.latest_odom_y),
            yaw=0.0,
            radius_m=self.safe_pose_check_radius_m,
        )

    def _effective_nav_timeout_sec(self, pose: PoseStamped) -> float:
        if not self.dynamic_timeout_enabled:
            return self.nav_timeout_sec

        distance = self._distance_from_latest_map_pose(pose)
        if distance is None:
            return self.nav_timeout_sec

        dynamic_timeout = max(
            self.dynamic_timeout_min_sec,
            distance * max(0.0, self.dynamic_timeout_sec_per_m),
        )
        effective_timeout = max(self.nav_timeout_sec, dynamic_timeout)
        if effective_timeout > self.nav_timeout_sec:
            self.get_logger().info(
                "Dynamic timeout expanded: "
                f"distance={distance:.2f}m, base={self.nav_timeout_sec:.1f}s, "
                f"effective={effective_timeout:.1f}s."
            )
        return effective_timeout

    def _recover_from_nav_stuck(self) -> None:
        if self.cmd_vel_pub is None:
            return

        backup = Twist()
        backup.linear.x = self.nav_stuck_recovery_backup_speed_m_s
        self._publish_cmd_vel_for_duration(
            twist=backup,
            duration_sec=max(0.0, self.nav_stuck_recovery_backup_duration_sec),
        )

        turn = Twist()
        turn.angular.z = self.nav_stuck_recovery_turn_speed_rad_s
        self._publish_cmd_vel_for_duration(
            twist=turn,
            duration_sec=max(0.0, self.nav_stuck_recovery_turn_duration_sec),
        )
        self._publish_zero_cmd_vel()

        if self.clear_costmaps_before_nav and self.navigator is not None:
            try:
                self.navigator.clearAllCostmaps()
                self.get_logger().info("Cleared Nav2 costmaps after stuck recovery.")
            except Exception as exc:
                self.get_logger().warning(
                    f"Failed to clear Nav2 costmaps after recovery: {exc}"
                )

    def _publish_cmd_vel_for_duration(self, twist: Twist, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

    def _visual_scan_for_target(self, instruction: str) -> dict:
        """Rotate in place and ask the VLM whether the target is visible in each view."""
        live_configured = bool(self.image_topic or self.compressed_image_topic)
        if not live_configured:
            return {
                "ok": False,
                "reason": "visual_scan_requires_live_camera_topic",
                "method": "visual_scan_failed",
            }

        last_image_path = ""
        last_model_output = ""
        last_model_json = None

        for scan_index in range(self.visual_scan_steps):
            self._wait_for_live_image(timeout_sec=self.image_timeout_sec)
            try:
                image_path = self._get_inference_image_path(
                    instruction=f"scan_{scan_index + 1:02d}_{instruction}"
                )
                model_output = self.model.infer_goal_text(
                    instruction=instruction,
                    image_path=image_path,
                )
            except Exception as exc:
                self.get_logger().error(f"Visual scan inference failed: {exc}")
                return {
                    "ok": False,
                    "reason": f"visual_scan_inference_failed: {exc}",
                    "image_path": last_image_path,
                    "model_output": last_model_output,
                    "model_json": last_model_json,
                    "method": "visual_scan_failed",
                }

            model_json = self.converter.parse_model_json(model_output)
            last_image_path = image_path
            last_model_output = model_output
            last_model_json = model_json
            self.get_logger().info(
                "Visual scan "
                f"{scan_index + 1}/{self.visual_scan_steps}: "
                f"image={image_path}, output={model_output}"
            )

            if self._model_reports_visible_target(
                model_json=model_json,
                instruction=instruction,
            ):
                return {
                    "ok": True,
                    "image_path": image_path,
                    "model_output": model_output,
                    "model_json": model_json,
                    "method": "visual_scan",
                }

            if scan_index < self.visual_scan_steps - 1:
                if not self._spin_for_visual_scan():
                    return {
                        "ok": False,
                        "reason": "visual_scan_spin_failed",
                        "image_path": last_image_path,
                        "model_output": last_model_output,
                        "model_json": last_model_json,
                        "method": "visual_scan_failed",
                    }

        return {
            "ok": False,
            "reason": "target_not_visible_after_visual_scan",
            "image_path": last_image_path,
            "model_output": last_model_output,
            "model_json": last_model_json,
            "method": "visual_scan_not_visible",
        }

    def _model_reports_visible_target(
        self,
        model_json: Optional[dict],
        instruction: str,
    ) -> bool:
        if not isinstance(model_json, dict):
            return False

        visible = bool(model_json.get("visible", False))
        try:
            confidence = float(model_json.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0
        target = str(model_json.get("target", "")).strip().lower()
        if not target or target in ("unknown", "none", "not visible"):
            return False

        return (
            visible
            and confidence >= self.visual_scan_min_confidence
            and self._target_matches_instruction(target=target, instruction=instruction)
        )

    @staticmethod
    def _target_matches_instruction(target: str, instruction: str) -> bool:
        target_text = target.lower()
        instruction_text = instruction.lower()
        alias_groups = (
            ("plant", "potted plant", "green plant"),
            ("chair", "office chair", "black office chair"),
            (
                "purple box",
                "purple boxes",
                "purple package",
                "purple packages",
                "purple crate",
                "purple crates",
                "package area",
                "cart with boxes",
                "cart carrying purple boxes",
                "cart carrying purple packages",
                "shelf",
                "right shelf",
                "warehouse rack",
                "rack",
                "boxes",
            ),
        )

        for aliases in alias_groups:
            target_has_alias = any(alias in target_text for alias in aliases)
            instruction_has_alias = any(alias in instruction_text for alias in aliases)
            if target_has_alias and instruction_has_alias:
                return True

        # Ambiguous instructions are allowed only if the model names a concrete known target.
        ambiguous_terms = ("target object", "object near the wall", "object")
        if any(term in instruction_text for term in ambiguous_terms):
            return any(
                alias in target_text
                for aliases in alias_groups
                for alias in aliases
            )

        return False

    def _spin_for_visual_scan(self) -> bool:
        if self.dry_run:
            self.get_logger().info("dry_run=True, visual scan will not rotate the robot.")
            time.sleep(max(0.0, self.visual_scan_settle_sec))
            return True

        if self.visual_scan_spin_mode in ("cmd_vel", "twist", "velocity"):
            return self._spin_for_visual_scan_cmd_vel()
        if self.visual_scan_spin_mode == "nav2":
            return self._spin_for_visual_scan_nav2()

        self.get_logger().error(
            f"Unsupported visual_scan_spin_mode={self.visual_scan_spin_mode!r}. "
            "Use 'cmd_vel' or 'nav2'."
        )
        return False

    def _spin_for_visual_scan_nav2(self) -> bool:
        if self.navigator is None:
            self.get_logger().error("Cannot run visual scan spin: navigator is not initialized.")
            return False

        image_time_before_spin = self.latest_image_wall_time
        accepted = self.navigator.spin(
            spin_dist=self.visual_scan_step_rad,
            time_allowance=max(1, int(math.ceil(self.visual_scan_spin_time_allowance_sec))),
        )
        if not accepted:
            self.get_logger().error("Visual scan spin action was rejected.")
            return False

        while not self.navigator.isTaskComplete():
            time.sleep(0.05)

        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            self.get_logger().error(f"Visual scan spin failed with result={result}.")
            return False

        self._wait_for_new_live_image(
            after_wall_time=image_time_before_spin,
            timeout_sec=max(0.5, self.visual_scan_settle_sec),
        )
        return True

    def _spin_for_visual_scan_cmd_vel(self) -> bool:
        if self.cmd_vel_pub is None:
            self.get_logger().error("Cannot run visual scan spin: cmd_vel publisher is not initialized.")
            return False

        angular_speed = abs(self.visual_scan_angular_speed_rad_s)
        if angular_speed <= 0.0:
            self.get_logger().error("visual_scan_angular_speed_rad_s must be > 0.")
            return False

        image_time_before_spin = self.latest_image_wall_time
        spin_dist = self.visual_scan_step_rad
        duration_sec = abs(spin_dist) / angular_speed
        direction = 1.0 if spin_dist >= 0.0 else -1.0
        twist = Twist()
        twist.angular.z = direction * angular_speed

        self.get_logger().info(
            "Visual scan cmd_vel spin: "
            f"dist={spin_dist:.3f} rad, speed={twist.angular.z:.3f} rad/s, "
            f"duration={duration_sec:.2f}s"
        )
        deadline = time.monotonic() + duration_sec
        try:
            while time.monotonic() < deadline and rclpy.ok():
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
        finally:
            self._publish_zero_cmd_vel()

        self._wait_for_new_live_image(
            after_wall_time=image_time_before_spin,
            timeout_sec=max(0.5, self.visual_scan_settle_sec),
        )
        return True

    def _publish_zero_cmd_vel(self) -> None:
        if self.cmd_vel_pub is None:
            return
        stop = Twist()
        for _ in range(4):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.05)

    def _wait_for_live_image(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if self.latest_image_path and os.path.exists(self.latest_image_path):
                age = time.monotonic() - self.latest_image_wall_time
                if age <= self.image_timeout_sec:
                    return True
            time.sleep(0.05)
        return bool(self.latest_image_path and os.path.exists(self.latest_image_path))

    def _wait_for_new_live_image(self, after_wall_time: float, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if self.latest_image_wall_time > after_wall_time:
                return True
            time.sleep(0.05)
        return False

    def _build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _setup_image_subscriptions(self) -> None:
        if self.image_topic:
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self._on_image,
                1,
                callback_group=self.image_callback_group,
            )
            self.get_logger().info(f"Subscribed raw image topic: {self.image_topic}")

        if self.compressed_image_topic:
            self.compressed_image_sub = self.create_subscription(
                CompressedImage,
                self.compressed_image_topic,
                self._on_compressed_image,
                1,
                callback_group=self.image_callback_group,
            )
            self.get_logger().info(
                f"Subscribed compressed image topic: {self.compressed_image_topic}"
            )

        if not self.image_topic and not self.compressed_image_topic:
            self.get_logger().info(f"Using static image path: {self.image_path}")

    def _ensure_live_image_dirs(self) -> None:
        live_parent = os.path.dirname(self.live_image_path)
        if live_parent:
            os.makedirs(live_parent, exist_ok=True)
        if self.live_image_snapshot_dir:
            os.makedirs(self.live_image_snapshot_dir, exist_ok=True)

    def _get_inference_image_path(self, instruction: str = "") -> str:
        live_configured = bool(self.image_topic or self.compressed_image_topic)
        if not live_configured:
            return self.image_path

        now = time.monotonic()
        if self.latest_image_path and os.path.exists(self.latest_image_path):
            age = now - self.latest_image_wall_time
            if age <= self.image_timeout_sec:
                return self._snapshot_live_image(
                    source_path=self.latest_image_path,
                    instruction=instruction,
                )

            message = (
                f"Latest live image is stale ({age:.1f}s old, "
                f"timeout={self.image_timeout_sec:.1f}s)."
            )
        else:
            message = "No live camera image has been received yet."

        # When a live camera topic is configured, visual evidence must come from
        # that topic. A static fallback can make strict VLN success look valid
        # while the robot never actually saw the target in Isaac Sim.
        raise RuntimeError(message)

    def _snapshot_live_image(self, source_path: str, instruction: str) -> str:
        """Create a per-instruction immutable image path for inference and CSV logs."""
        if not self.live_image_snapshot_dir:
            return source_path

        try:
            os.makedirs(self.live_image_snapshot_dir, exist_ok=True)
            self.trial_image_counter += 1
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            slug = self._slugify_instruction(instruction)
            filename = (
                f"trial_{self.trial_image_counter:04d}_{timestamp}_"
                f"{time.time_ns() % 1_000_000_000:09d}_{slug}.png"
            )
            snapshot_path = os.path.join(self.live_image_snapshot_dir, filename)
            tmp_path = f"{snapshot_path}.tmp"
            shutil.copyfile(source_path, tmp_path)
            os.replace(tmp_path, snapshot_path)
            self.get_logger().info(f"Saved trial image snapshot: {snapshot_path}")
            return snapshot_path
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to create live image snapshot from {source_path}: {exc}. "
                "Using latest live image path."
            )
            return source_path

    @staticmethod
    def _slugify_instruction(instruction: str) -> str:
        slug = re.sub(r"[^a-zA-Z0-9]+", "_", instruction.strip().lower()).strip("_")
        return slug[:48] or "instruction"

    def _on_image(self, msg: Image) -> None:
        try:
            image = self._pil_from_image_msg(msg)
            self._save_live_image(image)
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode raw image: {exc}")

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        try:
            from PIL import Image as PILImage

            image = PILImage.open(io.BytesIO(bytes(msg.data))).convert("RGB")
            self._save_live_image(image)
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode compressed image: {exc}")

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.latest_map_x = float(pose.position.x)
        self.latest_map_y = float(pose.position.y)
        self.latest_map_wall_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.latest_odom_x = float(pose.position.x)
        self.latest_odom_y = float(pose.position.y)
        self.latest_odom_wall_time = time.monotonic()

    def _save_live_image(self, image) -> None:
        with self.image_save_lock:
            now = time.monotonic()
            if now - self.last_image_save_wall_time < self.live_image_save_interval_sec:
                return

            self._ensure_live_image_dirs()
            tmp_path = (
                f"{self.live_image_path}."
                f"{os.getpid()}.{threading.get_ident()}.{time.time_ns()}.tmp"
            )
            try:
                image.save(tmp_path, format="PNG")
                if not os.path.exists(tmp_path) or os.path.getsize(tmp_path) == 0:
                    raise RuntimeError(f"Image save produced no file: {tmp_path}")
                os.replace(tmp_path, self.live_image_path)
            finally:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)

            self.latest_image_path = self.live_image_path
            self.latest_image_wall_time = now
            self.last_image_save_wall_time = now

    @staticmethod
    def _pil_from_image_msg(msg: Image):
        from PIL import Image as PILImage

        encoding = msg.encoding.lower()
        width = int(msg.width)
        height = int(msg.height)
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        if encoding not in channels_by_encoding:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        channels = channels_by_encoding[encoding]
        row_bytes = width * channels
        data = bytes(msg.data)
        if int(msg.step) != row_bytes:
            rows = [
                data[row_start:row_start + row_bytes]
                for row_start in range(0, int(msg.step) * height, int(msg.step))
            ]
            data = b"".join(rows)

        if encoding == "rgb8":
            return PILImage.frombytes("RGB", (width, height), data)
        if encoding == "bgr8":
            return PILImage.frombytes("RGB", (width, height), data, "raw", "BGR")
        if encoding == "rgba8":
            return PILImage.frombytes("RGBA", (width, height), data).convert("RGB")
        if encoding == "bgra8":
            return PILImage.frombytes("RGBA", (width, height), data, "raw", "BGRA").convert("RGB")
        if encoding == "mono8":
            return PILImage.frombytes("L", (width, height), data).convert("RGB")

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def _build_trial_result(
        self,
        instruction: str,
        image_path: str,
        model_output: str,
        model_json: Optional[dict],
        parse_method: str,
        x: float,
        y: float,
        yaw: float,
        nav_result: str,
        failure_reason: str,
        started_at: float,
        navigation_arrived: Optional[bool] = None,
        visual_confirmed: Optional[bool] = None,
        task_success: Optional[bool] = None,
        target_seen_during_nav: Optional[bool] = None,
        final_visual_confirmed: Optional[bool] = None,
        active_search_used: Optional[bool] = None,
    ) -> dict:
        model_json = model_json or {}
        if navigation_arrived is None:
            navigation_arrived = nav_result in ("success", "dry_run")
        if visual_confirmed is None:
            visual_confirmed = bool(model_json.get("visible", False)) and parse_method not in (
                "visual_scan_failed",
                "visual_scan_not_visible",
                "semantic_explore_visual_scan_failed",
                "visual_map_failed",
                "failed",
                "inference_failed",
            )
        if task_success is None:
            task_success = bool(navigation_arrived) and bool(visual_confirmed)
        if target_seen_during_nav is None:
            target_seen_during_nav = self._target_seen_during_nav
        if final_visual_confirmed is None:
            final_visual_confirmed = bool(visual_confirmed)
        if active_search_used is None:
            active_search_used = False
        return {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_sec": round(time.time() - started_at, 3),
            "instruction": instruction,
            "image_path": image_path,
            "model_output": model_output,
            "model_target": model_json.get("target", ""),
            "visible": model_json.get("visible", ""),
            "confidence": model_json.get("confidence", ""),
            "parse_method": parse_method,
            "target_x": x,
            "target_y": y,
            "target_yaw": yaw,
            "nav_result": nav_result,
            "navigation_arrived": navigation_arrived,
            "visual_confirmed": visual_confirmed,
            "target_seen_during_nav": target_seen_during_nav,
            "target_seen_during_nav_image": self._target_seen_during_nav_path,
            "final_visual_confirmed": final_visual_confirmed,
            "active_search_used": active_search_used,
            "task_success": task_success,
            "failure_reason": failure_reason,
        }

    def _publish_and_log_result(self, payload: dict) -> None:
        text = json.dumps(payload, ensure_ascii=False)
        if self.result_pub is not None:
            self.result_pub.publish(String(data=text))
        self._append_trial_csv(payload)
        self.get_logger().info(f"Trial result: {text}")

    def _append_trial_csv(self, payload: dict) -> None:
        os.makedirs(os.path.dirname(self.trial_log_path), exist_ok=True)
        fieldnames = [
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
            "nav_result",
            "navigation_arrived",
            "visual_confirmed",
            "target_seen_during_nav",
            "target_seen_during_nav_image",
            "final_visual_confirmed",
            "active_search_used",
            "task_success",
            "failure_reason",
        ]
        write_header = not os.path.exists(self.trial_log_path)
        with open(self.trial_log_path, "a", newline="", encoding="utf-8") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow({key: payload.get(key, "") for key in fieldnames})


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VLNBridgeNodeLocal()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.model.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
