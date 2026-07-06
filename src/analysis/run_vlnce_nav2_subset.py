#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[2]
BRIDGE_SRC = ROOT / "src" / "vln_nav2_bridge"
sys.path.insert(0, str(BRIDGE_SRC))

from vln_nav2_bridge.node6_map_preflight import DEFAULT_MAP_YAML, SimpleOccupancyMap


Pose = Tuple[float, float, float]


@dataclass(frozen=True)
class TrialSpec:
    trial_id: int
    instruction: str
    target_key: str
    group: str


@dataclass(frozen=True)
class GoalPlan:
    policy: str
    target_key: str
    waypoints: Tuple[Pose, ...]
    goal_source: str
    candidate_poses: Tuple[Pose, ...] = ()
    selected_candidate_index: int = -1
    candidate_selection_reason: str = ""
    candidate_safety: str = ""
    candidate_evaluations: Tuple[Dict[str, object], ...] = ()
    candidate_path_diagnostics: str = ""
    selected_candidate_path_length_m: object = ""
    route_planner_result: str = ""
    route_planner_path_length_m: object = ""
    route_execution_mode: str = ""
    planner_start_source: str = ""
    planner_start_x: object = ""
    planner_start_y: object = ""
    planner_start_yaw: object = ""


@dataclass(frozen=True)
class ExecutionResult:
    nav_result: str
    timed_out: bool
    stuck: bool
    recovery_count: object = ""
    waypoint_status: str = ""


TRIALS: Tuple[TrialSpec, ...] = (
    TrialSpec(1, "Go to the plant.", "plant", "near"),
    TrialSpec(2, "Navigate to the green plant near the chair.", "plant", "near"),
    TrialSpec(3, "Go to the black office chair.", "chair", "near"),
    TrialSpec(4, "Go to the purple boxes.", "shelf_package_area", "long"),
    TrialSpec(5, "Move to the right shelf with purple boxes.", "shelf_package_area", "long"),
    TrialSpec(6, "Move to the package area.", "shelf_package_area", "long"),
)

OBJECT_CENTERS: Dict[str, Pose] = {
    "plant": (-0.43, -2.92, 0.0),
    "chair": (-0.54, -0.69, 1.57),
    "shelf_package_area": (-6.78, 10.96, 0.0),
}

OBSERVATION_POSES: Dict[str, Pose] = {
    "plant": (0.35, -2.92, math.pi),
    "chair": (0.20, -0.69, math.pi),
    "shelf_package_area": (-6.30, 10.80, math.pi),
}

CANDIDATE_OBSERVATION_XY: Dict[str, Tuple[Tuple[float, float], ...]] = {
    "plant": (
        (0.35, -2.92),
        (-0.45, -2.15),
        (-1.05, -2.92),
        (0.00, -3.55),
    ),
    "chair": (
        (-0.20, -1.30),
        (0.45, -1.20),
        (-0.90, -1.10),
        (0.20, -0.69),
    ),
    "shelf_package_area": (
        (-6.30, 10.80),
        (-5.80, 10.20),
        (-6.80, 10.20),
        (-5.60, 11.20),
    ),
}

WAYPOINT_SEQUENCES: Dict[str, Tuple[Pose, ...]] = {
    "plant": ((0.35, -2.92, math.pi),),
    "chair": ((0.20, -0.69, math.pi),),
    "shelf_package_area": (
        (-1.20, 3.20, 1.95),
        (-3.80, 7.20, 2.15),
        (-6.30, 10.80, math.pi),
    ),
}

POLICIES = ("object_center", "observation_pose", "candidate_observation_pose", "waypoint_sequence")
SMOKE_TEST_POLICY = "single_goal"


def build_goal_plan(
    policy: str,
    target_key: str,
    occupancy: Optional[SimpleOccupancyMap] = None,
    args: Optional[argparse.Namespace] = None,
    start_pose: Optional[Pose] = None,
) -> GoalPlan:
    if policy == "object_center":
        return GoalPlan(policy, target_key, (OBJECT_CENTERS[target_key],), "object_center")
    if policy == "observation_pose":
        return GoalPlan(policy, target_key, (OBSERVATION_POSES[target_key],), "observation_pose")
    if policy == "candidate_observation_pose":
        if args is None:
            raise ValueError("candidate_observation_pose requires parsed args.")
        return build_candidate_observation_plan(
            target_key=target_key,
            occupancy=occupancy,
            safe_radius_m=args.safe_radius_m,
            min_free_ratio=args.min_free_ratio,
            selection_mode=args.candidate_selection_mode,
            start_pose=start_pose,
        )
    if policy == "waypoint_sequence":
        return GoalPlan(policy, target_key, WAYPOINT_SEQUENCES[target_key], "waypoint_sequence")
    raise ValueError(f"Unsupported policy: {policy}")


def pose_distance(a: Pose, b: Pose) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_length(poses: Sequence[Pose], start: Pose = (0.0, 0.0, 0.0)) -> float:
    total = 0.0
    prev = start
    for pose in poses:
        total += pose_distance(prev, pose)
        prev = pose
    return total


def spl_score(success: bool, shortest_path_m: object, trajectory_length_m: object) -> object:
    if not success:
        return 0.0
    try:
        shortest = float(shortest_path_m)
        trajectory = float(trajectory_length_m)
    except (TypeError, ValueError):
        return ""
    if shortest <= 0.0:
        return ""
    return round(shortest / max(shortest, trajectory), 3)


def yaw_toward(source_x: float, source_y: float, target_key: str) -> float:
    target = OBJECT_CENTERS[target_key]
    return math.atan2(target[1] - source_y, target[0] - source_x)


def candidate_observation_poses(target_key: str) -> Tuple[Pose, ...]:
    return tuple(
        (x, y, yaw_toward(x, y, target_key))
        for x, y in CANDIDATE_OBSERVATION_XY[target_key]
    )


def build_candidate_observation_plan(
    target_key: str,
    occupancy: Optional[SimpleOccupancyMap],
    safe_radius_m: float,
    min_free_ratio: float,
    selection_mode: str,
    start_pose: Optional[Pose],
) -> GoalPlan:
    candidates = candidate_observation_poses(target_key)
    evaluations = evaluate_candidate_safety(
        candidates=candidates,
        occupancy=occupancy,
        safe_radius_m=safe_radius_m,
        min_free_ratio=min_free_ratio,
    )
    selected_index, reason = select_candidate_index(
        candidates=candidates,
        evaluations=evaluations,
        selection_mode=selection_mode,
        start_pose=start_pose,
    )
    return GoalPlan(
        policy="candidate_observation_pose",
        target_key=target_key,
        waypoints=(candidates[selected_index],),
        goal_source=f"candidate_observation_pose:{selection_mode}",
        candidate_poses=candidates,
        selected_candidate_index=selected_index,
        candidate_selection_reason=reason,
        candidate_safety=format_candidate_evaluations(evaluations),
        candidate_evaluations=tuple(evaluations),
    )


def replace_candidate_selection(
    plan: GoalPlan,
    selected_index: int,
    reason: str,
    path_diagnostics: str,
    selected_path_length_m: object,
) -> GoalPlan:
    return replace(
        plan,
        waypoints=(plan.candidate_poses[selected_index],),
        selected_candidate_index=selected_index,
        candidate_selection_reason=reason,
        candidate_path_diagnostics=path_diagnostics,
        selected_candidate_path_length_m=selected_path_length_m,
    )


def replace_route_diagnostics(
    plan: GoalPlan,
    route_planner_result: str,
    route_planner_path_length_m: object,
    route_execution_mode: str,
) -> GoalPlan:
    return replace(
        plan,
        route_planner_result=route_planner_result,
        route_planner_path_length_m=route_planner_path_length_m,
        route_execution_mode=route_execution_mode,
    )


def replace_planner_start(plan: GoalPlan, source: str, pose: Optional[Pose]) -> GoalPlan:
    if pose is None:
        return replace(
            plan,
            planner_start_source=source,
            planner_start_x="",
            planner_start_y="",
            planner_start_yaw="",
        )
    return replace(
        plan,
        planner_start_source=source,
        planner_start_x=round(pose[0], 3),
        planner_start_y=round(pose[1], 3),
        planner_start_yaw=round(pose[2], 3),
    )


def evaluate_candidate_safety(
    candidates: Sequence[Pose],
    occupancy: Optional[SimpleOccupancyMap],
    safe_radius_m: float,
    min_free_ratio: float,
) -> List[Dict[str, object]]:
    evaluations: List[Dict[str, object]] = []
    for index, pose in enumerate(candidates):
        if occupancy is None:
            evaluations.append(
                {
                    "index": index,
                    "center_state": "not_checked",
                    "free_ratio": "",
                    "safe": True,
                }
            )
            continue

        check = occupancy.check_pose(
            name=f"candidate_{index}",
            x=pose[0],
            y=pose[1],
            yaw=pose[2],
            radius_m=safe_radius_m,
        )
        evaluations.append(
            {
                "index": index,
                "center_state": check.center_state,
                "free_ratio": round(check.free_ratio, 3),
                "safe": check.ok(min_free_ratio),
            }
        )
    return evaluations


def select_candidate_index(
    candidates: Sequence[Pose],
    evaluations: Sequence[Dict[str, object]],
    selection_mode: str,
    start_pose: Optional[Pose],
) -> Tuple[int, str]:
    safe_indices = [
        int(evaluation["index"])
        for evaluation in evaluations
        if bool(evaluation.get("safe", False))
    ]
    if safe_indices:
        if selection_mode == "planner_shortest":
            return safe_indices[0], f"planner_shortest_unavailable_mock_first_static_safe:{safe_indices[0]}"
        if selection_mode == "nearest_safe" and start_pose is not None:
            selected = min(safe_indices, key=lambda index: pose_distance(start_pose, candidates[index]))
            return selected, f"nearest_safe_from_start:{selected}"
        if selection_mode == "highest_free_ratio":
            selected = max(
                safe_indices,
                key=lambda index: float(evaluations[index].get("free_ratio") or 0.0),
            )
            return selected, f"highest_free_ratio:{selected}"
        return safe_indices[0], f"first_static_safe:{safe_indices[0]}"

    if evaluations:
        selected = max(
            range(len(evaluations)),
            key=lambda index: float(evaluations[index].get("free_ratio") or 0.0),
        )
        return selected, f"no_safe_candidate_fallback_best_free_ratio:{selected}"

    return 0, "no_candidates_fallback:0"


def check_goal_safety(
    occupancy: SimpleOccupancyMap,
    pose: Pose,
    radius_m: float,
    min_free_ratio: float,
) -> Dict[str, object]:
    check = occupancy.check_pose("goal", pose[0], pose[1], pose[2], radius_m)
    return {
        "goal_center_state": check.center_state,
        "goal_free_ratio": round(check.free_ratio, 3),
        "goal_safe": check.ok(min_free_ratio),
    }


def make_mock_row(
    trial: TrialSpec,
    plan: GoalPlan,
    occupancy: SimpleOccupancyMap,
    radius_m: float,
    min_free_ratio: float,
    success_radius_m: float,
) -> Dict[str, object]:
    final_goal = plan.waypoints[-1]
    safety = check_goal_safety(occupancy, final_goal, radius_m, min_free_ratio)
    planned_length = path_length(plan.waypoints)
    return {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "mode": "mock",
        "trial_id": trial.trial_id,
        "instruction": trial.instruction,
        "group": trial.group,
        "target_key": trial.target_key,
        "policy": plan.policy,
        "goal_source": plan.goal_source,
        "candidate_count": len(plan.candidate_poses),
        "candidate_selected_index": plan.selected_candidate_index,
        "candidate_selection_reason": plan.candidate_selection_reason,
        "candidate_poses": format_waypoints(plan.candidate_poses),
        "candidate_safety": plan.candidate_safety,
        "candidate_path_diagnostics": plan.candidate_path_diagnostics,
        "selected_candidate_path_length_m": plan.selected_candidate_path_length_m,
        "route_planner_result": plan.route_planner_result,
        "route_planner_path_length_m": plan.route_planner_path_length_m,
        "route_execution_mode": plan.route_execution_mode,
        "planner_start_source": plan.planner_start_source,
        "planner_start_x": plan.planner_start_x,
        "planner_start_y": plan.planner_start_y,
        "planner_start_yaw": plan.planner_start_yaw,
        "waypoint_count": len(plan.waypoints),
        "waypoints": format_waypoints(plan.waypoints),
        "target_x": round(final_goal[0], 3),
        "target_y": round(final_goal[1], 3),
        "target_yaw": round(final_goal[2], 3),
        "planned_path_length_m": round(planned_length, 3),
        "nav_result": "mock_not_run",
        "duration_sec": "",
        "trajectory_length_m": round(planned_length, 3),
        "spl": "",
        "final_pose_source": "mock",
        "final_x": "",
        "final_y": "",
        "final_yaw": "",
        "final_navigation_error_m": "",
        "physical_arrival": "",
        "success_radius_m": success_radius_m,
        "timeout": False,
        "stuck": False,
        "recovery_count": 0,
        "waypoint_status": "",
        "amcl_odom_disagreement_m": "",
        **safety,
    }


def format_waypoints(waypoints: Iterable[Pose]) -> str:
    return ";".join(f"{x:.3f},{y:.3f},{yaw:.3f}" for x, y, yaw in waypoints)


def format_candidate_evaluations(evaluations: Iterable[Dict[str, object]]) -> str:
    return ";".join(
        (
            f"{evaluation.get('index')}:"
            f"center={evaluation.get('center_state')}:"
            f"free={evaluation.get('free_ratio')}:"
            f"safe={evaluation.get('safe')}"
        )
        for evaluation in evaluations
    )


def format_candidate_path_diagnostics(evaluations: Iterable[Dict[str, object]]) -> str:
    return ";".join(
        (
            f"{evaluation.get('index')}:"
            f"path={evaluation.get('path_found')}:"
            f"length={evaluation.get('path_length_m')}:"
            f"reason={evaluation.get('reason')}"
        )
        for evaluation in evaluations
    )


def requested_trial_specs(args: argparse.Namespace) -> List[TrialSpec]:
    if args.single_goal is not None:
        return [TrialSpec(0, f"Smoke test to {args.single_goal_label}.", args.single_goal_label, "smoke")]

    return [
        trial
        for trial in TRIALS
        if args.trial_ids is None or trial.trial_id in args.trial_ids
    ]


def requested_trial_plans(
    args: argparse.Namespace,
    occupancy: Optional[SimpleOccupancyMap] = None,
    start_pose: Optional[Pose] = None,
) -> List[Tuple[TrialSpec, GoalPlan]]:
    if args.single_goal is not None:
        goal = (args.single_goal[0], args.single_goal[1], args.single_goal[2])
        trial = requested_trial_specs(args)[0]
        return [
            (
                trial,
                GoalPlan(SMOKE_TEST_POLICY, trial.target_key, (goal,), "manual_single_goal"),
            )
        ]

    return [
        (
            trial,
            build_goal_plan(
                policy=policy,
                target_key=trial.target_key,
                occupancy=occupancy,
                args=args,
                start_pose=start_pose,
            ),
        )
        for trial in requested_trial_specs(args)
        for policy in args.policies
    ]


def write_rows(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "timestamp",
        "mode",
        "trial_id",
        "instruction",
        "group",
        "target_key",
        "policy",
        "goal_source",
        "candidate_count",
        "candidate_selected_index",
        "candidate_selection_reason",
        "candidate_poses",
        "candidate_safety",
        "candidate_path_diagnostics",
        "selected_candidate_path_length_m",
        "route_planner_result",
        "route_planner_path_length_m",
        "route_execution_mode",
        "planner_start_source",
        "planner_start_x",
        "planner_start_y",
        "planner_start_yaw",
        "waypoint_count",
        "waypoints",
        "target_x",
        "target_y",
        "target_yaw",
        "planned_path_length_m",
        "nav_result",
        "duration_sec",
        "trajectory_length_m",
        "spl",
        "final_pose_source",
        "final_x",
        "final_y",
        "final_yaw",
        "amcl_x",
        "amcl_y",
        "amcl_yaw",
        "final_navigation_error_m",
        "physical_arrival",
        "success_radius_m",
        "timeout",
        "stuck",
        "recovery_count",
        "waypoint_status",
        "amcl_odom_disagreement_m",
        "cmd_vel_nav_count",
        "cmd_vel_nav_nonzero_count",
        "cmd_vel_nav_max_linear_x",
        "cmd_vel_nav_max_angular_z",
        "cmd_vel_count",
        "cmd_vel_nonzero_count",
        "cmd_vel_max_linear_x",
        "cmd_vel_max_angular_z",
        "goal_center_state",
        "goal_free_ratio",
        "goal_safe",
    ]
    with path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def run_mock(args: argparse.Namespace) -> List[Dict[str, object]]:
    occupancy = SimpleOccupancyMap.from_yaml(args.map_yaml)
    rows: List[Dict[str, object]] = []
    for trial, plan in requested_trial_plans(
        args,
        occupancy=occupancy,
        start_pose=(0.0, 0.0, 0.0),
    ):
        rows.append(
            make_mock_row(
                trial=trial,
                plan=plan,
                occupancy=occupancy,
                radius_m=args.safe_radius_m,
                min_free_ratio=args.min_free_ratio,
                success_radius_m=args.success_radius_m,
            )
        )
    write_rows(Path(args.output), rows)
    return rows


def run_online(args: argparse.Namespace) -> List[Dict[str, object]]:
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    class RuntimeWatcher(Node):
        def __init__(
            self,
            odom_topic: str,
            amcl_pose_topic: str,
            cmd_vel_nav_topic: str,
            cmd_vel_topic: str,
        ) -> None:
            super().__init__("vlnce_subset_runtime_watcher")
            self.pose: Optional[Pose] = None
            self.amcl_pose: Optional[Pose] = None
            self.distance_traveled = 0.0
            self._last_xy: Optional[Tuple[float, float]] = None
            self._cmd_stats = {
                "cmd_vel_nav": {"count": 0, "nonzero_count": 0, "max_linear_x": 0.0, "max_angular_z": 0.0},
                "cmd_vel": {"count": 0, "nonzero_count": 0, "max_linear_x": 0.0, "max_angular_z": 0.0},
            }
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
            self.create_subscription(PoseWithCovarianceStamped, amcl_pose_topic, self._on_amcl_pose, 10)
            self.create_subscription(Twist, cmd_vel_nav_topic, self._on_cmd_vel_nav, 10)
            self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)

        def _on_odom(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            xy = (float(p.x), float(p.y))
            if self._last_xy is not None:
                self.distance_traveled += math.hypot(
                    xy[0] - self._last_xy[0],
                    xy[1] - self._last_xy[1],
                )
            self._last_xy = xy
            self.pose = (xy[0], xy[1], yaw)

        def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.amcl_pose = (float(p.x), float(p.y), yaw)

        def amcl_odom_disagreement(self) -> object:
            if self.pose is None or self.amcl_pose is None:
                return ""
            return round(pose_distance(self.pose, self.amcl_pose), 3)

        def _update_cmd_stats(self, name: str, msg: Twist) -> None:
            stats = self._cmd_stats[name]
            linear_x = abs(float(msg.linear.x))
            angular_z = abs(float(msg.angular.z))
            stats["count"] += 1
            if linear_x > 1e-4 or angular_z > 1e-4:
                stats["nonzero_count"] += 1
            stats["max_linear_x"] = max(stats["max_linear_x"], linear_x)
            stats["max_angular_z"] = max(stats["max_angular_z"], angular_z)

        def _on_cmd_vel_nav(self, msg: Twist) -> None:
            self._update_cmd_stats("cmd_vel_nav", msg)

        def _on_cmd_vel(self, msg: Twist) -> None:
            self._update_cmd_stats("cmd_vel", msg)

        def cmd_snapshot(self) -> Dict[str, Dict[str, float]]:
            return {name: dict(values) for name, values in self._cmd_stats.items()}

        def cmd_delta(self, start: Dict[str, Dict[str, float]]) -> Dict[str, object]:
            delta: Dict[str, object] = {}
            for name, current in self._cmd_stats.items():
                initial = start[name]
                delta[f"{name}_count"] = int(current["count"] - initial["count"])
                delta[f"{name}_nonzero_count"] = int(current["nonzero_count"] - initial["nonzero_count"])
                delta[f"{name}_max_linear_x"] = round(current["max_linear_x"], 3)
                delta[f"{name}_max_angular_z"] = round(current["max_angular_z"], 3)
            return delta

    def make_pose(navigator: BasicNavigator, pose: Pose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = navigator.get_clock().now().to_msg()
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.orientation.w = math.cos(pose[2] / 2.0)
        return msg

    def nav_result_text(result_code) -> str:
        if result_code == TaskResult.SUCCEEDED:
            return "success"
        if result_code == TaskResult.CANCELED:
            return "canceled"
        return "failed"

    def path_msg_length(path_msg) -> Optional[float]:
        poses = getattr(path_msg, "poses", None)
        if not poses or len(poses) < 2:
            return None
        total = 0.0
        prev = poses[0].pose.position
        for stamped in poses[1:]:
            current = stamped.pose.position
            total += math.hypot(float(current.x) - float(prev.x), float(current.y) - float(prev.y))
            prev = current
        return total

    def select_planner_start_pose() -> Tuple[Optional[Pose], str]:
        if args.planner_start_source == "amcl":
            return watcher.amcl_pose, "amcl" if watcher.amcl_pose is not None else "amcl_unavailable"
        if args.planner_start_source == "odom":
            return watcher.pose, "odom" if watcher.pose is not None else "odom_unavailable"
        if watcher.amcl_pose is not None:
            return watcher.amcl_pose, "amcl"
        if watcher.pose is not None:
            return watcher.pose, "odom"
        return None, "unavailable"

    def wait_for_initial_pose() -> None:
        deadline = time.monotonic() + args.initial_pose_wait_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(watcher, timeout_sec=0.1)
            pose, _source = select_planner_start_pose()
            if pose is not None:
                return

    def spin_runtime_callbacks(duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(watcher, timeout_sec=0.05)

    def attach_planner_start(plan: GoalPlan, pose: Optional[Pose], source: str) -> GoalPlan:
        return replace_planner_start(plan, source, pose)

    def evaluate_candidate_paths_with_nav2(plan: GoalPlan) -> GoalPlan:
        if (
            plan.policy != "candidate_observation_pose"
            or args.candidate_selection_mode != "planner_shortest"
            or not plan.candidate_poses
        ):
            pose, source = select_planner_start_pose()
            return attach_planner_start(plan, pose, source)

        start_pose, start_source = select_planner_start_pose()
        plan = attach_planner_start(plan, start_pose, start_source)
        if start_pose is None:
            return plan

        start_msg = make_pose(navigator, start_pose)
        diagnostics: List[Dict[str, object]] = []
        best_index = -1
        best_length = float("inf")
        safe_indices = {
            int(evaluation["index"])
            for evaluation in plan.candidate_evaluations
            if bool(evaluation.get("safe", False))
        }
        for index, candidate in enumerate(plan.candidate_poses):
            if plan.candidate_evaluations and index not in safe_indices:
                diagnostics.append(
                    {
                        "index": index,
                        "path_found": False,
                        "path_length_m": "",
                        "reason": "skipped_static_unsafe",
                    }
                )
                continue
            goal_msg = make_pose(navigator, candidate)
            path_msg = None
            path_length_value: Optional[float] = None
            try:
                path_msg = navigator.getPath(start_msg, goal_msg, use_start=True)
                path_length_value = path_msg_length(path_msg)
            except Exception as exc:
                diagnostics.append(
                    {
                        "index": index,
                        "path_found": False,
                        "path_length_m": "",
                        "reason": f"getPath_failed:{exc}",
                    }
                )
                continue

            path_found = path_msg is not None and path_length_value is not None
            diagnostics.append(
                {
                    "index": index,
                    "path_found": path_found,
                    "path_length_m": "" if path_length_value is None else round(path_length_value, 3),
                    "reason": "" if path_found else "no_path",
                }
            )
            if path_found and path_length_value < best_length:
                best_index = index
                best_length = path_length_value

        if best_index >= 0:
            return replace_candidate_selection(
                plan=plan,
                selected_index=best_index,
                reason=f"planner_shortest:{best_index}",
                path_diagnostics=format_candidate_path_diagnostics(diagnostics),
                selected_path_length_m=round(best_length, 3),
            )

        fallback_index = plan.selected_candidate_index if plan.selected_candidate_index >= 0 else 0
        if safe_indices and fallback_index not in safe_indices:
            fallback_index = min(safe_indices)
        return replace_candidate_selection(
            plan=plan,
            selected_index=fallback_index,
            reason=f"planner_shortest_no_valid_path_fallback:{fallback_index}",
            path_diagnostics=format_candidate_path_diagnostics(diagnostics),
            selected_path_length_m="",
        )

    def evaluate_route_with_nav2(plan: GoalPlan) -> GoalPlan:
        start_pose, start_source = select_planner_start_pose()
        plan = attach_planner_start(plan, start_pose, start_source)
        if not args.route_plan_diagnostics:
            return replace_route_diagnostics(
                plan=plan,
                route_planner_result="not_requested",
                route_planner_path_length_m="",
                route_execution_mode=route_execution_mode(plan),
            )
        if start_pose is None:
            return replace_route_diagnostics(
                plan=plan,
                route_planner_result="no_start_pose",
                route_planner_path_length_m="",
                route_execution_mode=route_execution_mode(plan),
            )

        start_msg = make_pose(navigator, start_pose)
        goal_msgs = [make_pose(navigator, waypoint) for waypoint in plan.waypoints]
        if not goal_msgs:
            return replace_route_diagnostics(
                plan=plan,
                route_planner_result="no_waypoints",
                route_planner_path_length_m="",
                route_execution_mode=route_execution_mode(plan),
            )

        try:
            if len(goal_msgs) > 1:
                path_msg = navigator.getPathThroughPoses(start_msg, goal_msgs, use_start=True)
            else:
                path_msg = navigator.getPath(start_msg, goal_msgs[0], use_start=True)
            route_length = path_msg_length(path_msg)
        except Exception as exc:
            return replace_route_diagnostics(
                plan=plan,
                route_planner_result=f"getPath_failed:{exc}",
                route_planner_path_length_m="",
                route_execution_mode=route_execution_mode(plan),
            )

        if path_msg is None or route_length is None:
            return replace_route_diagnostics(
                plan=plan,
                route_planner_result="no_path",
                route_planner_path_length_m="",
                route_execution_mode=route_execution_mode(plan),
            )

        return replace_route_diagnostics(
            plan=plan,
            route_planner_result="path_found",
            route_planner_path_length_m=round(route_length, 3),
            route_execution_mode=route_execution_mode(plan),
        )

    def route_execution_mode(plan: GoalPlan) -> str:
        if plan.policy == "waypoint_sequence" and args.waypoint_execution_mode == "through_poses":
            return "goThroughPoses"
        return "sequential_goToPose"

    def recovery_count_from_feedback() -> object:
        feedback = navigator.getFeedback()
        if feedback is None:
            return ""
        value = getattr(feedback, "number_of_recoveries", "")
        if value == "":
            return ""
        return int(value)

    def poses_remaining_from_feedback() -> object:
        feedback = navigator.getFeedback()
        if feedback is None:
            return ""
        value = getattr(feedback, "number_of_poses_remaining", "")
        if value == "":
            return ""
        return int(value)

    def format_waypoint_status(statuses: Sequence[Dict[str, object]]) -> str:
        return ";".join(
            (
                f"{status.get('index')}:"
                f"result={status.get('result')}:"
                f"recoveries={status.get('recoveries')}:"
                f"poses_remaining={status.get('poses_remaining')}"
            )
            for status in statuses
        )

    def reset_nav_feedback() -> None:
        if hasattr(navigator, "feedback"):
            navigator.feedback = None

    def check_progress_to_goal(
        waypoint: Pose,
        start_time: float,
        best_distance: float,
        last_progress_time: float,
    ) -> Tuple[float, float, bool, bool]:
        rclpy.spin_once(watcher, timeout_sec=0.05)
        if watcher.pose is not None:
            distance_to_waypoint = pose_distance(watcher.pose, waypoint)
            if distance_to_waypoint + args.stuck_min_progress_m < best_distance:
                best_distance = distance_to_waypoint
                last_progress_time = time.monotonic()

        timed_out = time.monotonic() - start_time > args.timeout_sec
        stuck = time.monotonic() - last_progress_time > args.stuck_timeout_sec
        return best_distance, last_progress_time, timed_out, stuck

    def execute_plan(plan: GoalPlan, start_time: float) -> ExecutionResult:
        if (
            plan.policy == "waypoint_sequence"
            and args.waypoint_execution_mode == "through_poses"
            and len(plan.waypoints) > 1
        ):
            poses = [make_pose(navigator, waypoint) for waypoint in plan.waypoints]
            reset_nav_feedback()
            accepted = navigator.goThroughPoses(poses)
            if not accepted:
                return ExecutionResult(
                    nav_result="failed",
                    timed_out=False,
                    stuck=False,
                    recovery_count="",
                    waypoint_status=format_waypoint_status(
                        [
                            {
                                "index": "all",
                                "result": "rejected",
                                "recoveries": "",
                                "poses_remaining": len(plan.waypoints),
                            }
                        ]
                    ),
                )
            best_distance = float("inf")
            last_progress_time = time.monotonic()
            final_waypoint = plan.waypoints[-1]
            max_recoveries: object = ""
            min_poses_remaining: object = len(plan.waypoints)
            while not navigator.isTaskComplete():
                best_distance, last_progress_time, timed_out, stuck = check_progress_to_goal(
                    waypoint=final_waypoint,
                    start_time=start_time,
                    best_distance=best_distance,
                    last_progress_time=last_progress_time,
                )
                current_recoveries = recovery_count_from_feedback()
                if current_recoveries != "":
                    max_recoveries = max(int(max_recoveries or 0), int(current_recoveries))
                current_poses_remaining = poses_remaining_from_feedback()
                if current_poses_remaining != "":
                    min_poses_remaining = min(int(min_poses_remaining), int(current_poses_remaining))
                if timed_out:
                    navigator.cancelTask()
                    return ExecutionResult(
                        nav_result="timeout",
                        timed_out=True,
                        stuck=False,
                        recovery_count=max_recoveries,
                        waypoint_status=format_waypoint_status(
                            [
                                {
                                    "index": "through_poses",
                                    "result": "timeout",
                                    "recoveries": max_recoveries,
                                    "poses_remaining": min_poses_remaining,
                                }
                            ]
                        ),
                    )
                if stuck:
                    navigator.cancelTask()
                    return ExecutionResult(
                        nav_result="stuck",
                        timed_out=False,
                        stuck=True,
                        recovery_count=max_recoveries,
                        waypoint_status=format_waypoint_status(
                            [
                                {
                                    "index": "through_poses",
                                    "result": "stuck",
                                    "recoveries": max_recoveries,
                                    "poses_remaining": min_poses_remaining,
                                }
                            ]
                        ),
                    )
            nav_result = nav_result_text(navigator.getResult())
            final_recoveries = recovery_count_from_feedback()
            if final_recoveries != "":
                max_recoveries = max(int(max_recoveries or 0), int(final_recoveries))
            final_poses_remaining = poses_remaining_from_feedback()
            if final_poses_remaining != "":
                min_poses_remaining = min(int(min_poses_remaining), int(final_poses_remaining))
            return ExecutionResult(
                nav_result=nav_result,
                timed_out=False,
                stuck=False,
                recovery_count=max_recoveries,
                waypoint_status=format_waypoint_status(
                    [
                        {
                            "index": "through_poses",
                            "result": nav_result,
                            "recoveries": max_recoveries,
                            "poses_remaining": min_poses_remaining,
                        }
                    ]
                ),
            )

        timed_out = False
        stuck = False
        nav_result = "not_started"
        waypoint_statuses: List[Dict[str, object]] = []
        max_recoveries: object = ""
        for index, waypoint in enumerate(plan.waypoints):
            reset_nav_feedback()
            navigator.goToPose(make_pose(navigator, waypoint))
            best_distance = float("inf")
            last_progress_time = time.monotonic()
            while not navigator.isTaskComplete():
                best_distance, last_progress_time, timed_out, stuck = check_progress_to_goal(
                    waypoint=waypoint,
                    start_time=start_time,
                    best_distance=best_distance,
                    last_progress_time=last_progress_time,
                )
                current_recoveries = recovery_count_from_feedback()
                if current_recoveries != "":
                    max_recoveries = max(int(max_recoveries or 0), int(current_recoveries))
                if timed_out:
                    navigator.cancelTask()
                    break
                if stuck:
                    navigator.cancelTask()
                    break
            final_recoveries = recovery_count_from_feedback()
            if final_recoveries != "":
                max_recoveries = max(int(max_recoveries or 0), int(final_recoveries))
            if timed_out:
                waypoint_statuses.append(
                    {
                        "index": index,
                        "result": "timeout",
                        "recoveries": final_recoveries,
                        "poses_remaining": "",
                    }
                )
                return ExecutionResult(
                    nav_result="timeout",
                    timed_out=True,
                    stuck=False,
                    recovery_count=max_recoveries,
                    waypoint_status=format_waypoint_status(waypoint_statuses),
                )
            if stuck:
                waypoint_statuses.append(
                    {
                        "index": index,
                        "result": "stuck",
                        "recoveries": final_recoveries,
                        "poses_remaining": "",
                    }
                )
                return ExecutionResult(
                    nav_result="stuck",
                    timed_out=False,
                    stuck=True,
                    recovery_count=max_recoveries,
                    waypoint_status=format_waypoint_status(waypoint_statuses),
                )
            nav_result = nav_result_text(navigator.getResult())
            waypoint_statuses.append(
                {
                    "index": index,
                    "result": nav_result,
                    "recoveries": final_recoveries,
                    "poses_remaining": "",
                }
            )
            if nav_result != "success":
                break
        return ExecutionResult(
            nav_result=nav_result,
            timed_out=False,
            stuck=False,
            recovery_count=max_recoveries,
            waypoint_status=format_waypoint_status(waypoint_statuses),
        )

    def make_online_row(
        trial: TrialSpec,
        plan: GoalPlan,
        start_time: float,
        start_distance: float,
        execution: ExecutionResult,
        cmd_start: Dict[str, Dict[str, float]],
    ) -> Dict[str, object]:
        final_goal = plan.waypoints[-1]
        final_pose = watcher.pose
        final_error = ""
        physical_arrival = ""
        final_x = final_y = final_yaw = ""
        amcl_x = amcl_y = amcl_yaw = ""
        if final_pose is not None:
            final_x = round(final_pose[0], 3)
            final_y = round(final_pose[1], 3)
            final_yaw = round(final_pose[2], 3)
            final_error_value = pose_distance(final_pose, final_goal)
            final_error = round(final_error_value, 3)
            physical_arrival = final_error_value <= args.success_radius_m
        if watcher.amcl_pose is not None:
            amcl_x = round(watcher.amcl_pose[0], 3)
            amcl_y = round(watcher.amcl_pose[1], 3)
            amcl_yaw = round(watcher.amcl_pose[2], 3)
        planned_length = round(path_length(plan.waypoints), 3)
        trajectory_length = round(watcher.distance_traveled - start_distance, 3)
        shortest_path_for_spl = plan.route_planner_path_length_m or planned_length
        spl = spl_score(
            success=bool(physical_arrival),
            shortest_path_m=shortest_path_for_spl,
            trajectory_length_m=trajectory_length,
        )

        safety = check_goal_safety(
            occupancy,
            final_goal,
            args.safe_radius_m,
            args.min_free_ratio,
        )
        return {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "mode": "online",
            "trial_id": trial.trial_id,
            "instruction": trial.instruction,
            "group": trial.group,
            "target_key": trial.target_key,
            "policy": plan.policy,
            "goal_source": plan.goal_source,
            "candidate_count": len(plan.candidate_poses),
            "candidate_selected_index": plan.selected_candidate_index,
            "candidate_selection_reason": plan.candidate_selection_reason,
            "candidate_poses": format_waypoints(plan.candidate_poses),
            "candidate_safety": plan.candidate_safety,
            "candidate_path_diagnostics": plan.candidate_path_diagnostics,
            "selected_candidate_path_length_m": plan.selected_candidate_path_length_m,
            "route_planner_result": plan.route_planner_result,
            "route_planner_path_length_m": plan.route_planner_path_length_m,
            "route_execution_mode": plan.route_execution_mode,
            "planner_start_source": plan.planner_start_source,
            "planner_start_x": plan.planner_start_x,
            "planner_start_y": plan.planner_start_y,
            "planner_start_yaw": plan.planner_start_yaw,
            "waypoint_count": len(plan.waypoints),
            "waypoints": format_waypoints(plan.waypoints),
            "target_x": round(final_goal[0], 3),
            "target_y": round(final_goal[1], 3),
            "target_yaw": round(final_goal[2], 3),
            "planned_path_length_m": planned_length,
            "nav_result": execution.nav_result,
            "duration_sec": round(time.monotonic() - start_time, 3),
            "trajectory_length_m": trajectory_length,
            "spl": spl,
            "final_pose_source": "odom",
            "final_x": final_x,
            "final_y": final_y,
            "final_yaw": final_yaw,
            "amcl_x": amcl_x,
            "amcl_y": amcl_y,
            "amcl_yaw": amcl_yaw,
            "final_navigation_error_m": final_error,
            "physical_arrival": physical_arrival,
            "success_radius_m": args.success_radius_m,
            "timeout": execution.timed_out,
            "stuck": execution.stuck,
            "recovery_count": execution.recovery_count,
            "waypoint_status": execution.waypoint_status,
            "amcl_odom_disagreement_m": watcher.amcl_odom_disagreement(),
            **watcher.cmd_delta(cmd_start),
            **safety,
        }

    occupancy = SimpleOccupancyMap.from_yaml(args.map_yaml)
    rows: List[Dict[str, object]] = []
    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    watcher = RuntimeWatcher(
        args.odom_topic,
        args.amcl_pose_topic,
        args.cmd_vel_nav_topic,
        args.cmd_vel_topic,
    )
    wait_for_initial_pose()
    if args.single_goal is not None:
        trial_plan_specs = requested_trial_plans(args, occupancy=occupancy)
    else:
        trial_plan_specs = [
            (trial, policy)
            for trial in requested_trial_specs(args)
            for policy in args.policies
        ]
    try:
        for trial_plan in trial_plan_specs:
                if args.single_goal is not None:
                    trial, plan = trial_plan
                else:
                    trial, policy = trial_plan
                    spin_runtime_callbacks(0.2)
                    plan = build_goal_plan(
                        policy=policy,
                        target_key=trial.target_key,
                        occupancy=occupancy,
                        args=args,
                        start_pose=select_planner_start_pose()[0],
                    )
                    plan = evaluate_candidate_paths_with_nav2(plan)
                    plan = evaluate_route_with_nav2(plan)
                start_distance = watcher.distance_traveled
                cmd_start = watcher.cmd_snapshot()
                start_time = time.monotonic()
                execution = execute_plan(plan, start_time=start_time)

                rows.append(
                    make_online_row(
                        trial,
                        plan,
                        start_time,
                        start_distance,
                        execution,
                        cmd_start,
                    )
                )
                write_rows(Path(args.output), rows)
                spin_runtime_callbacks(args.settle_sec)
    except KeyboardInterrupt:
        navigator.cancelTask()
        if rows:
            write_rows(Path(args.output), rows)
        raise
    finally:
        navigator.cancelTask()
        write_rows(Path(args.output), rows)
        watcher.destroy_node()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="VLN-CE-style Nav2 subset runner for object_center / observation_pose / waypoint_sequence ablations."
    )
    parser.add_argument("--mock", action="store_true", help="Do not start ROS/Nav2; only generate the planned CSV.")
    parser.add_argument(
        "--output",
        default=str(ROOT / "data" / "vlnce_nav2_subset_mock_2026-07-02.csv"),
    )
    parser.add_argument("--map-yaml", default=DEFAULT_MAP_YAML)
    parser.add_argument("--odom-topic", default="/chassis/odom")
    parser.add_argument("--amcl-pose-topic", default="/amcl_pose")
    parser.add_argument("--cmd-vel-nav-topic", default="/cmd_vel_nav")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    parser.add_argument("--stuck-timeout-sec", type=float, default=90.0)
    parser.add_argument("--stuck-min-progress-m", type=float, default=0.05)
    parser.add_argument("--settle-sec", type=float, default=2.0)
    parser.add_argument(
        "--initial-pose-wait-sec",
        type=float,
        default=5.0,
        help="Online only: wait this long for AMCL/odom before Nav2 path diagnostics.",
    )
    parser.add_argument(
        "--planner-start-source",
        choices=("auto", "amcl", "odom"),
        default="auto",
        help="Online only: pose source for Nav2 getPath/getPathThroughPoses diagnostics.",
    )
    parser.add_argument("--success-radius-m", type=float, default=0.8)
    parser.add_argument("--safe-radius-m", type=float, default=0.30)
    parser.add_argument("--min-free-ratio", type=float, default=0.95)
    parser.add_argument(
        "--route-plan-diagnostics",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Call Nav2 getPath/getPathThroughPoses online before execution and record route path length.",
    )
    parser.add_argument(
        "--waypoint-execution-mode",
        choices=("sequential", "through_poses"),
        default="sequential",
        help="Use sequential goToPose or Nav2 goThroughPoses for waypoint_sequence policies.",
    )
    parser.add_argument(
        "--candidate-selection-mode",
        choices=("first_safe", "nearest_safe", "highest_free_ratio", "planner_shortest"),
        default="first_safe",
        help=(
            "How candidate_observation_pose selects among candidates. "
            "planner_shortest uses Nav2 getPath online and falls back to static first-safe in mock."
        ),
    )
    parser.add_argument(
        "--policies",
        nargs="+",
        choices=POLICIES,
        default=list(POLICIES),
    )
    parser.add_argument("--trial-ids", nargs="+", type=int, choices=[trial.trial_id for trial in TRIALS])
    parser.add_argument(
        "--single-goal",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW"),
        help="Run one manual smoke-test goal instead of the predefined VLN subset.",
    )
    parser.add_argument("--single-goal-label", default="manual_goal")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = run_mock(args) if args.mock else run_online(args)
    print(f"Wrote {len(rows)} rows to {args.output}")


if __name__ == "__main__":
    main()
