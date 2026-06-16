# Node 7 Clean Online Rerun Report

Date: 2026-06-15

## Objective

This run evaluates the current Node 7 online stack after the safe-start, semantic-confirmation, dynamic-timeout, and ambiguous-target policy updates. Unlike the offline ablation in `docs/node7_ablation_report.md`, this is a live Isaac Sim + ROS 2 + Nav2 + Qwen bridge rerun over the full 15-instruction evaluation set.

## Runtime Protocol

The robot was manually reset in Isaac Sim before the run. The following runtime stack was then used:

- Isaac Sim publishing live odometry, camera, LiDAR, TF, and clock topics.
- `/front_3d_lidar/lidar_points` converted to `/scan` through `pointcloud_to_laserscan`.
- Static transform published for `base_link -> base_footprint`.
- Nav2 launched with AMCL and a valid `map -> odom` transform after `/set_initial_pose`.
- `vln_node_local` launched with:
  - Qwen backend enabled.
  - Realtime left camera topic: `/front_stereo_camera/left/image_raw`.
  - `require_fresh_image:=true`.
  - `visual_scan_enabled:=true`.
  - `semantic_exploration_enabled:=true`.
  - `nav_timeout_sec:=900.0`.
- Preflight checks passed for live odometry and known semantic targets:
  - current odom near reset origin.
  - plant target valid.
  - chair target valid.
  - shelf/package target valid.

Output CSV:

```text
data/node7_online_trials_clean_2026-06-15.csv
```

## Summary Metrics

| Metric | Result |
|---|---:|
| Total trials | 15 |
| Bridge `nav_result=success` | 8/15 |
| Final pose within 0.80 m | 13/15 |
| Visual confirmed | 8/15 |
| Task success | 8/15 |
| Mean final error, all trials | 2.006 m |
| Mean final error, arrived trials | 0.306 m |

Parse-method distribution:

| Parse method | Count |
|---|---:|
| `visual_semantic_map` | 8 |
| `semantic_explore_visual_scan_failed` | 4 |
| `semantic_explore_failed` | 1 |
| `ambiguous_target` | 2 |

Failure distribution:

| Failure type | Count |
|---|---:|
| Target not visible after visual scan | 4 |
| Ambiguous target rejected | 2 |
| Nav2 no meaningful progress near goal | 1 |

## Per-Trial Results

| Trial | Instruction | Nav result | Parse method | Error (m) | Arrived | Visual | Task | Failure reason |
|---:|---|---|---|---:|---:|---:|---:|---|
| 1 | Go to the plant. | stuck | `semantic_explore_failed` | 0.167 | True | False | False | no meaningful progress near goal |
| 2 | Move to the potted plant on the floor. | failed | `semantic_explore_visual_scan_failed` | 0.162 | True | False | False | target not visible after visual scan |
| 3 | Navigate to the green plant near the chair. | failed | `semantic_explore_visual_scan_failed` | 0.060 | True | False | False | target not visible after visual scan |
| 4 | Go to the black office chair. | success | `visual_semantic_map` | 0.236 | True | True | True |  |
| 5 | Move to the chair near the robot. | failed | `semantic_explore_visual_scan_failed` | 0.191 | True | False | False | target not visible after visual scan |
| 6 | Navigate to the chair beside the plant. | failed | `semantic_explore_visual_scan_failed` | 0.151 | True | False | False | target not visible after visual scan |
| 7 | Go to the purple boxes. | success | `visual_semantic_map` | 0.430 | True | True | True |  |
| 8 | Move to the right shelf with purple boxes. | success | `visual_semantic_map` | 0.430 | True | True | True |  |
| 9 | Navigate to the shelf area containing purple packages. | success | `visual_semantic_map` | 0.430 | True | True | True |  |
| 10 | Go to the shelf. | success | `visual_semantic_map` | 0.430 | True | True | True |  |
| 11 | Move to the right shelf. | success | `visual_semantic_map` | 0.430 | True | True | True |  |
| 12 | Navigate to the warehouse rack near the boxes. | success | `visual_semantic_map` | 0.430 | True | True | True |  |
| 13 | Go to the object near the wall. | failed | `ambiguous_target` | 13.057 | False | False | False | ambiguous target rejected |
| 14 | Move to the package area. | success | `visual_semantic_map` | 0.430 | True | True | True |  |
| 15 | Navigate to the target object. | failed | `ambiguous_target` | 13.057 | False | False | False | ambiguous target rejected |

## Interpretation

The clean online rerun supports the Node 7 claims, but it does not support a claim of perfect end-to-end performance. The strongest defensible result is:

- The current Node 7 stack completes a full live 15-trial rerun without crashing.
- Shelf/package-area instructions are now robust online: Trials 7-12 and 14 succeed with `visual_semantic_map`.
- Physical navigation is substantially stronger than strict visual task success: 13/15 trials end inside the 0.80 m success radius.
- The plant/chair failures are mostly evaluator/confirmation failures after physical arrival, not gross navigation failures.
- Ambiguous target handling is intentionally strict: `object near the wall` and `target object` are rejected rather than silently mapped to an arbitrary landmark.

For a poster or paper, report both `navigation_arrived` and `task_success`. Reporting only task success hides the fact that the robot physically arrived in 13/15 trials. Reporting only final-position success hides the current visual-confirmation limitation.

## Limitations To State Explicitly

- The evaluation is a single live clean rerun in one Isaac Sim warehouse scene.
- The model/backend was the local Qwen bridge used in this project, not a multi-seed comparison against external VLN baselines.
- The current camera viewpoint can fail to re-observe plant/chair targets after arrival, causing strict visual confirmation failures even when final pose is within radius.
- Broad noun phrases without a concrete semantic class are rejected as ambiguous by design.

## Poster-Ready Claim

In a 15-instruction live Isaac Sim rerun, Node 7 achieved 13/15 physical arrivals within 0.80 m and 8/15 strict visual task successes. The major improvement is robust online handling of shelf/package-area language, while remaining failures expose a visual-confirmation gap for nearby plant/chair targets and an intentional rejection policy for ambiguous references.

## Reproduction Command

After Isaac Sim, `/scan`, Nav2, AMCL, `map -> odom`, and `vln_node_local` are active:

```bash
source /opt/ros/humble/setup.bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
source install/setup.bash
ros2 run vln_nav2_bridge node6_auto_trials -- \
  --output data/node7_online_trials_clean_2026-06-15.csv \
  --timeout-sec 900.0 \
  --settle-sec 2.0
```

Use ROS 2's system Python path for ROS scripts because ROS 2 Humble `rclpy` is bound to Python 3.10. Conda Python versions can fail to import the Humble C extension.
