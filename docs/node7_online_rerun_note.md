# Node 7 Online Rerun Attempt

Date: 2026-06-15

## Scope

This was an attempt to run the full 15-trial Node 7 online evaluation with Isaac Sim, real camera input, Qwen server inference, Nav2, AMCL, semantic exploration, relaxed confirmation, and safe-start enabled.

Partial output:

```text
data/node7_online_trials_2026-06-15.csv
```

The run was stopped after two completed trials because both reproduced the same startup-state blocker. Continuing all 15 trials from the same invalid start state would have produced repeated, low-value failures rather than a useful Node 7 online evaluation.

## Runtime Stack

The following runtime pieces were brought online:

- Isaac Sim was already running.
- Isaac ROS topics were available, including `/front_3d_lidar/lidar_points`, `/front_stereo_camera/left/image_raw`, `/chassis/odom`, `/clock`, `/cmd_vel`, and `/tf`.
- `pointcloud_to_laserscan` converted `/front_3d_lidar/lidar_points` to `/scan`.
- Static `base_link -> base_footprint` TF was published.
- Nav2 was launched with `data/warehouse_map.yaml`.
- `/set_initial_pose` was called for AMCL.
- `vln_node_local` was started with:

```text
image_topic:=/front_stereo_camera/left/image_raw
require_fresh_image:=true
nav_timeout_sec:=900.0
visual_scan_enabled:=true
semantic_exploration_enabled:=true
dynamic_timeout_enabled:=true
```

## Partial Results

| Trial | Instruction | Result | Parse method | Failure reason |
|---:|---|---|---|---|
| 1 | Go to the plant. | stuck | `semantic_explore_failed` | Nav2 made no meaningful progress for 45.0s with best distance 0.45 m. |
| 2 | Move to the potted plant on the floor. | stuck | `semantic_explore_failed` | Nav2 made no meaningful progress for 45.0s with best distance 0.40 m. |

Both trials first ran the real-time visual scan. The model did not see the plant from the current view, so the bridge correctly fell back to semantic exploration toward the known plant coordinate. Before moving to the plant, Node 7 safe-start detected that the current odom pose was near an occupied region and selected a nearest-free recovery candidate.

## Blocker

The blocker was not model startup or bridge wiring. The online stack reached the point where Node 7 safe-start was triggered and Nav2 accepted the recovery goal.

The issue was a map/odom startup-state mismatch:

- Preflight reported the current odom pose around `(-4.96, 11.81)` in an occupied cell, with free ratio around `20-30%`.
- Safe-start selected a nearby free recovery candidate around `(-5.39, 11.9)`.
- The odom distance to the recovery candidate was only about `0.40-0.45 m`.
- Nav2 feedback nevertheless reported a remaining path distance around `13.7 m` and made no meaningful progress.

This indicates that the robot/AMCL/map state at the start of the rerun was not a valid clean evaluation start. A full 15-trial rerun should be restarted only after resetting the Isaac robot pose or manually placing the robot at a known free map pose, then verifying `node6_map_preflight` reports `[OK] current_odom`.

## Next Clean Rerun Procedure

1. Reset or manually move the Isaac robot to a known free map pose near `(0.0, 0.0)`.
2. Restart `/scan`, static `base_link -> base_footprint`, Nav2, and AMCL.
3. Call `/set_initial_pose` with the same pose used by the simulator.
4. Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
source install/setup.bash
ros2 run vln_nav2_bridge node6_map_preflight --odom-topic /chassis/odom --robot-radius-m 0.30
```

5. Continue only if current odom is `[OK]`.
6. Start `vln_node_local`, then run:

```bash
ros2 run vln_nav2_bridge node6_auto_trials -- \
  --output data/node7_online_trials_YYYY-MM-DD.csv \
  --timeout-sec 900.0 \
  --settle-sec 2.0
```
