# VLN-CE-style Nav2 Subset Protocol

Date: 2026-07-02

This protocol prepares a small ablation before running another full Isaac Sim benchmark.

## Subset

Six instructions are used:

- 2 plant instructions
- 1 chair instruction
- 3 long-distance shelf / purple-box / package-area instructions

## Policies

- `object_center`: navigate to the semantic object center.
- `observation_pose`: navigate to a single camera-friendly observation pose.
- `candidate_observation_pose`: select one observation pose from a static candidate set after occupancy-map safety filtering.
- `waypoint_sequence`: navigate through intermediate waypoints, mainly for long shelf/package routes.

## 2026-07-06 Candidate Policy Update

`candidate_observation_pose` was added after the fixed chair observation pose
`(0.20, -0.69, pi)` failed online while the chair object center remained
navigable. The policy currently keeps four candidate poses per semantic target,
computes yaw toward the object center, checks each candidate against the static
occupancy map, optionally scores the candidates with Nav2 `getPath()` online,
and records candidate diagnostics in the CSV:

- `candidate_count`
- `candidate_selected_index`
- `candidate_selection_reason`
- `candidate_poses`
- `candidate_safety`
- `candidate_path_diagnostics`
- `selected_candidate_path_length_m`

The online watcher now also subscribes to `/amcl_pose` and writes:

- `amcl_x`
- `amcl_y`
- `amcl_yaw`
- `amcl_odom_disagreement_m`
- `route_planner_result`
- `route_planner_path_length_m`
- `route_execution_mode`
- `planner_start_source`
- `planner_start_x`
- `planner_start_y`
- `planner_start_yaw`
- `spl`
- `recovery_count`
- `waypoint_status`

This keeps the runner aligned with VLN-CE-style continuous metrics and makes
localization drift visible without invoking the VLN bridge.

`waypoint_sequence` can now be executed in two modes:

- `sequential`: send each waypoint as an individual Nav2 `goToPose()`.
- `through_poses`: send the whole route through Nav2 `goThroughPoses()`.

When online, route diagnostics use `getPath()` or `getPathThroughPoses()` before
execution and record whether Nav2 can produce a global path and its length. The
planner start pose is selected by `--planner-start-source`; the default `auto`
mode prefers AMCL and falls back to odom. `--initial-pose-wait-sec` waits for an
initial AMCL/odom pose before candidate scoring and route diagnostics.

The runner now writes an SPL-style efficiency field. Online rows compute `spl`
as `shortest_path / max(shortest_path, trajectory_length)` for physical-arrival
successes and `0.0` for failures. When a Nav2 route path length is available, it
is used as the shortest path estimate; otherwise the static waypoint length is
used as a fallback.

Online execution also records `recovery_count` from Nav2 action feedback. In
sequential mode, `waypoint_status` stores one entry per waypoint result. In
`through_poses` mode, Humble exposes aggregate feedback, so `waypoint_status`
stores the final route-level result, recovery count, and the minimum observed
`number_of_poses_remaining`.

## Mock Test

Run without Isaac Sim:

```bash
python3 src/analysis/run_vlnce_nav2_subset.py \
  --mock \
  --output data/vlnce_nav2_subset_mock_candidate_2026-07-06.csv
```

Expected output after the candidate update: 24 rows, one for each
6-instruction x 4-policy pair. This checks goal generation, map safety metadata,
candidate diagnostics, and the VLN-CE-style metric schema.

Focused chair candidate mock:

```bash
python3 src/analysis/run_vlnce_nav2_subset.py \
  --mock \
  --trial-ids 3 \
  --policies candidate_observation_pose \
  --candidate-selection-mode first_safe \
  --output data/vlnce_nav2_trial3_candidate_mock_2026-07-06.csv
```

The first selected chair candidate is currently `(-0.20, -1.30, 2.079)`,
which intentionally differs from the weak fixed observation pose
`(0.20, -0.69, pi)`.

Planner-shortest mock check:

```bash
python3 src/analysis/run_vlnce_nav2_subset.py \
  --mock \
  --trial-ids 3 \
  --policies candidate_observation_pose \
  --candidate-selection-mode planner_shortest \
  --output data/vlnce_nav2_trial3_candidate_planner_mock_2026-07-06.csv
```

In mock mode, `planner_shortest` records a static fallback reason because no
Nav2 planner is running. In online mode, it calls `BasicNavigator.getPath()`
for each statically safe candidate and selects the shortest valid global path.
Static-unsafe candidates are skipped before invoking Nav2 and are marked as
`skipped_static_unsafe` in candidate path diagnostics.

## Online Run

After Isaac Sim, `/scan`, Nav2, RViz, and TF are running from a clean reset:

```bash
python3 src/analysis/run_vlnce_nav2_subset.py \
  --output data/vlnce_nav2_subset_online_2026-07-02.csv \
  --timeout-sec 900 \
  --settle-sec 2.0
```

Recommended next focused run after a clean Isaac reset:

```bash
/usr/bin/python3 src/analysis/run_vlnce_nav2_subset.py \
  --output data/vlnce_nav2_trial3_candidate_online_2026-07-06.csv \
  --trial-ids 3 \
  --policies candidate_observation_pose object_center observation_pose \
  --candidate-selection-mode planner_shortest \
  --planner-start-source auto \
  --initial-pose-wait-sec 5.0 \
  --timeout-sec 180 \
  --stuck-timeout-sec 45 \
  --stuck-min-progress-m 0.03 \
  --success-radius-m 0.35
```

Recommended long-route Nav2-native waypoint run:

```bash
/usr/bin/python3 src/analysis/run_vlnce_nav2_subset.py \
  --output data/vlnce_nav2_long_waypoints_through_poses_2026-07-06.csv \
  --trial-ids 4 5 6 \
  --policies waypoint_sequence \
  --waypoint-execution-mode through_poses \
  --route-plan-diagnostics \
  --planner-start-source auto \
  --initial-pose-wait-sec 5.0 \
  --timeout-sec 900 \
  --stuck-timeout-sec 90 \
  --stuck-min-progress-m 0.05 \
  --success-radius-m 0.8
```

Watch RViz during the run:

- `/vln_goal_pose` if separately published by bridge experiments
- global plan
- local costmap
- `/scan`
- AMCL pose
- robot footprint
- executed trajectory

This runner is intentionally Nav2-only. It does not call the VLN bridge, Qwen, final visual scan, active search, or visual-anchor AMCL reanchor.
