# Analysis Utilities

This folder contains lightweight analysis and evidence-generation scripts for Node 6/7.

## `plot_node6_node7_results.py`

Purpose:

- Reads the fixed Node 6 final CSV and Node 7 ablation CSV.
- Generates poster/report figures under `assets/`.

Inputs:

```text
data/node6_auto_trials_2026-06-13_final.csv
data/node7_ablation_2026-06-13.csv
```

Outputs:

```text
assets/node6_target_vs_final_pose.png
assets/node6_failure_taxonomy.png
assets/node7_ablation_comparison.png
```

Run:

```bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
python3 src/analysis/plot_node6_node7_results.py
```

Dependencies:

- Python standard library
- Pillow

## `run_node7_safe_recovery_trial.py`

Purpose:

- Runs or records a targeted Node 7 safe-start recovery check.
- Uses the static warehouse occupancy map to detect unsafe starts and find nearby free recovery candidates.
- Can use Nav2 online when Isaac Sim, `/scan`, AMCL, and Nav2 are active.

Default output:

```text
data/node7_safe_recovery_trials_2026-06-15.csv
```

Recommended deterministic replay evidence is currently stored in:

```text
data/node7_safe_recovery_replay_2026-06-15.csv
docs/node7_safe_recovery_note.md
```

Run with ROS Python:

```bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
source /opt/ros/humble/setup.bash
/usr/bin/python3 src/analysis/run_node7_safe_recovery_trial.py --timeout-sec 900.0
```

Important:

- Use `/usr/bin/python3` for ROS scripts. ROS 2 Humble `rclpy` is compiled for system Python 3.10.
- Do not use Conda Python for scripts that import `rclpy`.
- Full online runs require Isaac Sim topics, `pointcloud_to_laserscan`, Nav2, AMCL initial pose, and the static `base_link -> base_footprint` transform.

## Full Online Node 7 Evaluation

After the full simulation stack and `vln_node_local` are active, run:

```bash
source /opt/ros/humble/setup.bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
source install/setup.bash
ros2 run vln_nav2_bridge node6_auto_trials -- \
  --output data/node7_online_trials_2026-06-15.csv \
  --timeout-sec 900.0 \
  --settle-sec 2.0
```

This produces an online CSV for the current Node 7 bridge logic. Keep it separate from the offline ablation CSV so the evidence scope remains clear.
