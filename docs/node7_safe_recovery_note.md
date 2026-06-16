# Node 7 Safe Recovery Note

Date: 2026-06-15

## Scope

This note documents the targeted safe-start recovery check added after Node 7. It validates the occupancy-map policy that detects unsafe or near-obstacle starts and selects a nearby free recovery candidate before executing the next semantic goal.

Output CSV:

```text
data/node7_safe_recovery_replay_2026-06-15.csv
```

## Online Environment Check

Isaac Sim was running and publishing:

```text
/front_3d_lidar/lidar_points
/front_stereo_camera/left/image_raw
/front_stereo_camera/right/image_raw
/chassis/odom
/cmd_vel
/clock
/tf
```

For this check, the missing runtime pieces were started manually:

- `pointcloud_to_laserscan` remapped `/front_3d_lidar/lidar_points` to `/scan`.
- Nav2 bringup was started with `warehouse_map.yaml`.
- Static `base_link -> base_footprint` was added.
- `/set_initial_pose` was called; AMCL then published `map -> odom`.

The full online attempt to reposition the robot from the origin to the shelf-edge setup pose was started, and Nav2 accepted the goal. Because the setup leg was long and slow, it was cancelled rather than used as final evidence in this note. The CSV below is therefore a deterministic safe-start policy replay over poses already observed during Node 6 debugging, not a full online rerun.

## Safe-Start Replay Results

| Case | Center state | Free ratio | Safe threshold | Recovery candidate | Interpretation |
|---|---|---:|---:|---|---|
| shelf-edge start from Node 6 | free | 0.920 | 0.950 | `(-6.725, 10.815)` | Near-obstacle start is detected and moved 0.050 m to a safer candidate. |
| occupied start from Node 6 | occupied | 0.451 | 0.950 | `(-1.290, -1.465)` | Occupied-map start is detected and receives a recovery candidate. |
| default chair goal | free | 1.000 | 0.950 | unchanged | Safe-goal validation does not perturb a valid semantic goal. |

## Interpretation

This supports the Node 7 safe-start/safe-goal extension at the policy level. It shows that the known shelf-edge failure mode would trigger recovery before the next chair navigation request. A full online demonstration can be added later by starting the robot directly near the shelf edge or by allowing the long setup navigation to complete.
