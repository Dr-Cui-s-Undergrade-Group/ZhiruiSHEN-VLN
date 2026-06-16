# Node 7 Observation-Pose Optimization Note

Date: 2026-06-15

## Motivation

The clean online rerun showed that plant/chair trials often reached the target radius but failed strict visual confirmation. Inspection of saved post-arrival images showed the camera looking at walls, floors, or chair undersides rather than the requested object. This is an embodied continuous-navigation failure: an object-center coordinate is not necessarily a good final camera viewpoint.

## Code Change

`src/vln_nav2_bridge/vln_nav2_bridge/text_to_pose_converter.py` now maps plant/chair semantic targets to safe observation poses instead of object-center poses:

| Semantic target | Previous object-center target | New navigation observation pose |
|---|---|---|
| plant | `(-0.43, -2.92, 0.00)` | `(0.35, -2.92, 3.14)` |
| chair | `(-0.54, -0.69, 1.57)` | `(0.20, -0.69, 3.14)` |
| shelf/package area | `(-6.78, 10.96, 0.00)` | unchanged |

The new parse method is `visual_semantic_map_observation_pose` for plant/chair targets. This keeps the metric traceable: the robot is evaluated against the navigation pose it was actually asked to reach, while the semantic instruction remains plant/chair.

## Static Validation

The observation poses were checked against the occupancy map with a 0.30 m robot radius and `free_ratio >= 0.95`:

| Pose | Safe | Center state | Free ratio |
|---|---|---|---:|
| plant observation | True | free | 1.000 |
| chair observation | True | free | 1.000 |
| shelf/package | True | free | 1.000 |

The package was rebuilt successfully:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vln_nav2_bridge
```

## Partial Online Verification

Output:

```text
data/node7_observation_pose_targeted_2026-06-15.csv
data/node7_observation_pose_targeted_instructions.txt
data/node7_observation_pose_targeted_clean_2026-06-15.csv
```

The targeted verification was started from the shelf/package area left by the previous clean rerun. It was stopped after the same cross-map controller blocker repeated.

| Trial | Instruction | Parse method | Target | Visual confirmed | Nav result | Final error |
|---:|---|---|---|---:|---|---:|
| 1 | Go to the plant. | `visual_semantic_map_observation_pose` | `(0.35, -2.92)` | True | stuck | 15.115 m |

The bridge also processed the second instruction and logged it to `data/node6_trials.csv`; it again resolved to `visual_semantic_map_observation_pose` with visible plant evidence, but Nav2 made no meaningful progress from the shelf-edge start.

## Clean Targeted Online Verification

After resetting the robot to the origin and reinitializing AMCL, `/scan`, Nav2, and the VLN bridge, the six plant/chair targeted instructions were rerun from a clean start.

Output:

```text
data/node7_observation_pose_targeted_clean_2026-06-15.csv
```

Summary:

| Metric | Result |
|---|---:|
| Total targeted trials | 6 |
| Final pose within 0.80 m | 6/6 |
| Visual confirmed | 5/6 |
| Strict task success | 5/6 |

Per-trial result:

| Trial | Instruction | Nav | Parse method | Error | Arrived | Visual | Task | Failure |
|---:|---|---|---|---:|---:|---:|---:|---|
| 1 | Go to the plant. | success | `semantic_explore_then_visual_semantic_map_observation_pose` | 0.656 | True | True | True |  |
| 2 | Move to the potted plant on the floor. | success | `visual_semantic_map_observation_pose` | 0.412 | True | True | True |  |
| 3 | Navigate to the green plant near the chair. | success | `visual_semantic_map_observation_pose` | 0.412 | True | True | True |  |
| 4 | Go to the black office chair. | failed | `semantic_explore_visual_scan_failed` | 0.515 | True | False | False | target not visible after visual scan |
| 5 | Move to the chair near the robot. | success | `semantic_explore_then_visual_semantic_map_observation_pose` | 0.496 | True | True | True |  |
| 6 | Navigate to the chair beside the plant. | success | `visual_semantic_map_observation_pose` | 0.515 | True | True | True |  |

## Interpretation

The observation-pose optimization is active and improves the semantic target selection. In the clean targeted rerun, plant/chair strict task success improved from the prior clean online rerun's 1/6 to 5/6 while preserving 6/6 physical arrival.

The remaining failure is the literal instruction `Go to the black office chair.` The robot reached the chair observation pose within radius, but the final visual scan did not confirm the chair. This should be treated as a remaining visual/viewpoint limitation, not a navigation failure.

## Required Next Verification

For paper/poster evidence, reset the robot to the origin or another known clean start before rerunning the full 15-instruction online evaluation. The targeted result supports the observation-pose change, but the headline Node 7 result should only be updated after a complete clean 15-trial rerun.
