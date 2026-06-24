# Node 8 Evidence Matrix

Date: 2026-06-24

## Status

Node 8 has a reproducible targeted odom-truth success for the long purple-box route, plus one 2026-06-24 strict targeted success using visual-anchor gated AMCL reanchoring and final active visual search. It does not yet solve scan-only AMCL-localized long-distance navigation.

## Use as Positive Evidence

| Evidence | Result | Use |
|---|---|---|
| `data/node8_odom_truth_baseline_from_origin_to_purple_boxes_2026-06-23.csv` | success, raw odom error `0.348 m` | Shows origin -> purple boxes long-distance pure Nav2 works when `map -> odom` is stable. |
| `data/node8_reboot_odom_truth_pure_nav2_origin_to_purple_boxes_2026-06-23.csv` | success, raw odom error `0.343 m` | Shows the odom-truth pure Nav2 result reproduced after Isaac restart. |
| `data/node8_reboot_odom_truth_return_to_origin_retry_2026-06-23.csv` | success, raw odom error `0.345 m` | Shows the odom-truth stack can also return from shelf area to origin. |
| `data/node6_trials.csv` rows with `semantic_explore_final_visual_confirmed` and `visual_semantic_map_observation_pose_final_visual_confirmed` | `nav_result=success`, confidence `0.95` | Shows strict active-visual purple-box confirmation after the odom-truth targeted run. |
| `data/node8_active_visual_reanchor_final_search_2026-06-24.csv` | `task_success=True`, raw error `0.255 m`, final visual confidence `0.95` | Shows Node 8 strict long-distance targeted success with visual-anchor gated AMCL reanchor and final active visual search. |
| `data/runtime/map_diagnostics_node8_reanchor_overlay.png` | origin, target, final pose, and reanchor points are in free cells | Supports the conclusion that failure is not caused by the target being inside an occupied grid cell. |
| `data/runtime/map_diagnostics_node8_target_crop.png` | target-area crop | Shows the sparse/coarse map representation around the purple-box shelf area. |
| `data/runtime/trial_images/trial_0013_20260623_180722_187547852_scan_05_go_to_the_purple_boxes.png` | purple boxes visible in final scan | Final visual evidence for targeted strict success. |
| `data/runtime/trial_images/trial_0009_20260623_184124_788871813_scan_01_go_to_the_purple_boxes.png` | purple boxes final scan evidence | Reboot strict visual evidence. |

## Do Not Use as Completion Evidence

| Evidence | Result | Reason |
|---|---|---|
| `data/node8_odom_truth_long_distance_subset_strict_2026-06-23.csv` | 0/2 task success | This is the current failed strict subset; it must be replaced by a clean rerun before claiming subset completion. |
| `data/node8_rpp_v2_purple_boxes_health_2026-06-23.csv` | bridge success but AMCL/raw disagreement `9.094 m` | False or weak physical-success evidence; raw odom did not support strict arrival. |
| `data/node8_rpp_amcl_v2_active_visual_purple_boxes_health_2026-06-23.csv` | timeout | `navigation_arrived=False`; cannot be reported as success. |
| `data/node8_reboot_amcl_tf_broadcast_pure_nav2_current_to_purple_boxes_2026-06-23.csv` | timeout | AMCL/raw disagreement remains large; AMCL mode is not solved. |
| `data/node8_amcl_conservative_origin_to_purple_boxes_2026-06-24.csv` | canceled after 618.8 s | Conservative AMCL delayed drift, but final raw error was `6.698 m` and AMCL/raw disagreement reached `5.315 m`; cannot be reported as success. |
| `data/node8_amcl_v6_odom_dominant_origin_to_purple_boxes_2026-06-24.csv` | canceled_diverged after 804.0 s | v6 improved best raw error to `3.044 m` and reduced final disagreement to `3.323 m`, but still failed the 0.8 m physical arrival gate. |

## Current AMCL Test Branch

`config/nav2_params_amcl_test.yaml` has been advanced to a v6 odom-dominant test after the 2026-06-24 conservative failure. The next clean run should start from Isaac origin and use:

```text
laser_likelihood_max_dist: 0.75
max_beams: 80
sigma_hit: 0.50
update_min_a: 0.25
update_min_d: 0.20
z_hit: 0.15
z_rand: 0.75
```

The clean v6 run improved the AMCL failure but did not solve it. Treat scan-only AMCL-localized long-distance navigation as a limitation unless a later method reaches the raw-odom 0.8 m success radius and passes final visual confirmation.

## Current Positive Claim

Use this wording for the 2026-06-24 Node 8 result:

```text
Node 8 strict long-distance targeted VLN succeeded with visual-anchor gated AMCL reanchoring and final active visual search.
```

Do not shorten this to “AMCL solved.” The successful run used 10 reanchors, which is useful engineering evidence but also evidence that the static map / scan model is still not stable enough for pure AMCL over this route.

## Minimum Next Rerun

Run from clean Isaac reset/origin:

```text
Go to the purple boxes.
Move to the right shelf with purple boxes.
```

Required output:

```text
data/node8_odom_truth_long_distance_subset_strict_clean_<date>.csv
```

Required pass condition for every row:

```text
navigation_arrived=True
visual_confirmed=True
task_success=True
```

Final image evidence must be retained under:

```text
data/runtime/trial_images/
```
