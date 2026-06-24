# Final Simulation Report

Date: 2026-06-24

## Scope

This report consolidates the current simulation evidence for Nodes 6-8. Node 6 and Node 7 provide the main 15-trial VLN benchmark evidence. Node 8 is a targeted long-distance navigation closure effort for shelf/package-area goals, especially `Go to the purple boxes.`.

The current Node 8 result should be reported as a strict targeted long-distance success with visual-anchor gated AMCL reanchoring and final active visual search. It should not be reported as solved scan-only AMCL localization. Strict VLN success remains:

```text
task_success = navigation_arrived AND visual_confirmed
```

## Evidence Files

| Area | Evidence |
|---|---|
| Node 6 final 15-trial baseline | `data/node6_auto_trials_2026-06-13_final.csv`, `docs/node6_final_report.md` |
| Node 7 metric split and shelf/package confirmation ablation | `data/node7_ablation_2026-06-13.csv`, `docs/node7_ablation_report.md` |
| Node 7 clean online 15-trial rerun | `data/node7_online_trials_clean_2026-06-15.csv`, `docs/node7_online_clean_rerun_report.md` |
| Node 7 observation-pose clean full baseline | `data/node7_observation_pose_full_clean_2026-06-16.csv` |
| Node 8 odom-truth origin to purple boxes | `data/node8_odom_truth_baseline_from_origin_to_purple_boxes_2026-06-23.csv` |
| Node 8 reboot odom-truth repeat | `data/node8_reboot_odom_truth_pure_nav2_origin_to_purple_boxes_2026-06-23.csv` |
| Node 8 reboot return-to-origin repeat | `data/node8_reboot_odom_truth_return_to_origin_retry_2026-06-23.csv` |
| Node 8 strict visual confirmation evidence | `data/node6_trials.csv`, `data/runtime/trial_images/trial_0013_20260623_180722_187547852_scan_05_go_to_the_purple_boxes.png`, `data/runtime/trial_images/trial_0009_20260623_184124_788871813_scan_01_go_to_the_purple_boxes.png` |
| Node 8 failed strict long-distance subset | `data/node8_odom_truth_long_distance_subset_strict_2026-06-23.csv` |
| Node 8 AMCL failure evidence | `data/node8_reboot_amcl_tf_broadcast_pure_nav2_current_to_purple_boxes_2026-06-23.csv`, `data/node8_rpp_amcl_v2_active_visual_purple_boxes_health_2026-06-23.csv` |
| Node 8 2026-06-24 AMCL conservative failure | `data/node8_amcl_conservative_origin_to_purple_boxes_2026-06-24.csv`, `docs/node8_long_distance_status_2026-06-24.docx` |
| Node 8 2026-06-24 AMCL v6 odom-dominant failure | `data/node8_amcl_v6_odom_dominant_origin_to_purple_boxes_2026-06-24.csv` |
| Node 8 2026-06-24 strict active-visual reanchor success | `data/node8_active_visual_reanchor_final_search_2026-06-24.csv` |
| Node 8 AMCL map diagnostics | `data/runtime/map_diagnostics_node8_reanchor_overlay.png`, `data/runtime/map_diagnostics_node8_target_crop.png` |
| Node 7/8 full-15 rerun attempt | `data/node7_node8_full15_interrupted_amcl_pose_2026-06-24.csv`, `data/node7_node8_remaining_12_15_odom_2026-06-24.csv`, `docs/data_inventory_2026-06-24.md` |

## Summary Metrics

| Experiment | Trials | Physical arrival | Visual confirmed | Strict task success | Notes |
|---|---:|---:|---:|---:|---|
| Node 6 final baseline | 15 | 11/15 | 11/15 visible, 7/15 accepted | 7/15 | Original strict bridge confirmation undercounted shelf/package tasks. |
| Node 7 offline ablation C | 15 | 11/15 | 11/15 | 11/15 | Offline reclassification over fixed Node 6 data; not a separate online rerun. |
| Node 7 clean online rerun | 15 | 13/15 | 8/15 | 8/15 | Strongest full online Node 7 benchmark evidence. |
| Node 7 observation-pose full clean baseline | 15 | 6/15 | 6/15 | 5/15 | Conservative long-distance baseline from 2026-06-16. |
| Node 8 odom-truth pure Nav2 purple-box baseline | 1 | 1/1 | N/A | N/A | Pure navigation only; final raw odom error 0.348 m. |
| Node 8 reboot odom-truth pure Nav2 repeat | 1 | 1/1 | N/A | N/A | Reproduced after Isaac restart; final raw odom error 0.343 m. |
| Node 8 reboot return-to-origin repeat | 1 | 1/1 | N/A | N/A | Return target reached with final raw odom error 0.345 m. |
| Node 8 strict active-visual purple-box bridge | 2 targeted successes in `data/node6_trials.csv` | 2/2 inferred from successful Nav2 result in odom-truth mode | 2/2 | 2/2 | Rows use final-visual-confirmed parse methods and confidence 0.95. |
| Node 8 strict active-visual AMCL reanchor | 1 | 1/1 | 1/1 | 1/1 | 2026-06-24 run; raw error 0.255 m, final AMCL/raw disagreement 0.077 m, final visual confidence 0.95. |
| Node 8 strict long-distance subset CSV | 2 | 0/2 | 0/2 | 0/2 | Both rows are stuck failures; this file does not prove subset completion. |

## Node 8 Technical Changes

Node 8 changed the navigation stack in three major ways:

- Replaced DWB with Regulated Pure Pursuit in `config/nav2_params_custom.yaml`.
- Added a motion-compensated scan accumulator in `src/vln_nav2_bridge/vln_nav2_bridge/node8_scan_accumulator.py`.
- Added stricter active visual confirmation in `src/vln_nav2_bridge/vln_nav2_bridge/vln_node_local.py` and CSV export support in `src/vln_nav2_bridge/vln_nav2_bridge/node6_auto_trials.py`.

The scan accumulator now caches incoming point clouds in a fixed frame, normally `odom`, and projects the accumulated points into `base_link` only at publish time. This avoids treating old points as if they were still in the robot's current body frame during long movements.

The strict visual flow now supports:

- low-frequency visual checks during navigation;
- required final visual scan after Nav2 reports arrival;
- bounded active local search if final confirmation initially fails;
- no fallback to static test images when a live camera topic is configured.

The 2026-06-24 active reanchor runner extends this for the Node 8 long route:

- initial 8-direction visual scan for the `purple boxes` landmark;
- visual-anchor gated AMCL reanchor through `/set_initial_pose`;
- AMCL/raw disagreement monitoring during navigation;
- final active visual search after physical arrival.

## Node 8 Odom-Truth Result

The most important positive Node 8 evidence is the odom-truth localization baseline:

```text
data/node8_odom_truth_baseline_from_origin_to_purple_boxes_2026-06-23.csv
```

Result:

| Field | Value |
|---|---:|
| Target | `(-6.3, 10.8)` |
| Result | `success` |
| Final raw odom | `(-6.082, 10.529)` |
| Raw odom target error | `0.348 m` |
| Final Nav2 feedback distance | `0.365 m` |

The same origin-to-purple-box pure Nav2 run was reproduced after an Isaac Sim restart:

```text
data/node8_reboot_odom_truth_pure_nav2_origin_to_purple_boxes_2026-06-23.csv
```

Result:

| Field | Value |
|---|---:|
| Result | `success` |
| Final raw odom | `(-6.085, 10.532)` |
| Raw odom target error | `0.343 m` |
| Final Nav2 feedback distance | `0.365 m` |

The return-to-origin repeat also succeeded:

```text
data/node8_reboot_odom_truth_return_to_origin_retry_2026-06-23.csv
```

Result:

| Field | Value |
|---|---:|
| Result | `success` |
| Final raw odom | `(-0.210, 0.273)` |
| Raw odom target error | `0.345 m` |
| Final Nav2 feedback distance | `0.352 m` |

These runs prove that the RPP controller, local/global costmap configuration, and rolling scan pipeline can physically complete the long shelf/package route when the `map -> odom` relationship is stable.

## Node 8 Strict Visual Result

The strict active-visual bridge produced targeted purple-box successes after the odom-truth navigation baseline. The relevant rows are appended to:

```text
data/node6_trials.csv
```

The key parse methods are:

```text
semantic_explore_final_visual_confirmed
visual_semantic_map_observation_pose_final_visual_confirmed
```

Both rows have:

```text
instruction = Go to the purple boxes.
nav_result = success
confidence = 0.95
```

Final visual evidence images:

```text
data/runtime/trial_images/trial_0013_20260623_180722_187547852_scan_05_go_to_the_purple_boxes.png
data/runtime/trial_images/trial_0009_20260623_184124_788871813_scan_01_go_to_the_purple_boxes.png
```

The first final image was manually inspected in the 2026-06-23 log: purple boxes are visible on the left side of the frame, partly occluded by yellow fork/rail geometry. This satisfies the strict targeted-case requirement because both physical arrival and final visual evidence are present.

## Failed or Incomplete Evidence

The following evidence must not be reported as Node 8 completion:

| File | Result | Interpretation |
|---|---|---|
| `data/node8_odom_truth_long_distance_subset_strict_2026-06-23.csv` | 0/2 task success | Both long-distance subset rows are `stuck`; the subset still needs a clean rerun. |
| `data/node8_rpp_amcl_v2_active_visual_purple_boxes_health_2026-06-23.csv` | timeout | AMCL final error looked close, but `navigation_arrived=False`; strict failure. |
| `data/node8_reboot_amcl_tf_broadcast_pure_nav2_current_to_purple_boxes_2026-06-23.csv` | timeout | Raw odom stayed 6.223 m from target while AMCL claimed 3.748 m; AMCL/raw disagreement 4.197 m. |
| `data/node8_rpp_v2_purple_boxes_health_2026-06-23.csv` | nominal success but old relaxed confirmation | AMCL/raw disagreement reached 9.094 m, so it is not valid physical success evidence. |
| `data/node8_amcl_conservative_origin_to_purple_boxes_2026-06-24.csv` | canceled | Conservative AMCL delayed divergence but still ended with raw target error 6.698 m and AMCL/raw disagreement 5.315 m. |
| `data/node8_amcl_v6_odom_dominant_origin_to_purple_boxes_2026-06-24.csv` | canceled_diverged | v6 improved raw best error to 3.044 m and final disagreement to 3.323 m, but still failed the 0.8 m arrival gate. |
| `data/node7_node8_full15_interrupted_amcl_pose_2026-06-24.csv` | interrupted wrapper run | Trial 7 bridge success arrived after the old 900 s auto-trial timeout; later rows use AMCL final pose after drift and must not be reported as clean full-15 evidence. |

## Current Limitation

Node 8 isolates the dominant long-distance failure to scan-only AMCL localization and map/scan consistency, not to the visual language model or the RPP controller. In odom-truth mode, long-distance purple-box navigation and final visual confirmation work. In the 2026-06-24 active-visual AMCL reanchor run, the robot also completed the strict long-distance task. However, scan-only AMCL runs still accumulate large AMCL/raw odom disagreement in the shelf/corridor region and can produce false progress estimates.

The current Node 8 claim should therefore be:

> Node 8 completed the long purple-box targeted task with visual-anchor gated AMCL reanchoring and final active visual search. This proves a strict long-distance targeted VLN closure, while scan-only AMCL localization and the 2+ instruction long-distance subset remain open.

## Poster-Ready Text

Method:

> The final simulation stack combines local VLM grounding, semantic target selection, Nav2 execution, motion-compensated LiDAR scan accumulation, and strict post-arrival visual confirmation. For shelf/package-area goals, the robot navigates to a semantic candidate pose and only counts success when both geometric arrival and final visual evidence pass.

Result:

> On the Node 6 15-trial baseline, the original strict bridge achieved 7/15 task success while 11/15 trials physically reached the target radius. Node 7 separated physical arrival from visual confirmation and recovered shelf/package-area undercounts in offline ablation, reaching 11/15 on the fixed baseline and 8/15 strict task success in a clean online rerun. Node 8 then targeted the remaining long-distance shelf/package navigation failure: the robot reached the purple-box target from the origin with 0.255 m final raw-odom error, 0.077 m final AMCL/raw disagreement after visual-anchor gated reanchoring, and final visual confirmation at 0.95 confidence.

Limitation:

> The scan-only AMCL-localized configuration remains unstable in the shelf/corridor region. AMCL can diverge from raw odometry by several meters and cause Nav2 to overestimate progress. The current Node 8 result is a targeted active-visual reanchor closure, not a full scan-only AMCL-localized 15-trial benchmark.

## Next Clean Rerun Gate

Before claiming Node 7/8 full-15 completion, run a fresh clean Isaac reset and generate a new strict full-15 CSV. The minimum gate is:

1. Reset Isaac Sim / Carter to origin and yaw near zero.
2. Start static `base_link -> base_footprint`.
3. Start static `map -> odom` identity for odom-truth mode, with `amcl.tf_broadcast=false`.
4. Start `node8_scan_accumulator` with `fixed_frame=odom` and `target_frame=base_link`.
5. Start Nav2 and the bridge with live camera input and final visual confirmation required.
6. Run the full 15 instructions with:
   - `--timeout-sec 1500`
   - `--final-pose-source odom`
7. Save a new CSV separate from `data/node7_node8_full15_interrupted_amcl_pose_2026-06-24.csv`.
8. Accept full-15 completion only if the CSV is not interrupted and each non-ambiguous target row records physical arrival and final visual confirmation, with final image evidence retained.
