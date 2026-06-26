# Data Inventory

Date: 2026-06-24

## Commit-Ready Evidence

| File | Status | Use |
|---|---|---|
| `data/node8_active_visual_reanchor_final_search_2026-06-24.csv` | positive | Node 8 strict long-distance targeted success with visual-anchor gated AMCL reanchor and final active visual search. |
| `data/node8_amcl_conservative_origin_to_purple_boxes_2026-06-24.csv` | limitation | Scan-only AMCL conservative failure evidence. |
| `data/node8_amcl_v6_odom_dominant_origin_to_purple_boxes_2026-06-24.csv` | limitation | Scan-only AMCL odom-dominant failure evidence. |
| `data/node7_node8_full15_clean_odom_2026-06-24.csv` | clean online rerun | Full 15-trial online rerun using raw odom final metrics: 12/15 strict task success, 13/15 physical arrival. |
| `data/node7_node8_trial11_hotfix_odom_2026-06-24.csv` | hotfix validation | Single-trial rerun showing Trial 11 succeeds after safe-start accepts safe raw odom when AMCL has drifted. |
| `data/node7_node8_remaining_12_15_odom_2026-06-24.csv` | partial online rerun | Clean from-current shelf-area rerun for trials 12-15 using raw odom final metrics. |
| `data/node7_node8_full15_scanonly_amcl_2026-06-25.csv` | scan-only result | Scan-only AMCL full-15 with truth map: 5/15 strict success (near-range all pass), long-range AMCL drift. |
| `data/warehouse_map_truth.{pgm,yaml}` | truth map | Odom-truth occupancy grid built by node8_odom_truth_mapper, replaces broken original warehouse_map. |
| `data/node8_amcl_truth_expanded_disagreement_2026-06-25.csv` | diagnostic | AMCL vs odom disagreement tracking with truth map (307 samples, 11.4m traveled). |
| `config/nav2_params_lowload.yaml` | config | Low-load Nav2 config (controller 5Hz, costmap 1/0.2Hz) to reduce Isaac sim-time jump-back. |
| `config/nav2_params_odom_dominant.yaml` | config | Odom-dominant AMCL config (laser_likelihood_max_dist 0.3, z_hit 0.85) for scan-only tuning. |
| `assets/node8_map_diagnostics_reanchor_overlay_2026-06-24.png` | figure | README-safe copy of the Node 8 map diagnostic overlay. |
| `assets/node8_map_diagnostics_target_crop_2026-06-24.png` | figure | README-safe copy of the Node 8 target-area map crop. |

## Interrupted / Diagnostic Only

| File | Status | Reason |
|---|---|---|
| `data/node7_node8_full15_interrupted_amcl_pose_2026-06-24.csv` | diagnostic only | Trial 7 exceeded the old 900 s auto-trial timeout while the bridge later succeeded. Later rows use AMCL final pose and show false large errors after AMCL drift. Do not report as clean full-15 evidence. |
| `data/node8_active_visual_reanchor_fast_2026-06-24.csv` | diagnostic only | Earlier active visual reanchor run before final active visual search was required. |
| `data/node8_active_visual_relocalization_2026-06-24.csv` | diagnostic only | Early active visual relocalization trial. |
| `data/node8_active_visual_relocalization_yaw_closed_loop_2026-06-24.csv` | diagnostic only | Early yaw-closed-loop visual spin trial. |

## Current Rerun Status

The clean full-15 rerun is:

```text
data/node7_node8_full15_clean_odom_2026-06-24.csv
```

Summary:

| Metric | Result |
|---|---:|
| Total trials | 15 |
| Navigation success | 12/15 |
| Physical arrival within 0.8 m | 13/15 |
| Visual confirmed | 12/15 |
| Strict task success | 12/15 |

Trial 11 `Move to the right shelf.` was a safe-start false negative: raw odom final error was `0.300 m`, but AMCL had drifted to an outside-map pose and caused safe-start rejection. After patching safe-start to accept a safe fresh raw-odom pose when AMCL is invalid, the single-trial hotfix rerun succeeded:

```text
data/node7_node8_trial11_hotfix_odom_2026-06-24.csv
```

Therefore, all 13 concrete/non-ambiguous instructions have online success evidence. Trial 13 and Trial 15 remain intentional ambiguous-target rejections.

## Earlier Interrupted Rerun

The 2026-06-24 attempted full 15-trial rerun reached 6/6 strict success before the long purple-box route. The purple-box route itself succeeded in the bridge after 959.787 s, but the auto-trial wrapper timed out at 900 s and recorded the row as timeout. This exposed two wrapper issues:

- full long-distance trials need `--timeout-sec` greater than 960 s;
- odom-truth baseline summaries should use `--final-pose-source odom`, because AMCL may drift even when Nav2 is controlled by static `map -> odom`.

`node6_auto_trials.py` now supports:

```text
--final-pose-source odom
```

The later clean full-15 rerun fixed this by using:

```text
--timeout-sec 1500 --final-pose-source odom
```
