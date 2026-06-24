# Data Inventory

Date: 2026-06-24

## Commit-Ready Evidence

| File | Status | Use |
|---|---|---|
| `data/node8_active_visual_reanchor_final_search_2026-06-24.csv` | positive | Node 8 strict long-distance targeted success with visual-anchor gated AMCL reanchor and final active visual search. |
| `data/node8_amcl_conservative_origin_to_purple_boxes_2026-06-24.csv` | limitation | Scan-only AMCL conservative failure evidence. |
| `data/node8_amcl_v6_odom_dominant_origin_to_purple_boxes_2026-06-24.csv` | limitation | Scan-only AMCL odom-dominant failure evidence. |
| `data/node7_node8_remaining_12_15_odom_2026-06-24.csv` | partial online rerun | Clean from-current shelf-area rerun for trials 12-15 using raw odom final metrics. |
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

The 2026-06-24 attempted full 15-trial rerun reached 6/6 strict success before the long purple-box route. The purple-box route itself succeeded in the bridge after 959.787 s, but the auto-trial wrapper timed out at 900 s and recorded the row as timeout. This exposed two wrapper issues:

- full long-distance trials need `--timeout-sec` greater than 960 s;
- odom-truth baseline summaries should use `--final-pose-source odom`, because AMCL may drift even when Nav2 is controlled by static `map -> odom`.

`node6_auto_trials.py` now supports:

```text
--final-pose-source odom
```

For the next clean full-15 rerun, reset Isaac/Carter to origin and run with:

```text
--timeout-sec 1500 --final-pose-source odom
```
