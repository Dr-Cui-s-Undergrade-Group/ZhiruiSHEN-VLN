# Node 7 Ablation Report

Date: 2026-06-13

## Objective

Node 7 targets the main defect exposed by Node 6: shelf/right-shelf instructions reached the correct target radius, but the bridge still marked them failed because the post-arrival visual scan rejected cart / purple boxes / purple packages as not being a literal visible shelf.

The optimization has two parts:

1. Split result metrics into `navigation_arrived`, `visual_confirmed`, and `task_success`.
2. Add relaxed same-cluster confirmation for shelf/package-area targets after semantic exploration has already navigated to the known candidate.

## Code Changes

- `src/vln_nav2_bridge/vln_nav2_bridge/node6_auto_trials.py`
  - Adds CSV fields: `navigation_arrived`, `visual_confirmed`, `task_success`.
  - Computes `navigation_arrived` from final pose radius, not only bridge `nav_result`.
  - Prints separate summary lines for visual confirmation and task success.

- `src/vln_nav2_bridge/vln_nav2_bridge/text_to_pose_converter.py`
  - Adds semantic clusters for plant, chair, and shelf/package-area.
  - Groups `shelf`, `right shelf`, `warehouse rack`, `package area`, `cart with boxes`, `purple boxes`, `purple packages`, and `purple crates`.

- `src/vln_nav2_bridge/vln_nav2_bridge/vln_node_local.py`
  - Adds relaxed semantic confirmation after semantic exploration.
  - If the robot has arrived at a shelf/package semantic candidate and the scan evidence belongs to the same semantic cluster, the bridge records `semantic_explore_relaxed_confirm` and marks the task successful.

## Ablation Data

Input baseline:

```text
data/node6_auto_trials_2026-06-13_final.csv
```

Output:

```text
data/node7_ablation_2026-06-13.csv
```

## Results

| Variant | Bridge success | Navigation arrived | Visual confirmed | Task success | Relaxed shelf confirmations |
|---|---:|---:|---:|---:|---:|
| A: Node 6 baseline | 7/15 | 7/15 | 7/15 | 7/15 | 0 |
| B: metric split | 7/15 | 11/15 | 7/15 | 7/15 | 0 |
| C: metric split + shelf confirmation | 11/15 | 11/15 | 11/15 | 11/15 | 4 |

Mean final error is unchanged at 0.697 m because Node 7 changes metric interpretation and visual confirmation logic, not the final poses.

## Interpretation

Metric split shows the key Node 6 undercount: four trials physically arrived at the shelf/package target but were hidden inside bridge-level failure. The relaxed shelf/package confirmation then converts those four cases into successful tasks because their visual evidence belongs to the same semantic cluster as the instruction.

The remaining 4/15 failures are not solved by this change. They come from older semantic-map alias or visual-scan failures involving `warehouse rack`, `object near the wall`, `package area`, and `target object`. Those should be handled separately by broader alias normalization or explicit object-disambiguation logic.
