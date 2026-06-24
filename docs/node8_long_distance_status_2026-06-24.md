# Node 8 长距离导航状态报告

日期：2026-06-24

## 当前结论

Node 8 的长距离 purple-box targeted case 已经有两类正证据：

- odom-truth localization baseline 可以物理到达，并通过 strict final visual confirmation；
- 2026-06-24 的 AMCL 路线在加入主动视觉识别、AMCL/raw disagreement 监控、visual-anchor gated reanchor、到达后 final active visual search 后，完成一次 strict VLN 闭环。

当前仍不能写成“纯 AMCL scan matching 已解决”。准确表述应为：

> Node 8 has a strict long-distance targeted VLN success with visual-anchor gated AMCL reanchoring and final active visual search, while scan-only AMCL long-distance localization remains a documented limitation.

## 已有正证据

| 文件 | 结果 | 说明 |
|---|---|---|
| `data/node8_odom_truth_baseline_from_origin_to_purple_boxes_2026-06-23.csv` | success, raw error 0.348 m | odom-truth origin -> purple boxes 成功。 |
| `data/node8_reboot_odom_truth_pure_nav2_origin_to_purple_boxes_2026-06-23.csv` | success, raw error 0.343 m | Isaac 重启后复现成功。 |
| `data/node8_reboot_odom_truth_return_to_origin_retry_2026-06-23.csv` | success, raw error 0.345 m | 从货架区域返回原点成功。 |
| `data/node6_trials.csv` final-visual-confirmed rows | nav_result=success, confidence 0.95 | strict active visual confirmation 通过。 |
| `data/node8_active_visual_reanchor_final_search_2026-06-24.csv` | task_success=True, raw error 0.255 m | AMCL 路线 + 主动视觉重锚 + final active visual search 的 strict 成功。 |

## 2026-06-24 Strict Active-Visual Reanchor 成功

输出：

```text
data/node8_active_visual_reanchor_final_search_2026-06-24.csv
```

结果：

| 指标 | 数值 |
|---|---:|
| nav_result | success |
| navigation_arrived | True |
| final_visual_confirmed | True |
| task_success | True |
| final raw error | 0.255 m |
| final AMCL/raw disagreement | 0.077 m |
| max AMCL/raw disagreement | 1.570 m |
| reanchor_count | 10 |

该结果满足严格 VLN 条件：

```text
task_success = navigation_arrived AND final_visual_confirmed
```

但该结果依赖主动视觉闭环：

- 起点 8 向视觉旋转识别 `purple boxes`；
- 视觉命中后用 raw odom 对 AMCL 做 pre-nav reanchor；
- 导航中当 AMCL/raw disagreement 超过阈值时继续 reanchor；
- 到达物理半径后执行 final active visual search；
- final search 第 4 个视角确认 `purple boxes`，confidence `0.95`。

因此它是“主动视觉辅助的 AMCL 路线成功”，不是“纯 AMCL scan-only 成功”。

## 2026-06-24 AMCL Conservative 测试

测试配置：

- `config/nav2_params_amcl_test.yaml`
- `amcl.tf_broadcast=true`
- 不启动静态 `map -> odom`
- rolling `/scan` 使用 `node8_scan_accumulator`
- AMCL conservative scan model：
  - `laser_likelihood_max_dist: 1.5`
  - `sigma_hit: 0.30`
  - `z_hit: 0.35`
  - `z_rand: 0.55`

输出：

```text
data/node8_amcl_conservative_origin_to_purple_boxes_2026-06-24.csv
```

结果：

| 指标 | 数值 |
|---|---:|
| result | canceled |
| duration | 618.8 s |
| final raw odom | `(-0.375, 7.676)` |
| final AMCL | `(-5.348, 5.799)` |
| raw target error | 6.698 m |
| AMCL target error | 5.090 m |
| AMCL/raw disagreement | 5.315 m |
| best raw target error | 6.375 m |
| final Nav2 feedback distance | 5.034 m |

过程解释：

- 前 380 秒左右明显好于之前 AMCL run，AMCL/raw disagreement 基本低于 0.5 m。
- 进入货架 / 走廊区域后分歧快速放大：
  - 约 403 s：1.01 m
  - 约 423 s：2.41 m
  - 约 564 s：4.66 m
  - cancel 前：5.315 m
- Nav2 feedback 和 AMCL 继续显示距离下降，但 raw odom 显示机器人没有进入目标区域。

结论：

保守 scan model 延缓了 AMCL 漂移，但没有解决 shelf/corridor 区域的 scan-map mismatch。剩余问题仍是 AMCL localization / frame consistency，而不是 VLM 或 RPP 控制器本身。

## AMCL v6 / Odom-Dominant 更新

已更新 `config/nav2_params_amcl_test.yaml`：

| 参数 | 旧值 | 新值 |
|---|---:|---:|
| `laser_likelihood_max_dist` | 1.5 | 0.75 |
| `max_beams` | 120 | 80 |
| `sigma_hit` | 0.30 | 0.50 |
| `update_min_a` | 0.15 | 0.25 |
| `update_min_d` | 0.08 | 0.20 |
| `z_hit` | 0.35 | 0.15 |
| `z_rand` | 0.55 | 0.75 |

设计目标：

- 让 Isaac raw odom 主导长距离位姿连续性。
- 保留 scan correction，但降低错误局部几何对粒子云的支配性。
- 如果 v6 仍在同一区域漂移超过 2 m，应停止 AMCL 微调，把 AMCL 问题作为 limitation 写入最终报告。

## 下一步 Clean Test

前置条件：

1. 在 Isaac Sim 中把 Carter / world reset 到 origin。
2. 确认 `/chassis/odom` 接近 `(0, 0)`，yaw 接近 `0`。
3. 重新启动：
   - static `base_link -> base_footprint`
   - `node8_scan_accumulator`
   - Nav2 with `config/nav2_params_amcl_test.yaml`
4. 用 `/set_initial_pose` 设置 AMCL 初始位姿。
5. 运行 pure Nav2 origin -> purple boxes。

成功标准：

- raw odom 进入目标 0.8 m 半径；
- AMCL/raw disagreement 不在 shelf/corridor 区域快速超过 2 m；
- 若 pure Nav2 成功，再运行 strict active visual bridge。

## AMCL v6 Clean-Origin 结果

用户 reset Isaac Sim / Carter 到原点后，使用 v6 odom-dominant AMCL 配置重新运行 origin -> purple boxes。

输出：

```text
data/node8_amcl_v6_odom_dominant_origin_to_purple_boxes_2026-06-24.csv
```

结果：

| 指标 | 数值 |
|---|---:|
| result | canceled_diverged |
| duration | 804.0 s |
| failure reason | AMCL/raw disagreement exceeded 3.0m for 40.0s |
| final raw odom | `(-3.362, 10.002)` |
| final AMCL | `(-5.663, 7.605)` |
| raw target error | 3.045 m |
| AMCL target error | 3.258 m |
| final AMCL/raw disagreement | 3.323 m |
| max AMCL/raw disagreement | 3.333 m |
| best raw target error | 3.044 m |
| final Nav2 feedback distance | 3.120 m |

与 conservative AMCL 对比：

| 配置 | best raw error | final disagreement | 结论 |
|---|---:|---:|---|
| conservative AMCL | 6.375 m | 5.315 m | 仍严重漂移，未到达 |
| v6 odom-dominant AMCL | 3.044 m | 3.323 m | 明显改善，但仍未到达 |

最终判断：

v6 证明降低 scan match 支配性是正确方向，但 scan-only AMCL-localized long-distance navigation 仍没有满足 0.8 m 物理到达标准。当前 Node 8 应收束为 strict active-visual reanchor success + AMCL scan-only limitation，不应继续把 AMCL 参数微调作为主线。

## AMCL 地图诊断

诊断图：

```text
data/runtime/map_diagnostics_node8_reanchor_overlay.png
data/runtime/map_diagnostics_node8_target_crop.png
```

overlay 显示 origin、purple-box 目标、最终位姿和 10 个 reanchor 点均落在 `warehouse_map.png` 的 free cell 内。因此当前问题不是“目标点在障碍物里”，而更像是静态 occupancy map 对 shelf / forklift / purple-box 区域的几何表达不足，和 Isaac rolling `/scan` 看到的实际结构不一致。
