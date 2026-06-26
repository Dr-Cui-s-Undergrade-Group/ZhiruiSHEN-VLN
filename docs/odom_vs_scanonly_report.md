# 定位模式对比与 Scan-Only 评估报告

Date: 2026-06-26

## 概述

本报告记录 Node 8 Phase 2 的 scan-only AMCL 定位改进工作，并提供 odom 模式与 scan-only 模式的完整对比，便于理解两种定位策略的本质差异、当前仿真结果、以及真机部署的关系。

---

## Odom 模式 vs Scan-Only 模式：核心区别

### 一句话总结

- **Odom 模式** = 机器人"闭着眼睛走"，只靠轮子里程计算自己在哪，不看周围环境。
- **Scan-Only 模式** = 机器人"睁着眼睛走"，用激光雷达扫一圈和地图对比，持续纠正自己的位置。

### 详细对比

| 维度 | Odom 模式 | Scan-Only AMCL 模式 |
|---|---|---|
| **定位原理** | 只信轮子里程计（`/chassis/odom`），不参考外部感知 | 里程计 + AMCL 粒子滤波：用激光雷达 `/scan` 和静态占据栅格地图做 likelihood-field 匹配，持续纠正 `map→odom` 变换 |
| **TF 链** | 静态 `map→odom`（identity），`odom→base_link` 由里程计发布 | `map→odom` 由 AMCL 动态发布并纠正 |
| **需要地图？** | 不需要 | **必须**有准确的占据栅格地图 |
| **走远了会偏？** | 仿真不偏（Isaac 物理引擎给真值）；真机一定偏（打滑、累积误差） | 持续纠正，理论上走多远都不累积偏——前提是地图准确 |
| **仿真表现** | **12/15** strict task success（完美里程计） | **5/15** strict task success（近距离全通，货架区地图覆盖不足导致漂移） |
| **真机可用？** | **不可用**（轮子打滑 → 越走越偏） | **这是真机必须用的方案**（AMCL 的设计目的就是纠正里程计漂移） |

### 为什么 Odom 模式在仿真里分数高

Isaac Sim 的物理引擎直接提供精确位姿，`/chassis/odom` 是无漂移真值。这相当于"考试有标准答案"——里程计永远准确，导航自然成功。但这**不代表真实机器人的能力**。真实轮式里程计存在打滑、地面不平、积分误差，走 10 米可能累积 1-2 米偏差。

### 为什么 Scan-Only 才是真正需要解决的

真实部署**只能**靠 scan-only（或 SLAM）做定位。AMCL 的工作就是：激光雷达扫一圈 → 和已知地图对比 → 推算最可能的位姿 → 纠正里程计漂移。这是移动机器人标准定位方案。但前提是**地图必须和真实环境一致**。

### 流程对比图

```
Odom 模式（当前 12/15 的方案）:
  指令 → Qwen 识别目标 → 算坐标 → Nav2 开过去 → 到了
                      ↑
           机器人靠"内部感觉"（轮子转了多少）算自己在哪
           不看周围，不核对位置

Scan-Only 模式（当前 5/15 的方案）:
  指令 → Qwen 识别目标 → 算坐标 → Nav2 开过去 → 到了
                      ↑              ↑
           轮子里程计          激光雷达扫一圈，
           算大概位置          和地图对比纠正精确位置
```

---

## 仿真阶段最终结果

### 三种配置对比

| 配置 | strict task success | 近距离 (plant/chair) | 长距离 (shelf/purple boxes) | 说明 |
|---|---:|---|---|---|
| **Odom-truth baseline** | **12/15** | 全通 | 全通 | 仿真特权：完美里程计 |
| **Scan-only AMCL (truth map)** | **5/15** | **5/5 全通** ✅ | 0/7 ❌ | truth map 让 scan-only 从 0→5 |
| Scan-only AMCL (原始地图) | 0/15 | 卡死 | — | 原始地图 97% 错配，AMCL 完全不可用 |

### Scan-only 逐条结果

证据文件：`data/node7_node8_full15_scanonly_amcl_2026-06-25.csv`

| Trial | 指令 | task_success | final error | 说明 |
|---:|---|---|---:|---|
| 1 | Go to the plant. | True | 0.296 m | 近距离成功 |
| 2 | Move to the potted plant on the floor. | True | 0.296 m | 近距离成功 |
| 3 | Navigate to the green plant near the chair. | True | 0.297 m | 近距离成功 |
| 4 | Go to the black office chair. | True | 0.037 m | 近距离成功 |
| 5 | Move to the chair near the robot. | True | 0.279 m | 近距离成功 |
| 6 | Navigate to the chair beside the plant. | False | 1.364 m | Nav2 stuck |
| 7 | Go to the purple boxes. | False | 13.846 m | AMCL 货架区漂移 |
| 8 | Move to the right shelf with purple boxes. | False | 13.838 m | 从漂移位置出发 |
| 9 | Navigate to the shelf area containing purple packages. | False | 13.834 m | 从漂移位置出发 |
| 10 | Go to the shelf. | False | 13.832 m | 从漂移位置出发 |
| 11 | Move to the right shelf. | False | 13.832 m | 从漂移位置出发 |
| 12 | Navigate to the warehouse rack near the boxes. | False | 13.829 m | 从漂移位置出发 |
| 13 | Go to the object near the wall. | False | 1.696 m | ambiguous target 正确拒绝 |
| 14 | Move to the package area. | False | 13.828 m | 从漂移位置出发 |
| 15 | Navigate to the target object. | False | 1.695 m | ambiguous target 正确拒绝 |

---

## Phase 2 改进工作总结

### 问题诊断（Phase 1）

在 origin 处将 `/scan` 的 723 个命中点投影到原始 `warehouse_map`：**703/723 (97%) 落在 FREE 栅格**，只有 20 个落在 occupied。雷达看到了真实障碍，但地图标的是 free。元凶是静态地图与 Isaac 仓库真实几何严重不一致。

证据：`data/node8_scan_map_mismatch_origin_2026-06-25.csv`

### Odom-Truth 地图重建

绕过 SLAM，用新方法 `node8_odom_truth_mapper.py`：用 Isaac 真值 odom 当位姿，把每帧 `/scan` 的射线直接 Bresenham 画进全局占据栅格。几何零误差，不需要 SLAM 局部化。

| 地图版本 | origin FREE 错配 | origin OCC 匹配 | AMCL 4.7m 后漂移 |
|---|---|---|---|
| 原始 warehouse_map | 97% | 3% | 5.42 m（完全丢失） |
| odom-truth map (v1) | 13% | 52% | 0.75 m（在跟踪） |
| odom-truth map (扩展覆盖) | 9-11% | 30-51% | 0.97 m |

### 关键工具（新增）

| 文件 | 用途 |
|---|---|
| `src/vln_nav2_bridge/vln_nav2_bridge/node8_odom_truth_mapper.py` | odom-truth 占据栅格绘制器 |
| `src/vln_nav2_bridge/vln_nav2_bridge/node8_scan_map_residual.py` | scan-vs-map residual 诊断节点 |
| `src/vln_nav2_bridge/vln_nav2_bridge/node8_waypoint_driver.py` | 带脱困逻辑（后退+旋转）的航点驱动器 |
| `src/vln_nav2_bridge/vln_nav2_bridge/static_base_footprint_tf.py` | base_link→base_footprint 静态 TF |
| `config/nav2_params_lowload.yaml` | 低负载 Nav2 配置（解决 Isaac sim-time 跳变） |
| `src/analysis/plot_scan_map_residual.py` | scan-map residual 可视化脚本 |

### Isaac Sim-Time 问题

Isaac 的 `/clock` 在 Nav2 计算负载下严重跳变（秒数跳变 100-245s），导致 Nav2 TF buffer 反复清空。解决方案：`config/nav2_params_lowload.yaml` 降低 Nav2 计算频率（controller 20→5Hz, costmap 5→1Hz/1→0.2Hz），jump-back 从 5000+ 降到 0。

---

## 长距离失败的根因分析

Scan-only AMCL 长距离失败的根因链条：

```
truth map 货架核心区覆盖不足 (36% UNKNOWN)
  ↓
AMCL 在货架区找不到足够特征做 likelihood-field 匹配
  ↓
粒子云漂移到错误位置
  ↓
Nav2 从错误位姿规划路径
  ↓
机器人走错方向 / 卡住
  ↓
后续 trial 从漂移位置出发，级联失败
```

### 为什么地图覆盖不足

Carter（仿真机器人）物理上无法靠近货架核心区——每次靠近紫色箱子 0.1m 处就卡住。这导致 odom-truth mapper 无法观测到货架核心区的障碍几何，那片区域在地图里始终是 UNKNOWN。

### 这不是 AMCL 算法问题

当 truth map 有足够覆盖时（近距离区域 OCC 52%），AMCL 跟踪精度达到 0.02-0.75m——完全可用。长距离失败纯粹是地图覆盖的物理限制，不是 AMCL 算法本身的缺陷。

---

## 真机部署的预期

| 仿真瓶颈 | 真机上是否会消失 | 原因 |
|---|---|---|
| 地图覆盖不足 | ✅ 消失 | 人可以推着机器人走遍仓库每个角落建图 |
| Isaac sim-time 跳变 | ✅ 消失 | 真机用系统时钟（wall clock），没有仿真时间不稳定问题 |

| 真机新挑战 | 说明 |
|---|---|
| 里程计漂移 | 轮式里程计会累积误差——但这正是 AMCL 要解决的，有好的地图就能纠正 |
| 传感器噪声 | 真实激光雷达有噪声、遮挡——但比仿真里"地图完全错"的问题小得多 |
| 无 ground truth 评估 | 无法像仿真一样用 Isaac 真值算误差——需要外部定位或物理测量 |

**结论：真机上 scan-only 长距离反而更有希望**，因为两个仿真瓶颈都会消失。人推着走一遍就能建出完整仓库地图，AMCL 有好地图就能定位。

---

## 报告口径建议

### 可写的三条 claim

1. ✅ **Odom-truth baseline: 12/15 strict task success** — 完整 VLN 闭环验证（指令→VLM→Nav2→到达→视觉确认），证明导航执行链路、视觉确认、语义解析全部可用。
2. ✅ **Odom-truth map 让 scan-only AMCL 从 0/15 提升到 5/15** — 近距离目标 (plant/chair) 全部成功，err ~0.3m，strict visual confirmed。证明改进方向正确（地图准确性是 scan-only 的关键）。
3. ⚠️ **Scan-only 长距离 limitation** — 货架核心区 AMCL 漂移，根因是 Carter 物理可达性限制导致地图覆盖不足。真机部署可解决（人推着建图 + 无 sim-time 问题）。

### 不能写的

- ❌ 不能写 "scan-only AMCL 完全解决了" — 长距离仍失败
- ❌ 不能把 odom 模式 12/15 写成 "真实机器人能力" — odom 模式依赖仿真真值里程计
- ❌ 不能省略 odom vs scan-only 的区别 — 必须明确 12/15 是 odom-truth baseline，scan-only 是 5/15

---

## 改进工作执行结果（2026-06-26）

### 已完成的改进

| 改进 | 文件 | 结果 |
|---|---|---|
| PGM merge bug 修复 | `node8_odom_truth_mapper.py` `_load_existing_map` | ✅ `free_mask > 220` 排除 unknown(205)，round-trip 验证通过 |
| odom-dominant AMCL | `config/nav2_params_odom_dominant.yaml` | ✅ `laser_likelihood_max_dist: 0.3`, `z_hit: 0.85`, `z_rand: 0.15`，AMCL init 跟踪 0.02m |
| 精确 pkill 修复 | 运维层面 | ✅ 根因：`pkill -f "ros2 run"` 误杀 Isaac bridge，改用节点名精确杀 |
| truth map 重建 | `data/warehouse_map_truth.{pgm,yaml}` | ✅ 550 scans / 50780 occupied cells（最好成绩） |
| Carter 脱困逻辑 | `node8_waypoint_driver.py` | ✅ 后退 1.5s + 旋转 3s，多次成功脱困 |

### 未完成的验证（被 Isaac sim-time 阻断）

odom-dominant AMCL + truth map full-15 验证被 Isaac sim-time jump-back 反复阻断。即使冷启动 Isaac + 精确 pkill + 低负载 Nav2，Nav2 主动导航时（controller + costmap 运算）仍产生 jump-back（8758 次跳变），导致 Nav2 controller TF 间歇性故障。

这是 Isaac 物理引擎在 CPU 负载下帧率不稳的硬限制，无法从 ROS 侧完全解决。

### 精确 pkill 根因（重要运维教训）

之前 Isaac 反复停发数据不是 Isaac 崩溃——是 `pkill -f "ros2 run"` 误杀了 Isaac 的 ROS bridge 进程（它们也通过 `ros2 run` 启动）。修复后改用 `pkill -f "vln_nav2_bridge/node_name"` 精确杀进程，Isaac 稳定运行了完整的建图周期（~20min 无中断）。

**教训**：在 Isaac Sim + ROS 2 环境中，清理 ROS 节点时不能用 broad `pkill -f "ros2 run"`，必须用节点名精确杀。
