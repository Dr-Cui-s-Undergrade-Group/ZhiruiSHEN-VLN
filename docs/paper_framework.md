# 论文框架

> 基于 Node 1-8 的全部仿真工作，搭建可投稿的论文结构。
> 目标会议/期刊：ICRA / IROS / RAL 级别机器人会议。
> 核心卖点：轻量本地 VLM + 严格视觉确认的 VLN 闭环系统，附带定位模式对比分析。

---

## 暂定标题

**Language-Guided Navigation with Strict Visual Confirmation: A Modular VLN Pipeline with Local VLM Grounding and Localization Mode Analysis in Warehouse Environments**

备选标题：
- From Instruction to Confirmation: A Modular Vision-Language Navigation System with On-Device Inference
- Bridging Language and Navigation: Semantic Grounding, Strict Visual Verification, and Localization Analysis for Warehouse VLN

---

## Abstract（200 词以内）

本文提出一种模块化视觉语言导航（VLN）系统，在 Isaac Sim 大型仓库场景中实现自然语言指令到严格视觉确认的完整闭环。系统采用本地 4-bit 量化 Qwen3-VL-2B 进行实时语义 grounding，将指令映射到语义目标坐标，通过 Nav2 Regulated Pure Pursuit 控制器执行导航，并采用分层视觉确认策略（导航中低频检查 + 到达后强制最终扫描 + 有界主动搜索）保证 task success 的严格性。

在 15 条标准化指令基准上，系统在 odom-truth 定位下达到 12/15 strict task success（navigation_arrived AND final_visual_confirmed）和 13/15 physical arrival。我们进一步分析了 odom-truth 与 scan-only AMCL 两种定位模式的本质差异：通过 odom-truth occupancy map 重建，将 scan-only AMCL 从完全不可用（0/15）提升到近距离全通（5/15），并定位长距离失败根因为地图覆盖的物理可达性限制。系统还实现了 shelf/package 语义簇确认修复、safe-start/safe-goal 安全导航检查、以及 Carter 差速底盘脱困逻辑。

**关键词**：Vision-Language Navigation, AMCL, Nav2, Visual Confirmation, Warehouse Robotics, On-Device VLM

---

## I. Introduction

### 1.1 问题背景
- VLN 是机器人领域的核心挑战：自然语言指令 → 自主导航 → 到达目标
- 现有 VLN 研究多聚焦 Habitat/Matterport 离散环境或端到端策略学习
- 工业仓库场景的特殊挑战：长距离导航、货架遮挡、实时定位、严格确认

### 1.2 本文贡献
1. **完整模块化 VLN 管线**：从语言指令到视觉确认的全闭环，各模块可独立替换
2. **严格视觉确认策略**：navigation_arrived AND final_visual_confirmed 双条件，避免假成功
3. **定位模式对比分析**：odom-truth vs scan-only AMCL 的系统性实验，揭示地图准确性是 scan-only 的关键
4. **Odom-truth occupancy map 重建方法**：绕过 SLAM，用真值里程计直接绘制几何精确的占据栅格

### 1.3 论文结构
Section II 相关工作 → Section III 系统设计 → Section IV 实验设置 → Section V 结果 → Section VI 定位模式分析 → Section VII 讨论 → Section VIII 结论

---

## II. Related Work

### 2.1 Vision-Language Navigation
- **R2R** (Anderson et al., CVPR 2018)：VLN 基准任务定义，离散导航图
- **VLN-CE** (Krantz et al., ECCV 2020)：连续环境 VLN，更接近真实机器人
- **VLFM** (Yokoyama et al., 2024)：language-grounded frontier/value maps，在线视觉证据更新
- **VL-Nav** (2025)：RGB/odometry/LiDAR/open-vocabulary detection，在线地图
- **3D-Aware ObjectNav** (Zhang et al., CVPR 2023)：同时探索与识别的 ObjectGoal 导航

### 2.2 视觉语言模型在机器人中的应用
- **Uni-NaVid** (2025)：VLA 架构，5Hz 实时推理，token merging
- **MapNav** (2025)：ASM 标注语义地图，恒定内存
- 本工作区别：使用轻量本地量化 VLM（2B），不依赖云端，在 Isaac Sim 实时仿真中在线推理

### 2.3 移动机器人定位
- **AMCL** (Fox et al., 2003)：粒子滤波激光定位
- **Nav2** 框架：Regulated Pure Pursuit、costmap、lifecycle
- 本工作贡献：odom-truth vs scan-only 的系统性对比，揭示仿真/真机的定位差异

---

## III. System Design

### 3.1 系统架构（附系统图）

```
自然语言指令
    ↓
[Qwen3-VL-2B 语义 Grounding] ← Isaac Sim 实时相机图像
    ↓ target + confidence + horizontal_position
[Text-to-Pose 语义地图映射] ← 预定义语义目标坐标表
    ↓ (target_x, target_y, target_yaw)
[Safe-Start / Safe-Goal 检查] ← 静态占据栅格
    ↓ 安全目标位姿
[Nav2 导航执行] ← RPP 控制器 + rolling scan accumulator
    ↓
[分层视觉确认]
  ├─ 导航中低频视觉检查 (5s interval)
  ├─ 到达后强制最终视觉扫描 (8-step 360°)
  └─ 有界主动搜索 (≤2 moves)
    ↓
task_success = navigation_arrived AND final_visual_confirmed
```

### 3.2 语义 Grounding 模块
- Qwen3-VL-2B-Instruct，4-bit NF4 量化，本地 GPU 推理
- 常驻 worker 进程（非每条指令重新加载），~1.2s 单次推理延迟
- 输出 JSON：target 名称、visible、confidence、horizontal_position、evidence
- 禁止推理失败时静默关键词兜底

### 3.3 语义地图与坐标映射
- 预定义语义目标表：plant (-0.43,-2.92)、chair (-0.54,-0.69)、shelf/package (-6.78,10.96)
- Shelf/package 语义簇：shelf, right shelf, warehouse rack, package area, cart with boxes, purple boxes, purple packages, purple crates
- Observation pose 策略：plant/chair 先导航到朝向目标的安全观测位

### 3.4 Nav2 导航执行
- Controller：Regulated Pure Pursuit（从 DWB 切换），`use_rotate_to_heading: false`
- Motion-compensated scan accumulator：3D 点云在 odom 固定帧累积，发布前转到 base_link
- Safe-start：检查当前位姿在地图中的 free ratio，不安全时先移到 nearest free
- Safe-goal：检查目标位姿，不安全时替换为 nearby free
- Dynamic timeout：根据 odom-to-goal 距离动态扩大

### 3.5 分层视觉确认策略
- **导航中检查**（visual_check_during_nav）：每 5s 或 0.75m 旅行触发一次视觉判断
- **最终视觉扫描**（final_visual_scan）：Nav2 报告到达后，强制 8-step 360° 扫描
- **主动搜索**（active_search）：最终确认失败时，最多 2 步前进搜索
- **semantic_nav_first**：对已知远距离目标，先导航到 semantic observation pose，不做起点 360° 扫描
- 严格条件：`task_success = navigation_arrived AND final_visual_confirmed`

### 3.6 安全导航与脱困
- Safe-start/safe-goal 检查 + nearest free candidate
- Carter 差速底盘脱困：检测 10s 无进展 → 后退 1.5s + 旋转 3s → 恢复导航
- Stuck 检测：AMCL-to-goal 欧氏距离 + Nav2 feedback distance 双重判断

---

## IV. Experimental Setup

### 4.1 仿真环境
- Isaac Sim 仓库场景（warehouse.usd），大型仓库（~20m × 30m）
- Carter 差速驱动机器人，3D 激光雷达 + 立体相机 + IMU
- ROS 2 Humble + Nav2

### 4.2 指令集（15 条）
| 类型 | 指令数 | 示例 |
|---|---|---|
| Plant (近距离) | 3 | "Go to the plant." |
| Chair (近距离) | 3 | "Go to the black office chair." |
| Purple boxes/Shelf (长距离 ~12m) | 6 | "Go to the purple boxes." |
| Package area (长距离) | 1 | "Move to the package area." |
| Ambiguous target | 2 | "Go to the object near the wall." |

### 4.3 评估指标
| 指标 | 定义 |
|---|---|
| Physical arrival | 最终位姿进入 0.8m 成功半径 |
| Navigation arrived | Nav2 报告到达 + bridge odom 确认 |
| Visual confirmed | 最终视觉扫描中目标可见 (confidence ≥ 0.6) |
| **Strict task success** | **navigation_arrived AND final_visual_confirmed** |
| Final error | 最终 odom 位姿到目标中心的欧氏距离 |

### 4.4 定位模式
- **Odom-truth baseline**：`amcl.tf_broadcast: false` + 静态 `map→odom` identity
- **Scan-only AMCL**：`amcl.tf_broadcast: true`，AMCL 粒子滤波激光定位
- **Truth map**：odom-truth mapper 用真值里程计直接 Bresenham 绘制占据栅格

---

## V. Results

### 5.1 Odom-Truth Baseline（主结果）

| 指标 | 结果 |
|---|---|
| Strict task success | **12/15** |
| Physical arrival | 13/15 |
| Final visual confirmed | 12/15 |
| Navigation arrived | 13/15 |
| Concrete target success (hotfix 后) | 13/13 |
| Ambiguous target rejection | 2/2 |

15 条逐条结果见 `data/node7_node8_full15_clean_odom_2026-06-24.csv`。

### 5.2 Scan-Only AMCL

| 配置 | Strict task success | 近距离 | 长距离 |
|---|---|---|---|
| 原始地图 | 0/15 | 卡死 | — |
| Truth map | **5/15** | **5/5 全通** | 0/7 |

### 5.3 Node 8 长距离定向成功
- Odom-truth origin→purple boxes：success，final error 0.348m，复现两次
- Visual-anchor gated AMCL reanchor：strict success，final error 0.255m，AMCL/raw disagreement 0.077m

---

## VI. Localization Mode Analysis

### 6.1 Odom vs Scan-Only 本质差异

| 维度 | Odom 模式 | Scan-Only AMCL |
|---|---|---|
| 定位原理 | 只信轮子里程计 | 里程计 + 激光雷达对比地图 |
| 走远偏不偏 | 仿真不偏；真机一定偏 | 持续纠正 |
| 需要地图 | 不需要 | 必须 |
| 仿真表现 | 12/15 | 5/15 |
| 真机可用 | 不可用 | **必须用** |

### 6.2 地图准确性是 scan-only 的关键
- Phase 1 诊断：原始地图 97% scan 命中落在 FREE（地图和真实环境严重不一致）
- Odom-truth map 重建：origin 错配从 97% 降到 13%，AMCL 漂移从 5.4m 降到 0.75m
- Scan-only 从 0/15 提升到 5/15（近距离全通）

### 6.3 长距离失败根因
- Truth map 货架核心区覆盖不足（36% UNKNOWN）
- Carter 物理上无法靠近货架核心区建图（0.1m 处物理卡住）
- 这是地图覆盖的物理限制，不是 AMCL 算法缺陷

---

## VII. Discussion

### 7.1 严格视觉确认的必要性
- Physical arrival 13/15 但 strict task success 只有 12/15：到达 ≠ 看到目标
- 连续环境 VLN 的特有挑战：相机视角决定能否确认，不仅仅是坐标到达

### 7.2 仿真 vs 真机的定位差异
- Odom 模式的 12/15 是仿真特权（完美里程计），不代表真机能力
- Scan-only AMCL 的 5/15 代表了真实定位链路
- 真机上两个仿真瓶颈（地图覆盖 + sim-time）都会消失

### 7.3 局限性
- 仿真环境：Isaac sim-time 在 Nav2 计算负载下不稳定
- 物理限制：Carter 差速底盘在狭窄区域转向困难
- 地图覆盖：odom-truth mapper 受机器人可达性限制
- 指令集规模：15 条指令，需要更大规模 benchmark 验证泛化性

---

## VIII. Conclusion

本文提出了一种模块化 VLN 系统并在 Isaac Sim 仓库场景中验证了完整闭环。主要贡献：
1. Odom-truth baseline 12/15 strict task success
2. Odom-truth map 让 scan-only AMCL 从 0/15 提升到 5/15
3. 系统性定位模式对比分析，揭示地图准确性是 scan-only 的关键

未来工作：在实体机器人上验证 scan-only 长距离导航（真机无地图覆盖限制和 sim-time 问题），扩展指令集规模，探索端到端 VLA 策略替代模块化管线。

---

## 附：图表清单

| 图表 | 内容 | 来源 |
|---|---|---|
| Fig. 1 | 系统架构图 | 新制作 |
| Fig. 2 | 15 条指令的目标 vs 最终位姿散点图 | `assets/node6_target_vs_final_pose.png` |
| Fig. 3 | 失败分类柱状图 | `assets/node6_failure_taxonomy.png` |
| Fig. 4 | Node 7 ablation 对比图 | `assets/node7_ablation_comparison.png` |
| Fig. 5 | Odom vs scan-only AMCL disagreement 曲线 | `data/node8_amcl_truth_expanded_disagreement_2026-06-25.csv` |
| Fig. 6 | Scan-map residual 热力图 | `src/analysis/plot_scan_map_residual.py` |
| Fig. 7 | AMCL map diagnostic overlay | `assets/node8_map_diagnostics_reanchor_overlay_2026-06-24.png` |
| Table 1 | 15 条逐条结果（odom-truth） | `data/node7_node8_full15_clean_odom_2026-06-24.csv` |
| Table 2 | 定位模式对比 | 本文 Section VI |
| Table 3 | Scan-only 逐条结果 | `data/node7_node8_full15_scanonly_amcl_2026-06-25.csv` |

## 附：参考文献清单

1. Anderson et al., "Room-to-Room: Vision-Language Navigation," CVPR 2018.
2. Krantz et al., "Beyond the Nav-Graph: VLN in Continuous Environments," ECCV 2020.
3. Fox et al., "Monte Carlo Localization: Efficient Position Estimation for Mobile Robots," AAAI 1999.
4. Macenski et al., "Nav2: A 2D Costmap-Based Navigation Framework," (arXiv:2305.20026), 2023.
5. Yokoyama et al., "VLFM: Vision-Language Frontier Maps," 2024.
6. Zhang et al., "3D-Aware Object Goal Navigation," CVPR 2023.
7. BEVBert, "Hybrid Map Pre-training," 2023.
8. MapNav, "Annotated Semantic Map," 2025.
9. Uni-NaVid, "Streaming VLA Architecture," 2025.
10. Qwen3-VL Technical Report, 2025.
