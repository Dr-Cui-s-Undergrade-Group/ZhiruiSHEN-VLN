# Node 6 集成仿真评估启动手册

Node 6 的目标是跑完整闭环：

```text
Isaac Sim 实时场景/相机 -> ROS 2/Nav2 -> Qwen3-VL-2B 本地 GPU 推理 -> 坐标解析 -> Nav2 导航
```

当前默认使用 `Qwen3-VL-2B-Instruct`，并让模型推理走 0 号 GPU。只有显存不够或调试兼容问题时，才考虑 `force_cpu:=true`。

---

## Node 6 任务定义

Node 6 不是只证明系统能跑通一次，而是要做**集成仿真评估**。也就是说，需要把 Node 5 已经打通的链路放到 Isaac Sim 仓库场景中，连续运行一组语言条件导航任务，并记录成功率和失败原因。

需要完成：

1. 准备至少 `15` 条自然语言导航指令。
2. 每条指令都执行完整闭环：
   ```text
   语言指令 -> Qwen 视觉/语言解析 -> 坐标解析 -> Nav2 规划控制 -> 机器人实际移动
   ```
3. 每条任务记录：
   - 指令内容
   - 模型原始输出
   - 解析方式：`json` / `instruction_fallback` / `fallback` / `failed`
   - 目标坐标
   - 是否成功到达
   - 失败原因和备注
4. 统计：
   - 导航成功率
   - 语言 grounding 准确率
   - JSON 解析率
   - fallback 使用率
5. 最后形成 Node 6 中期评估报告，说明当前系统失败主要来自视觉识别、语言歧义、坐标映射还是 Nav2 执行。

当前进度：

| Trial | Instruction | Result | Notes |
| --- | --- | --- | --- |
| 1 | Go to the plant | success | 已正确移动到植物边上，说明实时相机 + Qwen GPU 推理 + 坐标解析 + Nav2 执行闭环可用 |

结论：现在已经完成 Node 6 的第 1 条有效试验。接下来不是继续证明能跑，而是按下面的实验表继续跑满至少 15 条，并记录结果。

---

## 0. 启动前检查

确认当前工程目录：

```bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
```

检查 GPU：

```bash
nvidia-smi
```

推荐状态：

- Isaac Sim 启动后仍至少剩余约 5GB 显存。
- Node 6 在线主模型继续使用 `Qwen3-VL-2B-Instruct`。
- 不建议同机直接换 8B 模型做在线主链路，容易 OOM 或明显变慢。

---

## 1. 终端 1：启动 Isaac Sim

```bash
conda activate isaaclab
isaacsim
```

进入 Isaac Sim 后需要确认：

1. 仓库场景已加载。
2. 机器人已在场景中。
3. ROS 2 Bridge 已启用。
4. RGB Camera 已配置并通过 ROS2 Camera Helper 发布图像 topic。
5. 3D LiDAR 或点云话题正常发布。

如果还没有相机 topic，后面的 Qwen 只能看静态样例图，Node 6 的实时视觉评估就不成立。

---

## 2. 终端 2：启动必要静态 TF

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint --ros-args -p use_sim_time:=True
```

保持这个终端运行。

注意：使用 `nav2_bringup bringup_launch.py` 的 AMCL 定位模式时，不要再额外启动静态 `map -> odom`。`map -> odom` 应由 AMCL 发布；如果同时运行静态 `map -> odom`，容易让机器人在 map 中落到障碍物上，Nav2 `GridBased` 会直接无法规划。只有不启动 AMCL、只做最小 TF 连通性冒烟测试时，才临时使用静态 `map -> odom`。

---

## 3. 终端 3：点云转 2D 激光

```bash
source /opt/ros/humble/setup.bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p use_sim_time:=True \
  -r cloud_in:=/front_3d_lidar/lidar_points \
  -r scan:=/scan
```

保持这个终端运行。

如果 Nav2 收不到 `/scan`，先检查 Isaac 是否还在发布点云：

```bash
ros2 topic list | rg 'lidar|point|scan'
```

当前 Isaac Sim 仓库场景验证过的点云输入是：

```text
/front_3d_lidar/lidar_points
```

---

## 4. 终端 4：启动 Nav2 和 RViz

```bash
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  autostart:=True \
  map:=/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/warehouse_map.yaml \
  rviz:=True
```

确认 RViz 中：

- 地图正常显示。
- 机器人定位正常。
- `/scan` 正常刷新。
- TF 不报 `map` / `odom` 缺失。

如果当前还有旧的静态 `map -> odom` 进程，先停掉它，再启动 Nav2/AMCL：

```bash
pgrep -af "static_transform_publisher .* map odom"
```

如果上面命令有输出，关掉对应的静态 TF 终端后再启动 Nav2；不要让这个进程和 AMCL 同时存在。

如果刚重置过小车位置，给 AMCL 一个初始位姿。可以在 RViz 用 `2D Pose Estimate` 点到地图原点附近，也可以发布一次 `/initialpose`：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"
```

### 4.1 Nav2 地图预检

在重跑 Node 6 前，先检查当前 `/chassis/odom` 位姿和默认语义目标是否落在 `warehouse_map` 可通行区域：

```bash
source /opt/ros/humble/setup.bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
source install/setup.bash
ros2 run vln_nav2_bridge node6_map_preflight \
  --odom-topic /chassis/odom \
  --robot-radius-m 0.30
```

如果输出类似下面这样，说明当前机器人在地图占据栅格中，Nav2 规划失败是预期现象，需要先修正 Isaac 初始位姿、AMCL 初始位姿或 TF 链路：

```text
[BLOCKED] current_odom:/chassis/odom: world=(-1.061, -1.730, yaw=1.692), center=occupied
```

默认目标 `plant`、`chair`、`shelf/package_area` 如果都是 `[OK]`，优先处理当前机器人 map 位姿，而不是先改语义目标坐标。

---

## 5. 终端 5：查找实时相机 topic

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | rg 'image|rgb|camera'
```

常见 topic 可能类似：

```text
/rgb
/camera/rgb
/camera/image
/front_camera/rgb
/front_camera/image_raw
```

查看 topic 类型：

```bash
ros2 topic info /YOUR_CAMERA_IMAGE_TOPIC
```

如果类型是：

- `sensor_msgs/msg/Image`：后面使用 `image_topic:=...`
- `sensor_msgs/msg/CompressedImage`：后面使用 `compressed_image_topic:=...`

如果这里找不到任何图像 topic，需要先回 Isaac Sim 里配置 ROS2 Camera Helper。

---

## 6. 终端 6：启动 Node 5 桥接节点，用 GPU 跑 Qwen

先进入工程并加载 ROS 包：

```bash
source /opt/ros/humble/setup.bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
source install/setup.bash
```

如果相机 topic 是 `sensor_msgs/msg/Image`，使用：

```bash
ros2 launch vln_nav2_bridge vln_nav2_bridge_local.launch.py \
  image_topic:=/YOUR_CAMERA_IMAGE_TOPIC \
  require_fresh_image:=true \
  force_cpu:=false \
  gpu_device:=0
```

如果相机 topic 是 `sensor_msgs/msg/CompressedImage`，使用：

```bash
ros2 launch vln_nav2_bridge vln_nav2_bridge_local.launch.py \
  compressed_image_topic:=/YOUR_COMPRESSED_IMAGE_TOPIC \
  require_fresh_image:=true \
  force_cpu:=false \
  gpu_device:=0
```

关键参数含义：

- `force_cpu:=false`：不强制 CPU。
- `gpu_device:=0`：让 Qwen 子进程使用 0 号 GPU。
- `require_fresh_image:=true`：强制要求收到实时相机图像，避免偷偷退回静态样例图。

如果启动后看到：

```text
force_cpu=False, gpu_device=0
Subscribed raw image topic: ...
```

说明桥接节点参数正确。

---

## 7. 终端 7：确认 Qwen 是否真的占用显存

在发指令前后都可以看：

```bash
nvidia-smi
```

或只看计算进程：

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader
```

正常情况：

- 发送第一条指令后，会出现 `python` 或 `python3.11` 推理进程。
- Qwen3-VL-2B 4-bit 推理大约占用 3GB 到 4GB 级别显存，具体取决于 Isaac Sim 已占用多少和当前图像大小。

---

## 8. 终端 8：发布自然语言导航指令

这是手动单条测试方式。确认某一类目标是否能跑通时使用。

```bash
source /opt/ros/humble/setup.bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
source install/setup.bash
```

发布一条测试指令：

```bash
ros2 topic pub /vln_instruction std_msgs/msg/String "{data: 'Go to the plant'}" -1
```

其他示例：

```bash
ros2 topic pub /vln_instruction std_msgs/msg/String "{data: 'Go to the black office chair'}" -1
ros2 topic pub /vln_instruction std_msgs/msg/String "{data: 'Go to the right shelf with purple boxes'}" -1
ros2 topic pub /vln_instruction std_msgs/msg/String "{data: 'Move to the shelf near the purple boxes'}" -1
```

观察 Node 5 终端中的关键日志：

```text
Received instruction: ...
Model output: ...
Resolved target (json): ...
Published pose to /vln_goal_pose
Navigation succeeded.
```

重点看 `Resolved target (...)`：

- `json`：模型返回 JSON，模型参与了目标解析。
- `instruction_fallback`：没有成功解析模型 JSON，靠关键词兜底。
- `fallback`：靠模型文本和指令混合关键词兜底。

Node 6 评估时，不能只记录导航是否成功，也要记录解析方式。

---

## 9. 推荐：全自动跑 Node 6

现在推荐使用自动评估脚本，不需要手动一条一条发布 `/vln_instruction`。

前提：

1. Isaac Sim 已经启动，场景正在播放。
2. 静态 TF、点云转 `/scan`、Nav2/RViz 已经启动。
3. Node 5 桥接节点已经重启到最新版，并且能看到：
   ```text
   Listening on /vln_instruction
   Subscribed raw image topic: /front_stereo_camera/left/image_raw
   force_cpu=False, gpu_device=0
   ```
4. 存在 `/vln_trial_result`、`/amcl_pose` 和 `/chassis/odom`：
   ```bash
   ros2 topic list | rg 'vln_trial_result|amcl_pose|chassis/odom'
   ```

启动自动 15 条评估：

```bash
source /opt/ros/humble/setup.bash
cd /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN
source install/setup.bash
ros2 run vln_nav2_bridge node6_auto_trials \
  --output /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/node6_auto_trials.csv \
  --timeout-sec 900 \
  --pose-topic /amcl_pose \
  --odom-topic /chassis/odom \
  --success-radius-m 0.8
```

如果 `--output` 误写成目录，例如：

```bash
--output /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/
```

新版脚本会自动写到：

```text
/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/node6_auto_trials.csv
```

如果中途崩溃或手动停止，可以从指定 trial 继续。例如 Trial 1 已经成功，要从 Trial 2 继续：

```bash
ros2 run vln_nav2_bridge node6_auto_trials \
  --output /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/node6_auto_trials_from_2.csv \
  --start-index 2 \
  --timeout-sec 900 \
  --pose-topic /amcl_pose \
  --odom-topic /chassis/odom \
  --success-radius-m 0.8
```

它会自动完成：

1. 按默认 15 条 Node 6 指令逐条发布到 `/vln_instruction`。
2. 等待 bridge 返回 `/vln_trial_result`。
3. 记录模型输出的目标坐标：`target_x, target_y, target_yaw`。
4. 记录模型侧信息：`model_target, visible, confidence, parse_method`。
5. 记录导航结果：`nav_result, failure_reason`。
6. 优先订阅 `/amcl_pose`，记录机器人最终位姿：`final_x, final_y, final_yaw`。
7. 如果 `/amcl_pose` 暂时没有数据，使用 `/chassis/odom` 兜底，并在 `final_pose_source` 里标记来源。
8. 自动计算最终误差：`final_error_m` 和 `within_success_radius`。

自动评估结束后主要看两个文件：

```text
data/node6_trials.csv       # bridge 每条任务的原始结果追加日志
data/node6_auto_trials.csv  # 自动评估汇总表，含 trial_id 和最终位姿误差
```

如果只想跑自定义指令，把指令写到文本文件，每行一条：

```bash
ros2 run vln_nav2_bridge node6_auto_trials \
  --instructions-file /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/my_node6_trials.txt \
  --output /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/node6_auto_trials_custom.csv
```

注意：这个脚本负责自动发任务和统计结果，不负责自动启动 Isaac Sim。Isaac、Nav2 和 bridge 仍然要先处于运行状态。

### 本次已验证的全自动结果

2026-06-10 重置小车并只保留 Isaac Sim 后，重新启动 TF、点云转 `/scan`、Nav2、`base_footprint` TF、AMCL 初始位姿和 VLN bridge，然后运行：

```bash
ros2 run vln_nav2_bridge node6_auto_trials \
  --output /home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/<历史输出文件>.csv \
  --start-index 1 \
  --timeout-sec 900 \
  --pose-topic /amcl_pose \
  --odom-topic /chassis/odom \
  --success-radius-m 0.8
```

该次历史 CSV 已在数据清理时删除，结果摘要保留在本节和 `logs/2026-06-10.md` 中；当前 Node 6 结果以 `data/node6_auto_trials_2026-06-12_combined.csv` 为准。

汇总结果：

```text
Navigation success: 15/15
Final pose within 0.80 m: 15/15
JSON parse rate: 15/15
Fallback rate: 0/15
Max final error: 0.268 m
```

需要注意：这说明 Nav2 执行和坐标闭环已经稳定，但不能直接等价为视觉 grounding 满分。部分指令的模型输出 `visible=False`，但仍返回固定坐标；最后 3 条模糊指令也都被解析到 `shelf with purple boxes`。所以 Node 6 报告里应把导航成功率和 grounding 准确率分开写。

---

## 10. Node 6 要做的实验

至少准备 15 条语言条件导航任务。建议分成 5 类，每类 3 条。

### A. Plant 类

```text
Go to the plant.
Move to the potted plant on the floor.
Navigate to the green plant near the chair.
```

### B. Chair 类

```text
Go to the black office chair.
Move to the chair near the robot.
Navigate to the chair beside the plant.
```

### C. Purple Boxes 类

```text
Go to the purple boxes.
Move to the right shelf with purple boxes.
Navigate to the shelf area containing purple packages.
```

### D. Shelf 类

```text
Go to the shelf.
Move to the right shelf.
Navigate to the warehouse rack near the boxes.
```

### E. 模糊/失败用例

```text
Go to the object near the wall.
Move to the package area.
Navigate to the target object.
```

模糊用例不是为了全部成功，而是用来分析失败原因。

---

## 11. 每次实验记录格式

建议记录到 `data/node6_trials.csv` 或日志 markdown 中。

字段建议：

```text
trial_id
instruction
camera_topic
model_output
parse_method
target_x
target_y
target_yaw
final_pose_source
final_x
final_y
final_yaw
final_error_m
within_success_radius
visible
confidence
nav_result
failure_reason
notes
```

示例表格：

| Trial | Instruction | Parse Method | Target | Final Pose | Error | Visible | Confidence | Nav Result | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | Go to the plant | json | (-0.43, -2.92) | (-0.51, -2.40) | 0.53m | true | 0.95 | success | 正常 |
| 2 | Go to the chair | instruction_fallback | (-0.54, -0.69) | (-0.60, -0.83) | 0.15m | unknown | unknown | success | 模型未返回 JSON |
| 3 | Go to the target object | failed | - | - | - | - | - | failed | 指令过于模糊 |

---

## 12. 建议先 dry run，再真导航

如果只想先检查模型输出和坐标解析，不让车动，可以启动时加：

```bash
ros2 run vln_nav2_bridge vln_node_local --ros-args \
  -p use_sim_time:=True \
  -p dry_run:=True \
  -p image_topic:=/YOUR_CAMERA_IMAGE_TOPIC \
  -p require_fresh_image:=True \
  -p force_cpu:=False \
  -p gpu_device:=0
```

确认输出稳定后，再改回：

```bash
-p dry_run:=False
```

---

## 13. 常见问题

### 1. 没有相机 topic

现象：

```text
ros2 topic list | rg 'image|rgb|camera'
```

没有输出。

处理：

- 回 Isaac Sim 检查 ROS2 Camera Helper。
- 确认 ROS 2 Bridge 已启用。
- 确认仿真正在播放。

### 2. Node 5 报没有实时图像

如果使用了：

```bash
require_fresh_image:=true
```

但没有收到图像，Node 5 会拒绝使用静态图。

处理：

- 检查 `image_topic` 是否写对。
- 检查 topic 类型是否是 `sensor_msgs/msg/Image`。
- 如果是压缩图，改用 `compressed_image_topic:=...`。

### 3. Qwen 没有用显存

检查启动参数：

```bash
force_cpu:=false
gpu_device:=0
```

检查显存：

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader
```

如果仍然没有 GPU 进程，检查终端里是否手动设置过：

```bash
echo $CUDA_VISIBLE_DEVICES
```

正常情况下 Node 5 子进程会覆盖为 `CUDA_VISIBLE_DEVICES=0`。

### 4. 显存不够

处理优先级：

1. 降低 Isaac Sim 视窗分辨率。
2. 关闭不必要的渲染窗口或高质量显示。
3. 降低相机发布分辨率。
4. 保持 `Qwen3-VL-2B-Instruct`，不要换 8B。
5. 最后才考虑 `force_cpu:=true`，但会明显变慢。

### 5. 模型识别不稳定

优先检查：

- 相机是否真的看到目标。
- 目标是否太小。
- 光照是否过暗或过曝。
- 图像是否来自机器人视角，而不是 Isaac 编辑器 Perspective 视角。
- `Resolved target` 是否是 `json`，还是 fallback。

如果小目标仍然差，Node 7 可以加 GroundingDINO 或 YOLO-World 做开放词表检测前端。

---

## 14. Node 6 完成标准

Node 6 至少需要形成：

1. 15 条语言导航任务。
2. 每条任务的模型输出、解析方式、目标坐标和导航结果。
3. 成功率统计。
4. 失败案例分析。
5. 至少一张 RViz/Isaac 截图或视频证据。
6. 一段总结：当前系统失败主要来自视觉识别、语言歧义、坐标映射还是 Nav2 执行。

建议最终汇总指标：

```text
Navigation Success Rate = 成功导航次数 / 总任务次数
Grounding Accuracy = 正确目标解析次数 / 总任务次数
JSON Parse Rate = method=json 次数 / 总任务次数
Fallback Rate = fallback 次数 / 总任务次数
```
