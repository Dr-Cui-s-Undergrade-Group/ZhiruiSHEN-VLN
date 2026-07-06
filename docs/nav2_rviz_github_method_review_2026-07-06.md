# Nav2 / RViz / VLN 方法调研与仓库落地记录 - 2026-07-06

## 已阅读的 GitHub / 官方资料

- `ros-navigation/navigation2`：官方 Nav2 仓库，重点看 `nav2_bringup` 和默认 RViz 配置。
- `ros-navigation/navigation2_tutorials`：官方教程仓库，用于确认 Nav2 项目通常如何组织 bringup 和调试显示。
- `ros-perception/pointcloud_to_laserscan`：官方 ROS 2 点云转 LaserScan 仓库，用于确认 3D 点云投影为 2D scan 的参数边界。
- `SteveMacenski/spatio_temporal_voxel_layer`：3D lidar / RGBD 点云进入局部或全局代价地图的进阶方案。
- `rai-opensource/vlfm`、`vlmaps/vlmaps`、`jacobkrantz/VLN-CE`、`Ram81/goat-bench`：VLN / ObjectNav 方向项目，用于判断语言视觉层与几何导航层的合理分工。

## 资料结论

### 1. Nav2 的正确使用方式

官方 `nav2_bringup` 明确说明它是一个可修改模板，实际机器人项目应该有自己的 `<robot>_nav` 或项目内 bringup。对当前仓库来说，应该保留 `config/nav2_params_lowload.yaml` 这种项目内参数文件，并为 RViz、TF、轨迹、雷达显示做自己的 launch，而不是每次手动敲散落命令。

Nav2 应该继续负责：

- AMCL / map / odom / base_link 的定位链路；
- 全局规划；
- 局部控制；
- recovery 行为；
- `goToPose()` 和 `goThroughPoses()` 执行。

VLN/VLM 不应该直接长期接管 `/cmd_vel`，除非是短时视觉扫描或受控 recovery 诊断。

### 2. RViz 的正确显示方式

Nav2 默认 RViz 配置使用：

- `Fixed Frame: map`
- `Map`
- `LaserScan`
- `Path`
- `Global Costmap`
- `Local Costmap`
- `TF`
- `Footprint`

这和当前仓库方向一致，但你的 3D 雷达不能只用 `/scan` 显示。`/scan` 在本项目里应被视为 Nav2/AMCL 用的二维投影或调试输入，不代表完整 3D 雷达，会丢掉高度和完整空间结构。RViz 主显示应该看完整 3D 点云，导航调试看 `/scan`。

### 3. 点云与 LaserScan 的正确分工

`pointcloud_to_laserscan` 官方 README 说明它是把 `PointCloud2` 投影成 `LaserScan`，并且有这些关键参数：

- `target_frame`
- `min_height`
- `max_height`
- `angle_min`
- `angle_max`
- `range_min`
- `range_max`
- `transform_tolerance`

因此当前仓库最合理的传感器分工是：

- `/front_3d_lidar/lidar_points`：完整 3D 雷达源；
- `/front_3d_lidar/map_points`：完整 3D 雷达转到 `map` 后给 RViz 显示；
- `/scan`：过滤/投影后的 2D 输入，只服务 AMCL 和 local costmap，不作为 3D 雷达主显示。

不要把 `/scan` 当成“雷达是否完整工作”的唯一证据。

### 4. 3D 代价地图的进阶方案

STVL 和 Nav2 VoxelLayer 都可以处理 3D 点云，但不应该作为当前第一优先级直接替换现有链路。

推荐路线：

1. 当前阶段：稳定 2D `/scan` + AMCL + local costmap。
2. 下一阶段：用 Nav2 VoxelLayer 接 PointCloud2 做局部 3D 障碍基线。
3. 再下一阶段：如果局部动态障碍/长期障碍记忆明显影响导航，再评估 STVL。

原因是 STVL 的优势是时序体素和 decay，但会增加参数复杂度。当前主要 blocker 仍是地图一致性、AMCL/odom 漂移、候选目标点和 RViz/TF 显示稳定性。

### 5. VLN / ObjectNav 给当前仓库的启发

VLFM 的核心思路是建立 occupancy / frontier / language-grounded value map，然后选择最有希望的探索方向。VLN-CE 强调连续环境里的轨迹、路径长度、SPL、成功率。GOAT-Bench 强调多模态目标、连续子任务和效率。

对应到本仓库，最合适的方法是：

- VLN/VLM 层负责语言解析、目标类别、候选 observation pose、候选排序；
- Nav2 planner 用于筛选候选点可达性和路径长度；
- Nav2 controller 执行运动；
- 最终指标分开记录 `physical_arrival`、`visual_confirmed`、`task_success`、`trajectory_length`、`SPL`、`AMCL/odom disagreement`。

## 已在仓库中完成的落地

新增正式 ROS 包内节点：

- `vln_nav2_bridge.odom_tf_broadcaster`
- `vln_nav2_bridge.odom_to_path`
- `vln_nav2_bridge.pointcloud_to_map_cloud`

新增统一可视化 launch：

```bash
ros2 launch vln_nav2_bridge rviz_track.launch.py
```

这个 launch 会启动：

- `/chassis/odom -> /tf` 的 odom TF broadcaster；
- `/executed_path`，在 `map` 坐标系下显示实际轨迹；
- `/front_3d_lidar/map_points`，完整 3D 雷达转到 `map` 坐标系；
- RViz，使用 `use_sim_time:=true` 和项目内 `robot_track.rviz`。

新增/安装的文件：

```text
src/vln_nav2_bridge/launch/rviz_track.launch.py
src/vln_nav2_bridge/config/robot_track.rviz
src/vln_nav2_bridge/vln_nav2_bridge/odom_tf_broadcaster.py
src/vln_nav2_bridge/vln_nav2_bridge/odom_to_path.py
src/vln_nav2_bridge/vln_nav2_bridge/pointcloud_to_map_cloud.py
```

已更新：

```text
src/vln_nav2_bridge/setup.py
src/vln_nav2_bridge/package.xml
config/robot_track.rviz
```

## 当前验证结果

构建通过：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vln_nav2_bridge --symlink-install
```

统一 launch 已启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vln_nav2_bridge rviz_track.launch.py start_base_footprint_tf:=false
```

话题验证：

```text
/front_3d_lidar/map_points
  frame_id: map
  width: about 7600 points/frame

/executed_path
  frame_id: map

/scan
  still available for Nav2 / AMCL
```

RViz 日志没有新的 raw lidar queue full 报警。现在 RViz 中应主要观察：

- `Full 3D Lidar Map Points`
- `Executed Path`
- `Nav2 Global Plan`
- `Nav2 Local Plan`
- `Static Map`
- `TF`

## 当前最合适的实验方法

1. 启动 Isaac Sim。
2. 启动导航用 scan：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vln_nav2_bridge pointcloud_to_scan_bridge.launch.py
```

3. 启动 Nav2：

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  autostart:=True \
  map:=/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/warehouse_map.yaml \
  rviz:=False \
  params_file:=/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/config/nav2_params_lowload.yaml
```

4. 设置 AMCL 初始位姿。
5. 启动 RViz/轨迹/完整雷达：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vln_nav2_bridge rviz_track.launch.py start_base_footprint_tf:=false
```

6. 运行候选点策略或短程 smoke test。

## 最终判断

当前最适合本项目的方式是：

- 不把完整 3D 雷达压缩成 `/scan` 后再拿它代表雷达显示；
- `/scan` 只作为 Nav2/AMCL 的二维输入；
- RViz 用 `/front_3d_lidar/map_points` 显示完整 3D 点云；
- VLN 层做语言、目标候选、候选选择和记忆；
- Nav2 层做定位、规划、控制和恢复；
- 实验结果按连续导航指标和视觉确认指标拆开记录。

这个方案最贴近官方 Nav2 使用方式，也最符合 VLFM / VLN-CE / GOAT-Bench 这类项目给出的架构方向。
