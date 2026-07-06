# Orange Pi 真车部署依赖说明

这个文件是给人看的部署说明，面向 Orange Pi 真车端。这里刻意不包含
Isaac Sim 依赖，也不建议把 Qwen3-VL 大模型直接放到 Orange Pi 上跑。

## 推荐架构

建议分成两台机器：

- Orange Pi：跑 ROS 2、Nav2、传感器驱动、小车控制和 VLN bridge。
- 外部模型机：跑 Qwen3-VL 推理，最好是 x86_64 + NVIDIA GPU。

原因：当前项目的 Qwen3-VL 推理依赖 PyTorch、Transformers 和 bitsandbytes
4bit 量化。这套东西在大多数 Orange Pi ARM 板子上不现实，尤其是没有 CUDA
时，会非常慢，甚至装不上完整量化依赖。

## Orange Pi 系统要求

- Ubuntu 22.04，或其他兼容 ROS 2 Humble 的 64 位 Linux 系统
- 64 位 ARM 用户态，也就是 `aarch64`
- ROS 2 Humble
- Nav2
- 小车驱动必须能提供这些接口：
  - RGB 相机话题，类型最好是 `sensor_msgs/msg/Image` 或 `CompressedImage`
  - 真实 3D 雷达话题，类型是 `sensor_msgs/msg/PointCloud2`
  - Nav2 用二维投影/调试话题，类型是 `sensor_msgs/msg/LaserScan`
  - 里程计话题，类型是 `nav_msgs/msg/Odometry`
  - TF 树：`map -> odom -> base_link`
  - 速度控制话题，通常是 `/cmd_vel`，类型是 `geometry_msgs/msg/Twist`

## Orange Pi 上用 apt 安装的包

在小车上安装：

```bash
sudo apt update
sudo apt install -y \
  python3-pip python3-venv python3-colcon-common-extensions \
  ros-humble-ros-base \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander \
  ros-humble-tf-transformations \
  ros-humble-tf2-ros ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs \
  ros-humble-sensor-msgs-py \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-slam-toolbox \
  ros-humble-rviz2
```

如果真车已经直接发布 `/scan`，`pointcloud-to-laserscan` 可以不装。
但如果要保留完整 3D 雷达 RViz 显示，仍然应该保留真实 `PointCloud2`
话题，并用本仓库的 `pointcloud_to_map_cloud` 发布 map-frame 点云。
如果已经有建好的真实地图，`slam-toolbox` 只在重新建图时需要。

## Orange Pi 上用 pip 安装的包

先 source ROS，再安装：

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install -U pip
python3 -m pip install -r requirements-orange-pi.txt
```

`requirements-orange-pi.txt` 里只放车端轻量依赖，目前是：

```text
numpy
pillow
```

不要在 Orange Pi 上安装 `torch`、`transformers`、`bitsandbytes`，除非你明确接受
CPU-only 本地推理非常慢，或者你已经换成了适配 Orange Pi NPU 的小模型。

## 外部模型机依赖

如果用外部 GPU 机器跑 Qwen3-VL，在模型机上安装：

```bash
python3 -m pip install -U pip
python3 -m pip install -r requirements-model-server.txt
```

`requirements-model-server.txt` 是给模型机用的，不是给 Orange Pi 用的。里面包含：

```text
torch
torchvision
transformers
accelerate
bitsandbytes
pillow
safetensors
```

模型机上需要拷贝完整模型目录，不能只拷 `model.safetensors`：

```text
models/Qwen3-VL-2B-Instruct/
```

这个目录里至少要有：

```text
config.json
model.safetensors
tokenizer.json
tokenizer_config.json
preprocessor_config.json
chat_template.json
vocab.json
merges.txt
```

## 真车上需要带过去的项目文件

建议把整个项目 clone 到 Orange Pi，然后只 build ROS 包。运行所需核心文件是：

```text
src/vln_nav2_bridge/
src/vln_inference/              # 如果本地推理或沿用当前 wrapper 才需要
config/
data/<real_robot_map>.yaml
data/<real_robot_map>.pgm
```

这些生成目录不要拷：

```text
build/
install/
log/
data/runtime/
```

到 Orange Pi 上重新 build：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vln_nav2_bridge
source install/setup.bash
```

## 真车部署时必须改的参数

这些参数不能直接沿用仿真默认值：

- `use_sim_time`：真车上必须设为 `false`
- `model_path`：如果模型不在 Orange Pi 上，要改成远程推理方案
- `model_python_executable`：当前是本机 conda 路径，真车上不能直接用
- `image_topic` 或 `compressed_image_topic`：改成真车相机话题
- `odom_topic`：改成真车里程计话题
- `cmd_vel_topic`：改成真车速度控制话题
- `safe_map_yaml`：改成真车地图 YAML
- `text_to_pose_converter.py` 里的语义目标坐标：必须把 Isaac 仿真坐标换成真实地图坐标

## 运行前检查清单

跑 VLN 之前先确认：

1. 真实 3D 雷达 `PointCloud2` 话题正常发布。
2. Nav2 用 `/scan` 或其他二维投影话题正常发布。
3. 相机图像正常发布。
4. `/odom` 或真车自己的里程计话题正常发布。
5. `map -> odom -> base_link` TF 是通的。
6. Nav2 已经用真车地图启动。
7. 已经在 RViz 里设置 initial pose，或通过 `/initialpose` 设置初始位姿。
8. 普通 Nav2 goal 能成功导航。
9. 再启动 VLN bridge，并设置 `require_fresh_image:=true`。

## 重要限制

Orange Pi 应该被当作“小车控制器”，不是“大模型推理服务器”。如果必须在
Orange Pi 上做端到端本地推理，建议换更小的视觉模型，或者导出成 Orange Pi NPU
能跑的格式，并相应改造当前 bridge 的推理后端。

## 3D 雷达与 `/scan` 的关系

本项目后续应按这个分工理解传感器：

- 真实 3D 雷达：`sensor_msgs/msg/PointCloud2`，用于 RViz 主显示和未来 3D costmap。
- `/scan`：给 Nav2/AMCL/local costmap 用的二维投影或已有二维感知输入，不代表完整 3D 雷达。
- RViz 主显示应看 map-frame PointCloud2，而不是只看 `/scan`。
