# Go2 MID-360 Navigation

[English](README.md) | **简体中文**

基于 ROS 2 Humble 和 Gazebo Classic 的 Unitree Go2 仿真与导航项目，集成 MID-360 激光雷达、D435i RGB-D 相机、FAST-LIO 建图、ICP 定位和 Nav2 导航，提供桌面控制台、Go2 Workshop 场景及配套地图。

如果这个项目对你有所帮助，欢迎点一个 Star，感谢你的支持。

![Go2 MID-360 Navigation 演示](media/demo.gif)

[完整视频](media/demo.mp4) · [快速开始](#快速开始) · [控制台使用](#控制台使用) · [功能使用](#功能使用) · [地图管理](#地图管理) · [开发参考](#开发参考)

## 功能概览

| 功能 | 内容 |
| --- | --- |
| 机器人仿真 | Go2 模型、CHAMP 步态控制、MID-360 支架与传感器装配 |
| 三维建图 | FAST-LIO 激光惯性里程计、点云累积与 PCD 保存 |
| 全局定位 | 基于已有点云地图的 ICP 定位与全局位姿估计 |
| 自主导航 | Nav2 单点导航、多点顺序导航与局部避障 |
| RGB-D 感知 | D435i 彩色图、深度图与深度点云 |
| 桌面控制台 | 模式切换、键盘遥控、实时运动仪表、地图保存与转换、日志查看 |

默认场景 **Go2 Workshop** 包含围墙、工作台、货架、木箱等分散布置的障碍物，可直接使用配套地图开展定位与导航。

## 快速开始

### 环境

| 组件 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 仿真器 | Gazebo Classic 11 |

### 安装与编译

先安装 ROS 2 Humble，再克隆并编译项目：

```bash
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
git clone https://github.com/max-niuniu/go2-mid360-navigation.git
cd ~/go2_ws
bash src/go2-mid360-navigation/scripts/setup_mid360_dependencies.bash
```

安装脚本完成系统依赖安装、固定版本源码下载、兼容补丁应用、编译和默认地图准备。首次运行需要网络与管理员权限；已有同名运行地图不覆盖。

**以下命令均在工作空间根目录 `go2_ws/` 执行。**

### 启动控制台

```bash
bash src/go2-mid360-navigation/scripts/run_control_panel.bash
```

## 控制台使用

![Go2 控制台界面](media/control-panel.png)

### 启动与停止

1. 在 **运行控制** 中选择仿真、建图、定位或导航。
2. 按需勾选 **Gazebo 窗口、RViz、日志终端**，点击对应的启动按钮。
3. 等待 **系统状态** 就绪后操作。切换模式前，点击 **停止任务**。

选择 **导航** 可直接加载配套地图，在 RViz 中使用 **Nav2 Goal** 设置目标位置和朝向。

### 遥控与状态显示

点击 **键盘遥控** 打开控制终端，将输入焦点切换到该终端后，使用 `W/X` 前进与后退、`A/D` 转向、空格停止。点击 **结束遥控** 关闭遥控；导航模式下禁用键盘遥控。

顶部仪表显示机器人关节姿态、运动方向、线速度和角速度；**系统状态** 显示仿真时钟、雷达、FAST-LIO、定位、TF 和 Nav2 的运行状态。RViz 中的 **D435i Color**、**D435i Depth** 窗口分别显示彩色图与深度图。

### 保存与转换地图

1. 在 **地图目录** 中选择输出目录，启动建图并遥控机器人扫描场景。
2. 点击 **保存三维点云**，生成该目录下的 `map.pcd`。
3. 停止任务，点击 **转换二维地图**，生成 `map.yaml` 和 `map.pgm`。
4. 保持该地图目录，切换至定位或导航模式启动。

转换的高度范围和分辨率可在 **高级设置** 中调整。

### RViz 与日志

**重启 RViz** 仅重开显示窗口，不停止当前仿真任务。**运行日志** 支持查看、导出和清空显示；**打开日志终端** 可查看完整输出。详细操作见 [控制台说明](go2_mid360_panel/README.md)。

## 功能使用

以下模式分别启动。脚本自动加载 ROS 环境，按 `Ctrl+C` 停止。

### 1. 仿真与遥控

```bash
bash src/go2-mid360-navigation/scripts/run_mid360_sim.bash gui:=true rviz:=true
```

通过控制台打开键盘遥控，或在另一终端执行：

```bash
bash src/go2-mid360-navigation/scripts/run_teleop.bash
```

| 按键 | 功能 |
| --- | --- |
| `W` / `X` | 前进 / 后退 |
| `A` / `D` | 原地左转 / 右转 |
| `Q` / `E` / `Z` / `C` | 斜向转弯 |
| `Shift` + 方向键 | 平移 |
| `S` / 空格 | 停止 |
| `R` / `F` | 整体加速 / 减速 |
| `T` / `G` | 调整线速度 |
| `Y` / `H` | 调整角速度 |

默认线速度 **0.5 m/s**，角速度 **1.0 rad/s**；每次调速幅度为 10%。按住方向键持续运动，松键后由控制器的 0.5 秒超时保护停止。导航与键盘遥控互斥。

### 2. 三维建图

为新地图建立独立目录：

```bash
mkdir -p maps/my_scene
bash src/go2-mid360-navigation/scripts/run_fast_lio_3d_mapping.bash \
  gui:=true show_rviz:=true map_output:="$PWD/maps/my_scene/map.pcd"
```

初始化完成后，遥控机器人扫描场景，在 RViz 中观察点云累积。通过控制台的 **保存三维点云** 保存地图，或在另一终端执行：

```bash
source src/go2-mid360-navigation/scripts/setup_env.bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

正常 `Ctrl+C` 退出时也会保存点云。未指定 `map_output` 时，默认写入 `maps/go2_workshop/map.pcd`，会覆盖同名文件。

### 3. 点云定位

```bash
bash src/go2-mid360-navigation/scripts/run_fast_lio_3d_localization.bash \
  gui:=true show_rviz:=true
```

加载已有三维地图，使用 FAST-LIO 里程计与 ICP 进行定位，此模式不启动 Nav2。遥控运动时，可在 RViz 中观察实时点云与参考地图的对齐情况。

### 4. 自主导航

```bash
bash src/go2-mid360-navigation/scripts/run_navigation.bash \
  gui:=true show_rviz:=true
```

启动顺序为 **仿真与定位 → TF 就绪 → Nav2 激活 → RViz**。

- **单点导航**：使用 RViz 的 **Nav2 Goal** 设置位置和朝向。
- **多点导航**：通过 Nav2 面板的航点模式依次添加目标，再执行顺序导航。
- **手动初始化**：启动时传入 `auto_initial_pose:=false`，通过 **2D Pose Estimate** 设置初始位姿。

默认使用 Go2 Workshop 地图，以地图坐标 `(x=0, y=0, yaw=0)` 初始化定位。

## 地图管理

| 位置 | 用途 |
| --- | --- |
| `go2_mid360_bringup/maps/go2_workshop/` | 仓库内的配套地图 |
| `go2_ws/maps/go2_workshop/` | 默认运行地图 |
| `go2_ws/maps/<场景名称>/` | 用户地图与建图输出 |

每组地图包含 `map.pcd`（三维定位参考）、`map.yaml` 和 `map.pgm`（二维导航地图）。场景说明见 [Go2 Workshop](go2_mid360_sim/worlds/README.md)。

### 点云查看与二维转换

查看默认点云地图：

```bash
bash src/go2-mid360-navigation/scripts/view_mid360_3d_map.bash
```

停止仿真后，可在控制台选择地图目录并执行 **转换二维地图**，也可使用命令：

```bash
source src/go2-mid360-navigation/scripts/setup_env.bash
ros2 run go2_fastlio_localization pcd_to_nav2_map \
  "$PWD/maps/my_scene/map.pcd" "$PWD/maps/my_scene/map" \
  --resolution 0.05 --z-min 0.35 --z-max 2.60 \
  --padding 0.30 --min-points 2 --dilation-cells 1
```

高度范围以 PCD 坐标系的 Z 轴为准。转换会覆盖同名二维地图及其手工编辑结果；更新后需重启导航。

### 自定义场景

导航支持 `world`、`pcd_map`、`nav_map`、`initial_x`、`initial_y` 和 `initial_yaw` 参数。场景、三维点云和二维地图应匹配，初始位姿使用地图坐标。

```bash
bash src/go2-mid360-navigation/scripts/run_navigation.bash \
  world:=/absolute/path/scene.world \
  pcd_map:="$PWD/maps/my_scene/map.pcd" \
  nav_map:="$PWD/maps/my_scene/map.yaml"
```

## 开发参考

### 工作空间

```text
go2_ws/
├── src/
│   ├── go2-mid360-navigation/
│   │   ├── go2_*/              功能包
│   │   ├── scripts/            安装、编译与启动脚本
│   │   ├── patches/            第三方依赖补丁
│   │   ├── media/              演示视频与截图
│   │   └── dependencies.repos  依赖来源与固定版本
│   └── third_party/            第三方源码
├── maps/                       运行地图与建图输出
├── build/
├── install/
└── log/
```

| 功能包 | 职责 |
| --- | --- |
| `go2_mid360_description` | Go2、传感器模型与安装关系 |
| `go2_mid360_config` | 步态、关节控制与通信配置 |
| `go2_mid360_sim` | Gazebo、传感器仿真与场景 |
| `go2_fastlio_localization` | 建图、ICP 定位、里程计桥接与地图转换 |
| `go2_mid360_bringup` | 系统启动、Nav2、参考地图与 RViz 配置 |
| `go2_mid360_panel` | 桌面控制台与运行状态显示 |

### 编译与依赖

日常编译：

```bash
bash src/go2-mid360-navigation/scripts/build.bash
source src/go2-mid360-navigation/scripts/setup_env.bash
```

单包增量编译：

```bash
source src/go2-mid360-navigation/scripts/setup_env.bash
colcon build --symlink-install --packages-select go2_fastlio_localization
source src/go2-mid360-navigation/scripts/setup_env.bash
```

依赖版本固定在 [dependencies.repos](dependencies.repos)，兼容修改保存在 [patches/](patches/)。`prepare_sources.bash` 下载源码并应用补丁；`apply_compatibility.bash` 校验已有源码并应用补丁。二者均可重复执行，版本不匹配或补丁冲突时停止，不自动替换已有源码。

### ROS 启动入口

直接运行 ROS 命令前，每个新终端执行 `source src/go2-mid360-navigation/scripts/setup_env.bash`。

| 模式 | 命令 |
| --- | --- |
| 仿真 | `ros2 launch go2_mid360_sim simulation.launch.py gui:=true rviz:=true` |
| 建图 | `ros2 launch go2_mid360_bringup mapping.launch.py gui:=true show_rviz:=true` |
| 定位 | `ros2 launch go2_mid360_bringup localization.launch.py gui:=true show_rviz:=true` |
| 导航 | `ros2 launch go2_mid360_bringup navigation.launch.py gui:=true show_rviz:=true` |
| 点云查看 | `ros2 launch go2_mid360_bringup view_3d_map.launch.py` |

### 配置入口

| 配置 | 文件 |
| --- | --- |
| MID-360 安装位姿 | [mid360_mount.yaml](go2_mid360_description/config/mid360_mount.yaml) · [支架模型](go2_mid360_description/meshes/mid360_mount/README.md) |
| D435i 开关、安装位姿与图像参数 | [d435i.yaml](go2_mid360_description/config/d435i.yaml) |
| Gazebo | [gazebo.yaml](go2_mid360_sim/config/gazebo.yaml) |
| 关节控制器 | [ros_control.yaml](go2_mid360_config/config/ros_control/ros_control.yaml) |
| FAST-LIO | [fast_lio_mid360_sim.yaml](go2_fastlio_localization/config/fast_lio_mid360_sim.yaml) |
| Nav2 | [fast_lio_nav2.yaml](go2_mid360_bringup/config/autonomy/fast_lio_nav2.yaml) |

### 数据与坐标系

```text
map → odom → base_footprint → base_link → mid360_mount_link → mid360_link
```

| 话题 | 内容 |
| --- | --- |
| `/livox/lidar` | `PointCloud2` 点云 |
| `/livox/lidar_custom` | Livox `CustomMsg` 点云 |
| `/imu/data` | IMU 数据 |
| `/Odometry` | FAST-LIO 里程计 |
| `/odom` | 导航里程计 |
| `/scan` | 二维激光扫描 |
| `/camera/color/image_raw` | D435i 彩色图，默认 640 × 360、15 Hz |
| `/camera/depth/image_raw` | D435i 深度图，默认 640 × 360、15 Hz |
| `/camera/color/camera_info` | 彩色相机参数 |
| `/camera/depth/camera_info` | 深度相机参数 |
| `/camera/depth/points` | 深度点云，按订阅生成 |

`/odom` 由 FAST-LIO 里程计转换得到，ICP 提供全局定位，Nav2 使用二维地图与 `/scan` 规划及避障。控制台仪表使用 Gazebo 真值速度进行展示，导航定位不依赖真值里程计。

## 常见问题

| 问题 | 处理方式 |
| --- | --- |
| Gazebo 端口被占用 | 在上一启动终端按 `Ctrl+C`，等待 `gzserver` 退出后再启动 |
| 新终端找不到功能包 | 重新加载 `scripts/setup_env.bash` |
| RViz 无响应 | 在控制台点击“重启 RViz”；依赖准备包含 tf2 0.25.24 的死锁修复 |
| 地图更新后显示未变化 | 重启导航以重新加载地图 |
| RViz 显示 Localization inactive | 本项目使用 ICP 定位；检查点云对齐、`ICP accepted` 日志与 `map → base_footprint` TF |

## 依赖与许可

本项目新增代码采用 [BSD-3-Clause](LICENSE)。第三方代码、模型及补丁沿用各自许可：[依赖版本](dependencies.repos) · [补丁许可](patches/LICENSE) · [Go2 模型](go2_mid360_description/meshes/LICENSE) · [D435i 模型](go2_mid360_description/meshes/d435i/LICENSE)。

## 项目团队

- **维护者**：[Tianwei Niu](mailto:tianwei.niu@cau.edu.cn)
- **指导**：Man Zhang、Han Li
