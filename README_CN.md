# 自动泊车仿真

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-FB8C00)](https://classic.gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Qt](https://img.shields.io/badge/Qt-5-41CD52?logo=qt&logoColor=white)](https://doc.qt.io/qt-5/qtwidgets-index.html)

**Language / 语言**: [English](README.md) • **中文**

基于 **ROS 2 Humble、Gazebo Classic 11 和 RViz2** 的 Prius 自动泊车仿真项目，支持侧方位停车与垂直泊车。项目保留两种运行方式：固定的预规划路线演示，以及通过 RViz 控制面板操作的近场 Hybrid A* 自动泊车。

| 方式 | 启动入口 | 工作流程 |
| --- | --- | --- |
| 预规划路线 | `planned_park.launch.py` | 启动后自动执行固定路线：垂直倒库 → 驶出并沿通道行驶 → 侧方位泊车 |
| Hybrid A* 在线泊车 | `auto_park.launch.py` | 手动驾驶接近车位 → 选中空闲车位 → 满足准入条件 → 点击开始 → 在线规划与跟踪 |

**在线泊车不是从任意远处导航到车位的系统。自动泊车前需要手动驾驶接近目标车位；从已停好的车位前往另一个车位，也需要先手动安全驶出。**

## 环境与安装

当前开发环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo **Classic 11**、Python 3 和 Qt5。这里的 Gazebo 不是 Gazebo Sim / Ignition Gazebo。

先安装 ROS 2 Humble。获取项目后，在包含 `src/` 的仓库根目录执行以下命令。首次安装 `python3-rosdep` 后、执行 `rosdep update` 前，若尚未初始化 rosdep，请先执行一次 `sudo rosdep init`。

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs ros-humble-rviz2 \
  libgazebo-dev libignition-math6-dev qtbase5-dev

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

每个用于启动或调试的新终端都需要加载 ROS 环境和本工作空间的 `install/setup.bash`。

### 仿真场景的外部模型

项目自带 Prius、车位和锥桶等资源，但默认 [park.world](src/car_demo/car_demo/worlds/park.world) 还依赖以下 **未包含在本仓库中的 Gazebo 模型**，可从上游 [OSRF Gazebo 模型库](https://github.com/osrf/gazebo_models) 获取：

```text
~/.gazebo/models/
├── suv/
├── hatchback/
├── hatchback_blue/
├── hatchback_red/
└── house_1/
```

首次启动前，请将这些模型的完整目录准备到上述位置，或将它们所在的父目录加入 `GAZEBO_MODEL_PATH`。需保留 `model.config`、SDF、网格、材质和纹理；其中 `hatchback` 提供蓝色与红色车辆共用的资源。启动文件禁用了在线模型数据库，**不会在启动时自动下载缺失模型**。

## 方式一：预规划路线演示

```bash
ros2 launch auto_parking planned_park.launch.py
```

该入口启动 Gazebo、车辆、RViz 和固定路线跟踪节点，车辆就绪后自动执行任务，不需要点击车位。路线由直线和圆弧组成，针对默认停车场及默认出生位姿设计，不进行 Hybrid A* 在线搜索。

参数见 [parking.yaml](src/auto_parking/config/parking.yaml)，路线定义见 [mission.py](src/auto_parking/auto_parking/mission.py)。当前该模式的 `safety_enabled` 为 `false`，应在默认场景中作为固定路线演示使用，不应视为具备动态避障能力。

## 方式二：手动接近 + Hybrid A* 自动泊车

```bash
ros2 launch auto_parking auto_park.launch.py
```

该入口启动整套仿真，并在 RViz 左侧加载原生 `ParkingPanel`：

1. 保持面板中的 **手动驾驶** 模式，用面板按钮或键盘驾驶车辆。
2. 使用 RViz 工具栏的 **Publish Point**，点击绿色空闲车位。红色表示占用，黄色表示选中。
3. 选择车位只显示目标车身和附近准入边界，不触发规划，也不接管车辆。远处选择车位后，按面板提示手动接近。
4. 在车位附近停车，等待状态变为 `READY_TO_PARK`、**开始自动泊车** 按钮可用。
5. 点击 **开始自动泊车**。系统自动切换到自动模式，规划并跟踪泊车路径；无需提前切换模式下拉框。
6. 完成后查看 `COMPLETE` 及位置、航向、车身在位等指标。需要再次驾驶时，切回 **手动驾驶**。

当前默认准入条件是：后轴参考点到目标车位矩形的最短距离不超过 **12 m**、速度绝对值不超过 **0.15 m/s**、目标车位空闲、车辆未完整停在其他车位内，且当前车身和目标车身具有所需障碍间隙。**不限制起始航向**；按钮可用不代表一定存在可行路线，完整路径的碰撞与运动学约束在启动规划后检查。

此范围是“车位附近”的准入边界，不是全局导航范围，也不是固定预停点或必须对齐的预停走廊。当前默认值是 12 m，而非早期设计中的约 6 m。

浅蓝色路径表示前进，深蓝色表示倒车，黄色点表示换挡位置。在线规划允许前进、倒车及揉库调整；**当前并非强制倒车入库策略**，实际入位方向由可行性和代价决定。

取消、急停或失败后车辆会停止；切回 **手动驾驶** 后再操作。侧方位停好后若选择另一处垂直车位，应先手动驶出、接近新车位，再开始自动泊车。当前没有专门的自动出库策略。

### 面板手动驾驶

键盘焦点需位于 ParkingPanel；按住驾驶按钮或按键行驶，松开或面板失去焦点时刹车。

| 按键 | 操作 |
| --- | --- |
| `W` / `Q` / `E` | 前进 / 前进左转 / 前进右转 |
| `X` / `Z` / `C` | 倒车 / 倒车左转 / 倒车右转 |
| `A` / `D` | 仅调整左 / 右转向，不加油门 |
| `S` / `Space` | 刹车并挂空挡 |
| `R` / `F` | 增加 / 减少油门幅度 |
| `T` / `G` | 增加 / 减少转向幅度 |

两种泊车启动文件都包含完整仿真，**请分别运行，不要同时启动**。在线模式已集成手动控制，不要另行启动默认直接发布 `/prius/control` 的终端键盘节点，以免绕过控制仲裁。

## 算法、配置与接口

在线模式使用局部 Hybrid A*、Reeds–Shepp 启发式及解析连接，结合障碍启发式进行搜索；代价考虑路径长度、前进/倒车、换挡、转向及障碍间隙。跟踪采用曲率前馈与横向、航向误差反馈，支持真实终点纠偏，以及路径被新障碍阻挡后的停车重规划。

主要参数位于 [hybrid.yaml](src/auto_parking/config/hybrid.yaml)：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `activation_radius` | 12.0 m | 后轴到车位矩形的准入距离 |
| `entry_speed` | 0.15 m/s | 启动泊车允许的最大速度绝对值 |
| `position_tolerance` | 0.10 m | 完成时纵向、横向误差分别满足的上限 |
| `yaw_tolerance` | 0.05236 rad（3°） | 完成时航向误差上限 |
| `stop_speed` | 0.05 m/s | 完成时速度绝对值上限 |
| `minimum_clearance` | 0.10 m | 最小车身障碍间隙 |
| `max_adjustments` | 5 | 终点纠偏与障碍变化重规划共用次数上限 |
| `speed_forward` / `speed_reverse` | 0.70 / 0.45 m/s | 常规前进 / 倒车目标速度 |
| `speed_fine` | 0.20 m/s | 精细调整目标速度 |

完成判定还要求车身完整位于车位内。参数或源码修改后，重新构建并加载工作空间。

| ROS 接口 | 用途 |
| --- | --- |
| `/clicked_point` | RViz 选择车位 |
| `/parking/command` | `manual`、`auto`、`start`、`cancel`、`estop` 请求 |
| `/parking/status` | 模式、状态、禁用原因和泊车指标 |
| `/parking/manual_control` / `/parking/auto_control` | 手动 / 自动控制输入 |
| `/prius/control` | 在线模式由 `control_mux` 统一发布的车辆控制 |
| `/prius/ground_truth` | Gazebo 真值位姿 |
| `/prius/center_laser/scan` | 激光点云，消息类型为 `sensor_msgs/msg/PointCloud2` |

调试时可查看 `ros2 topic echo /parking/status`。面板未显示时，确认已构建并加载当前工作空间，或在 RViz 的 `Panels → Add New Panel` 中添加 `auto_parking/ParkingPanel`。

## 测试与适用范围

完成基础构建后，可单独启用自动泊车包的测试：

```bash
colcon build --packages-select auto_parking --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select auto_parking --event-handlers console_direct+
colcon test-result --test-result-base build/auto_parking --verbose
```

测试覆盖几何与碰撞检测、局部规划、离线闭环跟踪、感知、状态管理、控制仲裁、ROS 消息交互和面板加载。这些测试不能替代 Gazebo 实际动力学下的完整泊车验收。

当前系统针对默认停车场：车位候选与静态障碍先验由 [spots.py](src/auto_parking/auto_parking/spots.py) 和 [collision.py](src/auto_parking/auto_parking/collision.py) 定义，激光感知补充占用与障碍信息；不是任意停车场的通用车位检测。更换场景时需要同步更新这些先验，预规划模式还需要更新固定路线。定位使用仿真真值，项目不面向真实车辆直接部署。

## 目录与发布说明

```text
car_parking/
├── src/
│   ├── auto_parking/           # 在线/预规划泊车、控制仲裁、RViz Panel、配置与测试
│   └── car_demo/
│       ├── car_demo/           # Gazebo 启动、停车场、车辆插件与键盘控制
│       ├── prius_description/  # Prius URDF、网格与纹理
│       └── prius_msgs/         # 车辆控制消息
├── README.md                   # English
├── README_CN.md                # 中文
└── .gitignore
```

[.gitignore](.gitignore) 排除构建、安装、日志、缓存、本地工具配置及常见录制数据；车辆 `.obj` 网格和纹理属于必需资源，保留在源码中。

当前工作目录中的 `src/car_demo` 仍是含本地修改的嵌套 Git 仓库。**发布前应明确采用完整源码纳入，还是提交修改后作为正式 submodule 引用**；直接在外层 `git add` 可能只记录 gitlink，不能保证他人克隆后获得当前修改。根 `.gitignore` 不会解决这一问题。

## 致谢与模型来源

本项目复用了已有开源项目的 Prius 车辆资源，车辆模型并非本自动泊车项目从零制作。引用与致谢如下：

| 上游项目 | 贡献与来源说明 |
| --- | --- |
| Open Robotics / OSRF [`car_demo`](https://github.com/osrf/car_demo) | 原始 Prius 网格、URDF、传感器配置与车辆控制插件；本项目在该仿真基础上修改适配 |
| [`NovoG93/car_demo`](https://github.com/NovoG93/car_demo) | `humble` 分支提供 ROS 2 Humble / Gazebo 11 适配 |
| [`saonam/car_demo`](https://github.com/saonam/car_demo) | 本地 `src/car_demo` 的 Git remote 所记录的直接导入来源 |
| OSRF [`gazebo_models`](https://github.com/osrf/gazebo_models) | 停车场使用的外部 SUV、hatchback 系列车辆及房屋模型 |

可查看原始 [Prius 模型目录](https://github.com/osrf/car_demo/tree/master/prius_description) 和 Open Robotics 的 [Simulated Car Demo 官方介绍](https://www.osrfoundation.org/simulated-car-demo/)。自动泊车算法与 RViz 控制面板位于 `auto_parking` 包中；车辆模型及基础仿真的贡献归原作者和上游贡献者所有。原始车辆描述包标注维护者 Shane Loretz；外部 SUV/hatchback 模型标注作者 Nate Koenig，`house_1` 标注作者 Cole Biesemeyer 与 Nate Koenig。

## 许可说明

上游 [Prius 包元数据](https://github.com/osrf/car_demo/blob/master/prius_description/package.xml) 与本地各 ROS 包的 `package.xml` 声明 Apache-2.0。当前项目尚无根目录 `LICENSE` 文件，因此没有添加代表整个项目许可的徽章。第三方资源仍遵循各自上游许可，发布前需补齐许可文件与必要署名。外部 Gazebo 模型未随本仓库分发。
