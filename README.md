#  ROS 2 + Nav2 + Gazebo 差速小车自主导航项目

这是一个基于 ROS 2、Navigation2 (Nav2) 和 Gazebo 经典仿真环境的自主导航小车（Fishbot）项目。项目集成了机器人底盘描述、Gazebo 仿真物理世界、地图与导航配置文件，并实现了一套具备平滑曲线切弯、碰撞自愈和断点重试的多点自主导航算法。

---

##  项目主要特点

*   **差速真值无漂移定位**
    *   在 `fishbot_description` 中启用了 Gazebo 原生差速控制插件（基于 `libgazebo_ros_diff_drive.so`），并配置为直接读取仿真器物理引擎的 `world` 坐标系作为里程计源。
    *   彻底解决了基于里程计算法（open_loop）推算造成的累积漂移以及频繁发生的 TF 抖动冲突。
*   **方形碰撞特征与雷达识别优化**
    *   针对原本在仿真中锥桶模型过小且下陷的视觉问题进行了修复（进行了 Scale 等比放大）。
    *   在仿真世界中为锥桶设计了专属的方形碰撞箱（35cm x 35cm x 60cm），加高的碰撞箱确保了单线 2D 雷达在任何地面厚度或微小起伏下，扫描线都能精准打在障碍物上并建图。
*   **双模式多点导航（Through / Follow）**
    *   **Through 模式**：通过调用 Nav2 Simple Commander API 的 `goThroughPoses` 实现连续穿越，小车在经过中间点时不停车，规划出平滑的连贯全局行驶轨迹。
    *   **Follow 模式**：小车会在每一个坐标路点精准降速、停靠并修正朝向（利用 `goToPose` 单点迭代）。
*   **提前预判平滑切弯算法**
    *   在 Follow 模式下，项目设计了基于实时 TF 坐标变换的“距离前瞻判定”机制。在距离中间目标点达到设定的阈值距离（默认 1.0 米）时，直接用下一个路点目标抢占（Goal Preempt）当前任务，实现平滑的“抹圆切弯”，极大改善了传统到点急停、原地旋转造成的顿挫感。
*   **碰撞自愈与增量切片重试**
    *   **代价地图自动清空**：在中途若因临时障碍物或擦碰导致规划超时失败时，小车不会直接放弃，而是自动清空全局和局部代价地图（Costmap），静止 2 秒重新扫描定位并最多重试 3 次。
    *   **断点记忆**：在 Through 模式下，系统动态维护已通过路点的索引，重试时采用切片操作，**仅将剩下未完成的路点重新下发**，避免了折返回起点的怪异掉头行为。

---

## 依赖安装

本项目推荐在以下系统环境运行：
*   **操作系统**：Ubuntu 22.04 LTS (支持通过 WSL 2 或原生安装) / Windows (原生或虚拟环境)
*   **ROS 2 版本**：ROS 2 Humble Hawksbill
*   **仿真器**：Gazebo Classic (Gazebo 11)

### 安装核心依赖包：
在你的 Linux 终端中运行以下命令安装必要的 ROS 2 工具箱：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-xacro \
                 ros-humble-joint-state-publisher \
                 ros-humble-robot-state-publisher
```

---

## 工作空间构建

请确保你的源码已放入 ROS 2 工作空间（例如 `bot_ws/src`），然后进行编译：

```bash
# 进入工作空间根目录
cd c:/Users/春晖/Desktop/bot_ws  # (根据你的终端系统调整路径)

# 编译所有包
colcon build

# 刷新工作空间环境变量
source install/setup.bash       # 如果是 Linux 终端 (bash)
# 或
.\install\setup.ps1             # 如果是 Windows PowerShell
```

---

##  核心功能运行指南

项目支持模块化启动，请严格按照以下三个步骤，在三个不同的终端窗口中分别运行。

### 步骤 1：启动 Gazebo 仿真环境与小车模型
该步骤将加载小车 URDF、斑马线地图、锥桶等物理障碍物，并启动仿真物理引擎。
```bash
ros2 launch bot sim.launch.py
```
*启动后，你将能在 Gazebo 窗口中看到 Fishbot 小车与对应的仿真跑道。*

### 步骤 2：启动 Nav2 导航与定位系统
该步骤将加载已建立的静态地图（`fishbot_map`）、AMCL 定位节点、代价地图、路径规划器以及可视化的 RViz 界面。
```bash
ros2 launch bot_nav2 nav2_launch.py
```
*启动后，在自动弹出的 RViz 中，你会看到雷达点云与当前建好的地图。可以通过 RViz 上方的 **"2D Pose Estimate"** 按钮对小车进行初始定位校准。*

### 步骤 3：启动多点巡航/导航控制脚本
该节点通过 Python 代码向 Nav2 自动下发预设的 8 个目标点坐标，开始自动导航。
```bash
# 模式 A(默认模式）：Through 连续穿越模式（推荐，行进极其连贯平滑）
ros2 launch waypoint pose_launch.py

# 模式 B：Follow 单点停靠模式（含 1.0米 提前平滑切弯）
ros2 launch waypoint pose_launch.py nav_mode:=follow lookahead_dist:=1.0 max_retries:=3
```

---

##  目录结构与关键文件介绍

*   **`src/fishbot_description/`**：包含小车 Fishbot 的三维网格、传感器（雷达、IMU、深度相机）和执行器轮子的 URDF/Xacro 宏定义，以及 Gazebo 物理属性插件。
*   **`src/bot/`**：包含 Gazebo 的仿真世界定义（`worlds/bot.world`，内含优化后的锥桶碰撞模型）以及 Gazebo 的启动 launch 脚本。
*   **`src/bot_nav2/`**：包含导航配置文件（`config/nav2_params.yaml`，内含调优后的膨胀层半径 `0.25` 等）、建好的栅格地图（`maps/`）以及 Nav2 启动 launch 脚本。
*   **`src/waypoint/`**：多点导航包，包含节点核心逻辑 `mutil_pose.py`、预设航点文件 `config/waypoints.yaml` 和启动脚本 `pose_launch.py`。
