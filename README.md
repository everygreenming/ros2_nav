#  ROS 2 + Nav2 + Gazebo 差速小车自主导航项目

这是一个基于 ROS 2、Navigation2 (Nav2) 和 Gazebo 仿真环境的自主导航小车项目。项目集成了机器人底盘描述、Gazebo 仿真物理世界、地图与导航配置文件，并实现了一套具备平滑曲线切弯、碰撞自愈和断点重试的多点自主导航算法。


## 依赖安装

本项目是在以下系统环境开发：
*   **操作系统**：Ubuntu 22.04 LTS
*   **ROS 2 版本**：ROS 2 Humble Hawksbill
*   **仿真器**：Gazebo Classic (Gazebo 11)

### 1. 自动安装（推荐，支持任意 ROS 2 版本）：
由于在各功能包的 `package.xml` 中已经声明了完整的依赖关系，可以利用 `rosdep` 自动根据当前的 ROS 2 版本进行检测和补全。

请在终端中执行：

```bash
sudo rosdep init
rosdep update
# 在 bot_ws 工作空间根目录下执行：
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

### 2. 手动安装（备用）：
如果你更倾向于手动通过 `apt` 安装，请使用环境变量 `$ROS_DISTRO` 来确保版本适配：

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-nav2-bringup \
                 ros-${ROS_DISTRO}-nav2-simple-commander \
                 ros-${ROS_DISTRO}-gazebo-ros-pkgs \
                 ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-joint-state-publisher \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-tf2-ros \
                 ros-${ROS_DISTRO}-tf-transformations \
                 python3-transforms3d
```

---

## 工作空间构建

请确保你的源码已放入 ROS 2 工作空间（例如 `bot_ws/src`），然后进行编译：

```bash
# 进入工作空间根目录
cd /bot_ws  # (根据你的终端系统调整路径)

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
进入后可以在rviz中手动初始化位姿（2D Pose Estimate）,使用nav goal进行单点导航，使用自带的nav through手动设置路点进行多点导航。

### 步骤 3：启动多点巡航/导航控制脚本
该节点通过 Python 代码向 Nav2 自动下发预设的 8 个目标点坐标，并初始化位姿，同时开始自动导航。
```bash
# 模式 A(默认模式）：Through 连续穿越模式（推荐，行进极其连贯平滑）
ros2 launch waypoint pose_launch.py

# 模式 B：Follow 单点停靠模式（含 1.0米 提前平滑切弯）
ros2 launch waypoint pose_launch.py nav_mode:=follow

```

---

##  目录结构与关键文件介绍


```text
bot_ws/src/
├── bot/                              # Gazebo 仿真世界与启动包
│   ├── launch/
│   │   └── sim.launch.py             # 仿真总启动脚本 (加载小车模型与仿真环境)
│   ├── models/bot_map/               # 仿真地图模型文件 (包含 map.stl（在cad绘制） 及纹理配置) 
│   └── worlds/
│       └── bot.world                 # 核心物理世界场景定义 
│
├── bot_nav2/                         # Navigation2 导航配置核心包
│   ├── behavior_trees/
│   │   └── my_nav_through_poses.xml  # 自定义行为树 (包含清图自愈与特定恢复逻辑)
│   ├── config/
│   │   └── nav2_params.yaml          # Nav2 核心调优参数 (包含 AMCL定位、代价地图膨胀、DWB局部规划器等)
│   ├── launch/
│   │   └── nav2_launch.py            # 导航系统总启动脚本 (加载地图、AMCL与规划器)
│   └── maps/
│       ├── fishbot_map.pgm           # 2D 栅格地图文件
│       └── fishbot_map.yaml          # 2D 地图元数据 (分辨率、原点等信息)
│
├── fishbot_description/              # 机器人 URDF 描述与物理控制包
│   ├── config/rviz/
│   │   └── dispaly_model.rviz        # RViz 可视化预设配置
│   └── urdf/fishbot/
│       ├── fishbot.urdf.xacro        # 小车 URDF 主入口文件
│       ├── base.urdf.xacro           # 底盘几何定义
│       ├── common_inertia.xacro      # 惯性矩阵通用宏
│       ├── actuator/
│       │   ├── wheel.urdf.xacro      # 驱动轮定义 
│       │   └── caster.urdf.xacro     # 万向支撑轮定义
│       ├── sensor/
│       │   ├── laser.urdf.xacro      # 2D 激光雷达结构
│       │   ├── camera.urdf.xacro     # 深度相机结构
│       │   └── imu.urdf.xacro        # IMU 传感器结构
│       └── plugins/
│           ├── gazebo_control_plugin.xacro # 差速控制真值插件 (解决里程计漂移的关键)
│           └── gazebo_sensor_plugin.xacro  # 传感器仿真数据输出插件
│
└── waypoint/                         # 自动多点巡航控制算法包
    ├── config/
    │   └── waypoints.yaml            # 预设的巡航路线坐标点 (包含8个途径点)
    ├── launch/
    │   └── pose_launch.py            # 巡航节点启动脚本 (可传入 nav_mode、lookahead_dist 等参数)
    └── waypoint/
        ├── mutil_pose.py             # 多点巡航核心业务逻辑 (含 Through/Follow 双模式、三次碰撞自愈重试功能)
        ├── init_bot_pose.py          # 小车初始位姿标定辅助测试脚本
        ├── go_to_pose.py             # 单点导航测试脚本
        └── get_pose.py               # 航点坐标拾取辅助脚本
```

## 特别说明
* 由于比赛中将使用实车，所以小车采用的是鱼香ros提供的开源代码，但是对激光雷达安装位置进行了降低，从而更好地实现避障。并将里程计来源更改为gazebo坐标系，使定位更精准。同时提高了小车的扭矩，提升了速度性能，完成导航项目大概在24s左右，仿真数据显示速度基本在1.5m/s左右。
* 由于gazebo中的锥桶底部是方形，导致小车在高速下，进行极端避障测试时（设置离锥桶很近的导航点）会出现小车撞到锥桶底座的情况，由于物理限制，激光雷达高度不能再低，所以将锥桶的碰撞箱更改为矩形，从而实现更好的避障。（或者采用深度相机，进行辅助避障；将yolo部署到小车中等方法）。
