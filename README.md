#  ROS 2 + Nav2 + Gazebo 差速小车自主导航项目

这是一个基于 ROS 2、Navigation2 (Nav2) 和 Gazebo 仿真环境的自主导航小车项目。项目集成了机器人底盘描述、Gazebo 仿真物理世界、地图与导航配置文件，并实现了一套具备平滑曲线切弯、碰撞自愈和断点重试的多点自主导航算法。


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

*   **`src/fishbot_description/`**：包含小车的三维网格、传感器（雷达、IMU、深度相机）和执行器轮子的 URDF/Xacro 宏定义，以及 Gazebo 物理属性插件。
*   **`src/bot/`**：包含 Gazebo 的仿真世界定义（`worlds/bot.world`，内含优化后的锥桶碰撞模型）以及 Gazebo 的启动 launch 脚本。
*   **`src/bot_nav2/`**：包含导航配置文件（`config/nav2_params.yaml`）、调参后的through行为树、建好的栅格地图（`maps/`）以及 Nav2 启动 launch 脚本。
*   **`src/waypoint/`**：多点导航包，包含节点核心逻辑 `mutil_pose.py`、预设航点文件 `config/waypoints.yaml` 和启动脚本 `pose_launch.py`。

## 特别说明
* 由于比赛中将使用实车，所以小车采用的是鱼香ros提供的开源代码，但是对激光雷达安装位置进行了降低，从而更好地实现避障。并将里程计来源更改为gazebo坐标系，使定位更精准。同时提高了小车的扭矩，提升了速度性能，完成导航项目大概在24s左右，仿真数据显示速度基本在1.5m/s左右。
* 由于gazebo中的锥桶底部是方形，导致小车在高速下，进行极端避障测试时（设置离锥桶很近的导航点）会出现小车撞到锥桶底座的情况，由于物理限制，激光雷达高度不能再低，所以将锥桶的碰撞箱更改为矩形，从而实现更好的避障。（或者采用深度相机，进行辅助避障；将yolo部署到小车中等方法）。
