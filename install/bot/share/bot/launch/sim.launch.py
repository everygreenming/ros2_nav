import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 核心路径解析
    # 确保你的包名分别是 fishbot_description 和 bot
    fishbot_pkg_dir = get_package_share_directory('fishbot_description')
    bot_pkg_dir = get_package_share_directory('bot')
    
    # 【深度校准】根据你的反馈，xacro 文件在 urdf/fishbot/ 目录下
    xacro_file = os.path.join(fishbot_pkg_dir, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    world_file = os.path.join(bot_pkg_dir, 'worlds', 'bot.world')

    # 2. 环境变量合并逻辑 (解决看不到地图和小车的关键)
    # 你的障碍物/地图模型路径
    bot_models_path = os.path.join(bot_pkg_dir, 'models')
    # 机器人包的 share 路径 (让 Gazebo 认识 model://fishbot_description)
    # get_package_share_directory 返回的是 .../share/fishbot_description
    # 我们需要的是它的父目录 .../share/
    fishbot_share_path = os.path.dirname(fishbot_pkg_dir)
    
    # 合并路径并注入环境变量
    combined_model_path = bot_models_path + ":" + fishbot_share_path
    
    # 3. 机器人描述 (Xacro 解析)
    robot_description_content = launch.substitutions.Command(['xacro ', xacro_file])
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        robot_description_content, value_type=str
    )

    # 4. 节点定义
    # 机器人状态发布器
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True # 仿真必开
        }]
    )

    # Gazebo 启动
    action_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # 机器人空投
    node_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'fishbot',
            '-x', '0.3814',
            '-y', '3.4835',
            '-z', '0.1' # 稍微抬高一点，防止和地面发生初始碰撞导致飞掉
        ],
        output='screen'
    )

    # 5. 控制器加载逻辑 (使用事件处理器确保顺序)
    load_joint_state_broadcaster = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fishbot_diff_drive_controller'],
        output='screen'
    )

    return launch.LaunchDescription([
        # A. 设置 Gazebo 搜索路径 (环境变量必须最先设置)
        launch.actions.SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=combined_model_path),
        
        # B. 启动核心节点
        node_robot_state_publisher,
        action_gazebo,
        node_spawn_entity,
        
        # C. 链式启动控制器：空投成功 -> 加载状态广播器 -> 加载差速控制器
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=node_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_controller],
            )
        ),
    ])