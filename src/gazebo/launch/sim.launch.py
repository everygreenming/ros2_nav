import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    description_pkg_dir = get_package_share_directory('bot_description')
    gazebo_pkg_dir = get_package_share_directory('gazebo')
    
    # 定位模型与世界文件
    xacro_file = os.path.join(description_pkg_dir, 'urdf', 'bot', 'bot.urdf.xacro')
    world_file = os.path.join(gazebo_pkg_dir, 'worlds', 'bot.world')

    # 配置环境变量，防止纹理丢失
    bot_models_path = os.path.join(gazebo_pkg_dir, 'models')
    description_share_path = os.path.dirname(description_pkg_dir)
    combined_model_path = bot_models_path + ":" + description_share_path
    
    # 动态解析 xacro 生成 URDF
    robot_description_content = launch.substitutions.Command(['xacro ', xacro_file])
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        robot_description_content, value_type=str
    )

    # 启动 TF 状态发布器
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 启动 Gazebo 引擎
    action_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # 在指定坐标实例化模型
    node_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'bot',
            '-x', '0.3814',
            '-y', '3.4835',
            '-z', '0.15'
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=combined_model_path),
        node_robot_state_publisher,
        action_gazebo,
        node_spawn_entity,
    ])