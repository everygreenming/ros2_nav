import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 获取与拼接默认路径
    # 【修正点1】包名替换为你的 bot_nav2
    bot_nav2_dir = get_package_share_directory('bot_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 默认使用 nav2 官方的 rviz 配置，省得我们自己一个个添加图层
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 2. 创建 Launch 配置变量
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    
    # 【修正点2】指向你刚才扫出来的 fishbot_map.yaml
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(bot_nav2_dir, 'maps', 'fishbot_map.yaml'))
        
    # 【修正点3】指向我们刚才深度定制的 nav2_params.yaml
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(bot_nav2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # 3. 声明新的 Launch 参数，暴露给命令行
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        # 4. 包含 Nav2 核心启动脚本
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 将我们的参数传递给 bringup 脚本
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        
        # 5. 启动 RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])