import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. 获取与拼接默认路径
    bot_nav2_dir = get_package_share_directory('bot_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 默认使用 nav2 官方的 rviz 配置，省得我们自己一个个添加图层
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 2. 动态解析自定义行为树路径（关键改动）
    bt_xml_path = os.path.join(
        bot_nav2_dir, 'behavior_trees', 'my_nav_through_poses.xml')

    # 3. 创建 Launch 配置变量
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(bot_nav2_dir, 'maps', 'fishbot_map.yaml'))
        
    nav2_param_path = os.path.join(bot_nav2_dir, 'config', 'nav2_params.yaml')

    # 4. 使用 RewrittenYaml 动态注入行为树路径
    #    这样 nav2_params.yaml 中的空字符串会被替换为实际的 install 路径
    configured_params = RewrittenYaml(
        source_file=nav2_param_path,
        param_rewrites={
            'default_nav_through_poses_bt_xml': bt_xml_path
        },
        convert_types=True
    )

    return launch.LaunchDescription([
        # 5. 声明新的 Launch 参数，暴露给命令行
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),

        # 6. 包含 Nav2 核心启动脚本（使用动态注入后的参数文件）
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': configured_params}.items(),
        ),
        
        # 7. 启动 RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])