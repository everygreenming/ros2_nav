import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # 获取各个功能包的 share 路径
    bot_pkg_dir = get_package_share_directory('bot')
    bot_nav2_pkg_dir = get_package_share_directory('bot_nav2')
    waypoint_pkg_dir = get_package_share_directory('waypoint')

    # 定义 Launch 变量
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(bot_nav2_pkg_dir, 'maps', 'fishbot_map.yaml'))
    start_patrol = LaunchConfiguration('start_patrol', default='false')

    # 声明命令行参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map = DeclareLaunchArgument(
        'map', default_value=map_yaml_path,
        description='Full path to map file to load'
    )

    declare_start_patrol = DeclareLaunchArgument(
        'start_patrol', default_value='false',
        description='Whether to start the waypoint patrol node automatically (true/false)'
    )

    # 1. 启动 Gazebo 仿真环境（包括小车模型加挂、小车空投、环境加载）
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_pkg_dir, 'launch', 'sim.launch.py')
        )
    )

    # 2. 启动 Nav2 导航框架（包括 AMCL 定位、Costmaps 代价地图、Planners 规划器、RViz 可视化）
    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_nav2_pkg_dir, 'launch', 'nav2_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_path
        }.items()
    )

    # 3. 启动多点巡航脚本（可选，默认关闭）
    launch_waypoint = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(waypoint_pkg_dir, 'launch', 'pose_launch.py')
        ),
        condition=IfCondition(start_patrol)
    )

    return LaunchDescription([
        # 声明参数
        declare_use_sim_time,
        declare_map,
        declare_start_patrol,
        
        # 启动仿真
        launch_sim,
        
        # 启动导航
        launch_nav2,
        
        # 启动巡航节点（按需）
        launch_waypoint
    ])
