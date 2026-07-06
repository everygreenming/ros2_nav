import os
import yaml
import tempfile
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def merge_yaml_files(yaml_files):
    merged_dict = {}
    for yaml_file in yaml_files:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            if data:
                merged_dict.update(data)
    
    tmp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(merged_dict, tmp_file, default_flow_style=False)
    tmp_file.close()
    return tmp_file.name

def generate_launch_description():
    # 路径解析
    nav2_pkg_dir = get_package_share_directory('nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 优先加载自定义 rviz 配置
    custom_rviz_config_dir = os.path.join(nav2_pkg_dir, 'rviz', 'nav2.rviz')
    if os.path.exists(custom_rviz_config_dir):
        rviz_config_dir = custom_rviz_config_dir
    else:
        rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    bt_xml_path = os.path.join(nav2_pkg_dir, 'behavior_trees', 'my_nav_through_poses.xml')

    # Launch 配置参数
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(nav2_pkg_dir, 'maps', 'bot_map.yaml'))
        
    # 合并所有的模块化配置文件
    config_dir = os.path.join(nav2_pkg_dir, 'config')
    yaml_files_to_merge = [
        os.path.join(config_dir, 'nav2_amcl.yaml'),
        os.path.join(config_dir, 'nav2_dwb_controller.yaml'),
        os.path.join(config_dir, 'nav2_global_planner.yaml'),
        os.path.join(config_dir, 'nav2_costmaps.yaml'),
        os.path.join(config_dir, 'nav2_behaviors.yaml'),
        os.path.join(config_dir, 'nav2_common.yaml')
    ]
    merged_nav2_param_path = merge_yaml_files(yaml_files_to_merge)

    # 动态注入行为树路径
    configured_params = RewrittenYaml(
        source_file=merged_nav2_param_path,
        param_rewrites={'default_nav_through_poses_bt_xml': bt_xml_path},
        convert_types=True
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),

        # 包含 Nav2 官方 bringup 启动脚本并传入合并参数
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
                'initial_pose_x': '0.3814',
                'initial_pose_y': '3.4835',
                'initial_pose_yaw': '0.0'}.items(),
        ),
        
        # 启动 RViz2 可视化
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])