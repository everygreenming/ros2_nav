import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_pkg_share = get_package_share_directory('waypoint')
    
    # 定位静态坐标表
    params_file = os.path.join(waypoint_pkg_share, 'config', 'waypoints.yaml')
    
    # 定义并声明控制模式参数
    nav_mode = LaunchConfiguration('nav_mode', default='through')
    lookahead_dist = LaunchConfiguration('lookahead_dist', default='1.0')
    max_retries = LaunchConfiguration('max_retries', default='3')
    
    declare_nav_mode_cmd = DeclareLaunchArgument(
        'nav_mode', default_value='through',
        description='Navigation mode: "through" or "follow"'
    )
    declare_lookahead_dist_cmd = DeclareLaunchArgument(
        'lookahead_dist', default_value='1.0',
        description='Switch threshold distance'
    )
    declare_max_retries_cmd = DeclareLaunchArgument(
        'max_retries', default_value='3',
        description='Maximum retry attempts'
    )
    
    # 启动自定义 Python 巡航节点
    patrol_node = Node(
        package='waypoint',           
        executable='mutil_pose', 
        name='patrol_node',      
        output='screen',
        parameters=[
            params_file,
            {
                'nav_mode': nav_mode,
                'lookahead_dist': lookahead_dist,
                'max_retries': max_retries
            }
        ]
    )
    
    return LaunchDescription([
        declare_nav_mode_cmd,
        declare_lookahead_dist_cmd,
        declare_max_retries_cmd,
        patrol_node
    ])