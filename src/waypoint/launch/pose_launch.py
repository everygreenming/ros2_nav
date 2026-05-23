import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取 waypoint 包在 install 目录下的 share 路径
    waypoint_pkg_share = get_package_share_directory('waypoint')
    
    # 2. 拼接参数文件的绝对路径
    # 注意：确保你的 waypoints.yaml 放在了 waypoint 包的 config 文件夹内
    params_file = os.path.join(waypoint_pkg_share, 'config', 'waypoints.yaml')
    
    # 3. 声明 Launch 参数，允许在命令行指定模式并默认为 through
    nav_mode = LaunchConfiguration('nav_mode', default='through')
    
    declare_nav_mode_cmd = DeclareLaunchArgument(
        'nav_mode',
        default_value='through',
        description='Navigation mode: "through" (continuous via-points) or "follow" (stop-at-waypoints)'
    )
    
    # 4. 定义节点动作
    patrol_node = Node(
        package='waypoint',           # 脚本所在的包名
        executable='mutil_pose',     # 你的 Python 脚本文件名
        name='patrol_node',           # 必须与 waypoints.yaml 顶层字段一致
        output='screen',
        parameters=[
            params_file,              # 将参数文件“喂”给节点
            {'nav_mode': nav_mode}    # 通过命令行传入的 nav_mode 参数覆盖 yaml 中的值
        ]
    )
    
    return LaunchDescription([
        declare_nav_mode_cmd,
        patrol_node
    ])