import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取 waypoint 包在 install 目录下的 share 路径
    waypoint_pkg_share = get_package_share_directory('waypoint')
    
    # 2. 拼接参数文件的绝对路径
    # 注意：确保你的 waypoints.yaml 放在了 waypoint 包的 config 文件夹内
    params_file = os.path.join(waypoint_pkg_share, 'config', 'waypoints.yaml')
    
    # 3. 定义节点动作
    patrol_node = Node(
        package='waypoint',           # 脚本所在的包名
        executable='mutil_pose',     # 你的 Python 脚本文件名
        name='patrol_node',           # 必须与 waypoints.yaml 顶层字段一致
        output='screen',
        parameters=[params_file]      # 核心：将参数文件“喂”给节点
    )
    
    return LaunchDescription([
        patrol_node
    ])