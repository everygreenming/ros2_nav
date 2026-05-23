#!/usr/bin/env python3
import math
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# 老师傅的数学工具：欧拉角转四元数
def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def main():
    rclpy.init()
    # 实例化 Nav2 领航员
    navigator = BasicNavigator()

    # 阻塞等待 Nav2 系统（包括代价地图和行为树）完全拉起
    print("正在等待 Nav2 导航栈就绪...")
    navigator.waitUntilNav2Active()
    print("Nav2 已就绪！准备下发巡航任务。")

    # 你的 8 个高精度业务目标点
    points = [
        (0.3814, 3.4835),  # 点 1 (起点)
        (9.3000, 3.4252),  # 点 2 (锥桶附近)
        (11.1020, 1.4488), # 点 3
        (13.5009, 1.4000), # 点 4
        (13.2923, 3.4000), # 点 5
        (11.0900, 3.4000), # 点 6 (蓝色台阶附近)
        (9.3000, 1.4401),  # 点 7
        (0.3582, 2.6580)   # 点 8 (终点)
    ]

    waypoints = []
    # 遍历坐标，将其转化为 ROS2 标准的 PoseStamped 消息
    for i in range(len(points)):
        pt = PoseStamped()
        pt.header.frame_id = 'map'
        pt.header.stamp = navigator.get_clock().now().to_msg()
        pt.pose.position.x = points[i][0]
        pt.pose.position.y = points[i][1]
        pt.pose.position.z = 0.0

        # 【核心逻辑】：计算车头朝向
        # 如果不是最后一个点，计算指向下一个点的向量角；如果是最后一个点，让其朝向 X 轴正向。
        if i < len(points) - 1:
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0 

        q = get_quaternion_from_euler(0.0, 0.0, yaw)
        pt.pose.orientation.x = q[0]
        pt.pose.orientation.y = q[1]
        pt.pose.orientation.z = q[2]
        pt.pose.orientation.w = q[3]

        waypoints.append(pt)

    # 提交多点跟随任务
    print("开始执行 8 点多点巡航任务！")
    navigator.followWaypoints(waypoints)

    # 状态机循环监控
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 15 == 0:
            print(f'目前状态：正在前往第 {feedback.current_waypoint + 1} 个目标点...')

    # 任务最终结果研判
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('✅ 完美！小车已成功跑完全部 8 个目标点！')
    elif result == TaskResult.CANCELED:
        print('⚠️ 任务被取消！')
    elif result == TaskResult.FAILED:
        print('❌ 任务失败！小车可能卡死了，请检查代价地图配置或环境障碍。')
    else:
        print('❓ 未知返回状态。')

    # 优雅关闭节点
    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()