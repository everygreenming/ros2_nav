#!/usr/bin/env python3
import rclpy
import numpy as np
# 兼容性补丁：NumPy 1.24+ 移除了 np.float，但 ROS 2 依赖的 transforms3d 仍在使用它
if not hasattr(np, 'float'):
    np.float = float

from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('nav_mode', 'through')
        self.declare_parameter('lookahead_dist', 1.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.nav_mode_ = self.get_parameter('nav_mode').value
        self.lookahead_dist_ = self.get_parameter('lookahead_dist').value
        self.max_retries_ = self.get_parameter('max_retries').value
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        
        # 实时位置获取 TF 相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

    def get_robot_pose(self):
        """
        通过 TF 获取小车当前在 map 坐标系下的位置
        """
        for base_frame in ['base_footprint', 'base_link']:
            try:
                tf = self.buffer_.lookup_transform('map', base_frame, rclpy.time.Time())
                return tf.transform.translation.x, tf.transform.translation.y
            except Exception:
                continue
        return None

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过 x,y,yaw 合成 PoseStamped
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # 填充实时时间戳（在 ROS 2 中至关重要，时间戳为零的消息会被 AMCL 丢弃）
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        # 1. 先等待 Nav2 系统完全启动并处于 Active 状态
        self.get_logger().info('正在等待 Nav2 导航及定位生命周期节点激活...')
        self.waitUntilNav2Active()
        
        # 2. 系统就绪后，再下发初始位姿，确保 AMCL 节点已经启动完毕并能接收到该位姿
        self.initial_point_ = self.get_parameter('initial_point').value
        self.get_logger().info(f'发送初始位姿: x={self.initial_point_[0]}, y={self.initial_point_[1]}')
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2]))
        
        # 3. 延时 3 秒给 AMCL 留出定位粒子收敛和建立 map->odom 坐标变换的时间
        import time
        self.get_logger().info('等待 3 秒以使定位（AMCL）收敛...')
        time.sleep(3.0)

    def get_target_points_list(self):
        """
        一次性获取所有目标点，并打包成 PoseStamped 列表
        """
        poses = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            
            # 将每个坐标直接转化为目标位姿
            target_pose = self.get_pose_by_xyyaw(x, y, yaw)
            poses.append(target_pose)
            
            self.get_logger().info(f'记录途径点: {index}->({x}, {y}, {yaw})')
            
        return poses

def main():
    rclpy.init()
    patrol = PatrolNode()
    
    patrol.get_logger().info('正在初始化机器人位姿...')
    patrol.init_robot_pose()
    patrol.get_logger().info('位姿初始化完成！准备生成全局路径。')

    # 【核心改动 1】：不再使用 for 循环一个个发，而是一次性打包所有点
    route_poses = patrol.get_target_points_list()
    
    if not route_poses:
        patrol.get_logger().error('没有读取到目标点，任务取消。')
        return

    # 获取导航模式并选择相应的 API
    nav_mode = patrol.get_parameter('nav_mode').value
    max_retries = patrol.get_parameter('max_retries').value
    
    import time
    
    if nav_mode == 'through':
        import math
        def get_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
        passed_idx = 0  # 记录已通过路点的索引数量
        total_points = len(route_poses)
        task_succeeded = False
        
        for attempt in range(max_retries):
            # 动态切片：只把剩下的未通过路点下发给 Nav2
            remaining_poses = route_poses[passed_idx:]
            
            if not remaining_poses:
                patrol.get_logger().info('所有路点实际上已全部通过，无需重试。')
                task_succeeded = True
                break
                
            patrol.get_logger().info(
                f'>>> 准备连贯穿越剩余的 {len(remaining_poses)}/{total_points} 个坐标点 (goThroughPoses) (第 {attempt+1} 次尝试) <<<'
            )
            patrol.goThroughPoses(remaining_poses)
            
            # 监控整个穿越任务的进度
            while not patrol.isTaskComplete():
                feedback = patrol.getFeedback()
                if feedback:
                    # 降低终端刷屏频率
                    if int(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) % 3 == 0:
                        patrol.get_logger().info(
                            f'正在导航中 (当前目标: {passed_idx+1}/{total_points})... 预计还需: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s'
                        )
                
                # 实时检测机器人是否通过了当前期望的路点
                robot_pose = patrol.get_robot_pose()
                if robot_pose is not None:
                    # 循环检查，防止在单次循环中漏掉快速经过的多个点
                    while passed_idx < total_points:
                        target_x = route_poses[passed_idx].pose.position.x
                        target_y = route_poses[passed_idx].pose.position.y
                        dist = get_distance(robot_pose, (target_x, target_y))
                        if dist < 0.7:  # 0.7 米范围内认为已通过
                            patrol.get_logger().info(f'检测到已成功通过目标点 {passed_idx + 1}/{total_points}，距离: {dist:.2f} 米')
                            passed_idx += 1
                        else:
                            break
                            
                time.sleep(0.1)  # 10Hz 监控频率
            
            # 最终任务结果研判
            result = patrol.getResult()
            if result == TaskResult.SUCCEEDED:
                patrol.get_logger().info('✅ 完美！小车已丝滑穿越全部点位！')
                task_succeeded = True
                break
            elif result == TaskResult.CANCELED:
                patrol.get_logger().warn('⚠️ 穿越任务被取消')
                break
            elif result == TaskResult.FAILED:
                # 失败时先最后检查一次在失败的这一刻有没有最新通过的点
                robot_pose = patrol.get_robot_pose()
                if robot_pose is not None:
                    while passed_idx < total_points:
                        target_x = route_poses[passed_idx].pose.position.x
                        target_y = route_poses[passed_idx].pose.position.y
                        dist = get_distance(robot_pose, (target_x, target_y))
                        if dist < 0.7:
                            passed_idx += 1
                        else:
                            break
                            
                if attempt < max_retries - 1:
                    patrol.get_logger().warn(
                        f'❌ 穿越任务第 {attempt+1} 次失败 (已通过 {passed_idx}/{total_points} 个点)。正在清除地图，准备从当前位置对剩余路点进行重试...'
                    )
                    patrol.clearAllCostmaps()
                    time.sleep(2.0)
                else:
                    patrol.get_logger().error('❌ 穿越任务已达最大重试次数，仍未成功，退出任务。')

    elif nav_mode == 'follow':
        lookahead_dist = patrol.get_parameter('lookahead_dist').value
        patrol.get_logger().info(f'>>> 准备逐点停靠导航 {len(route_poses)} 个坐标点 (提前 {lookahead_dist:.2f} 米切换，最大重试 {max_retries} 次) <<<')
        
        current_idx = 0
        total_points = len(route_poses)
        
        # 下发第一个目标点
        patrol.goToPose(route_poses[current_idx])
        
        import math
        
        def get_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
        task_failed = False
        attempts = 0  # 当前目标点导航重试计数
        
        while current_idx < total_points:
            # 1. 检查任务是否由于意外原因结束（如发生故障或者动作被取消）
            if patrol.isTaskComplete():
                result = patrol.getResult()
                if result == TaskResult.SUCCEEDED:
                    patrol.get_logger().info(f'到达最终目标点 {current_idx + 1}/{total_points}')
                    current_idx += 1
                    attempts = 0  # 目标完成，重置计数器
                    if current_idx < total_points:
                        patrol.goToPose(route_poses[current_idx])
                    continue
                else:
                    if attempts < max_retries - 1:
                        attempts += 1
                        patrol.get_logger().warn(f'在目标点 {current_idx + 1} 处导航第 {attempts} 次失败，正在清除代价地图并进行第 {attempts+1} 次尝试...')
                        patrol.clearAllCostmaps()
                        time.sleep(2.0)
                        patrol.goToPose(route_poses[current_idx])
                    else:
                        patrol.get_logger().error(f'在目标点 {current_idx + 1} 处导航失败已达最大重试次数，退出任务。')
                        task_failed = True
                        break
            
            # 2. 获取当前机器人的位置，并计算距离当前目标点的距离
            robot_pose = patrol.get_robot_pose()
            if robot_pose is not None:
                target_x = route_poses[current_idx].pose.position.x
                target_y = route_poses[current_idx].pose.position.y
                dist = get_distance(robot_pose, (target_x, target_y))
                
                # 3. 如果不是最后一个点，且距离小于设定的阈值，提前切换到下一个点
                if current_idx < total_points - 1:
                    if dist < lookahead_dist:
                        patrol.get_logger().info(
                            f'距离目标点 {current_idx + 1} 余 {dist:.2f}米 (< {lookahead_dist:.2f}m)，提前规划并切换至目标点 {current_idx + 2}...'
                        )
                        current_idx += 1
                        attempts = 0  # 成功提前预判切换，重置计数器
                        patrol.goToPose(route_poses[current_idx])
                else:
                    # 最后一个点，定期打印剩余距离
                    if int(time.time() * 2) % 3 == 0:
                        patrol.get_logger().info(f'前往终点中... 距离终点还剩: {dist:.2f} 米')
            
            # 10Hz 循环频率
            time.sleep(0.1)
            
        if not task_failed:
            patrol.get_logger().info('✅ 完美！已平滑完成所有点位的导航！')
            
    else:
        patrol.get_logger().error(f'无效的 nav_mode: "{nav_mode}"，无法执行导航任务。')

    rclpy.shutdown()

if __name__ == '__main__':
    main()