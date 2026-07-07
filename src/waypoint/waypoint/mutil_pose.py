#!/usr/bin/env python3
import rclpy
import numpy as np
# 补丁：解决 NumPy 1.24+ 与 transforms3d 兼容性问题
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
        # 初始化参数
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
        
        # TF 初始化
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

    def get_robot_pose(self):
        # 获取机器人当前位置
        for base_frame in ['base_footprint', 'base_link']:
            try:
                tf = self.buffer_.lookup_transform('map', base_frame, rclpy.time.Time())
                return tf.transform.translation.x, tf.transform.translation.y
            except Exception:
                continue
        return None

    def get_pose_by_xyyaw(self, x, y, yaw):
        # 生成位姿目标
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # 强制设置时间戳
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
        # 初始化位置
        self.initial_point_ = self.get_parameter('initial_point').value
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2]))
        self.waitUntilNav2Active()

    def get_target_points_list(self):
        # 生成目标点列表
        poses = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            
            target_pose = self.get_pose_by_xyyaw(x, y, yaw)
            poses.append(target_pose)
            self.get_logger().info(f'目标点: {index}->({x}, {y}, {yaw})')
            
        return poses

def main():
    rclpy.init()
    patrol = PatrolNode()
    
    patrol.get_logger().info('初始化机器人位姿...')
    patrol.init_robot_pose()
    patrol.get_logger().info('位姿初始化完成。')

    route_poses = patrol.get_target_points_list()
    
    if not route_poses:
        patrol.get_logger().error('未读取到目标点。')
        return

    nav_mode = patrol.get_parameter('nav_mode').value
    max_retries = patrol.get_parameter('max_retries').value
    
    import time
    
    if nav_mode == 'through':
        import math
        def get_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
        passed_idx = 0
        total_points = len(route_poses)
        task_succeeded = False
        
        for attempt in range(max_retries):
            remaining_poses = route_poses[passed_idx:]
            
            if not remaining_poses:
                patrol.get_logger().info('路点已全部通过。')
                task_succeeded = True
                break
                
            patrol.get_logger().info(f'开始连贯穿越 (尝试 {attempt+1})')
            patrol.goThroughPoses(remaining_poses)
            
            while not patrol.isTaskComplete():
                feedback = patrol.getFeedback()
                if feedback and int(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) % 3 == 0:
                    patrol.get_logger().info(f'导航中... 剩余: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s')
                
                # 检测是否到达目标
                robot_pose = patrol.get_robot_pose()
                if robot_pose is not None:
                    while passed_idx < total_points:
                        target_x = route_poses[passed_idx].pose.position.x
                        target_y = route_poses[passed_idx].pose.position.y
                        dist = get_distance(robot_pose, (target_x, target_y))
                        if dist < 0.7:
                            patrol.get_logger().info(f'通过目标点 {passed_idx + 1}/{total_points}')
                            passed_idx += 1
                        else:
                            break
                            
                time.sleep(0.1)
            
            # 结果处理
            result = patrol.getResult()
            if result == TaskResult.SUCCEEDED:
                patrol.get_logger().info('导航成功。')
                task_succeeded = True
                break
            elif result == TaskResult.CANCELED:
                patrol.get_logger().warn('导航取消。')
                break
            elif result == TaskResult.FAILED:
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
                    patrol.get_logger().warn(f'导航失败，清除代价地图并重试...')
                    patrol.clearAllCostmaps()
                    time.sleep(2.0)
                else:
                    patrol.get_logger().error('已达最大重试次数，任务失败。')

    elif nav_mode == 'follow':
        lookahead_dist = patrol.get_parameter('lookahead_dist').value
        patrol.get_logger().info(f'开始逐点停靠导航')
        
        current_idx = 0
        total_points = len(route_poses)
        
        patrol.goToPose(route_poses[current_idx])
        
        import math
        def get_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
        task_failed = False
        attempts = 0
        
        while current_idx < total_points:
            # 检查任务状态
            if patrol.isTaskComplete():
                result = patrol.getResult()
                if result == TaskResult.SUCCEEDED:
                    patrol.get_logger().info(f'到达目标点 {current_idx + 1}/{total_points}')
                    current_idx += 1
                    attempts = 0
                    if current_idx < total_points:
                        patrol.goToPose(route_poses[current_idx])
                    continue
                else:
                    if attempts < max_retries - 1:
                        attempts += 1
                        patrol.get_logger().warn(f'导航失败，正在重试...')
                        patrol.clearAllCostmaps()
                        time.sleep(2.0)
                        patrol.goToPose(route_poses[current_idx])
                    else:
                        patrol.get_logger().error(f'目标点 {current_idx + 1} 导航失败，退出任务。')
                        task_failed = True
                        break
            else:
                # 检测距离提前切换
                robot_pose = patrol.get_robot_pose()
                if robot_pose is not None:
                    target_x = route_poses[current_idx].pose.position.x
                    target_y = route_poses[current_idx].pose.position.y
                    dist = get_distance(robot_pose, (target_x, target_y))
                    
                    if current_idx < total_points - 1:
                        if dist < lookahead_dist:
                            patrol.get_logger().info(f'提前切换至目标点 {current_idx + 2}...')
                            current_idx += 1
                            attempts = 0
                            patrol.goToPose(route_poses[current_idx])
                    else:
                        if int(time.time() * 2) % 3 == 0:
                            patrol.get_logger().info(f'距离终点: {dist:.2f} 米')
            
            time.sleep(0.1)
            
        if not task_failed:
            patrol.get_logger().info('导航完成。')
            
    else:
        patrol.get_logger().error(f'无效模式: "{nav_mode}"')

    rclpy.shutdown()

if __name__ == '__main__':
    main()