#!/usr/bin/env python3
import rclpy
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
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.nav_mode_ = self.get_parameter('nav_mode').value
        self.lookahead_dist_ = self.get_parameter('lookahead_dist').value
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
        self.initial_point_ = self.get_parameter('initial_point').value
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2]))
        self.waitUntilNav2Active()

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
    if nav_mode == 'through':
        patrol.get_logger().info(f'>>> 准备一次性连贯穿越 {len(route_poses)} 个坐标点 (goThroughPoses) <<<')
        patrol.goThroughPoses(route_poses)
        
        # 监控整个穿越任务的进度
        while not patrol.isTaskComplete():
            feedback = patrol.getFeedback()
            if feedback:
                # 降低终端刷屏频率
                if int(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) % 3 == 0:
                    patrol.get_logger().info(f'正在导航中... 预计全路段完成还需: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s')
        
        # 最终任务结果研判
        result = patrol.getResult()
        if result == TaskResult.SUCCEEDED:
            patrol.get_logger().info('✅ 完美！小车已丝滑穿越全部点位！')
        elif result == TaskResult.CANCELED:
            patrol.get_logger().warn('⚠️ 穿越任务被取消')
        elif result == TaskResult.FAILED:
            patrol.get_logger().error('❌ 穿越任务失败 (请检查中间点位是否在墙里，或者被障碍物彻底堵死)')

    elif nav_mode == 'follow':
        lookahead_dist = patrol.get_parameter('lookahead_dist').value
        patrol.get_logger().info(f'>>> 准备逐点停靠导航 {len(route_poses)} 个坐标点 (提前 {lookahead_dist:.2f} 米切换) <<<')
        
        current_idx = 0
        total_points = len(route_poses)
        
        # 下发第一个目标点
        patrol.goToPose(route_poses[current_idx])
        
        import math
        import time
        
        def get_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
        task_failed = False
        
        while current_idx < total_points:
            # 1. 检查任务是否由于意外原因结束（如发生故障或者动作被取消）
            if patrol.isTaskComplete():
                result = patrol.getResult()
                if result == TaskResult.SUCCEEDED:
                    patrol.get_logger().info(f'到达最终目标点 {current_idx + 1}/{total_points}')
                    current_idx += 1
                    if current_idx < total_points:
                        patrol.goToPose(route_poses[current_idx])
                    continue
                else:
                    patrol.get_logger().error(f'在目标点 {current_idx + 1} 处导航失败，退出任务。')
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