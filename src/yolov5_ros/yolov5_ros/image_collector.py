#!/usr/bin/env python3
import os
import cv2
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCollectorNode(Node):
    def __init__(self):
        super().__init__('image_collector')
        self.get_logger().info('Initializing image collector node...')
        
        # 声明采集周期（单位：秒，默认 0.5s）与保存路径
        self.declare_parameter('save_interval', 0.5)
        
        # 默认保存在虚拟机桌面工作空间下的 dataset_raw 文件夹里
        default_dir = os.path.expanduser('~/Desktop/bot_ws/dataset_raw')
        self.declare_parameter('save_dir', default_dir)
        
        self.save_interval = self.get_parameter('save_interval').value
        self.save_dir = self.get_parameter('save_dir').value
        
        # 创建目标文件夹
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir, exist_ok=True)
            self.get_logger().info(f"Created save directory: {self.save_dir}")
        else:
            self.get_logger().info(f"Saving images to existing directory: {self.save_dir}")
            
        self.bridge = CvBridge()
        self.img_counter = 0
        self.last_save_time = time.time()
        
        # 订阅相机图像话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info(f"Image collector is ready! Interval: {self.save_interval}s, Target: {self.save_dir}")
        
    def image_callback(self, msg):
        current_time = time.time()
        # 校验设定的时间间隔是否已过
        if current_time - self.last_save_time >= self.save_interval:
            try:
                # 转换图像
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                # 构造顺序排布的文件名（如 frame_000001.jpg）
                self.img_counter += 1
                filename = f"frame_{self.img_counter:06d}.jpg"
                filepath = os.path.join(self.save_dir, filename)
                
                # 写入本地
                cv2.imwrite(filepath, cv_image)
                self.last_save_time = current_time
                
                self.get_logger().info(f"[{self.img_counter}] Saved frame: {filename}")
            except Exception as e:
                self.get_logger().warn(f"Failed to capture frame: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Shutting down collector. Total frames collected: {node.img_counter}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
