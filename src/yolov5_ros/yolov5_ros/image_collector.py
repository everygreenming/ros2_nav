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
        
        # 动态寻找工作空间根目录（通过往上级目录查找包含 src 文件夹的目录）
        # 这样无论在物理机、无桌面虚拟机构建、还是通过软链接安装都能自动定位到工作空间根目录，防止硬编码 Desktop
        ws_root = os.path.expanduser('~')
        current_dir = os.path.dirname(os.path.abspath(__file__))
        while current_dir and current_dir != os.path.dirname(current_dir):
            if os.path.exists(os.path.join(current_dir, 'src')):
                ws_root = current_dir
                break
            current_dir = os.path.dirname(current_dir)
            
        default_dir = os.path.join(ws_root, 'dataset_raw')
        self.declare_parameter('save_dir', default_dir)
        
        self.save_interval = self.get_parameter('save_interval').value
        self.save_dir = self.get_parameter('save_dir').value
        
        # 创建目标文件夹
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir, exist_ok=True)
            self.get_logger().info(f"Created save directory: {self.save_dir}")
            self.img_counter = 0
        else:
            self.get_logger().info(f"Saving images to existing directory: {self.save_dir}")
            # 扫描已有图片，找到最大序号以防覆盖旧图片
            self.img_counter = self.get_existing_max_index()
            
        self.bridge = CvBridge()
        self.last_save_time = time.time()
        
        # 订阅相机图像话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info(f"Image collector is ready! Interval: {self.save_interval}s, Target: {self.save_dir}")

    def get_existing_max_index(self):
        """
        扫描保存目录，找到已存在的最大图片序号
        """
        try:
            files = os.listdir(self.save_dir)
            indices = []
            for f in files:
                if f.startswith('frame_') and f.endswith('.jpg'):
                    try:
                        # 提取 frame_000120.jpg 中的 120
                        num = int(f.split('_')[1].split('.')[0])
                        indices.append(num)
                    except (IndexError, ValueError):
                        continue
            if indices:
                max_idx = max(indices)
                self.get_logger().info(f"Found {len(indices)} existing frames. Resuming count from index {max_idx}...")
                return max_idx
        except Exception as e:
            self.get_logger().warn(f"Failed to scan directory for index: {str(e)}")
        return 0
        
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
