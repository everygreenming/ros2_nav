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
        
        self.declare_parameter('save_interval', 0.5)
        
        # 定位工作空间根目录
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
        
        # 准备输出目录
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir, exist_ok=True)
            self.img_counter = 0
        else:
            self.img_counter = self.get_existing_max_index()
            
        self.bridge = CvBridge()
        self.last_save_time = time.time()
        
        # 订阅图像
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

    def get_existing_max_index(self):
        # 扫描现有图片序列号
        try:
            files = os.listdir(self.save_dir)
            indices = [int(f.split('_')[1].split('.')[0]) for f in files if f.startswith('frame_') and f.endswith('.jpg')]
            if indices:
                return max(indices)
        except Exception:
            pass
        return 0
        
    def image_callback(self, msg):
        current_time = time.time()
        # 定时采集
        if current_time - self.last_save_time >= self.save_interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.img_counter += 1
                filename = f"frame_{self.img_counter:06d}.jpg"
                filepath = os.path.join(self.save_dir, filename)
                
                cv2.imwrite(filepath, cv_image)
                self.last_save_time = current_time
                self.get_logger().info(f"[{self.img_counter}] Saved frame: {filename}")
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ImageCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
