#!/usr/bin/env python3
import os
import sys
import json
import urllib.request
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# 官方默认的 80 类 COCO 类别名称
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.get_logger().info('Initializing yolov5_ros object detection node...')

        # 声明 ROS 2 参数
        self.declare_parameter('model_path', 'yolov5s.onnx')
        self.declare_parameter('classes', [])
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('score_threshold', 0.25)
        self.declare_parameter('nms_threshold', 0.45)

        # 获取参数
        self.model_name = self.get_parameter('model_path').value
        self.custom_classes = self.get_parameter('classes').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.score_threshold = self.get_parameter('score_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value

        # 决定类别列表
        if len(self.custom_classes) > 0:
            self.classes = self.custom_classes
            self.get_logger().info(f'Loading custom classes: {self.classes}')
        else:
            self.classes = COCO_CLASSES
            self.get_logger().info('No custom classes provided, loading default COCO 80 classes.')

        # 桥接转换器
        self.bridge = CvBridge()

        # 解析模型文件路径
        self.resolve_model_path()

        # 加载 ONNX 模型
        self.get_logger().info(f'Loading ONNX model: {self.model_file}')
        try:
            self.net = cv2.dnn.readNetFromONNX(self.model_file)
            # 开启硬件加速（如果可用，在 CPU 上这能启用加速计算）
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {str(e)}')
            sys.exit(1)

        # 订阅小车深度/彩色相机的彩色图
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 发布带有边界框的可视化图像
        self.image_pub = self.create_publisher(
            Image,
            '/yolo/annotated_image',
            10
        )

        # 发布检测信息的 JSON 文本话题
        self.detection_pub = self.create_publisher(
            String,
            '/yolo/detections',
            10
        )

        self.get_logger().info('YOLOv5 ROS 2 node is ready!')

    def resolve_model_path(self):
        """
        动态解析模型加载位置，如果不存在且是默认模型，则尝试从 GitHub 自动下载。
        """
        # 如果是绝对路径，直接使用
        if os.path.isabs(self.model_name):
            self.model_file = self.model_name
            return

        # 否则，优先去 package 的 share 目录下查找
        from ament_index_python.packages import get_package_share_directory
        try:
            share_dir = get_package_share_directory('yolov5_ros')
            self.model_file = os.path.join(share_dir, 'models', self.model_name)
        except Exception:
            # 备用方案：检查当前工作目录下的 models 文件夹
            self.model_file = os.path.join('models', self.model_name)

        # 如果文件不存在，且是默认模型 yolov5s.onnx，执行自动下载
        if not os.path.exists(self.model_file):
            if self.model_name == 'yolov5s.onnx':
                self.get_logger().info(f'Default model yolov5s.onnx not found at {self.model_file}')
                self.get_logger().info('Downloading pre-trained yolov5s.onnx (28MB) from official release...')
                os.makedirs(os.path.dirname(self.model_file), exist_ok=True)
                url = 'https://github.com/ultralytics/yolov5/releases/download/v6.1/yolov5s.onnx'
                try:
                    urllib.request.urlretrieve(url, self.model_file)
                    self.get_logger().info('Model download finished successfully.')
                except Exception as e:
                    self.get_logger().error(f'Failed to download model: {str(e)}. Please check internet connection.')
                    # 备用下载方案，尝试国内镜像或直接抛出错误
                    raise FileNotFoundError(f'Could not download or find default model: {self.model_file}')
            else:
                self.get_logger().error(f'Custom model file NOT found at: {self.model_file}')
                self.get_logger().error('Please place your best.onnx into src/yolov5_ros/models/ and colcon build.')
                raise FileNotFoundError(f'Custom model file not found: {self.model_file}')

    def image_callback(self, msg):
        """
        图像话题接收回调，运行 YOLOv5 目标检测。
        """
        try:
            # 1. 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert ROS Image to OpenCV: {str(e)}')
            return

        img_height, img_width = cv_image.shape[:2]

        # 2. 预处理图像为 YOLOv5 尺寸（640x640）
        blob = cv2.dnn.blobFromImage(cv_image, 1.0 / 255.0, (640, 640), swapRB=True, crop=False)

        # 3. 前向推理
        self.net.setInput(blob)
        outputs = self.net.forward()  # outputs shape: (1, 25200, 85) 或 (25200, 85)

        # 4. 高效解析输出（NumPy 向量化加速过滤）
        predictions = np.squeeze(outputs)
        if len(predictions.shape) == 1:
            predictions = np.expand_dims(predictions, axis=0)

        # 获取物体存在置信度（5th column）
        confidences_all = predictions[:, 4]
        mask = confidences_all >= self.conf_threshold
        filtered_preds = predictions[mask]

        boxes = []
        confidences = []
        class_ids = []

        x_factor = img_width / 640.0
        y_factor = img_height / 640.0

        if len(filtered_preds) > 0:
            # 获取类别预测得分（从 6th column 开始）
            class_scores = filtered_preds[:, 5:]
            best_class_ids = np.argmax(class_scores, axis=1)
            # 过滤类别最高得分低于阈值的行
            best_scores = class_scores[np.arange(len(filtered_preds)), best_class_ids]
            score_mask = best_scores > self.score_threshold

            filtered_preds = filtered_preds[score_mask]
            best_class_ids = best_class_ids[score_mask]
            best_scores = best_scores[score_mask]

            if len(filtered_preds) > 0:
                # 坐标尺度还原
                cx = filtered_preds[:, 0]
                cy = filtered_preds[:, 1]
                w = filtered_preds[:, 2]
                h = filtered_preds[:, 3]

                left = ((cx - w / 2.0) * x_factor).astype(int)
                top = ((cy - h / 2.0) * y_factor).astype(int)
                width = (w * x_factor).astype(int)
                height = (h * y_factor).astype(int)

                # DWB 需要原生 Python 列表格式
                boxes = np.stack([left, top, width, height], axis=1).tolist()
                confidences = (filtered_preds[:, 4] * best_scores).tolist() # 合成最终置信度
                class_ids = best_class_ids.tolist()

        # 5. 非极大值抑制（NMS）消除重叠边界框
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)

        detections = []
        
        # 处理 NMS 筛选后的检测物体
        if len(indices) > 0:
            # 兼容不同版本 OpenCV 返回结构体差异
            flat_indices = np.array(indices).flatten()
            for idx in flat_indices:
                box = boxes[idx]
                left, top, width, height = box
                conf = confidences[idx]
                cid = class_ids[idx]

                # 限制越界
                left = max(0, left)
                top = max(0, top)
                width = min(width, img_width - left)
                height = min(height, img_height - top)

                # 获取类别名称
                class_name = self.classes[cid] if cid < len(self.classes) else f"class_{cid}"

                detections.append({
                    "class": class_name,
                    "confidence": float(conf),
                    "bbox": [int(left), int(top), int(left + width), int(top + height)]
                })

                # 6. 在画面上绘制边界框
                cv2.rectangle(cv_image, (left, top), (left + width, top + height), (0, 255, 0), 2)
                label = f"{class_name}: {conf:.2f}"
                cv2.putText(cv_image, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 7. 发布标记图像
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish annotated image: {str(e)}')

        # 8. 发布文本检测数据 (JSON)
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detection_pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
