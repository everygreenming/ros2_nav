#!/usr/bin/env python3
import os
import sys
import json
import urllib.request
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

# 尝试导入 onnxruntime
try:
    import onnxruntime as ort
    HAS_ORT = True
except ImportError:
    HAS_ORT = False
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

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
        self.get_logger().info('Initializing yolov5 node...')

        # 声明参数
        self.declare_parameter('model_path', 'yolov5s.onnx')
        classes_param = Parameter('classes', Parameter.Type.STRING_ARRAY, [])
        self.declare_parameter(classes_param.name, classes_param.get_parameter_value())
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('score_threshold', 0.25)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('input_size', 416)

        self.model_name = self.get_parameter('model_path').value
        self.custom_classes = self.get_parameter('classes').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.score_threshold = self.get_parameter('score_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.input_size = self.get_parameter('input_size').value

        if len(self.custom_classes) > 0:
            self.classes = self.custom_classes
        else:
            self.classes = COCO_CLASSES

        self.bridge = CvBridge()
        self.resolve_model_path()

        self.get_logger().info(f'Loading model: {self.model_file}')
        self.use_ort = False

        # 加载 ONNX 模型
        if HAS_ORT:
            try:
                self.ort_session = ort.InferenceSession(self.model_file, providers=['CPUExecutionProvider'])
                self.ort_input_name = self.ort_session.get_inputs()[0].name
                self.use_ort = True
            except Exception as e:
                self.get_logger().warn(f'ONNX Runtime failed, fallback to OpenCV: {str(e)}')

        if not self.use_ort:
            try:
                self.net = cv2.dnn.readNetFromONNX(self.model_file)
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            except Exception as e:
                self.get_logger().error(f'OpenCV DNN failed: {str(e)}')
                sys.exit(1)

        # 订阅/发布
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        self.detection_pub = self.create_publisher(String, '/yolo/detections', 10)

    def resolve_model_path(self):
        # 解析模型路径并按需下载
        if os.path.isabs(self.model_name):
            self.model_file = self.model_name
            return

        from ament_index_python.packages import get_package_share_directory
        try:
            share_dir = get_package_share_directory('yolov5_ros')
            self.model_file = os.path.join(share_dir, 'models', self.model_name)
        except Exception:
            self.model_file = os.path.join('models', self.model_name)

        if not os.path.exists(self.model_file):
            if self.model_name == 'yolov5s.onnx':
                os.makedirs(os.path.dirname(self.model_file), exist_ok=True)
                url = 'https://github.com/ultralytics/yolov5/releases/download/v6.1/yolov5s.onnx'
                try:
                    urllib.request.urlretrieve(url, self.model_file)
                except Exception as e:
                    raise FileNotFoundError(f'Download failed: {self.model_file}')
            else:
                raise FileNotFoundError(f'Model not found: {self.model_file}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return

        img_height, img_width = cv_image.shape[:2]
        blob = cv2.dnn.blobFromImage(cv_image, 1.0 / 255.0, (self.input_size, self.input_size), swapRB=True, crop=False)

        # 推理
        if self.use_ort:
            try:
                outputs = self.ort_session.run(None, {self.ort_input_name: blob})
                predictions = np.squeeze(outputs[0])
            except Exception:
                return
        else:
            try:
                self.net.setInput(blob)
                outputs = self.net.forward()
                predictions = np.squeeze(outputs)
            except Exception:
                return

        # 解析预测结果
        if len(predictions.shape) == 1:
            predictions = np.expand_dims(predictions, axis=0)

        confidences_all = predictions[:, 4]
        mask = confidences_all >= self.conf_threshold
        filtered_preds = predictions[mask]

        boxes = []
        confidences = []
        class_ids = []
        x_factor = img_width / float(self.input_size)
        y_factor = img_height / float(self.input_size)

        if len(filtered_preds) > 0:
            class_scores = filtered_preds[:, 5:]
            best_class_ids = np.argmax(class_scores, axis=1)
            best_scores = class_scores[np.arange(len(filtered_preds)), best_class_ids]
            score_mask = best_scores > self.score_threshold

            filtered_preds = filtered_preds[score_mask]
            best_class_ids = best_class_ids[score_mask]
            best_scores = best_scores[score_mask]

            if len(filtered_preds) > 0:
                cx, cy = filtered_preds[:, 0], filtered_preds[:, 1]
                w, h = filtered_preds[:, 2], filtered_preds[:, 3]

                left = ((cx - w / 2.0) * x_factor).astype(int)
                top = ((cy - h / 2.0) * y_factor).astype(int)
                width = (w * x_factor).astype(int)
                height = (h * y_factor).astype(int)

                boxes = np.stack([left, top, width, height], axis=1).tolist()
                confidences = (filtered_preds[:, 4] * best_scores).tolist()
                class_ids = best_class_ids.tolist()

        # NMS 过滤
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        detections = []
        
        if len(indices) > 0:
            flat_indices = np.array(indices).flatten()
            for idx in flat_indices:
                box = boxes[idx]
                left, top, width, height = box
                conf = confidences[idx]
                cid = class_ids[idx]

                left, top = max(0, left), max(0, top)
                width, height = min(width, img_width - left), min(height, img_height - top)

                class_name = self.classes[cid] if cid < len(self.classes) else f"class_{cid}"

                detections.append({
                    "class": class_name,
                    "confidence": float(conf),
                    "bbox": [int(left), int(top), int(left + width), int(top + height)]
                })

                # 绘制包围盒
                cv2.rectangle(cv_image, (left, top), (left + width, top + height), (0, 255, 0), 2)
                label = f"{class_name}: {conf:.2f}"
                cv2.putText(cv_image, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 发布结果
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
        except Exception:
            pass

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
