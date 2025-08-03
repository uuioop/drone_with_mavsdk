#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import sys
import cv2
import torch
import numpy as np
from pathlib import Path

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# 获取包的路径
from ament_index_python.packages import get_package_share_directory

# 添加原始包的路径到sys.path
original_package_path = os.path.join(
    get_package_share_directory('plate_detection'), 
    '..', '..', 'src', 'Chinese_license_plate_detection_recognition'
)
if original_package_path not in sys.path:
    sys.path.append(original_package_path)

# 应用typing补丁
import typing
if not hasattr(typing, '_ClassVar'):
    typing._ClassVar = typing.ClassVar

# 导入原始包的功能
from ros2_plate_detection import (
    load_model, 
    detect_Recognition_plate, 
    draw_result,
    init_model
)

class SimplePlateDetectionNode(Node):
    def __init__(self):
        super().__init__('simple_plate_detection_node')
        
        # 声明参数
        self.declare_parameter('detect_model_path', 'weights/plate_detect.pt')
        self.declare_parameter('rec_model_path', 'weights/plate_rec_color.pth')
        self.declare_parameter('is_color', True)
        self.declare_parameter('img_size', 640)
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/plate_detection_result')
        self.declare_parameter('result_image_topic', '/plate_detection_result_image')
        
        # 获取参数
        detect_model_path = self.get_parameter('detect_model_path').value
        rec_model_path = self.get_parameter('rec_model_path').value
        self.is_color = self.get_parameter('is_color').value
        self.img_size = self.get_parameter('img_size').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        result_image_topic = self.get_parameter('result_image_topic').value
        
        # 构建模型路径（相对于原始包）
        detect_model_path = os.path.join(original_package_path, detect_model_path)
        rec_model_path = os.path.join(original_package_path, rec_model_path)
        
        # 检查模型文件是否存在
        if not os.path.exists(detect_model_path):
            self.get_logger().error(f'Detection model not found: {detect_model_path}')
            return
        if not os.path.exists(rec_model_path):
            self.get_logger().error(f'Recognition model not found: {rec_model_path}')
            return
        
        # Initialize models
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f'Using device: {self.device}')
        
        try:
            self.detect_model = load_model(detect_model_path, self.device)
            self.plate_rec_model = init_model(self.device, rec_model_path, is_color=self.is_color)
            self.get_logger().info('Models loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Error loading models: {str(e)}')
            return
        
        self.bridge = CvBridge()
        
        # Create publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        self.result_pub = self.create_publisher(
            String,
            output_topic,
            10
        )
        
        self.result_image_pub = self.create_publisher(
            Image,
            result_image_topic,
            10
        )
        
        self.get_logger().info('Simple plate detection node initialized')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing results to: {output_topic}')
        self.get_logger().info(f'Publishing result images to: {result_image_topic}')
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image
            dict_list = detect_Recognition_plate(
                self.detect_model, 
                cv_image, 
                self.device, 
                self.plate_rec_model, 
                self.img_size, 
                is_color=self.is_color
            )
            
            # Draw results on image
            result_image = draw_result(cv_image, dict_list, self.is_color)
            
            # Publish result image
            result_image_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            result_image_msg.header = msg.header
            self.result_image_pub.publish(result_image_msg)
            
            # Publish results
            if dict_list:
                result_str = ""
                for result in dict_list:
                    if result['plate_no']:
                        result_str += f"{result['plate_no']} "
                
                if result_str.strip():
                    result_msg = String()
                    result_msg.data = result_str.strip()
                    self.result_pub.publish(result_msg)
                    self.get_logger().info(f'Detected plates: {result_str.strip()}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = SimplePlateDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 