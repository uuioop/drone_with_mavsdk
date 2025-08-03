#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
import os

# PaddleOCR导入
try:
    import sys
    print(f"Python路径: {sys.path}")
    from paddleocr import PaddleOCR
    PADDLEOCR_AVAILABLE = True
    print("PaddleOCR导入成功")
except ImportError as e:
    PADDLEOCR_AVAILABLE = False
    print(f"警告: PaddleOCR导入失败: {e}")
    print("请运行: pip install paddlepaddle paddleocr")
except Exception as e:
    PADDLEOCR_AVAILABLE = False
    print(f"警告: PaddleOCR初始化异常: {e}")
    
class PaddleOCRNode(Node):
    def __init__(self):
        super().__init__('paddle_ocr_node')
        
        # 初始化 PaddleOCR
        self.ocr = PaddleOCR(
            use_doc_orientation_classify=False,
            use_doc_unwarping=False,
            use_textline_orientation=False)
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            'image_raw',  # 替换为你的图像话题名称
            self.image_callback,
            10)
        
        # 创建输出文件夹
        self.output_dir = "ocr_results"
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info("PaddleOCR节点已启动，等待图像输入...")

    def image_callback(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 执行OCR识别
            result = self.ocr.predict(cv_image)
            
            # 处理结果
            for idx, res in enumerate(result):
                # 打印结果到终端
                self.get_logger().info(f"识别结果: {res.print()}")
                
                # 保存结果到文件
                timestamp = self.get_clock().now().to_msg().sec
                output_prefix = os.path.join(self.output_dir, f"result_{timestamp}_{idx}")
                
                # 保存可视化结果
                res.save_to_img(output_prefix)
                
                # 保存JSON结果
                res.save_to_json(output_prefix)
                
                # 可选: 发布识别结果到新的ROS话题
                # self.publish_ocr_result(res)
                
        except Exception as e:
            self.get_logger().error(f"处理图像时出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PaddleOCRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
