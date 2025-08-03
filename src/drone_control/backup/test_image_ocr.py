#!/usr/bin/env python3
"""
PNG图片OCR测试节点
读取PNG图片进行OCR识别，并将结果传送给无人机控制节点
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
import threading

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

class TestImageOCRNode(Node):
    def __init__(self):
        super().__init__('test_image_ocr_node')
        self.get_logger().info("PNG图片OCR测试节点启动中...")
        
        # 初始化PaddleOCR
        if PADDLEOCR_AVAILABLE:
            try:
                # 使用中文模型，支持中英文识别
                self.ocr = PaddleOCR(use_angle_cls=True, lang='ch')
                self.get_logger().info("PaddleOCR初始化成功")
            except Exception as e:
                self.get_logger().error(f"PaddleOCR初始化失败: {e}")
                self.ocr = None
        else:
            self.ocr = None
            self.get_logger().error("PaddleOCR不可用，请安装相关依赖")
        
        # OCR结果发布
        self.ocr_result_pub = self.create_publisher(
            String, 'ocr/license_plate_result', 10)
        
        # 处理参数
        self.result_threshold = 0.8  # 置信度阈值
        self.min_text_length = 2     # 最小文字长度
        self.image_folder = "src/drone_control/test_images"  # 修改为正确的图片文件夹路径
        self.test_interval = 5.0     # 测试间隔（秒）
        
        # 创建处理线程
        self.processing_thread = None
        self.stop_thread = False
        
        self.get_logger().info("PNG图片OCR测试节点初始化完成")
        
        # 启动图片处理线程
        self._start_image_processing()

    def _start_image_processing(self):
        """启动图片处理线程"""
        def run_processing():
            while not self.stop_thread and rclpy.ok():
                try:
                    self._process_test_images()
                    time.sleep(self.test_interval)
                except Exception as e:
                    self.get_logger().error(f"图片处理线程错误: {e}")
                    time.sleep(1)
            
        self.processing_thread = threading.Thread(
            target=run_processing, daemon=True, name="ImageProcessingThread")
        self.processing_thread.start()

    def _process_test_images(self):
        """处理测试图片"""
        try:
            # 检查图片文件夹是否存在
            if not os.path.exists(self.image_folder):
                self.get_logger().warn(f"图片文件夹不存在: {self.image_folder}")
                self._create_test_images()
                return
            
            # 获取所有PNG图片（适配用户的命名方式：number0.png, number1.png, number2.png）
            image_files = [f for f in os.listdir(self.image_folder) 
                          if f.lower().endswith('.png') and f.startswith('number')]
            
            if not image_files:
                self.get_logger().warn(f"在 {self.image_folder} 中没有找到number*.png格式的图片")
                self._create_test_images()
                return
            
            # 按数字顺序排序
            image_files.sort(key=lambda x: int(x.replace('number', '').replace('.png', '')))
            
            self.get_logger().info(f"找到 {len(image_files)} 张测试图片: {image_files}")
            
            # 处理每张图片
            for image_file in image_files:
                image_path = os.path.join(self.image_folder, image_file)
                self._process_single_image(image_path)
                time.sleep(1)  # 图片间间隔
                
        except Exception as e:
            self.get_logger().error(f"处理测试图片失败: {e}")

    def _create_test_images(self):
        """创建测试图片（如果用户文件夹不存在）"""
        try:
            os.makedirs(self.image_folder, exist_ok=True)
            
            # 创建一些测试图片（包含号牌文字）
            test_texts = ["京A12345", "沪B67890", "粤C11111", "川D22222"]
            
            for i, text in enumerate(test_texts):
                # 创建简单的文字图片
                img = np.ones((200, 400, 3), dtype=np.uint8) * 255
                cv2.putText(img, text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 
                           2, (0, 0, 0), 3)
                
                image_path = os.path.join(self.image_folder, f"number{i}.png")
                cv2.imwrite(image_path, img)
                self.get_logger().info(f"创建测试图片: {image_path}")
                
        except Exception as e:
            self.get_logger().error(f"创建测试图片失败: {e}")

    def _process_single_image(self, image_path):
        """处理单张图片"""
        try:
            self.get_logger().info(f"处理图片: {image_path}")
            
            # 读取图片
            image = cv2.imread(image_path)
            if image is None:
                self.get_logger().error(f"无法读取图片: {image_path}")
                return
            
            self.get_logger().info(f"图片尺寸: {image.shape}")
            
            # 图像预处理
            processed_image = self._preprocess_image(image)
            
            # OCR识别
            if self.ocr:
                try:
                    self.get_logger().info("开始OCR识别...")
                    results = self.ocr.ocr(processed_image)
                    self.get_logger().info(f"OCR识别完成，结果类型: {type(results)}")
                    
                    # 处理识别结果
                    recognized_text = self._extract_text_from_results(results)
                    
                    if recognized_text:
                        # 发布识别结果
                        self._publish_license_plate_result(recognized_text, image_path)
                    else:
                        self.get_logger().warn(f"图片 {image_path} 未识别到文字")
                        
                except Exception as ocr_error:
                    self.get_logger().error(f"OCR识别失败: {ocr_error}")
                    import traceback
                    self.get_logger().error(f"详细错误: {traceback.format_exc()}")
                    
        except Exception as e:
            self.get_logger().error(f"处理图片 {image_path} 失败: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    def _preprocess_image(self, image):
        """图像预处理 - 针对号牌识别优化"""
        try:
            # 转换为灰度图
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 高斯模糊去噪
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # 自适应阈值处理
            thresh = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                cv2.THRESH_BINARY, 11, 2)
            
            # 形态学操作 - 针对号牌文字优化
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            processed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            
            # 转换回彩色图像（PaddleOCR需要3通道）
            processed_color = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)
            
            return processed_color
            
        except Exception as e:
            self.get_logger().error(f"图像预处理错误: {e}")
            return image

    def _extract_text_from_results(self, results):
        """从OCR结果中提取号牌文字，兼容新版PaddleOCR字典格式"""
        try:
            if not results:
                return ""
            self.get_logger().info(f"OCR结果类型: {type(results)}")
            self.get_logger().info(f"OCR结果内容: {results}")
            extracted_texts = []
            
            # 新版PaddleOCR返回list[dict]，每个dict包含'rec_texts'和'rec_scores'
            if isinstance(results, list) and len(results) > 0 and isinstance(results[0], dict):
                for page in results:
                    # 直接从字典中获取rec_texts和rec_scores
                    rec_texts = page.get('rec_texts', [])
                    rec_scores = page.get('rec_scores', [])
                    
                    self.get_logger().info(f"找到 {len(rec_texts)} 个文字: {rec_texts}")
                    self.get_logger().info(f"对应置信度: {rec_scores}")
                    
                    for text, score in zip(rec_texts, rec_scores):
                        if score > self.result_threshold and len(text) >= self.min_text_length:
                            if self._is_license_plate_text(text):
                                extracted_texts.append(text)
                                self.get_logger().info(f"新版识别到文字: {text}, 置信度: {score}")
            else:
                # 兼容旧版格式
                for line in results:
                    if isinstance(line, list) and len(line) >= 2:
                        text = line[1][0]
                        confidence = line[1][1]
                        if (confidence > self.result_threshold and len(text) >= self.min_text_length):
                            if self._is_license_plate_text(text):
                                extracted_texts.append(text)
                                self.get_logger().info(f"旧版识别到文字: {text}, 置信度: {confidence}")
            
            if extracted_texts:
                result_text = " ".join(extracted_texts)
                self.get_logger().info(f"最终提取的文字: {result_text}")
                return result_text
            else:
                self.get_logger().warn("未提取到任何符合条件的文字")
                return ""
        except Exception as e:
            self.get_logger().error(f"提取文字时出错: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")
            return ""

    def _is_license_plate_text(self, text):
        """判断是否为号牌文字"""
        # 这里可以添加号牌文字的特征判断
        # 例如：包含数字、字母、中文字符等
        # 目前简单返回True，后续可以根据实际需求优化
        return True

    def _publish_license_plate_result(self, text, image_path):
        """发布号牌识别结果"""
        try:
            result_msg = String()
            result_msg.data = text
            
            # 发布到话题
            self.ocr_result_pub.publish(result_msg)
            
            # 打印发布信息
            self.get_logger().info(f"[PNG测试发布] 图片 {image_path} 识别到号牌文字: {text}")
            
        except Exception as e:
            self.get_logger().error(f"发布号牌识别结果失败: {e}")

    def _cleanup(self):
        """清理资源"""
        self.stop_thread = True
        if self.processing_thread and self.processing_thread.is_alive():
            try:
                self.processing_thread.join(timeout=2.0)
                if self.processing_thread.is_alive():
                    self.get_logger().warn("图片处理线程未能正常停止")
            except Exception as e:
                self.get_logger().error(f"停止图片处理线程时出错: {e}")
        
        # 清理OCR资源
        if hasattr(self, 'ocr') and self.ocr is not None:
            try:
                # PaddleOCR没有显式的清理方法，但可以删除引用
                del self.ocr
                self.ocr = None
            except Exception as e:
                self.get_logger().error(f"清理OCR资源时出错: {e}")

    def destroy_node(self):
        """安全关闭节点"""
        self.get_logger().info("关闭PNG图片OCR测试节点中...")
        self._cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TestImageOCRNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 