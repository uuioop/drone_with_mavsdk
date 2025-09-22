#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import cv2
import numpy as np
import time
from pathlib import Path
import sys

# 添加当前包的路径到sys.path
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

try:
    import onnxruntime as ort
except ImportError:
    print("错误: 请安装 onnxruntime")
    print("pip install onnxruntime")
    sys.exit(1)

class ONNXPlateDetector:
    """使用ONNX模型的车牌检测器"""
    
    def __init__(self, detect_model_path, rec_model_path, device='cpu'):
        """
        初始化ONNX车牌检测器
        
        Args:
            detect_model_path: 检测模型路径
            rec_model_path: 识别模型路径
            device: 设备类型 ('cpu' 或 'gpu')
        """
        self.device = device
        
        # 加载检测模型
        print(f"加载检测模型: {detect_model_path}")
        self.detect_session = ort.InferenceSession(
            detect_model_path,
            providers=['CPUExecutionProvider'] if device == 'cpu' else ['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        
        # 加载识别模型
        print(f"加载识别模型: {rec_model_path}")
        self.rec_session = ort.InferenceSession(
            rec_model_path,
            providers=['CPUExecutionProvider'] if device == 'cpu' else ['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        
        # 车牌字符集
        self.plate_chars = r"#京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危0123456789ABCDEFGHJKLMNPQRSTUVWXYZ险品"
        self.color_names = ['黑色', '蓝色', '绿色', '白色', '黄色']
        
        # 图像预处理参数
        self.mean_value, self.std_value = (0.588, 0.193)
        
        print("ONNX模型加载完成")
    
    def preprocess_detection(self, img, img_size=640):
        """
        预处理检测图像
        
        Args:
            img: 输入图像
            img_size: 目标尺寸
            
        Returns:
            预处理后的图像
        """
        # 调整图像大小
        img_resized = cv2.resize(img, (img_size, img_size))
        
        # 转换为RGB
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        
        # 归一化
        img_normalized = img_rgb.astype(np.float32) / 255.0
        
        # 调整维度 (H, W, C) -> (C, H, W)
        img_transposed = np.transpose(img_normalized, (2, 0, 1))
        
        # 添加batch维度
        img_batch = np.expand_dims(img_transposed, axis=0)
        
        return img_batch, img_resized
    
    def preprocess_recognition(self, img):
        """
        预处理识别图像
        
        Args:
            img: 车牌图像
            
        Returns:
            预处理后的图像
        """
        # 调整到标准尺寸
        img_resized = cv2.resize(img, (168, 48))
        
        # 归一化
        img_normalized = (img_resized.astype(np.float32) / 255.0 - self.mean_value) / self.std_value
        
        # 调整维度 (H, W, C) -> (C, H, W)
        img_transposed = np.transpose(img_normalized, (2, 0, 1))
        
        # 添加batch维度
        img_batch = np.expand_dims(img_transposed, axis=0)
        
        return img_batch
    
    def decode_plate(self, preds):
        """
        解码车牌预测结果
        
        Args:
            preds: 预测结果
            
        Returns:
            解码后的车牌号和索引
        """
        pre = 0
        new_preds = []
        index = []
        
        for i in range(len(preds)):
            if preds[i] != 0 and preds[i] != pre:
                new_preds.append(preds[i])
                index.append(i)
            pre = preds[i]
        
        return new_preds, index
    
    def detect_plates(self, img, conf_threshold=0.3, iou_threshold=0.5):
        """
        检测车牌
        
        Args:
            img: 输入图像
            conf_threshold: 置信度阈值
            iou_threshold: IoU阈值
            
        Returns:
            检测结果列表
        """
        # 预处理
        input_tensor, img_resized = self.preprocess_detection(img)
        
        # 运行检测推理
        outputs = self.detect_session.run(None, {'input': input_tensor})
        
        # 处理输出 (这里需要根据实际的YOLO输出格式调整)
        # 假设输出是 [batch, num_detections, 85] 格式
        detections = outputs[0]
        
        # 过滤低置信度的检测
        valid_detections = []
        for detection in detections:
            confidence = detection[4]  # 假设第5个元素是置信度
            if confidence > conf_threshold:
                valid_detections.append(detection)
        
        return valid_detections, img_resized
    
    def recognize_plate(self, plate_img):
        """
        识别车牌文字
        
        Args:
            plate_img: 车牌图像
            
        Returns:
            识别结果字典
        """
        # 预处理
        input_tensor = self.preprocess_recognition(plate_img)
        
        # 运行识别推理
        outputs = self.rec_session.run(None, {'input': input_tensor})
        
        # 处理输出
        if len(outputs) == 2:
            # 包含颜色识别
            plate_preds, color_preds = outputs
        else:
            # 只有车牌识别
            plate_preds = outputs[0]
            color_preds = None
        
        # 解码车牌号
        plate_probs = self.softmax(plate_preds[0])
        plate_indices = np.argmax(plate_probs, axis=1)
        
        decoded_plate, plate_indices = self.decode_plate(plate_indices)
        plate_number = ''.join([self.plate_chars[i] for i in decoded_plate])
        
        # 计算置信度
        plate_conf = np.mean([plate_probs[i, idx] for i, idx in enumerate(plate_indices)])
        
        result = {
            'plate_number': plate_number,
            'confidence': plate_conf,
            'plate_type': 'unknown'
        }
        
        # 处理颜色识别
        if color_preds is not None:
            color_probs = self.softmax(color_preds[0])
            color_idx = np.argmax(color_probs)
            color_conf = color_probs[color_idx]
            
            result['plate_color'] = self.color_names[color_idx]
            result['color_confidence'] = color_conf
        
        return result
    
    def softmax(self, x):
        """计算softmax"""
        exp_x = np.exp(x - np.max(x, axis=-1, keepdims=True))
        return exp_x / np.sum(exp_x, axis=-1, keepdims=True)
    
    def process_image(self, img):
        """
        处理完整图像
        
        Args:
            img: 输入图像
            
        Returns:
            处理结果列表
        """
        results = []
        
        # 检测车牌
        detections, img_resized = self.detect_plates(img)
        
        # 处理每个检测结果
        for detection in detections:
            # 提取边界框坐标 (这里需要根据实际输出格式调整)
            x1, y1, x2, y2 = detection[:4]
            
            # 提取车牌区域
            plate_roi = img[int(y1):int(y2), int(x1):int(x2)]
            
            if plate_roi.size == 0:
                continue
            
            # 识别车牌
            recognition_result = self.recognize_plate(plate_roi)
            
            # 组合结果
            result = {
                'bbox': [x1, y1, x2, y2],
                'detection_confidence': detection[4],
                **recognition_result
            }
            
            results.append(result)
        
        return results

def test_onnx_inference():
    """测试ONNX推理"""
    
    # 模型路径
    detect_model_path = current_dir / "onnx_models" / "plate_detect_optimized.onnx"
    rec_model_path = current_dir / "onnx_models" / "plate_rec_color_optimized.onnx"
    
    # 检查模型文件
    if not detect_model_path.exists():
        print(f"错误: 检测模型不存在: {detect_model_path}")
        print("请先运行 export_to_onnx.py 导出模型")
        return
    
    if not rec_model_path.exists():
        print(f"错误: 识别模型不存在: {rec_model_path}")
        print("请先运行 export_to_onnx.py 导出模型")
        return
    
    # 创建检测器
    detector = ONNXPlateDetector(
        str(detect_model_path),
        str(rec_model_path),
        device='cpu'
    )
    
    # 创建测试图像
    test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # 测试推理
    print("开始测试推理...")
    start_time = time.time()
    
    results = detector.process_image(test_img)
    
    end_time = time.time()
    print(f"推理耗时: {(end_time - start_time) * 1000:.2f} ms")
    print(f"检测结果数量: {len(results)}")
    
    # 打印结果
    for i, result in enumerate(results):
        print(f"结果 {i+1}:")
        print(f"  车牌号: {result.get('plate_number', 'N/A')}")
        print(f"  置信度: {result.get('confidence', 0):.3f}")
        print(f"  车牌颜色: {result.get('plate_color', 'N/A')}")
        print(f"  边界框: {result.get('bbox', [])}")

if __name__ == "__main__":
    test_onnx_inference() 