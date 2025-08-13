#!/usr/bin/env python3
"""
门牌检测模型训练脚本
"""

import os
import yaml
import shutil
from pathlib import Path

def create_dataset_config():
    """
    创建数据集配置文件
    """
    config = {
        'path': './output',  # 数据集根目录
        'train': 'images',   # 训练图像路径
        'val': 'images',     # 验证图像路径
        'nc': 1,             # 类别数量
        'names': ['doorplate']  # 类别名称
    }
    
    # 保存配置文件
    config_path = 'data/doorplate.yaml'
    os.makedirs('data', exist_ok=True)
    
    with open(config_path, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
    
    print(f"✅ 数据集配置文件已创建: {config_path}")
    return config_path

def prepare_training_data():
    """
    准备训练数据
    """
    print("=== 准备训练数据 ===")
    
    # 检查数据集是否存在
    if not os.path.exists('output/images') or not os.path.exists('output/labels'):
        print("❌ 数据集不存在，请先生成数据集")
        return False
    
    # 统计数据集信息
    image_files = [f for f in os.listdir('output/images') if f.endswith('.jpg')]
    label_files = [f for f in os.listdir('output/labels') if f.endswith('.txt')]
    
    print(f"📊 图像文件数量: {len(image_files)}")
    print(f"📊 标签文件数量: {len(label_files)}")
    
    if len(image_files) != len(label_files):
        print("❌ 图像和标签文件数量不匹配")
        return False
    
    print("✅ 数据集准备完成")
    return True

def create_training_script():
    """
    创建训练脚本
    """
    script_content = '''#!/usr/bin/env python3
"""
门牌检测模型训练脚本
"""

import os
import sys
import subprocess
from pathlib import Path

def main():
    """
    主训练函数
    """
    print("=== 门牌检测模型训练 ===")
    
    # 检查YOLOv5是否可用
    try:
        import torch
        print(f"✅ PyTorch版本: {torch.__version__}")
    except ImportError:
        print("❌ PyTorch未安装，请先安装PyTorch")
        return False
    
    # 检查数据集配置文件
    config_path = 'data/doorplate.yaml'
    if not os.path.exists(config_path):
        print(f"❌ 数据集配置文件不存在: {config_path}")
        return False
    
    # 检查预训练权重
    weights_path = 'weights/yolov5s.pt'
    if not os.path.exists(weights_path):
        print(f"⚠️  预训练权重不存在: {weights_path}")
        print("将从头开始训练...")
        weights_path = 'yolov5s.pt'
    
    # 训练参数
    epochs = 100
    batch_size = 16
    img_size = 640
    
    print(f"\\n训练参数:")
    print(f"- 训练轮数: {epochs}")
    print(f"- 批次大小: {batch_size}")
    print(f"- 图像尺寸: {img_size}")
    print(f"- 数据集配置: {config_path}")
    print(f"- 预训练权重: {weights_path}")
    
    # 开始训练
    print("\\n🚀 开始训练...")
    
    # 构建训练命令
    cmd = [
        'python', 'train.py',
        '--data', config_path,
        '--weights', weights_path,
        '--epochs', str(epochs),
        '--batch-size', str(batch_size),
        '--img-size', str(img_size),
        '--project', 'runs/train',
        '--name', 'doorplate_detection',
        '--exist-ok'
    ]
    
    try:
        # 执行训练命令
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print("✅ 训练完成！")
        print("\\n训练结果保存在: runs/train/doorplate_detection/")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ 训练失败: {e}")
        print(f"错误输出: {e.stderr}")
        return False

if __name__ == "__main__":
    main()
'''
    
    with open('train_doorplate.py', 'w', encoding='utf-8') as f:
        f.write(script_content)
    
    print("✅ 训练脚本已创建: train_doorplate.py")

def create_inference_script():
    """
    创建推理脚本
    """
    script_content = '''#!/usr/bin/env python3
"""
门牌检测推理脚本
"""

import os
import cv2
import torch
import numpy as np
from pathlib import Path

def load_model(weights_path):
    """
    加载训练好的模型
    """
    try:
        # 加载YOLOv5模型
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path)
        model.eval()
        print(f"✅ 模型加载成功: {weights_path}")
        return model
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return None

def detect_doorplate(model, image_path, conf_threshold=0.5):
    """
    检测门牌
    """
    # 读取图像
    image = cv2.imread(image_path)
    if image is None:
        print(f"❌ 无法读取图像: {image_path}")
        return None
    
    # 进行检测
    results = model(image)
    
    # 处理结果
    detections = []
    for det in results.xyxy[0]:
        x1, y1, x2, y2, conf, cls = det.cpu().numpy()
        if conf > conf_threshold:
            detections.append({
                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                'confidence': float(conf),
                'class': int(cls)
            })
    
    return image, detections

def draw_detections(image, detections):
    """
    在图像上绘制检测结果
    """
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        conf = det['confidence']
        
        # 绘制边界框
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # 绘制标签
        label = f"Doorplate: {conf:.2f}"
        cv2.putText(image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return image

def main():
    """
    主函数
    """
    print("=== 门牌检测推理 ===")
    
    # 检查模型文件
    weights_path = 'runs/train/doorplate_detection/weights/best.pt'
    if not os.path.exists(weights_path):
        print(f"❌ 模型文件不存在: {weights_path}")
        print("请先完成训练")
        return
    
    # 加载模型
    model = load_model(weights_path)
    if model is None:
        return
    
    # 测试图像路径
    test_image = 'output/images/doorplate_0000.jpg'
    if not os.path.exists(test_image):
        print(f"❌ 测试图像不存在: {test_image}")
        return
    
    # 进行检测
    print(f"\\n🔍 检测图像: {test_image}")
    image, detections = detect_doorplate(model, test_image)
    
    if detections:
        print(f"✅ 检测到 {len(detections)} 个门牌")
        for i, det in enumerate(detections):
            print(f"  门牌 {i+1}: 置信度 {det['confidence']:.3f}")
        
        # 绘制检测结果
        result_image = draw_detections(image.copy(), detections)
        
        # 保存结果
        output_path = 'doorplate_detection_result.jpg'
        cv2.imwrite(output_path, result_image)
        print(f"\\n📸 检测结果已保存: {output_path}")
    else:
        print("❌ 未检测到门牌")

if __name__ == "__main__":
    main()
'''
    
    with open('inference_doorplate.py', 'w', encoding='utf-8') as f:
        f.write(script_content)
    
    print("✅ 推理脚本已创建: inference_doorplate.py")

def main():
    """
    主函数
    """
    print("=== 门牌检测训练准备 ===")
    
    # 1. 准备训练数据
    if not prepare_training_data():
        return
    
    # 2. 创建数据集配置
    config_path = create_dataset_config()
    
    # 3. 创建训练脚本
    create_training_script()
    
    # 4. 创建推理脚本
    create_inference_script()
    
    print("\\n=== 训练准备完成 ===")
    print("\\n📋 下一步操作:")
    print("1. 确保已安装YOLOv5: pip install ultralytics")
    print("2. 运行训练: python train_doorplate.py")
    print("3. 训练完成后运行推理: python inference_doorplate.py")
    print("\\n📁 生成的文件:")
    print("- data/doorplate.yaml: 数据集配置文件")
    print("- train_doorplate.py: 训练脚本")
    print("- inference_doorplate.py: 推理脚本")

if __name__ == "__main__":
    main() 