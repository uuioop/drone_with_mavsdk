#!/usr/bin/env python3
"""
门牌检测训练设置脚本
"""

import os
import yaml
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
    
    # 创建data目录
    os.makedirs('data', exist_ok=True)
    
    # 保存配置文件
    config_path = 'data/doorplate.yaml'
    with open(config_path, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
    
    print(f"✅ 数据集配置文件已创建: {config_path}")
    return config_path

def check_requirements():
    """
    检查训练要求
    """
    print("=== 检查训练要求 ===")
    
    # 检查数据集
    if not os.path.exists('output/images') or not os.path.exists('output/labels'):
        print("❌ 数据集不存在，请先生成数据集")
        return False
    
    # 统计数据集
    image_files = [f for f in os.listdir('output/images') if f.endswith('.jpg')]
    label_files = [f for f in os.listdir('output/labels') if f.endswith('.txt')]
    
    print(f"📊 图像文件数量: {len(image_files)}")
    print(f"📊 标签文件数量: {len(label_files)}")
    
    if len(image_files) != len(label_files):
        print("❌ 图像和标签文件数量不匹配")
        return False
    
    print("✅ 数据集检查通过")
    return True

def create_training_commands():
    """
    创建训练命令说明
    """
    print("\n=== 训练命令 ===")
    print("请按照以下步骤进行训练：")
    print()
    print("1. 安装YOLOv5依赖：")
    print("   pip install ultralytics")
    print()
    print("2. 下载YOLOv5代码：")
    print("   git clone https://github.com/ultralytics/yolov5.git")
    print("   cd yolov5")
    print()
    print("3. 复制数据集到YOLOv5目录：")
    print("   cp -r ../plate_detection/output ./")
    print("   cp ../plate_detection/data/doorplate.yaml ./data/")
    print()
    print("4. 开始训练：")
    print("   python train.py --data data/doorplate.yaml --weights yolov5s.pt --epochs 100 --batch-size 16 --img-size 640")
    print()
    print("5. 训练完成后，模型将保存在：")
    print("   runs/train/exp/weights/best.pt")
    print()
    print("6. 测试模型：")
    print("   python detect.py --weights runs/train/exp/weights/best.pt --source ../plate_detection/output/images/doorplate_0000.jpg")

def main():
    """
    主函数
    """
    print("=== 门牌检测训练设置 ===")
    
    # 检查要求
    if not check_requirements():
        return
    
    # 创建数据集配置
    config_path = create_dataset_config()
    
    # 创建训练命令说明
    create_training_commands()
    
    print("\n=== 设置完成 ===")
    print("现在您可以按照上述步骤开始训练门牌检测模型了！")

if __name__ == "__main__":
    main() 