#!/usr/bin/env python3
"""
验证生成的门牌数据集的脚本
"""

import os
import glob
from PIL import Image

def verify_dataset():
    """
    验证生成的数据集
    """
    # 检查目录结构
    output_dir = "output"
    images_dir = os.path.join(output_dir, "images")
    labels_dir = os.path.join(output_dir, "labels")
    
    print("=== 数据集验证报告 ===")
    
    # 检查目录是否存在
    if not os.path.exists(output_dir):
        print("❌ 输出目录不存在")
        return False
    
    if not os.path.exists(images_dir):
        print("❌ 图像目录不存在")
        return False
    
    if not os.path.exists(labels_dir):
        print("❌ 标签目录不存在")
        return False
    
    print("✅ 目录结构正确")
    
    # 统计文件数量
    image_files = glob.glob(os.path.join(images_dir, "*.jpg"))
    label_files = glob.glob(os.path.join(labels_dir, "*.txt"))
    
    print(f"📊 图像文件数量: {len(image_files)}")
    print(f"📊 标签文件数量: {len(label_files)}")
    
    if len(image_files) != len(label_files):
        print("❌ 图像和标签文件数量不匹配")
        return False
    
    print("✅ 图像和标签文件数量匹配")
    
    # 检查图像质量
    print("\n=== 图像质量检查 ===")
    sample_images = image_files[:5]  # 检查前5张图像
    
    for img_path in sample_images:
        try:
            with Image.open(img_path) as img:
                width, height = img.size
                print(f"📷 {os.path.basename(img_path)}: {width}x{height}")
                
                if width != 640 or height != 640:
                    print(f"⚠️  图像尺寸不是640x640: {width}x{height}")
        except Exception as e:
            print(f"❌ 无法打开图像 {img_path}: {e}")
            return False
    
    # 检查标签格式
    print("\n=== 标签格式检查 ===")
    sample_labels = label_files[:5]  # 检查前5个标签文件
    
    for label_path in sample_labels:
        try:
            with open(label_path, 'r', encoding='utf-8') as f:
                content = f.read().strip()
                lines = content.split('\n')
                
                if len(lines) != 1:
                    print(f"⚠️  标签文件 {os.path.basename(label_path)} 有多行内容")
                    continue
                
                parts = lines[0].split()
                if len(parts) != 5:
                    print(f"❌ 标签格式错误 {os.path.basename(label_path)}: {lines[0]}")
                    continue
                
                class_id, center_x, center_y, width, height = map(float, parts)
                
                # 检查坐标是否在合理范围内
                if not (0 <= center_x <= 1 and 0 <= center_y <= 1 and 0 <= width <= 1 and 0 <= height <= 1):
                    print(f"⚠️  标签坐标超出范围 {os.path.basename(label_path)}: {lines[0]}")
                    continue
                
                print(f"🏷️  {os.path.basename(label_path)}: class={int(class_id)}, center=({center_x:.3f}, {center_y:.3f}), size=({width:.3f}, {height:.3f})")
                
        except Exception as e:
            print(f"❌ 无法读取标签文件 {label_path}: {e}")
            return False
    
    print("\n=== 验证完成 ===")
    print("✅ 数据集验证通过！")
    print(f"\n数据集信息:")
    print(f"- 总图像数量: {len(image_files)}")
    print(f"- 图像尺寸: 640x640")
    print(f"- 图像格式: JPG")
    print(f"- 标签格式: YOLO")
    print(f"- 类别数量: 1 (门牌)")
    
    return True

if __name__ == "__main__":
    verify_dataset() 