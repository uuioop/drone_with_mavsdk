#!/usr/bin/env python3
"""
可视化生成的门牌图像和边界框
"""

import os
import glob
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import numpy as np

def visualize_plate_with_bbox(image_path, label_path, save_path=None):
    """
    可视化门牌图像和边界框
    """
    # 读取图像
    img = Image.open(image_path)
    img_array = np.array(img)
    
    # 读取标签
    with open(label_path, 'r') as f:
        line = f.read().strip()
        parts = line.split()
        class_id, center_x, center_y, width, height = map(float, parts)
    
    # 转换为像素坐标
    img_width, img_height = img.size
    x1 = (center_x - width/2) * img_width
    y1 = (center_y - height/2) * img_height
    x2 = (center_x + width/2) * img_width
    y2 = (center_y + height/2) * img_height
    
    # 创建图形
    fig, ax = plt.subplots(1, figsize=(10, 10))
    ax.imshow(img_array)
    
    # 绘制边界框
    rect = patches.Rectangle((x1, y1), x2-x1, y2-y1, 
                           linewidth=2, edgecolor='red', facecolor='none')
    ax.add_patch(rect)
    
    # 添加标题
    ax.set_title(f'门牌图像 - {os.path.basename(image_path)}\n'
                f'长宽比: {width/height:.3f} (目标: 1.679)', fontsize=12)
    ax.axis('off')
    
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=150)
        print(f"可视化结果已保存到: {save_path}")
    else:
        plt.show()
    
    plt.close()

def main():
    """
    主函数
    """
    images_dir = "output/images"
    labels_dir = "output/labels"
    
    if not os.path.exists(images_dir) or not os.path.exists(labels_dir):
        print("❌ 数据集目录不存在，请先生成数据集")
        return
    
    # 获取图像文件列表
    image_files = glob.glob(os.path.join(images_dir, "*.jpg"))
    
    if not image_files:
        print("❌ 没有找到图像文件")
        return
    
    print(f"=== 门牌可视化 ===")
    print(f"找到 {len(image_files)} 张图像")
    
    # 可视化前5张图像
    for i, img_path in enumerate(image_files[:5]):
        label_path = img_path.replace('images', 'labels').replace('.jpg', '.txt')
        
        if os.path.exists(label_path):
            print(f"\n📷 可视化第 {i+1} 张图像: {os.path.basename(img_path)}")
            save_path = f"plate_visualization_{i+1}.png"
            visualize_plate_with_bbox(img_path, label_path, save_path)
        else:
            print(f"❌ 找不到对应的标签文件: {label_path}")

if __name__ == "__main__":
    main() 