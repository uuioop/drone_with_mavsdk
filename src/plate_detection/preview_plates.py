#!/usr/bin/env python3
"""
预览生成的门牌图像样式
"""

import os
import glob
from PIL import Image

def preview_plates(num_samples=5):
    """
    预览生成的门牌图像
    """
    images_dir = "output/images"
    
    if not os.path.exists(images_dir):
        print("❌ 图像目录不存在，请先生成数据集")
        return
    
    # 获取图像文件列表
    image_files = glob.glob(os.path.join(images_dir, "*.jpg"))
    
    if not image_files:
        print("❌ 没有找到图像文件")
        return
    
    print(f"=== 门牌图像预览 (显示前{num_samples}张) ===")
    
    # 显示前几张图像的信息
    for i, img_path in enumerate(image_files[:num_samples]):
        try:
            with Image.open(img_path) as img:
                width, height = img.size
                file_size = os.path.getsize(img_path) / 1024  # KB
                
                print(f"\n📷 {os.path.basename(img_path)}:")
                print(f"   - 尺寸: {width}x{height}")
                print(f"   - 文件大小: {file_size:.1f}KB")
                
                # 显示图像的基本信息
                print(f"   - 模式: {img.mode}")
                
        except Exception as e:
            print(f"❌ 无法打开图像 {img_path}: {e}")
    
    print(f"\n✅ 总共找到 {len(image_files)} 张门牌图像")
    print(f"📁 图像目录: {images_dir}")

if __name__ == "__main__":
    preview_plates() 