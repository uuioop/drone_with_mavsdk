#!/bin/bash

# 门牌检测模型训练快速开始脚本

echo "=== 门牌检测模型训练快速开始 ==="

# 检查数据集是否存在
if [ ! -d "output/images" ] || [ ! -d "output/labels" ]; then
    echo "❌ 数据集不存在，正在生成数据集..."
    python3 generate_plates.py
fi

# 检查数据集配置文件
if [ ! -f "data/doorplate.yaml" ]; then
    echo "📝 创建数据集配置文件..."
    python3 setup_training.py
fi

# 检查YOLOv5是否已安装
if [ ! -d "yolov5" ]; then
    echo "📥 下载YOLOv5..."
    git clone https://github.com/ultralytics/yolov5.git
    cd yolov5
    pip install -r requirements.txt
    cd ..
else
    echo "✅ YOLOv5已存在"
fi

# 复制数据集到YOLOv5目录
echo "📁 准备训练数据..."
cp -r output yolov5/
cp data/doorplate.yaml yolov5/data/

# 进入YOLOv5目录
cd yolov5

echo "🚀 开始训练门牌检测模型..."
echo "训练参数："
echo "- 数据集：doorplate.yaml"
echo "- 预训练权重：yolov5s.pt"
echo "- 训练轮数：100"
echo "- 批次大小：16"
echo "- 图像尺寸：640"
echo ""

# 开始训练
python train.py \
    --data data/doorplate.yaml \
    --weights yolov5s.pt \
    --epochs 100 \
    --batch-size 16 \
    --img-size 640 \
    --project runs/train \
    --name doorplate_detection \
    --exist-ok

echo ""
echo "=== 训练完成 ==="
echo "模型保存在：runs/train/doorplate_detection/weights/best.pt"
echo ""
echo "测试模型："
echo "python detect.py --weights runs/train/doorplate_detection/weights/best.pt --source ../output/images/doorplate_0000.jpg" 