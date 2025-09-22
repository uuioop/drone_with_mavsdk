# 门牌检测模型训练指南

本指南将帮助您训练门牌检测模型。

## 前置要求

### 1. 环境准备

确保您的系统已安装以下依赖：

```bash
# 安装Python依赖
pip install torch torchvision
pip install ultralytics
pip install opencv-python
pip install matplotlib
```

### 2. 数据集准备

确保您已经生成了门牌数据集：

```bash
# 生成数据集（如果还没有）
python3 generate_plates.py

# 验证数据集
python3 verify_dataset.py
```

## 训练步骤

### 步骤1: 下载YOLOv5

```bash
# 克隆YOLOv5仓库
git clone https://github.com/ultralytics/yolov5.git
cd yolov5

# 安装YOLOv5依赖
pip install -r requirements.txt
```

### 步骤2: 准备数据集

```bash
# 复制数据集到YOLOv5目录
cp -r ../plate_detection/output ./
cp ../plate_detection/data/doorplate.yaml ./data/
```

### 步骤3: 开始训练

```bash
# 使用YOLOv5s预训练模型开始训练
python train.py \
    --data data/doorplate.yaml \
    --weights yolov5s.pt \
    --epochs 100 \
    --batch-size 16 \
    --img-size 640 \
    --project runs/train \
    --name doorplate_detection \
    --exist-ok
```

### 训练参数说明

- `--data`: 数据集配置文件路径
- `--weights`: 预训练权重文件
- `--epochs`: 训练轮数（建议100-200轮）
- `--batch-size`: 批次大小（根据GPU内存调整）
- `--img-size`: 输入图像尺寸
- `--project`: 项目保存目录
- `--name`: 实验名称
- `--exist-ok`: 允许覆盖现有实验

## 训练监控

### 1. 查看训练进度

训练过程中，您可以在以下位置查看进度：

- **实时日志**: 终端输出
- **TensorBoard**: `runs/train/doorplate_detection/`
- **训练图表**: `runs/train/doorplate_detection/results.png`

### 2. 训练指标

训练过程中会显示以下指标：

- **Loss**: 损失函数值（越小越好）
- **mAP**: 平均精度（越大越好）
- **Precision**: 精确率
- **Recall**: 召回率

## 模型评估

### 1. 查看训练结果

```bash
# 查看训练结果图表
ls runs/train/doorplate_detection/
```

### 2. 测试模型

```bash
# 使用训练好的模型进行检测
python detect.py \
    --weights runs/train/doorplate_detection/weights/best.pt \
    --source ../plate_detection/output/images/doorplate_0000.jpg \
    --conf 0.5
```

## 模型优化

### 1. 调整训练参数

如果训练效果不理想，可以尝试以下调整：

```bash
# 增加训练轮数
--epochs 200

# 调整学习率
--lr0 0.01

# 使用更大的模型
--weights yolov5m.pt  # 或 yolov5l.pt, yolov5x.pt

# 调整批次大小
--batch-size 8  # 如果GPU内存不足
```

### 2. 数据增强

可以在训练时启用数据增强：

```bash
# 启用数据增强
python train.py \
    --data data/doorplate.yaml \
    --weights yolov5s.pt \
    --epochs 100 \
    --batch-size 16 \
    --img-size 640 \
    --augment  # 启用数据增强
```

## 模型部署

### 1. 导出模型

```bash
# 导出为ONNX格式
python export.py \
    --weights runs/train/doorplate_detection/weights/best.pt \
    --include onnx
```

### 2. 集成到ROS2

训练好的模型可以集成到ROS2节点中：

```python
# 在plate_detection_node.py中使用
model = torch.hub.load('ultralytics/yolov5', 'custom', 
                      path='runs/train/doorplate_detection/weights/best.pt')
```

## 故障排除

### 常见问题

1. **CUDA内存不足**
   ```bash
   # 减少批次大小
   --batch-size 8
   ```

2. **训练速度慢**
   ```bash
   # 使用更小的模型
   --weights yolov5n.pt
   ```

3. **过拟合**
   ```bash
   # 增加数据增强
   --augment
   # 减少训练轮数
   --epochs 50
   ```

4. **欠拟合**
   ```bash
   # 增加训练轮数
   --epochs 200
   # 使用更大的模型
   --weights yolov5m.pt
   ```

## 性能评估

### 1. 评估指标

训练完成后，模型性能通常达到：

- **mAP@0.5**: > 0.95
- **Precision**: > 0.90
- **Recall**: > 0.90

### 2. 测试集评估

```bash
# 在测试集上评估模型
python val.py \
    --weights runs/train/doorplate_detection/weights/best.pt \
    --data data/doorplate.yaml \
    --img-size 640
```

## 下一步

训练完成后，您可以：

1. **集成到ROS2节点**: 将模型集成到现有的plate_detection_node.py中
2. **实时检测**: 使用摄像头进行实时门牌检测
3. **模型优化**: 进一步优化模型性能
4. **部署**: 将模型部署到实际应用中

## 联系支持

如果在训练过程中遇到问题，请：

1. 检查数据集是否正确生成
2. 确认所有依赖已正确安装
3. 查看训练日志中的错误信息
4. 参考YOLOv5官方文档 