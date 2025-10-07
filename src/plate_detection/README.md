# Plate Detection

这个包提供了号牌检测和识别功能，用于无人机视觉导航系统中的号牌识别任务。

## 功能
- 实时号牌检测（基于YOLO模型）
- 号牌号码识别
- 支持双层号牌的分割与合并处理
- 提供检测结果的可视化

## 主要组件

### 检测模块 (plate_detection)
使用YOLO模型检测图像中的号牌

### 识别模块 (plate_recognition)
识别检测到的号牌号码

### 工具模块 (utils)
提供图像处理和模型推理的辅助功能

## 使用方法

### 启动检测节点
```bash
ros2 run plate_detection plate_detection_node
```

### 通过启动文件启动
```bash
ros2 launch plate_detection plate_detection.launch.py
```

### 配置
配置文件位于 `config/plate_detection_params.yaml`，可以调整检测和识别参数。（需自行替换启动文件中的参数）

### 数据集
支持的门牌数据配置位于 `data/doorplate.yaml`。

## 依赖
- ROS2
- OpenCV
- PyTorch
- 预训练模型文件（位于 `weights/` 目录）