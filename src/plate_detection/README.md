# 中文车牌检测识别ROS2节点

这是一个基于YOLOv5的中文车牌检测和识别ROS2节点，支持实时图像处理和车牌识别。

## 功能特性

- 支持中文车牌检测和识别
- 支持彩色车牌识别
- 支持ROS2参数配置
- 支持实时图像处理
- 发布检测结果和标注后的图像

## 安装依赖

确保您的系统已安装以下依赖：

```bash
# ROS2 Humble
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Python依赖
pip install torch torchvision opencv-python numpy
```

## 构建包

```bash
# 在您的工作空间根目录
colcon build --packages-select plate_detection
source install/setup.bash
```

## 使用方法

### 1. 启动节点

```bash
# 使用launch文件启动
ros2 launch plate_detection plate_detection.launch.py

# 或者直接运行节点
ros2 run plate_detection plate_detection_node
```

### 2. 发布图像话题

节点会订阅 `/image_raw` 话题，您需要发布图像数据：

```bash
# 使用摄像头
ros2 run image_transport_tutorials publisher /dev/video0

# 或者使用其他图像源
ros2 run image_transport_tutorials publisher your_image.jpg
```

### 3. 查看结果

```bash
# 查看检测结果
ros2 topic echo /plate_detection_result

# 查看标注后的图像
ros2 run rqt_image_view rqt_image_view /plate_detection_result_image
```

## 参数配置

### 启动参数

- `detect_model_path`: 检测模型路径 (默认: weights/plate_detect.pt)
- `rec_model_path`: 识别模型路径 (默认: weights/plate_rec_color.pth)
- `is_color`: 是否启用彩色识别 (默认: true)
- `img_size`: 输入图像尺寸 (默认: 640)
- `input_topic`: 输入图像话题 (默认: /image_raw)
- `output_topic`: 输出结果话题 (默认: /plate_detection_result)
- `result_image_topic`: 输出结果图像话题 (默认: /plate_detection_result_image)

### 使用自定义参数

```bash
ros2 launch plate_detection plate_detection.launch.py \
  detect_model_path:=weights/your_model.pt \
  rec_model_path:=weights/your_rec_model.pth \
  input_topic:=/camera/image_raw \
  output_topic:=/my_plate_results
```

## 话题说明

### 订阅话题
- `/image_raw` (sensor_msgs/Image): 输入图像

### 发布话题
- `/plate_detection_result` (std_msgs/String): 检测到的车牌号码
- `/plate_detection_result_image` (sensor_msgs/Image): 标注了检测结果的图像

## 模型文件

请确保以下模型文件存在于 `weights/` 目录中：
- `plate_detect.pt`: 车牌检测模型
- `plate_rec_color.pth`: 车牌识别模型

## 故障排除

### 1. 模型文件未找到
确保模型文件路径正确，并且文件存在于包的共享目录中。

### 2. 导入错误
确保所有依赖模块（models, utils, plate_recognition）都可用。

### 3. CUDA错误
如果GPU不可用，节点会自动使用CPU。确保PyTorch安装正确。

## 示例

### 使用摄像头实时检测

```bash
# 终端1: 启动摄像头节点
ros2 run image_transport_tutorials publisher /dev/video0

# 终端2: 启动车牌检测节点
ros2 launch plate_detection plate_detection.launch.py

# 终端3: 查看结果
ros2 topic echo /plate_detection_result
```

### 使用图像文件测试

```bash
# 终端1: 发布测试图像
ros2 run image_transport_tutorials publisher path/to/test_image.jpg

# 终端2: 启动车牌检测节点
ros2 launch plate_detection plate_detection.launch.py

# 终端3: 查看结果图像
ros2 run rqt_image_view rqt_image_view /plate_detection_result_image
``` 