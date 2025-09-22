# Launch文件使用说明

## 概述

本目录包含用于启动无人机控制系统的launch文件，可以同时启动多个相关节点。

## Launch文件列表

### 1. `drone_plate_detection.launch.py`
完整功能的launch文件，包含所有节点（包括Intel RealSense D435支持）

### 2. `test_system.launch.py`
简化版launch文件，用于测试基本功能

## 使用方法

### 基本启动
```bash
# 启动所有节点
ros2 launch drone_control test_system.launch.py

# 启动完整版本（包含RealSense支持）
ros2 launch drone_control drone_plate_detection.launch.py
```

### 自定义参数启动
```bash
# 只启动车牌检测和USB摄像头
ros2 launch drone_control test_system.launch.py use_drone_control:=false

# 只启动无人机控制和车牌检测（不使用摄像头）
ros2 launch drone_control test_system.launch.py use_usb_cam:=false

# 只启动车牌检测
ros2 launch drone_control test_system.launch.py use_drone_control:=false use_usb_cam:=false
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_usb_cam` | `true` | 是否启动USB摄像头 |
| `use_realsense` | `false` | 是否启动Intel RealSense D435 |
| `use_plate_detection` | `true` | 是否启动车牌检测节点 |
| `use_drone_control` | `true` | 是否启动无人机控制节点 |

## 节点说明

### 1. 无人机控制节点 (`drone_control_node`)
- **包**: `drone_control`
- **功能**: 提供无人机控制服务
- **话题**: 服务接口

### 2. 车牌检测节点 (`plate_detection_node`)
- **包**: `plate_detection`
- **订阅**: `/image_raw` (图像输入)
- **发布**: 
  - `/plate_detection_result` (检测结果)
  - `/plate_detection_result_image` (标注图像)

### 3. USB摄像头节点 (`usb_cam_node`)
- **包**: `usb_cam`
- **发布**: `/image_raw` (图像输出)
- **参数**: 
  - `video_device`: `/dev/video0`
  - `image_width`: `640`
  - `image_height`: `480`

### 4. Intel RealSense D435节点（已注释）
- **包**: `realsense2_camera`
- **功能**: 提供深度相机支持
- **注意**: 需要安装 `ros-humble-realsense2-camera`

## 安装依赖

### USB摄像头
```bash
sudo apt install ros-humble-usb-cam
```

### Intel RealSense（可选）
```bash
sudo apt install ros-humble-realsense2-camera
```

## 故障排除

### 1. USB摄像头问题
```bash
# 检查摄像头设备
ls /dev/video*

# 测试摄像头
v4l2-ctl --list-devices
```

### 2. 话题检查
```bash
# 查看所有话题
ros2 topic list

# 查看图像话题
ros2 topic echo /image_raw
```

### 3. 节点状态检查
```bash
# 查看所有节点
ros2 node list

# 查看节点信息
ros2 node info /plate_detection_node
```

## 示例工作流程

### 1. 基本测试
```bash
# 终端1: 启动系统
ros2 launch drone_control test_system.launch.py

# 终端2: 查看车牌检测结果
ros2 topic echo /plate_detection_result

# 终端3: 查看标注图像
ros2 run rqt_image_view rqt_image_view /plate_detection_result_image
```

### 2. 无人机控制测试
```bash
# 终端1: 启动系统
ros2 launch drone_control test_system.launch.py use_usb_cam:=false

# 终端2: 调用无人机服务
ros2 service call /drone/connect std_srvs/srv/Trigger
``` 