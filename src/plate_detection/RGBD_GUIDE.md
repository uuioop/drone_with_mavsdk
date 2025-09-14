# RGBD车牌检测使用指南

本指南介绍如何使用Intel Realsense D435相机进行RGBD车牌检测，获取车牌的3D坐标信息。

## 功能概述

RGBD车牌检测节点在原有车牌识别基础上，增加了以下功能：
- ✅ 同步处理RGB图像和深度图
- ✅ 计算车牌相对于相机的3D坐标
- ✅ 实时显示车牌距离信息
- ✅ 支持Intel Realsense D435相机
- ✅ 可视化3D位置信息

## 快速开始

### 1. 启动Realsense相机

```bash
# 方式1：使用realsense2_camera驱动
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 depth_module.profile:=640x480x30 align_depth.enable:=true

# 方式2：使用自定义配置
ros2 launch plate_detection plate_detection_rgbd.launch.py
```

### 2. 启动RGBD车牌检测节点

```bash
# 使用launch文件启动
ros2 launch plate_detection plate_detection_rgbd.launch.py

# 或者直接运行节点
ros2 run plate_detection plate_detection_rgbd_node
```

## 话题配置

### 输入话题

| 话题名称 | 类型 | 描述 |
|---------|------|------|
| `/camera/color/image_raw` | sensor_msgs/Image | RGB彩色图像 |
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | 深度图像 |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | 相机内参 |

### 输出话题

| 话题名称 | 类型 | 描述 |
|---------|------|------|
| `/license_detection_result` | std_msgs/String | JSON格式的检测结果 |
| `/license_detection_result_image` | sensor_msgs/Image | 带标注的结果图像 |
| `/license_plate_positions` | geometry_msgs/PointStamped | 车牌3D坐标 |

## 配置参数

### 文件配置

编辑 `config/plate_detection_rgbd_params.yaml`：

```yaml
plate_detection:
  ros__parameters:
    rgb_topic: "/camera/color/image_raw"
    depth_topic: "/camera/depth/image_rect_raw"
    camera_info_topic: "/camera/color/camera_info"
    output_topic: "/license_detection_result"
    result_image_topic: "/license_detection_result_image"
    position_topic: "/license_plate_positions"
```

### 命令行参数

```bash
ros2 launch plate_detection plate_detection_rgbd.launch.py \
  rgb_topic:=/custom/rgb/topic \
  depth_topic:=/custom/depth/topic \
  camera_info_topic:=/custom/camera_info/topic
```

## 输出格式

### 检测结果JSON格式

```json
[
  {
    "plate_no": "京A12345",
    "confidence": 0.95,
    "position_3d": [1.234, -0.156, 2.789],
    "rect": [100, 200, 300, 250],
    "plate_color": "蓝色"
  }
]
```

### 3D坐标解释

- `position_3d`: [x, y, z] 相对于相机坐标系的3D坐标（单位：米）
- x: 水平方向（右为正）
- y: 垂直方向（下为正）  
- z: 深度方向（前为正）

## 相机设置

### Realsense D435推荐配置

```bash
# 启动相机时推荐参数
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.profile:=640x480x30 \
  depth_module.profile:=640x480x30 \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false
```

### 校准注意事项

1. **深度精度**: 确保相机已正确校准
2. **光照条件**: 避免强光直射，影响深度测量
3. **距离范围**: 有效检测距离0.1-10米
4. **角度限制**: 避免过大视角，影响检测精度

## 故障排除

### 常见问题

#### 1. 深度图无数据

**症状**: 所有车牌距离显示为0或无效

**解决**:
```bash
# 检查深度图话题
ros2 topic echo /camera/depth/image_rect_raw/header

# 检查相机驱动
ros2 launch realsense2_camera rs_launch.py
```

#### 2. 3D坐标计算错误

**症状**: 距离信息明显不准确

**解决**:
- 检查相机内参是否正确加载
- 验证深度图与RGB图是否对齐
- 确认相机已正确校准

#### 3. 同步失败

**症状**: RGB和深度图不同步

**解决**:
```bash
# 检查时间戳
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_rect_raw

# 调整同步参数
# 在launch文件中修改slop参数
```

### 调试工具

```bash
# 查看检测结果
ros2 topic echo /license_detection_result

# 查看3D坐标
ros2 topic echo /license_plate_positions

# 可视化结果
ros2 run rqt_image_view rqt_image_view /license_detection_result_image

# 检查相机信息
ros2 topic echo /camera/color/camera_info
```

## 性能优化

### 参数调优

1. **降低分辨率**: 如果性能不足，可降低RGB和深度图分辨率
2. **调整检测频率**: 修改帧率参数
3. **优化模型**: 使用更轻量的检测模型

### 硬件要求

- **CPU**: Intel i5或同等性能
- **GPU**: 推荐NVIDIA GTX 1050以上（可选）
- **内存**: 8GB以上
- **USB**: USB 3.0接口

## 集成示例

### 与无人机系统集成

```python
# 订阅3D位置信息
import rclpy
from geometry_msgs.msg import PointStamped

def position_callback(msg):
    print(f"车牌位置: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}")

# 在无人机控制节点中订阅
self.position_sub = self.create_subscription(
    PointStamped, 
    '/license_plate_positions', 
    position_callback, 
    10
)
```

## 版本信息

- **支持相机**: Intel Realsense D435/D435i
- **ROS版本**: ROS2 Foxy及以上
- **依赖**: realsense2_camera, message_filters, geometry_msgs

## 技术支持

如有问题，请检查：
1. 相机驱动是否正确安装
2. 话题名称是否匹配
3. 相机是否已校准
4. 查看节点日志获取详细错误信息