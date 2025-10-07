# ArUco Tracker

这个包用于检测和追踪ArUco标记，为无人机提供视觉定位和导航功能。

## 功能
- 实时检测和识别ArUco标记
- 计算标记的位置和姿态信息
- 提供标记跟踪数据用于无人机导航

## 使用方法

### 配置
配置文件位于 `cfg/params.yaml`，可以调整检测参数。（需自行替换启动文件中的参数）

### 启动

#### 在Gazebo仿真环境中启动
```bash
ros2 launch aruco_tracker gazebo_aruco_tracker.launch.py
```

#### 在真实环境中启动
```bash
ros2 launch aruco_tracker real_aruco_tracker.launch.py
```

## 依赖
- ROS2
- OpenCV
- ArUco标记库