# 无人机控制系统 (Drone Control System)

基于ROS2和MAVSDK的无人机控制系统，支持任务模式、板外模式和混合导航模式。

## 功能特性

### 🚁 核心功能
- **任务模式导航**: 支持GPS坐标导航
- **板外模式控制**: 支持精确位置和速度控制
- **混合导航模式**: 结合任务模式和板外模式的智能导航
- **相对位置导航**: 基于NED坐标系的相对运动控制
- **自动连接管理**: 智能端口管理和连接重试
- **状态监控**: 实时监控无人机连接、健康状态、位置和姿态

### 🛠️ 技术特性
- **异步编程**: 基于asyncio的高效异步控制
- **模块化设计**: 清晰的代码结构和组件分离
- **错误处理**: 完善的异常处理和错误恢复机制
- **日志系统**: 详细的日志记录和调试信息


## 安装和构建

### 环境要求
- ROS2 Humble 或更新版本
- Python 3.8+
- MAVSDK-Python
- 支持MAVLink的无人机（如PX4固件）

### 构建步骤
```bash
# 1. 进入ROS2工作空间
cd ~/your_ws

# 2. 安装依赖
pip install mavsdk

# 3. 构建包
colcon build --packages-select drone_control

# 4. 设置环境
source install/setup.bash
```

## 使用方法

### 1. 启动无人机控制节点

```bash
# 启动主控制节点
ros2 run drone_control drone_control_node
```

节点启动后会自动：
- 连接到无人机（UDP端口14540）
- 启动状态监控
- 初始化所有服务接口

### 2. 服务接口

#### 绝对位置导航服务 (`/drone/hybrid_navigation`)
```bash
# 导航到指定GPS坐标
ros2 service call /drone/hybrid_navigation drone_control/srv/Nav "{latitude_deg: 30.123456, longitude_deg: 120.123456, absolute_altitude_m: 100.0}"
```

#### 相对位置导航服务 (`/drone/relative_navigation`)
```bash
# 向北飞行10米，向东飞行5米，上升9米
ros2 service call /drone/relative_navigation drone_control/srv/Pos "{north: 10.0, east: 5.0, down: -9.0}"
```

### 3. 常用导航命令

#### 基础相对导航
```bash
# 向北飞行10米
ros2 service call /drone/relative_navigation drone_control/srv/Pos "{north: 10.0, east: 0.0, down: 0.0}"

# 向东飞行5米
ros2 service call /drone/relative_navigation drone_control/srv/Pos "{north: 0.0, east: 5.0, down: 0.0}"

# 上升5米
ros2 service call /drone/relative_navigation drone_control/srv/Pos "{north: 0.0, east: 0.0, down: -5.0}"

# 下降3米
ros2 service call /drone/relative_navigation drone_control/srv/Pos "{north: 0.0, east: 0.0, down: 3.0}"
```

#### 复合导航
```bash
# 对角线飞行：向北5米，向东5米，上升2米
ros2 service call /drone/relative_navigation drone_control/srv/Pos "{north: 5.0, east: 5.0, down: -2.0}"

# 复杂路径：向北15米，向西10米，上升3米
ros2 service call /drone/relative_navigation drone_control/srv/Pos "{north: 15.0, east: -10.0, down: -3.0}"
```

## 坐标系统

### NED坐标系 (North-East-Down)
- **N (North)**: 北向偏移，正值向北，负值向南
- **E (East)**: 东向偏移，正值向东，负值向西  
- **D (Down)**: 下向偏移，正值向下（下降），负值向上（上升）

### 示例转换
```
当前位置: 纬度=30.123456, 经度=120.123456, 高度=100.0m
相对偏移: NED(10.0, 5.0, -2.0)  # 向北10米，向东5米，上升2米
目标位置: 纬度=30.123546, 经度=120.123501, 高度=102.0m
```

## 工作流程

### 相对位置导航流程
1. **位置获取**: 获取无人机当前位置
2. **坐标转换**: 将NED相对偏移转换为经纬度绝对坐标
3. **任务模式**: 使用任务模式导航到目标位置
4. **板外模式**: 切换到板外模式进行精确位置控制
5. **完成确认**: 确认到达目标位置

### 混合导航流程
1. **系统检查**: 验证连接状态和起飞条件
2. **任务规划**: 创建任务计划并上传
3. **任务执行**: 执行任务模式导航
4. **精确控制**: 切换到板外模式进行精确控制
5. **状态监控**: 实时监控导航进度和位置误差

## 工具函数

### 几何计算工具 (`utils.py`)
- `calculate_distance()`: 计算GPS坐标间距离
- `calculate_relative_position_target()`: 相对位置目标计算
- `validate_gps_coordinates()`: GPS坐标验证
- `calculate_heading_from_points()`: 航向角计算

## 板外模式示例

系统包含多个板外模式示例：

### 位置控制 (`offboard_position_ned.py`)
```python
# 设置NED位置
setpoint = PositionNedYaw(0.0, 0.0, -10.0, 90.0)
await drone.offboard.set_position_ned(setpoint)
```

### 速度控制 (`offboard_velocity_ned.py`)
```python
# 设置NED速度
setpoint = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
await drone.offboard.set_velocity_ned(setpoint)
```

### 姿态控制 (`offboard_attitude.py`)
```python
# 设置姿态（仅适用于仿真）
setpoint = Attitude(0.0, 0.0, 0.0, 0.5)
await drone.offboard.set_attitude(setpoint)
```

## 错误处理和故障排除

### 常见错误

#### 连接问题
```
错误: "未连接无人机"
解决: 检查无人机连接和端口占用

```

#### 状态问题
```
错误: "无人机起飞状态不满足"
解决: 确保无人机已解锁且GPS信号良好
```

#### 位置问题
```
错误: "无法获取当前位置"
解决: 等待GPS定位完成
```

### 调试命令

#### 检查服务状态
```bash
# 列出所有服务
ros2 service list

# 检查服务类型
ros2 service type /drone/relative_navigation
```

#### 检查节点状态
```bash
# 列出所有节点
ros2 node list

# 查看节点信息
ros2 node info /drone_control_node
```

#### 端口管理
```bash
# 查看所有占用14540端口的进程
sudo lsof -i :14540
# 方法2：如果知道PID，使用kill
sudo kill -9 <PID>
```

## 开发指南

### 添加新功能
1. 在 `drone_control/core/` 中创建新模块
2. 在 `interface/` 中定义服务接口
3. 在 `drone_control_node.py` 中集成新功能
4. 更新 `CMakeLists.txt` 和 `package.xml`

### 代码结构
- **异步编程**: 所有MAVSDK操作都使用async/await
- **错误处理**: 使用try-except包装所有重要异步操作
- **日志记录**: 使用self.get_logger()记录关键信息
- **状态管理**: 通过DroneState类管理全局状态

## 精准降落功能

本项目已集成基于ArUco标记的精准降落功能，移植自C++版本的PrecisionLand节点。

### 功能特性
- **🎯 精准定位**: 基于ArUco标记的视觉定位
- **🔄 状态机控制**: Search → Approach → Descend → Finished
- **📡 PID控制**: XY方向速度的精确控制
- **🌀 搜索模式**: 目标丢失时的螺旋搜索
- **🛡️ 安全回退**: 精准降落失败时自动切换到普通降落

### 使用方法

#### 1. 基本配置
```python
from drone_control.core.aruco_tracker import ArucoTracker
from drone_control.core.offboard_navigation import OffboardNavigationController

# 创建ArUco跟踪器
aruco_tracker = ArucoTracker(drone, logger, drone_state)

# 设置ArUco参数
aruco_tracker.set_aruco_parameters(
    aruco_id=0,        # 目标标记ID
    dictionary=2,      # DICT_4X4_250
    marker_size=0.5    # 标记尺寸(米)
)

# 设置相机参数 (需要相机标定)
camera_matrix = np.array([[615.0, 0.0, 320.0],
                         [0.0, 615.0, 240.0],
                         [0.0, 0.0, 1.0]])
dist_coeffs = np.array([0.1, -0.2, 0.0, 0.0, 0.0])
aruco_tracker.set_camera_parameters(camera_matrix, dist_coeffs)

# 创建带精准降落的导航控制器
nav_controller = OffboardNavigationController(
    drone, logger, drone_state, license_plate_result, aruco_tracker
)
```

#### 2. 参数调整
```python
# 调整精准降落参数
nav_controller.precision_land.set_parameters(
    descent_vel=0.8,        # 下降速度 (m/s)
    vel_p_gain=1.2,         # P增益
    vel_i_gain=0.1,         # I增益
    max_velocity=2.0,       # 最大速度 (m/s)
    target_timeout=5.0      # 目标超时时间 (s)
)
```

#### 3. 执行精准降落
```python
# 导航到目标位置会自动使用精准降落
await nav_controller.navigate_to_position(
    target_north=10.0,
    target_east=5.0, 
    target_down=-8.0
)
```

### 配置文件
参考 `config/precision_land_config.yaml` 进行详细配置。

### 测试
```bash
# 安装测试依赖
pip3 install pytest pytest-asyncio

# 运行单元测试
python3 -m pytest test/test_precision_land.py -v

# 运行集成测试
python3 -m pytest test/test_integration.py -v

# 运行所有测试
python3 -m pytest test/ -v

# 手动测试服务
ros2 service call /drone/relative_navigation drone_control/srv/Pos "{north: 1.0, east: 0.0, down: 0.0}"
```

## 许可证

本项目基于MIT许可证开源。

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 联系方式

如有问题或建议，请通过GitHub Issues联系我们。 