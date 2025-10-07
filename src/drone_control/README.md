# Drone Control

这个包是无人机控制系统的核心，提供了无人机的飞行控制、状态管理、号牌识别确认和精准降落等功能。

## 功能
- 无人机状态管理和监控
- 飞行控制器接口（基于MAVSDK）
- 号牌识别与确认系统
- 精准降落功能
- 有限状态机（FSM）实现的状态管理
- 服务接口用于导航和控制

## 主要模块

### 状态机模块 (fsm)
提供状态机框架，管理无人机的各种运行状态

### 号牌处理 (SQLite)
管理号牌数据库和号牌识别结果处理

### 确认号牌 (confirm_license)
识别并确认目标号牌，实现状态转换

### 精准降落 (precision_land)
实现无人机的精准降落功能

## 使用方法

### 启动控制节点
```bash
ros2 run drone_control drone_control_node
```

### 配置
配置文件位于 `config/` 目录下，可以根据需要调整参数。（需自行替换启动文件中的参数）

## 依赖
- ROS2
- MAVSDK
- SQLite3
- Python 3.8+
- drone_control_interfaces