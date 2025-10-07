# Drone Control Interfaces

这个包提供了无人机控制系统中使用的ROS2接口定义，包括消息类型和服务类型。

## 功能
- 定义号牌信息消息格式
- 定义导航服务接口
- 定义位置服务接口

## 接口定义

### 消息类型 (msg)
- **LicenseInfo.msg**: 用于传输号牌识别信息

### 服务类型 (srv)
- **Nav.srv**: 导航服务接口，用于发送导航指令
- **Pos.srv**: 位置服务接口，用于获取或设置无人机位置

## 使用方法

在其他ROS2包中使用这些接口时，需要在`package.xml`和`CMakeLists.txt`中添加依赖：

### package.xml
```xml
<depend>drone_control_interfaces</depend>
```

### CMakeLists.txt
```cmake
find_package(drone_control_interfaces REQUIRED)
```

## 依赖
- ROS2 Interface Definition Language (IDL)