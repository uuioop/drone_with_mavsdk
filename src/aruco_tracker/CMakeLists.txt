# CMake构建配置文件 - ArUco标签跟踪器
# 功能：定义ROS2 C++节点的编译规则和依赖关系

cmake_minimum_required(VERSION 3.5)  # 最低CMake版本要求
project(aruco_tracker)               # 项目名称

# 编译选项配置：启用严格警告和错误检查
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

# 依赖包查找 - 核心数学和ROS2库
find_package(eigen3_cmake_module REQUIRED)  # Eigen3矩阵库CMake模块
find_package(Eigen3 REQUIRED)                 # Eigen3线性代数库（用于3D位姿计算）
find_package(ament_cmake REQUIRED)            # ROS2构建系统
find_package(rclcpp REQUIRED)                 # ROS2 C++客户端库

# 消息类型依赖
find_package(sensor_msgs REQUIRED)            # 传感器消息（图像、相机信息）
find_package(geometry_msgs REQUIRED)        # 几何消息（位姿、变换）
find_package(cv_bridge REQUIRED)            # OpenCV与ROS图像转换

# OpenCV库配置 - 使用自定义编译版本
# 注意：从源码构建OpenCV，路径指向/usr/local
find_package(OpenCV 4.10.0 REQUIRED PATHS /usr/local NO_DEFAULT_PATH)

# 头文件和库路径配置
include_directories(${Eigen3_INCLUDE_DIRS})   # Eigen3头文件路径
include_directories(${OpenCV_INCLUDE_DIRS}) # OpenCV头文件路径
link_directories(/usr/local/lib)          # OpenCV库文件路径

# 可执行文件构建
add_executable(aruco_tracker ArucoTracker.cpp)  # 创建aruco_tracker可执行文件

# 库链接配置
target_link_libraries(aruco_tracker ${OpenCV_LIBS})  # 链接OpenCV库

# ROS2依赖关系声明
ament_target_dependencies(aruco_tracker 
  Eigen3        # 3D数学运算
  rclcpp        # ROS2 C++接口
  sensor_msgs   # 传感器消息
  cv_bridge     # OpenCV-ROS桥接
  OpenCV        # 计算机视觉库
)

# 编译特性配置 - C99和C++17标准
target_compile_features(aruco_tracker PUBLIC c_std_99 cxx_std_17)

# 可执行文件安装配置
install(TARGETS aruco_tracker DESTINATION lib/${PROJECT_NAME})

# 测试配置（可选）
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# 启动文件安装 - 包含Gazebo和真实无人机的启动配置
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 配置文件安装 - 包含参数YAML文件
install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)

# ROS2包配置完成
ament_package()
