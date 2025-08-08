cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成自定义服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "interface/Nav.srv"
  "interface/Pos.srv"
  "interface/LicenseInfo.srv"
  DEPENDENCIES geometry_msgs
)

# 安装Python可执行文件
install(PROGRAMS
  scripts/drone_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装接口文件
install(DIRECTORY
  interface
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python模块到local包
install(DIRECTORY
  drone_control
  DESTINATION local/lib/python3.10/dist-packages
)

ament_package() 