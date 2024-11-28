cmake_minimum_required(VERSION 3.5)
project(camera_publisher)

# 引入 ROS 2 依赖
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(launch REQUIRED)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 添加头文件路径
include_directories(include)

# 添加可执行文件
add_executable(camera_publisher src/camera_publisher.cpp)

# 链接 ROS 2 依赖库
ament_target_dependencies(camera_publisher rclcpp sensor_msgs)

# 安装可执行文件
install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME})

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# 安装 Launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})

ament_package()
