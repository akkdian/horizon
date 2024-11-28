cmake_minimum_required(VERSION 3.5)
project(image_subscriber)

# 使用ament_cmake构建
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 查找并链接libcurl库
find_package(CURL REQUIRED)

# 创建可执行文件
add_executable(image_subscriber src/image_subscriber/src/image_subscriber.cpp)
# 为可执行文件链接需要的依赖
ament_target_dependencies(image_subscriber rclcpp sensor_msgs CURL)

# 安装可执行文件
install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
