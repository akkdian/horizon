# camera_streamer/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(camera_streamer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SharedImage.msg"
  DEPENDENCIES builtin_interfaces
)

include_directories(include)

add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher rclcpp)

add_executable(web_display_node src/web_display_node.cpp)
ament_target_dependencies(web_display_node rclcpp)

target_link_libraries(camera_publisher
  pthread
  rt
)

target_link_libraries(web_display_node
  pthread
  rt
  jpeg  # 链接 libjpeg 库
)

install(TARGETS
  camera_publisher
  web_display_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()