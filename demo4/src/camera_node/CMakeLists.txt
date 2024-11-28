cmake_minimum_required(VERSION 3.5)
project(camera_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(camera_publisher src/camera_publisher.cpp src/shared_memory_utils.cpp)
ament_target_dependencies(camera_publisher rclcpp std_msgs)

# Install
install(TARGETS camera_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()