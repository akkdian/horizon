cmake_minimum_required(VERSION 3.5)
project(web_server_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(web_server_subscriber src/web_server_subscriber.cpp src/shared_memory_utils.cpp)
ament_target_dependencies(web_server_subscriber rclcpp std_msgs)

# Install
install(TARGETS web_server_subscriber DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()