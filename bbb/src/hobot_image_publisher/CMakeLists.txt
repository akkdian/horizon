cmake_minimum_required(VERSION 3.5)
project(hobot_image_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(img_msgs REQUIRED)

set(FFMPEG_LIBRARIES avformat avutil avcodec)

if(PLATFORM_X3)
  message("build platform X3")
  set(SYS_ROOT ${CMAKE_SYSROOT})
  message("SYS_ROOT is " ${SYS_ROOT})
  include_directories(
    ${SYS_ROOT}/usr/include/opencv/
  )
  link_directories(
    ${SYS_ROOT}/usr/lib/hbbpu/
    ${SYS_ROOT}/usr/lib/
  )
  set(SYS_LIB hbmem opencv_world ${FFMPEG_LIBRARIES})
elseif(PLATFORM_Rdkultra)
  message("build platform Rdkultra")
  set(SYS_ROOT ${CMAKE_SYSROOT})
  message("SYS_ROOT is " ${SYS_ROOT})
  include_directories(
    ${SYS_ROOT}/usr/include/opencv/
  )
  link_directories(
    ${SYS_ROOT}/usr/lib/hbbpu/
    ${SYS_ROOT}/usr/lib/
  )
  set(SYS_LIB hbmem opencv_world ${FFMPEG_LIBRARIES})
elseif(PLATFORM_X86)
  message("build platform X86")
  find_package(OpenCV 4 REQUIRED)
  include_directories(
    /usr/include/
    /usr/include/opencv4
  )
  link_directories(
    /lib/
  )
  set(SYS_LIB ${OpenCV_LIBS} ${FFMPEG_LIBRARIES})
elseif(PLATFORM_X5)
  message("build platform X5")
  set(SYS_ROOT ${CMAKE_SYSROOT})
  message("SYS_ROOT is " ${SYS_ROOT})
  include_directories(
    ${SYS_ROOT}/usr/include/opencv/
  )
  link_directories(
    ${SYS_ROOT}/usr/lib/hbbpu/
    ${SYS_ROOT}/usr/lib/
  )
  set(SYS_LIB opencv_world ${FFMPEG_LIBRARIES})
else()
  message("invalid platform, build platform X3 default")
  set(SYS_ROOT ${CMAKE_SYSROOT})
  message("SYS_ROOT is " ${SYS_ROOT})
  include_directories(
    ${SYS_ROOT}/usr/include/opencv/
  )
  link_directories(
    ${SYS_ROOT}/usr/lib/hbbpu/
    ${SYS_ROOT}/usr/lib/
  )
  set(SYS_LIB hbmem opencv_world ${FFMPEG_LIBRARIES})
  set(PLATFORM_X3 ON)
endif()

add_executable(hobot_image_pub
src/main.cpp
src/image_pub_node.cpp
)

ament_target_dependencies(
  hobot_image_pub
  rclcpp
  sensor_msgs
  hbm_img_msgs
  img_msgs
)

target_link_libraries(hobot_image_pub ${SYS_LIB})

include_directories(include
${PROJECT_SOURCE_DIR}
)

# Install executables
install(
  TARGETS hobot_image_pub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
