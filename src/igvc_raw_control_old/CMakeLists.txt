cmake_minimum_required(VERSION 3.8)
project(igvc_raw_control_old)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)

add_executable(serial_bridge src/serial_bridge.cpp)

target_include_directories(serial_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(serial_bridge rclcpp std_msgs)

install(TARGETS serial_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
