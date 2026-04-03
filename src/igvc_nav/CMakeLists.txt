cmake_minimum_required(VERSION 3.8)
project(igvc_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake  REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs     REQUIRED)

# ── cmd_vel_bridge ────────────────────────────────────────────────────────────
add_executable(cmd_vel_bridge src/cmd_vel_bridge.cpp)

target_include_directories(cmd_vel_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cmd_vel_bridge rclcpp geometry_msgs std_msgs)

install(TARGETS cmd_vel_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

ament_package()
