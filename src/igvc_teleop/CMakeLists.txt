cmake_minimum_required(VERSION 3.8)
project(igvc_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(std_msgs       REQUIRED)
find_package(geometry_msgs  REQUIRED)

add_executable(teleop_node src/teleop_node.cpp)

target_include_directories(teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(teleop_node rclcpp std_msgs geometry_msgs)

install(TARGETS teleop_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
