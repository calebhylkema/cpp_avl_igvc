cmake_minimum_required(VERSION 3.8)
project(igvc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Hardware interface shared library
add_library(igvc_hardware SHARED src/igvc_hardware.cpp)
target_include_directories(igvc_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(igvc_hardware
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface igvc_control.xml)

install(TARGETS igvc_hardware
  EXPORT export_igvc_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_igvc_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp)

ament_package()
