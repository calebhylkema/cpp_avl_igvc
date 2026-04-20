cmake_minimum_required(VERSION 3.8)
project(igvc_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz DESTINATION share/${PROJECT_NAME})

ament_package()
