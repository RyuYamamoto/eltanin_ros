cmake_minimum_required(VERSION 3.20)
# NONE: this package compiles nothing, so no compiler needs to be probed for it.
project(eltanin_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
