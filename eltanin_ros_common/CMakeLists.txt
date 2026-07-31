cmake_minimum_required(VERSION 3.20)
project(eltanin_ros_common)

find_package(ament_cmake REQUIRED)
# eltanin comes from eltanin_vendor's install prefix, which ament puts on CMAKE_PREFIX_PATH (D-24).
find_package(eltanin REQUIRED)
find_package(eltanin_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# This is the first package where ELTANIN_ROS_ENABLE_WERROR has anything to act on: the other three
# compile nothing of their own. The flags go on targets with target_compile_options(PRIVATE) rather
# than through CMAKE_CXX_FLAGS, which would leak them into the eltanin_vendor ExternalProject and
# into rosidl generated code.
option(ELTANIN_ROS_ENABLE_WERROR "Treat compiler warnings as errors" OFF)
set(warning_options -Wall -Wextra -Wpedantic)
if(ELTANIN_ROS_ENABLE_WERROR)
  list(APPEND warning_options -Werror)
endif()

add_library(${PROJECT_NAME} SHARED
  src/cost_conversion.cpp
  src/geometry_conversion.cpp
  src/map_conversion.cpp
  src/scan_conversion.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# PUBLIC: the public headers expose eltanin and message types, so downstream needs both the include
# directories and the link interface. The three eltanin modules are the ones whose headers appear in
# include/: core (types, path), map (grid_map) and sensor (scan_projection).
target_link_libraries(${PROJECT_NAME} PUBLIC
  eltanin::core
  eltanin::map
  eltanin::sensor
  ${eltanin_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2)
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# Lets a downstream package consume this one with find_package(eltanin_ros_common) alone: the
# generated extras call find_package(<dep> QUIET REQUIRED) for each name here, which is what defines
# the eltanin:: imported targets our export set refers to (A-5).
ament_export_dependencies(
  eltanin
  eltanin_msgs
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # No node is started anywhere in these tests: the whole package is reachable without rclcpp::init.
  foreach(test_name
      test_conversion_result
      test_cost_conversion
      test_geometry_conversion
      test_map_conversion
      test_scan_conversion)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(${test_name} PRIVATE ${warning_options})
  endforeach()
endif()

ament_package()
