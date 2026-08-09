cmake_minimum_required(VERSION 3.20)
project(eltanin_planner)

find_package(ament_cmake REQUIRED)
# eltanin comes from eltanin_vendor's install prefix, which ament puts on CMAKE_PREFIX_PATH (D-24).
find_package(eltanin REQUIRED)
find_package(eltanin_msgs REQUIRED)
find_package(eltanin_ros_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# On targets with target_compile_options(PRIVATE), never through CMAKE_CXX_FLAGS, which would leak
# them into the eltanin_vendor ExternalProject and into rosidl generated code.
option(ELTANIN_ROS_ENABLE_WERROR "Treat compiler warnings as errors" OFF)
set(warning_options -Wall -Wextra -Wpedantic)
if(ELTANIN_ROS_ENABLE_WERROR)
  list(APPEND warning_options -Werror)
endif()

add_library(${PROJECT_NAME} SHARED
  src/global_path_planner.cpp
  src/plan_attempt.cpp
  src/planner_parameters.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
# src/diagnostic.hpp is private: the single shape of a line this package logs, not part of the API.
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
# eltanin_ros_common_runtime carries eltanin::core, eltanin::map, eltanin_msgs and nav_msgs, but not
# eltanin::planner: astar_planner.cpp and path_smoother.cpp live there and this is the first
# package to need them.
target_link_libraries(${PROJECT_NAME} PUBLIC
  eltanin::planner
  eltanin::collision
  eltanin_ros_common::eltanin_ros_common_runtime
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  tf2_ros::tf2_ros)
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})

# NO_UNDEFINED_SYMBOLS below calls check_cxx_compiler_flag, which rclcpp_components does not
# include itself, so the macro fails with "Unknown CMake command" without this line.
include(CheckCXXCompilerFlag)
# MultiThreadedExecutor: the search takes 0.147 s at -O2 and must not stop the costmap
# subscriptions, which is what the separate callback groups and the worker thread are for.
# NO_UNDEFINED_SYMBOLS turns a missing link into a build error rather than a dlopen failure.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "eltanin_planner::GlobalPathPlanner"
  EXECUTABLE global_path_planner
  EXECUTOR MultiThreadedExecutor
  NO_UNDEFINED_SYMBOLS)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# Required: the generated executable dlopens lib/libeltanin_planner.so to find the component.
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
      test_goal_validation
      test_planner_parameters)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(${test_name} PRIVATE ${warning_options})
  endforeach()

  # The only test in this package that calls rclcpp::init; the pure parts above do not need it.
  # 120 s rather than 60: an action round trip in a Debug CI build is slower than a service call.
  ament_add_gtest(test_global_path_planner test/test_global_path_planner.cpp TIMEOUT 120)
  target_link_libraries(test_global_path_planner ${PROJECT_NAME})
  target_include_directories(test_global_path_planner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(test_global_path_planner PRIVATE ${warning_options})
endif()

ament_package()
