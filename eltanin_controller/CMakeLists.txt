cmake_minimum_required(VERSION 3.20)
project(eltanin_controller)

find_package(ament_cmake REQUIRED)
# eltanin comes from eltanin_vendor's install prefix, which ament puts on CMAKE_PREFIX_PATH (D-24).
find_package(eltanin REQUIRED)
find_package(eltanin_msgs REQUIRED)
find_package(eltanin_ros_common REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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
  src/command_composition.cpp
  src/follower_parameters.cpp
  src/limiter_diagnostic.cpp
  src/limiter_inputs.cpp
  src/limiter_parameters.cpp
  src/path_follower.cpp
  src/path_input.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
# src/diagnostic.hpp is private: the single shape of a line this package logs, not part of the API.
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
# eltanin_ros_common_runtime carries eltanin::core, eltanin::map, eltanin_msgs, diagnostic_msgs,
# geometry_msgs, nav_msgs and rclcpp; eltanin::control and std_srvs are what this package adds.
target_link_libraries(${PROJECT_NAME} PUBLIC
  eltanin::control
  ${diagnostic_msgs_TARGETS}
  eltanin_ros_common::eltanin_ros_common_runtime
  rclcpp::rclcpp
  rclcpp_components::component
  tf2_ros::tf2_ros
  ${std_srvs_TARGETS})
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})

# NO_UNDEFINED_SYMBOLS below calls check_cxx_compiler_flag, which rclcpp_components does not
# include itself, so the macro fails with "Unknown CMake command" without this line.
include(CheckCXXCompilerFlag)
# MultiThreadedExecutor: the timer, the one path subscription and ~/reset sit in separate
# MutuallyExclusive callback groups, which a single-threaded executor would make meaningless.
# NO_UNDEFINED_SYMBOLS turns a missing link into a build error rather than a dlopen failure.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "eltanin_controller::PathFollower"
  EXECUTABLE path_follower
  EXECUTOR MultiThreadedExecutor
  NO_UNDEFINED_SYMBOLS)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# Required: the generated executable dlopens lib/libeltanin_controller.so to find the component.
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  foreach(test_name
      test_command_composition
      test_follower_parameters
      test_limiter_diagnostic
      test_limiter_inputs
      test_limiter_parameters
      test_path_input)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(${test_name} PRIVATE ${warning_options})
  endforeach()

  # The only test in this package that calls rclcpp::init; the pure parts above do not need it.
  # 120 s rather than 60: a 20 Hz timer watched over several cycles in a Debug CI build is slow.
  ament_add_gtest(test_path_follower test/test_path_follower.cpp TIMEOUT 120)
  target_link_libraries(test_path_follower ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
  target_include_directories(test_path_follower PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(test_path_follower PRIVATE ${warning_options})
endif()

ament_package()
