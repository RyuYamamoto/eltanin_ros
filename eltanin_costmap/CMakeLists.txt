cmake_minimum_required(VERSION 3.20)
project(eltanin_costmap)

find_package(ament_cmake REQUIRED)
# eltanin comes from eltanin_vendor's install prefix, which ament puts on CMAKE_PREFIX_PATH (D-24).
find_package(eltanin REQUIRED)
find_package(eltanin_msgs REQUIRED)
find_package(eltanin_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)

# On targets with target_compile_options(PRIVATE), never through CMAKE_CXX_FLAGS, which would leak
# them into the eltanin_vendor ExternalProject and into rosidl generated code.
option(ELTANIN_ROS_ENABLE_WERROR "Treat compiler warnings as errors" OFF)
set(warning_options -Wall -Wextra -Wpedantic)
if(ELTANIN_ROS_ENABLE_WERROR)
  list(APPEND warning_options -Werror)
endif()

add_library(${PROJECT_NAME} SHARED
  src/global_costmap.cpp
  src/global_costmap_state.cpp
  src/observation_store.cpp
  src/update_region.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
# One link line is enough: eltanin_ros_common_runtime carries eltanin::core, eltanin::map,
# eltanin::sensor, eltanin_msgs and nav_msgs in its public interface, and rclcpp with them.
target_link_libraries(${PROJECT_NAME} PUBLIC
  eltanin_ros_common::eltanin_ros_common_runtime
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_srvs_TARGETS})
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})

# NO_UNDEFINED_SYMBOLS below calls check_cxx_compiler_flag, which rclcpp_components does not
# include itself, so the macro fails with "Unknown CMake command" without this line.
include(CheckCXXCompilerFlag)
# MultiThreadedExecutor: with the default single-threaded one the 0.285 s of ~/update would stop
# the subscriptions outright, which is what splitting the callback groups is meant to avoid.
# NO_UNDEFINED_SYMBOLS turns a missing link into a build error rather than a dlopen failure.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "eltanin_costmap::GlobalCostmap"
  EXECUTABLE global_costmap
  EXECUTOR MultiThreadedExecutor
  NO_UNDEFINED_SYMBOLS)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# Required: the generated executable dlopens lib/libeltanin_costmap.so to find the component.
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  foreach(test_name
      test_global_costmap_state
      test_observation_store
      test_update_region)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
    target_compile_options(${test_name} PRIVATE ${warning_options})
  endforeach()

  # The only test in this package that calls rclcpp::init; the pure parts above do not need it.
  ament_add_gtest(test_global_costmap test/test_global_costmap.cpp TIMEOUT 60)
  target_link_libraries(test_global_costmap ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
  target_compile_options(test_global_costmap PRIVATE ${warning_options})
endif()

ament_package()
