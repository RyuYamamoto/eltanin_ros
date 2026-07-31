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

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
      test_update_region)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_compile_options(${test_name} PRIVATE ${warning_options})
  endforeach()
endif()

ament_package()
