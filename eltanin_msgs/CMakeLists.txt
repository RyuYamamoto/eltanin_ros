cmake_minimum_required(VERSION 3.20)
project(eltanin_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# No warning flags here. This package compiles nothing of its own, and -Wall -Wextra -Wpedantic
# would apply to rosidl generated code instead.
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MapMetaData2D.msg
  msg/Costmap.msg
  msg/CostmapUpdate.msg
  msg/TrajectoryPoint2D.msg
  msg/Trajectory2D.msg
  msg/DirectedPath.msg
  msg/NavigationState.msg
  msg/FollowerDiagnostic.msg
  action/NavigateToPose.action
  action/ComputePathToPose.action
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    geometry_msgs
    nav_msgs
    std_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
