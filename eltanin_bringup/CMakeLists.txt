cmake_minimum_required(VERSION 3.20)
# NONE: this package compiles nothing, so no compiler needs to be probed for it.
project(eltanin_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

# RENAME so that the launch file names an executable, not a file with an extension.
install(PROGRAMS scripts/goal_pose_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME goal_pose_relay)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Fences over the configuration itself: none of what they check fails at startup.
  foreach(test_name
      test_robot_profile_config
      test_navigation_config
      test_rviz_config
      test_launch_file)
    ament_add_pytest_test(${test_name} test/${test_name}.py)
  endforeach()
endif()

ament_package()
