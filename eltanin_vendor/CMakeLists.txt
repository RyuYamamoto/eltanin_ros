cmake_minimum_required(VERSION 3.20)
# LANGUAGES CXX rather than NONE: this package compiles nothing itself, but it has to forward the
# compiler colcon selected to the eltanin sub-build, and CMAKE_CXX_COMPILER is only defined when
# CXX is enabled (D-T3-11).
project(eltanin_vendor LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

set(ELTANIN_VENDOR_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../eltanin"
    CACHE PATH "eltanin source tree used when ELTANIN_VENDOR_GIT_URL is empty")
set(ELTANIN_VENDOR_GIT_URL ""
    CACHE STRING "clone eltanin from this URL instead of using ELTANIN_VENDOR_SOURCE_DIR")
set(ELTANIN_VENDOR_GIT_TAG ""
    CACHE STRING "revision to check out; required when ELTANIN_VENDOR_GIT_URL is set")
# RelWithDebInfo (= NDEBUG = assert() disabled) is the default on purpose. A precondition violation
# that aborts on a real robot kills the node, and a dead collision_predictor stops publishing
# /cmd_vel while the robot keeps driving on the last command (design section 4.3 / D-25). Pass
# -DELTANIN_VENDOR_BUILD_TYPE=Debug during development to get the asserts back.
set(ELTANIN_VENDOR_BUILD_TYPE "RelWithDebInfo"
    CACHE STRING "CMAKE_BUILD_TYPE for the eltanin sub-build")
# OFF matches eltanin's own default and keeps the build offline. ON adds the MPC follower, which
# pulls OSQP: with the "fetch" provider that is a git clone during the build. eltanin installs OSQP
# into this same prefix and its package config calls find_dependency(osqp), so nothing else here
# has to know about it - but ELTANIN_WITH_MPC does reach every downstream translation unit through
# the exported target, and eltanin_controller compiles differently with and without it.
option(ELTANIN_VENDOR_ENABLE_MPC "Build eltanin's MPC path follower and its QP backend" OFF)
set(ELTANIN_VENDOR_MPC_SOLVER_PROVIDER "fetch"
    CACHE STRING "Where OSQP comes from when ELTANIN_VENDOR_ENABLE_MPC is ON: fetch or package")

if(ELTANIN_VENDOR_GIT_URL)
  if(NOT ELTANIN_VENDOR_GIT_TAG)
    message(FATAL_ERROR
      "ELTANIN_VENDOR_GIT_URL is set to '${ELTANIN_VENDOR_GIT_URL}' but ELTANIN_VENDOR_GIT_TAG is "
      "empty. Pass -DELTANIN_VENDOR_GIT_TAG=<branch|tag|sha> as well.")
  endif()
  # Declared as a buildtool_depend, but diagnose it here too so that an environment which ignored
  # the declaration fails with a readable message instead of one from inside ExternalProject.
  find_program(GIT_EXECUTABLE git REQUIRED)
  set(eltanin_source_args
    GIT_REPOSITORY "${ELTANIN_VENDOR_GIT_URL}"
    GIT_TAG "${ELTANIN_VENDOR_GIT_TAG}"
    GIT_SHALLOW TRUE)
else()
  if(NOT EXISTS "${ELTANIN_VENDOR_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "No eltanin sources at '${ELTANIN_VENDOR_SOURCE_DIR}'. Run "
      "'vcs import src < src/eltanin_ros/.repos' from the workspace root, or pass "
      "-DELTANIN_VENDOR_SOURCE_DIR=<path> or -DELTANIN_VENDOR_GIT_URL=<url>.")
  endif()
  set(eltanin_source_args SOURCE_DIR "${ELTANIN_VENDOR_SOURCE_DIR}")
endif()

# Installs straight into this package's prefix, so that downstream packages resolve eltanin with
# find_package(eltanin REQUIRED) alone: ament puts AMENT_PREFIX_PATH on CMAKE_PREFIX_PATH (D-24).
ExternalProject_Add(eltanin_external
  ${eltanin_source_args}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE=${ELTANIN_VENDOR_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    # eltanin does not set CMAKE_POSITION_INDEPENDENT_CODE itself and declares every module STATIC.
    # Every node here is an rclcpp_components shared library (design section 3.3 / D-13), and
    # linking a non-PIC archive into a shared object fails with an R_X86_64_PC32 relocation error.
    # Measured without this flag: eltanin_core / map / map_io / planner fail, the other four link by
    # accident, so the breakage would surface only once a later node happens to use one of the four.
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DELTANIN_BUILD_TESTS=OFF
    -DELTANIN_BUILD_EXAMPLES=OFF
    -DELTANIN_ENABLE_MPC=${ELTANIN_VENDOR_ENABLE_MPC}
    -DELTANIN_MPC_SOLVER_PROVIDER=${ELTANIN_VENDOR_MPC_SOLVER_PROVIDER}
  # eltanin is developed alongside this repository, so a plain timestamp check would silently ship a
  # stale library. BUILD_ALWAYS re-runs the build and install steps every time; measured cost is
  # 0.53 s for a no-op rebuild against 16.8 s for a clean one.
  BUILD_ALWAYS ON)

# Proves that the installed eltanin can actually be consumed the way the nodes will consume it:
# find_package(eltanin) from a separate prefix, every module linked into a SHARED library. This is a
# nested ExternalProject rather than a ctest, because eltanin is installed by a build step of this
# same project, so find_package() cannot resolve at our configure time -- and because a module that
# cannot be linked is a build failure, not a test failure (D-T3-2).
ExternalProject_Add(eltanin_link_check
  DEPENDS eltanin_external
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/link_check"
  CMAKE_ARGS
    # install/eltanin_vendor is not on the environment's prefix path while this package is being
    # built, so point the sub-build at it explicitly. Passing it as a cache variable does not
    # disable the environment lookups, so Eigen3 and yaml-cpp still resolve.
    -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    # The outer CMAKE_BUILD_TYPE, not ELTANIN_VENDOR_BUILD_TYPE: link_check stands in for a
    # downstream ROS package, so it must be built the way downstream packages are.
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
  INSTALL_COMMAND ""
  BUILD_ALWAYS ON)

ament_package()
