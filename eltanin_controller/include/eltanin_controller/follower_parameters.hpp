// Copyright 2026 RyuYamamoto.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#ifndef ELTANIN_CONTROLLER__FOLLOWER_PARAMETERS_HPP_
#define ELTANIN_CONTROLLER__FOLLOWER_PARAMETERS_HPP_

#include <eltanin/control/follower_factory.hpp>
#include <eltanin/control/goal_approach.hpp>
#include <eltanin/control/path_follower.hpp>
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin_ros_common/conversion_result.hpp>
#include <eltanin_ros_common/robot_profile.hpp>

#include <array>
#include <optional>
#include <string_view>

namespace eltanin_controller
{

inline constexpr const char * KEY_UPDATE_FREQUENCY = "update_frequency";
inline constexpr const char * KEY_PATH_SOURCE = "path_source";
inline constexpr const char * KEY_FOLLOWER_TYPE = "follower_type";
inline constexpr const char * KEY_XY_GOAL_TOLERANCE = "xy_goal_tolerance";
inline constexpr const char * KEY_YAW_GOAL_TOLERANCE = "yaw_goal_tolerance";
inline constexpr const char * KEY_APPROACH_DISTANCE = "approach_distance";
inline constexpr const char * KEY_APPROACH_DECEL = "approach_decel";
inline constexpr const char * KEY_YAW_ALIGN_TIMEOUT = "yaw_align_timeout";
inline constexpr const char * KEY_TRAJECTORY_TIMEOUT = "trajectory_timeout";
inline constexpr const char * KEY_PATH_TIMEOUT = "path_timeout";

inline constexpr const char * KEY_DESIRED_LINEAR_VEL = "pure_pursuit.desired_linear_vel";
inline constexpr const char * KEY_YAW_TOLERANCE = "pure_pursuit.yaw_tolerance";
inline constexpr const char * KEY_LOOKAHEAD_TIME = "pure_pursuit.lookahead_time";
inline constexpr const char * KEY_MIN_LOOKAHEAD_DIST = "pure_pursuit.min_lookahead_dist";

#ifdef ELTANIN_WITH_MPC
inline constexpr const char * KEY_MPC_PREDICTION_HORIZON = "mpc.prediction_horizon";
inline constexpr const char * KEY_MPC_PREDICTION_DT = "mpc.prediction_dt";
inline constexpr const char * KEY_MPC_MAX_LINEAR_VEL = "mpc.max_linear_vel";
inline constexpr const char * KEY_MPC_MIN_LINEAR_VEL = "mpc.min_linear_vel";
inline constexpr const char * KEY_MPC_MAX_LINEAR_ACCEL = "mpc.max_linear_accel";
inline constexpr const char * KEY_MPC_MAX_ANGULAR_ACCEL = "mpc.max_angular_accel";
inline constexpr const char * KEY_MPC_WEIGHT_LATERAL = "mpc.weight_lateral";
inline constexpr const char * KEY_MPC_WEIGHT_LONGITUDINAL = "mpc.weight_longitudinal";
inline constexpr const char * KEY_MPC_WEIGHT_YAW = "mpc.weight_yaw";
inline constexpr const char * KEY_MPC_WEIGHT_LINEAR_VEL = "mpc.weight_linear_vel";
inline constexpr const char * KEY_MPC_WEIGHT_ANGULAR_VEL = "mpc.weight_angular_vel";
inline constexpr const char * KEY_MPC_WEIGHT_LINEAR_VEL_RATE = "mpc.weight_linear_vel_rate";
inline constexpr const char * KEY_MPC_WEIGHT_ANGULAR_VEL_RATE = "mpc.weight_angular_vel_rate";
inline constexpr const char * KEY_MPC_TERMINAL_WEIGHT_SCALE = "mpc.terminal_weight_scale";
inline constexpr const char * KEY_MPC_YAW_TOLERANCE = "mpc.yaw_tolerance";
inline constexpr const char * KEY_MPC_MAX_HEADING_ERROR = "mpc.max_heading_error";
inline constexpr const char * KEY_MPC_MAX_CONSECUTIVE_FAILURES = "mpc.max_consecutive_failures";
inline constexpr const char * KEY_MPC_SOLVER_MAX_ITERATIONS = "mpc.solver.max_iterations";
inline constexpr const char * KEY_MPC_SOLVER_EPS_ABS = "mpc.solver.eps_abs";
inline constexpr const char * KEY_MPC_SOLVER_EPS_REL = "mpc.solver.eps_rel";
inline constexpr const char * KEY_MPC_SOLVER_WARM_START = "mpc.solver.warm_start";
inline constexpr const char * KEY_MPC_SOLVER_POLISH = "mpc.solver.polish";
#endif

enum class PathSource { Path, Trajectory, DirectedPath };

const char * name_of(PathSource source) noexcept;

std::optional<PathSource> to_path_source(std::string_view name) noexcept;

/// False when this build of eltanin was configured without ELTANIN_ENABLE_MPC.
bool mpc_is_available() noexcept;

struct FollowerParameters
{
  double update_frequency{20.0};
  PathSource path_source{PathSource::Path};
  eltanin::control::FollowerFactoryParams follower{};
  eltanin::control::GoalApproachParams approach{};
  double trajectory_timeout{0.5};
  /// 0 means no deadline: a global path is published once per replan, not periodically.
  double path_timeout{0.0};
};

struct VelocityClamp
{
  bool clamped{false};
  const char * key{""};
  double requested{0.0};
  double applied{0.0};
};

/// The cruise speed and, for a follower that can reverse, the reverse floor.
using VelocityClamps = std::array<VelocityClamp, 2>;

/// Spreads robot.max_angular_vel over the approach and the followers, and caps both speed bounds.
VelocityClamps apply_velocity_limits(
  FollowerParameters & parameters, const eltanin_ros_common::VelocityLimits & limits);

/// The first violated condition only, for the selected follower and the goal approach.
eltanin_ros_common::ConversionStatus validate(const FollowerParameters & parameters);

}  // namespace eltanin_controller

#endif  // ELTANIN_CONTROLLER__FOLLOWER_PARAMETERS_HPP_
