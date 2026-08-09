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

#include <eltanin/control/goal_approach.hpp>
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin_ros_common/conversion_result.hpp>
#include <eltanin_ros_common/robot_profile.hpp>

#include <optional>
#include <string_view>

namespace eltanin_controller
{

inline constexpr const char * KEY_UPDATE_FREQUENCY = "update_frequency";
inline constexpr const char * KEY_PATH_SOURCE = "path_source";
inline constexpr const char * KEY_DESIRED_LINEAR_VEL = "desired_linear_vel";
inline constexpr const char * KEY_YAW_TOLERANCE = "yaw_tolerance";
inline constexpr const char * KEY_LOOKAHEAD_TIME = "lookahead_time";
inline constexpr const char * KEY_MIN_LOOKAHEAD_DIST = "min_lookahead_dist";
inline constexpr const char * KEY_XY_GOAL_TOLERANCE = "xy_goal_tolerance";
inline constexpr const char * KEY_YAW_GOAL_TOLERANCE = "yaw_goal_tolerance";
inline constexpr const char * KEY_APPROACH_DISTANCE = "approach_distance";
inline constexpr const char * KEY_APPROACH_DECEL = "approach_decel";
inline constexpr const char * KEY_YAW_ALIGN_TIMEOUT = "yaw_align_timeout";
inline constexpr const char * KEY_TRAJECTORY_TIMEOUT = "trajectory_timeout";
inline constexpr const char * KEY_PATH_TIMEOUT = "path_timeout";

/// Which of the two inputs is subscribed; exactly one subscription is ever created.
enum class PathSource { Path, Trajectory };

/// The parameter spelling of a PathSource.
const char * name_of(PathSource source) noexcept;

/// The inverse; nullopt for a name nobody defined.
std::optional<PathSource> to_path_source(std::string_view name) noexcept;

/// eltanin's own parameter structs are held by value, so the two sets of defaults cannot drift.
struct FollowerParameters
{
  double update_frequency{20.0};
  PathSource path_source{PathSource::Path};
  eltanin::control::PurePursuitParams pursuit{};
  eltanin::control::GoalApproachParams approach{};
  double trajectory_timeout{0.5};
  /// 0 means no deadline: a global path is published once per replan, not periodically.
  double path_timeout{0.0};
};

/// What the profile's limits did to the requested cruise speed; never a rejection.
struct VelocityClamp
{
  bool clamped{false};
  double requested{0.0};
  double applied{0.0};
};

/// Feeds robot.max_angular_vel into both generators and caps desired_linear_vel at the profile.
VelocityClamp apply_velocity_limits(
  FollowerParameters & parameters, const eltanin_ros_common::VelocityLimits & limits);

/// The first violated condition only, in the order the two create()s check them.
eltanin_ros_common::ConversionStatus validate(const FollowerParameters & parameters);

}  // namespace eltanin_controller

#endif  // ELTANIN_CONTROLLER__FOLLOWER_PARAMETERS_HPP_
