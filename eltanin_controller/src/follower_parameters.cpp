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

#include "eltanin_controller/follower_parameters.hpp"

#include "src/diagnostic.hpp"

#include <cmath>
#include <numbers>
#include <string>

namespace eltanin_controller
{

namespace
{

using eltanin_ros_common::ConversionStatus;

/// The name the angular limit carries in the profile; this node declares no key of its own for it.
constexpr const char * ANGULAR_LIMIT = "robot.max_angular_vel";

/// A period longer than an hour is a timer that never fires again in any run worth watching.
constexpr double LONGEST_PERIOD = 3600.0;

ConversionStatus require_finite(const char * key, double value)
{
  if (std::isfinite(value)) {
    return ConversionStatus::success();
  }
  return ConversionStatus::failure(
    diagnostic::rejected(key, "is " + std::to_string(value) + ", which must be finite"));
}

ConversionStatus require_positive(const char * key, double value)
{
  const ConversionStatus finite = require_finite(key, value);
  if (!finite.ok()) {
    return finite;
  }
  if (value <= 0.0) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must be greater than 0"));
  }
  return ConversionStatus::success();
}

ConversionStatus require_non_negative(const char * key, double value)
{
  const ConversionStatus finite = require_finite(key, value);
  if (!finite.ok()) {
    return finite;
  }
  if (value < 0.0) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must not be negative"));
  }
  return ConversionStatus::success();
}

/// eltanin refuses an angle at pi as well as below zero; a tolerance of half a turn is not one.
ConversionStatus require_angle_below_half_turn(const char * key, double value)
{
  const ConversionStatus positive = require_positive(key, value);
  if (!positive.ok()) {
    return positive;
  }
  if (value >= std::numbers::pi) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must be below pi"));
  }
  return ConversionStatus::success();
}

/// The conditions PurePursuit::create() returns nullopt on, in the order it checks them.
ConversionStatus validate_pursuit(const eltanin::control::PurePursuitParams & pursuit)
{
  const ConversionStatus linear =
    require_positive(KEY_DESIRED_LINEAR_VEL, pursuit.desired_linear_vel);
  if (!linear.ok()) {
    return linear;
  }
  const ConversionStatus angular = require_positive(ANGULAR_LIMIT, pursuit.max_angular_vel);
  if (!angular.ok()) {
    return angular;
  }
  const ConversionStatus yaw =
    require_angle_below_half_turn(KEY_YAW_TOLERANCE, pursuit.yaw_tolerance);
  if (!yaw.ok()) {
    return yaw;
  }
  const ConversionStatus time = require_non_negative(KEY_LOOKAHEAD_TIME, pursuit.lookahead_time);
  if (!time.ok()) {
    return time;
  }
  return require_positive(KEY_MIN_LOOKAHEAD_DIST, pursuit.min_lookahead_dist);
}

/// The conditions GoalApproach::create() returns nullopt on, in the order it checks them.
ConversionStatus validate_approach(const eltanin::control::GoalApproachParams & approach)
{
  const ConversionStatus xy = require_positive(KEY_XY_GOAL_TOLERANCE, approach.xy_goal_tolerance);
  if (!xy.ok()) {
    return xy;
  }
  const ConversionStatus distance =
    require_positive(KEY_APPROACH_DISTANCE, approach.approach_distance);
  if (!distance.ok()) {
    return distance;
  }
  const ConversionStatus decel = require_positive(KEY_APPROACH_DECEL, approach.approach_decel);
  if (!decel.ok()) {
    return decel;
  }
  const ConversionStatus timeout =
    require_positive(KEY_YAW_ALIGN_TIMEOUT, approach.yaw_align_timeout);
  if (!timeout.ok()) {
    return timeout;
  }
  const ConversionStatus angular = require_positive(ANGULAR_LIMIT, approach.max_angular_vel);
  if (!angular.ok()) {
    return angular;
  }
  const ConversionStatus yaw =
    require_angle_below_half_turn(KEY_YAW_GOAL_TOLERANCE, approach.yaw_goal_tolerance);
  if (!yaw.ok()) {
    return yaw;
  }
  if (approach.approach_distance < approach.xy_goal_tolerance) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_APPROACH_DISTANCE, "is " + std::to_string(approach.approach_distance) +
                               ", which is inside " + std::string(KEY_XY_GOAL_TOLERANCE) + " " +
                               std::to_string(approach.xy_goal_tolerance) +
                               " and could never be left"));
  }
  return ConversionStatus::success();
}

}  // namespace

const char * name_of(PathSource source) noexcept
{
  return source == PathSource::Trajectory ? "trajectory" : "path";
}

std::optional<PathSource> to_path_source(std::string_view name) noexcept
{
  if (name == "path") {
    return PathSource::Path;
  }
  if (name == "trajectory") {
    return PathSource::Trajectory;
  }
  return std::nullopt;
}

VelocityClamp apply_velocity_limits(
  FollowerParameters & parameters, const eltanin_ros_common::VelocityLimits & limits)
{
  parameters.pursuit.max_angular_vel = limits.max_angular_vel;
  parameters.approach.max_angular_vel = limits.max_angular_vel;

  VelocityClamp clamp;
  clamp.requested = parameters.pursuit.desired_linear_vel;
  clamp.applied = clamp.requested;
  if (std::isfinite(clamp.requested) && clamp.requested > limits.max_linear_vel) {
    clamp.clamped = true;
    clamp.applied = limits.max_linear_vel;
    parameters.pursuit.desired_linear_vel = limits.max_linear_vel;
  }
  return clamp;
}

ConversionStatus validate(const FollowerParameters & parameters)
{
  const ConversionStatus frequency =
    require_positive(KEY_UPDATE_FREQUENCY, parameters.update_frequency);
  if (!frequency.ok()) {
    return frequency;
  }
  if (1.0 / parameters.update_frequency > LONGEST_PERIOD) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_UPDATE_FREQUENCY, "is " + std::to_string(parameters.update_frequency) +
                              " Hz, a period longer than " + std::to_string(LONGEST_PERIOD) +
                              " s"));
  }
  if (parameters.path_source == PathSource::Trajectory) {
    const ConversionStatus timeout =
      require_positive(KEY_TRAJECTORY_TIMEOUT, parameters.trajectory_timeout);
    if (!timeout.ok()) {
      return timeout;
    }
  }
  const ConversionStatus deadline = require_non_negative(KEY_PATH_TIMEOUT, parameters.path_timeout);
  if (!deadline.ok()) {
    return deadline;
  }
  const ConversionStatus pursuit = validate_pursuit(parameters.pursuit);
  if (!pursuit.ok()) {
    return pursuit;
  }
  return validate_approach(parameters.approach);
}

}  // namespace eltanin_controller
