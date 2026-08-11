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

using eltanin::control::FollowerType;
using eltanin_ros_common::ConversionStatus;

/// The profile key the angular bound comes from; this node declares none of its own.
constexpr const char * ANGULAR_LIMIT = "robot.max_angular_vel";

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

#ifdef ELTANIN_WITH_MPC
ConversionStatus require_positive_int(const char * key, int value)
{
  if (value <= 0) {
    return ConversionStatus::failure(
      diagnostic::rejected(key, "is " + std::to_string(value) + ", which must be greater than 0"));
  }
  return ConversionStatus::success();
}
#endif

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

#ifdef ELTANIN_WITH_MPC
ConversionStatus validate_mpc(const eltanin::control::MpcFollowerParams & mpc)
{
  const ConversionStatus horizon =
    require_positive_int(KEY_MPC_PREDICTION_HORIZON, mpc.prediction_horizon);
  if (!horizon.ok()) {
    return horizon;
  }
  const ConversionStatus step = require_positive(KEY_MPC_PREDICTION_DT, mpc.prediction_dt);
  if (!step.ok()) {
    return step;
  }
  const ConversionStatus fastest = require_positive(KEY_MPC_MAX_LINEAR_VEL, mpc.max_linear_vel);
  if (!fastest.ok()) {
    return fastest;
  }
  const ConversionStatus slowest = require_finite(KEY_MPC_MIN_LINEAR_VEL, mpc.min_linear_vel);
  if (!slowest.ok()) {
    return slowest;
  }
  if (mpc.min_linear_vel > mpc.max_linear_vel) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_MPC_MIN_LINEAR_VEL, "is " + std::to_string(mpc.min_linear_vel) + ", above " +
                                std::string(KEY_MPC_MAX_LINEAR_VEL) + " " +
                                std::to_string(mpc.max_linear_vel)));
  }
  // A negative value is what allows reversing; max_linear_vel bounds the magnitude either way.
  if (mpc.min_linear_vel < -mpc.max_linear_vel) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_MPC_MIN_LINEAR_VEL, "is " + std::to_string(mpc.min_linear_vel) + ", below minus " +
                                std::string(KEY_MPC_MAX_LINEAR_VEL) + " " +
                                std::to_string(mpc.max_linear_vel)));
  }
  const ConversionStatus angular = require_positive(ANGULAR_LIMIT, mpc.max_angular_vel);
  if (!angular.ok()) {
    return angular;
  }
  const ConversionStatus accel = require_positive(KEY_MPC_MAX_LINEAR_ACCEL, mpc.max_linear_accel);
  if (!accel.ok()) {
    return accel;
  }
  const ConversionStatus turn_accel =
    require_positive(KEY_MPC_MAX_ANGULAR_ACCEL, mpc.max_angular_accel);
  if (!turn_accel.ok()) {
    return turn_accel;
  }
  const ConversionStatus yaw =
    require_angle_below_half_turn(KEY_MPC_YAW_TOLERANCE, mpc.yaw_tolerance);
  if (!yaw.ok()) {
    return yaw;
  }
  const ConversionStatus heading =
    require_angle_below_half_turn(KEY_MPC_MAX_HEADING_ERROR, mpc.max_heading_error);
  if (!heading.ok()) {
    return heading;
  }
  if (mpc.yaw_tolerance > mpc.max_heading_error) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_MPC_YAW_TOLERANCE, "is " + std::to_string(mpc.yaw_tolerance) + ", above " +
                               std::string(KEY_MPC_MAX_HEADING_ERROR) + " " +
                               std::to_string(mpc.max_heading_error)));
  }
  if (mpc.max_consecutive_failures < 0) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_MPC_MAX_CONSECUTIVE_FAILURES,
      "is " + std::to_string(mpc.max_consecutive_failures) + ", which must not be negative"));
  }

  const ConversionStatus lateral = require_non_negative(KEY_MPC_WEIGHT_LATERAL, mpc.weight_lateral);
  if (!lateral.ok()) {
    return lateral;
  }
  const ConversionStatus longitudinal =
    require_non_negative(KEY_MPC_WEIGHT_LONGITUDINAL, mpc.weight_longitudinal);
  if (!longitudinal.ok()) {
    return longitudinal;
  }
  const ConversionStatus heading_weight = require_non_negative(KEY_MPC_WEIGHT_YAW, mpc.weight_yaw);
  if (!heading_weight.ok()) {
    return heading_weight;
  }
  const ConversionStatus terminal =
    require_positive(KEY_MPC_TERMINAL_WEIGHT_SCALE, mpc.terminal_weight_scale);
  if (!terminal.ok()) {
    return terminal;
  }
  // Strictly positive, or the QP has no unique minimiser.
  const ConversionStatus speed_weight =
    require_positive(KEY_MPC_WEIGHT_LINEAR_VEL, mpc.weight_linear_vel);
  if (!speed_weight.ok()) {
    return speed_weight;
  }
  const ConversionStatus turn_weight =
    require_positive(KEY_MPC_WEIGHT_ANGULAR_VEL, mpc.weight_angular_vel);
  if (!turn_weight.ok()) {
    return turn_weight;
  }
  const ConversionStatus speed_rate =
    require_non_negative(KEY_MPC_WEIGHT_LINEAR_VEL_RATE, mpc.weight_linear_vel_rate);
  if (!speed_rate.ok()) {
    return speed_rate;
  }
  const ConversionStatus turn_rate =
    require_non_negative(KEY_MPC_WEIGHT_ANGULAR_VEL_RATE, mpc.weight_angular_vel_rate);
  if (!turn_rate.ok()) {
    return turn_rate;
  }

  const ConversionStatus iterations =
    require_positive_int(KEY_MPC_SOLVER_MAX_ITERATIONS, mpc.solver.max_iterations);
  if (!iterations.ok()) {
    return iterations;
  }
  const ConversionStatus eps_abs = require_positive(KEY_MPC_SOLVER_EPS_ABS, mpc.solver.eps_abs);
  if (!eps_abs.ok()) {
    return eps_abs;
  }
  return require_positive(KEY_MPC_SOLVER_EPS_REL, mpc.solver.eps_rel);
}
#endif

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

ConversionStatus validate_follower(const eltanin::control::FollowerFactoryParams & follower)
{
  if (follower.type == FollowerType::Mpc) {
#ifdef ELTANIN_WITH_MPC
    return validate_mpc(follower.mpc);
#else
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_FOLLOWER_TYPE,
      "is 'mpc', which this build of eltanin does not contain: configure "
      "eltanin_vendor with ELTANIN_VENDOR_ENABLE_MPC=ON"));
#endif
  }
  return validate_pursuit(follower.pure_pursuit);
}

}  // namespace

const char * name_of(PathSource source) noexcept
{
  switch (source) {
    case PathSource::Trajectory:
      return "trajectory";
    case PathSource::DirectedPath:
      return "directed_path";
    case PathSource::Path:
      break;
  }
  return "path";
}

std::optional<PathSource> to_path_source(std::string_view name) noexcept
{
  if (name == "path") {
    return PathSource::Path;
  }
  if (name == "trajectory") {
    return PathSource::Trajectory;
  }
  if (name == "directed_path") {
    return PathSource::DirectedPath;
  }
  return std::nullopt;
}

bool mpc_is_available() noexcept
{
#ifdef ELTANIN_WITH_MPC
  return true;
#else
  return false;
#endif
}

VelocityClamps apply_velocity_limits(
  FollowerParameters & parameters, const eltanin_ros_common::VelocityLimits & limits)
{
  parameters.approach.max_angular_vel = limits.max_angular_vel;
  parameters.follower.pure_pursuit.max_angular_vel = limits.max_angular_vel;
#ifdef ELTANIN_WITH_MPC
  parameters.follower.mpc.max_angular_vel = limits.max_angular_vel;
#endif

  double * speed = &parameters.follower.pure_pursuit.desired_linear_vel;
  VelocityClamps clamps;
  clamps[0].key = KEY_DESIRED_LINEAR_VEL;
#ifdef ELTANIN_WITH_MPC
  if (parameters.follower.type == FollowerType::Mpc) {
    speed = &parameters.follower.mpc.max_linear_vel;
    clamps[0].key = KEY_MPC_MAX_LINEAR_VEL;
  }
#endif

  clamps[0].requested = *speed;
  clamps[0].applied = clamps[0].requested;
  if (std::isfinite(clamps[0].requested) && clamps[0].requested > limits.max_linear_vel) {
    clamps[0].clamped = true;
    clamps[0].applied = limits.max_linear_vel;
    *speed = limits.max_linear_vel;
  }

#ifdef ELTANIN_WITH_MPC
  // robot.max_linear_vel is a magnitude, so it bounds the reverse floor from below just as hard.
  if (parameters.follower.type == FollowerType::Mpc) {
    double & floor = parameters.follower.mpc.min_linear_vel;
    clamps[1].key = KEY_MPC_MIN_LINEAR_VEL;
    clamps[1].requested = floor;
    clamps[1].applied = floor;
    if (std::isfinite(floor) && floor < -limits.max_linear_vel) {
      clamps[1].clamped = true;
      clamps[1].applied = -limits.max_linear_vel;
      floor = -limits.max_linear_vel;
    }
  }
#endif
  return clamps;
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
  const ConversionStatus follower = validate_follower(parameters.follower);
  if (!follower.ok()) {
    return follower;
  }
  return validate_approach(parameters.approach);
}

}  // namespace eltanin_controller
