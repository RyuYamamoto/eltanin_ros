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

#include "eltanin_controller/limiter_parameters.hpp"

#include "src/diagnostic.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace eltanin_controller::limiter
{

namespace
{

using eltanin_ros_common::ConversionStatus;

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

}  // namespace

std::vector<std::string> declared_keys()
{
  return {KEY_UPDATE_FREQUENCY,          KEY_CMD_TIMEOUT,           KEY_MAP_TIMEOUT,
          KEY_OUTPUT_ENABLED_ON_STARTUP, KEY_PREDICTION_STEPS,      KEY_REACTION_TIME,
          KEY_COLLISION_MARGIN,          KEY_EXACT_FOOTPRINT_CHECK, KEY_STOP_CLEARANCE,
          KEY_SLOW_DOWN_CLEARANCE,       KEY_MIN_PROXIMITY_SCALE,   KEY_RELEASE_TIME,
          KEY_CLEARANCE_MAX_DISTANCE};
}

ConversionStatus validate(
  const LimiterParameters & parameters, const eltanin::DistanceTraversabilityModel & model)
{
  const eltanin::collision::VelocityLimiterParams & limits = parameters.governor.limiter;

  const ConversionStatus rate = require_positive(KEY_UPDATE_FREQUENCY, parameters.update_frequency);
  if (!rate.ok()) {
    return rate;
  }
  // A non-positive timeout makes every get() stale, which would be a permanent zero command.
  const ConversionStatus command = require_positive(KEY_CMD_TIMEOUT, parameters.cmd_timeout);
  if (!command.ok()) {
    return command;
  }
  const ConversionStatus map = require_positive(KEY_MAP_TIMEOUT, parameters.map_timeout);
  if (!map.ok()) {
    return map;
  }
  if (limits.prediction_steps < 1) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_PREDICTION_STEPS,
      "is " + std::to_string(limits.prediction_steps) + ", which must be at least 1"));
  }
  const ConversionStatus reaction = require_positive(KEY_REACTION_TIME, limits.reaction_time);
  if (!reaction.ok()) {
    return reaction;
  }
  const ConversionStatus margin =
    require_non_negative(KEY_COLLISION_MARGIN, limits.collision_margin);
  if (!margin.ok()) {
    return margin;
  }
  const ConversionStatus stop = require_non_negative(KEY_STOP_CLEARANCE, limits.stop_clearance);
  if (!stop.ok()) {
    return stop;
  }
  const ConversionStatus slow = require_finite(KEY_SLOW_DOWN_CLEARANCE, limits.slow_down_clearance);
  if (!slow.ok()) {
    return slow;
  }
  if (limits.slow_down_clearance <= limits.stop_clearance) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_SLOW_DOWN_CLEARANCE, "is " + std::to_string(limits.slow_down_clearance) + ", not above " +
                                 std::string(KEY_STOP_CLEARANCE) + " " +
                                 std::to_string(limits.stop_clearance)));
  }
  // Zero would let the proximity ramp stop the robot, which makes a narrow corridor impassable.
  const ConversionStatus scale =
    require_positive(KEY_MIN_PROXIMITY_SCALE, limits.min_proximity_scale);
  if (!scale.ok()) {
    return scale;
  }
  if (limits.min_proximity_scale > 1.0) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_MIN_PROXIMITY_SCALE,
      "is " + std::to_string(limits.min_proximity_scale) + ", which must not exceed 1"));
  }
  const ConversionStatus release =
    require_positive(KEY_RELEASE_TIME, parameters.governor.release_time);
  if (!release.ok()) {
    return release;
  }
  const ConversionStatus saturation =
    require_positive(KEY_CLEARANCE_MAX_DISTANCE, parameters.distance_map.max_distance);
  if (!saturation.ok()) {
    return saturation;
  }
  // At or below the circumscribed radius the two-stage gate can never report Free again.
  if (parameters.distance_map.max_distance <= model.circumscribed_radius()) {
    return ConversionStatus::failure(diagnostic::rejected(
      KEY_CLEARANCE_MAX_DISTANCE, "is " + std::to_string(parameters.distance_map.max_distance) +
                                    ", not above the circumscribed radius " +
                                    std::to_string(model.circumscribed_radius()) +
                                    " m of robot.footprint"));
  }
  return require_positive(KEY_MAX_DECELERATION, limits.max_deceleration);
}

std::vector<std::string> warnings(
  const LimiterParameters & parameters, const eltanin_ros_common::RobotProfile & profile)
{
  const eltanin::collision::VelocityLimiterParams & limits = parameters.governor.limiter;
  std::vector<std::string> lines;

  if (parameters.update_frequency < MINIMUM_UPDATE_FREQUENCY) {
    lines.push_back(diagnostic::line(
      std::string(KEY_UPDATE_FREQUENCY) + " " + std::to_string(parameters.update_frequency) +
      " Hz is below the " + std::to_string(MINIMUM_UPDATE_FREQUENCY) +
      " Hz the driver watchdog needs"));
  }
  if (!limits.exact_footprint_check) {
    lines.push_back(diagnostic::line(
      std::string(KEY_EXACT_FOOTPRINT_CHECK) +
      " is false; the centre cell may short-circuit a footprint that overlaps an obstacle"));
  }

  const double speed = profile.limits().max_linear_vel;
  const double horizon = limits.reaction_time + speed / limits.max_deceleration;
  const double step_arc = speed * horizon / static_cast<double>(limits.prediction_steps);
  const double thickness = 2.0 * profile.distance_model().inscribed_radius();
  if (step_arc >= thickness) {
    lines.push_back(diagnostic::line(
      "one prediction step covers " + std::to_string(step_arc) + " m at " +
      std::string(KEY_MAX_LINEAR_VEL) + " " + std::to_string(speed) +
      " m/s, which is not below the footprint thickness " + std::to_string(thickness) +
      " m; raise " + std::string(KEY_PREDICTION_STEPS)));
  }
  return lines;
}

}  // namespace eltanin_controller::limiter
