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

#ifndef ELTANIN_CONTROLLER__LIMITER_PARAMETERS_HPP_
#define ELTANIN_CONTROLLER__LIMITER_PARAMETERS_HPP_

#include <eltanin/collision/velocity_governor.hpp>
#include <eltanin/core/footprint.hpp>
#include <eltanin/map/distance_map.hpp>
#include <eltanin_ros_common/conversion_result.hpp>
#include <eltanin_ros_common/robot_profile.hpp>

#include <string>
#include <vector>

namespace eltanin_controller::limiter
{

inline constexpr const char * KEY_UPDATE_FREQUENCY = "update_frequency";
inline constexpr const char * KEY_CMD_TIMEOUT = "cmd_timeout";
inline constexpr const char * KEY_MAP_TIMEOUT = "map_timeout";
inline constexpr const char * KEY_OUTPUT_ENABLED_ON_STARTUP = "output_enabled_on_startup";
inline constexpr const char * KEY_PREDICTION_STEPS = "prediction_steps";
inline constexpr const char * KEY_REACTION_TIME = "reaction_time";
inline constexpr const char * KEY_COLLISION_MARGIN = "collision_margin";
inline constexpr const char * KEY_EXACT_FOOTPRINT_CHECK = "exact_footprint_check";
inline constexpr const char * KEY_STOP_CLEARANCE = "stop_clearance";
inline constexpr const char * KEY_SLOW_DOWN_CLEARANCE = "slow_down_clearance";
inline constexpr const char * KEY_MIN_PROXIMITY_SCALE = "min_proximity_scale";
inline constexpr const char * KEY_RELEASE_TIME = "release_time";
inline constexpr const char * KEY_CLEARANCE_MAX_DISTANCE = "clearance_max_distance";

/// The profile key the braking law reads; a violation names the profile, not a key of this node.
inline constexpr const char * KEY_MAX_DECELERATION = "robot.max_decel";

/// The profile key the pass-through check reads.
inline constexpr const char * KEY_MAX_LINEAR_VEL = "robot.max_linear_vel";

/// Below this the kachaka bridge watchdog of 0.3 s can fire between two cycles (C-15).
inline constexpr double MINIMUM_UPDATE_FREQUENCY = 4.0;

struct LimiterParameters
{
  double update_frequency{20.0};
  double cmd_timeout{0.3};
  double map_timeout{0.5};
  /// False on the robot; the sim configuration is the one that turns the output on at startup.
  bool output_enabled_on_startup{false};
  eltanin::map::DistanceMapParams distance_map{};
  eltanin::collision::VelocityGovernorParams governor{};
};

/// Every key this node declares, in the order the shipped configuration lists them.
std::vector<std::string> declared_keys();

/// The first violated condition only, in the order of the README; the footprint is the profile's.
eltanin_ros_common::ConversionStatus validate(
  const LimiterParameters & parameters, const eltanin::DistanceTraversabilityModel & model);

/// Configurations that degrade safety without being wrong; empty when there is nothing to say.
std::vector<std::string> warnings(
  const LimiterParameters & parameters, const eltanin_ros_common::RobotProfile & profile);

}  // namespace eltanin_controller::limiter

#endif  // ELTANIN_CONTROLLER__LIMITER_PARAMETERS_HPP_
