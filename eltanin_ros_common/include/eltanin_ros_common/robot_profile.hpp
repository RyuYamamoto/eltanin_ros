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

#ifndef ELTANIN_ROS_COMMON__ROBOT_PROFILE_HPP_
#define ELTANIN_ROS_COMMON__ROBOT_PROFILE_HPP_

#include "eltanin_ros_common/conversion_result.hpp"

#include <eltanin/core/footprint.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/map/cost_model.hpp>
#include <rclcpp/node.hpp>

#include <string>

namespace eltanin_ros_common
{

/// Frame names read from frames.*, so no node writes "map" or "base_footprint" into its own code.
struct FrameIds
{
  std::string map;
  std::string odom;
  std::string base;
};

/// Absolute magnitude limits; forward and reverse share the linear cap, left and right the angular.
struct VelocityLimits
{
  /// Cap on the magnitude of the linear velocity [m/s], applied in both directions.
  double max_linear_vel{0.0};

  /// Cap on the magnitude of the angular velocity [rad/s], applied to both turn directions.
  double max_angular_vel{0.0};

  /// Positive acceleration magnitude [m/s^2].
  double max_accel{0.0};

  /// Positive deceleration magnitude [m/s^2]; a negative value is rejected, not negated.
  double max_decel{0.0};
};

/// A validated robot profile; declare_robot_profile() is the only thing that can build one.
class RobotProfile
{
public:
  /// Counter-clockwise footprint in the base frame, normalized after validation.
  const eltanin::Polygon2D & footprint() const noexcept { return footprint_; }

  /// inscribed <= circumscribed <= inflation, computed by eltanin and never recomputed here.
  const eltanin::CollisionRadii & radii() const noexcept { return inflation_.radii(); }

  /// Pass this to InflationLayer and read circumscribed_cost() from it; do not build a second one.
  const eltanin::map::InflationCostModel & inflation_cost_model() const noexcept
  {
    return inflation_;
  }

  double cost_scaling_factor() const noexcept { return cost_scaling_factor_; }

  const VelocityLimits & limits() const noexcept { return limits_; }

  const FrameIds & frames() const noexcept { return frames_; }

private:
  RobotProfile(
    eltanin::Polygon2D footprint, eltanin::map::InflationCostModel inflation,
    double cost_scaling_factor, VelocityLimits limits, FrameIds frames);

  friend ConversionResult<RobotProfile> declare_robot_profile(rclcpp::Node & node);

  eltanin::Polygon2D footprint_;
  eltanin::map::InflationCostModel inflation_;
  double cost_scaling_factor_{0.0};
  VelocityLimits limits_{};
  FrameIds frames_{};
};

/// Declares robot.* and frames.* with defaults, reads them and validates them; check ok() first.
ConversionResult<RobotProfile> declare_robot_profile(rclcpp::Node & node);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__ROBOT_PROFILE_HPP_
