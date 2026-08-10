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

#ifndef ELTANIN_ROS_COMMON__GEOMETRY_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__GEOMETRY_CONVERSION_HPP_

#include "eltanin_ros_common/conversion_result.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/core/types.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>

namespace eltanin_ros_common
{

/// Wide enough for float32 round-off, narrow enough to reject a zero or unnormalized quaternion.
inline constexpr double QUATERNION_NORM_TOLERANCE = 1e-3;

/// The only way to get a yaw out of a quaternion here; tf2::getYaw() is never called unchecked.
ConversionResult<double> to_yaw(const geometry_msgs::msg::Quaternion & msg);

/// Rotation this library cannot represent, kept as an observable rather than dropped.
struct RollPitch
{
  double roll{0.0};
  double pitch{0.0};
};

/// Same validation as to_yaw; tf2::getYaw() reports 0 for a rotation that is pure roll or pitch.
ConversionResult<RollPitch> to_roll_pitch(const geometry_msgs::msg::Quaternion & msg);

/// Built with setRPY(0, 0, yaw); note that setEuler takes yaw first. Precondition: yaw is finite.
geometry_msgs::msg::Quaternion to_quaternion(double yaw);

ConversionResult<eltanin::Pose2D> to_pose2d(const geometry_msgs::msg::Pose & msg);

/// position.z is 0: a Pose2D has no third dimension to write.
geometry_msgs::msg::Pose to_pose_msg(const eltanin::Pose2D & pose);

/// Only the fields that are read are validated, so a NaN in a discarded field is not a rejection.
ConversionResult<eltanin::Twist2D> to_twist2d(const geometry_msgs::msg::Twist & msg);

/// linear.y() is discarded: the robot is differential drive and cannot execute it.
geometry_msgs::msg::Twist to_twist_msg(const eltanin::Twist2D & twist);

/// How far out of plane a transform may be before the node should say so.
struct PlanarityTolerance
{
  double z{0.05};
  double roll{0.05};
  double pitch{0.05};
};

/// The out-of-plane part of a transform, which Transform2D drops.
struct PlanarityDeviation
{
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  bool within_tolerance{true};
};

struct Transform2DConversion
{
  eltanin::Transform2D transform;
  PlanarityDeviation deviation;
};

/// Succeeds even when out of plane: a tilted tf is a warning, not a reason to stop navigating.
ConversionResult<Transform2DConversion> to_transform2d(
  const geometry_msgs::msg::TransformStamped & msg, const PlanarityTolerance & tolerance = {});

/// Vertices in order, z = 0 and no closing repeat; RViz closes a PolygonStamped by itself.
geometry_msgs::msg::PolygonStamped to_polygon_msg(
  const eltanin::Polygon2D & polygon, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__GEOMETRY_CONVERSION_HPP_
