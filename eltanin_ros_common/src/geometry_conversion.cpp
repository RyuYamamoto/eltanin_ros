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

#include "eltanin_ros_common/geometry_conversion.hpp"

#include <eltanin/core/angle.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>

#include <cassert>
#include <cmath>
#include <string>

namespace eltanin_ros_common
{

namespace
{

/// tf2 only declares fromMsg() for geometry_msgs; its definition lives in tf2_geometry_msgs.
tf2::Quaternion to_tf2(const geometry_msgs::msg::Quaternion & msg)
{
  return tf2::Quaternion(msg.x, msg.y, msg.z, msg.w);
}

std::string describe(const geometry_msgs::msg::Quaternion & msg)
{
  return "quaternion (x=" + std::to_string(msg.x) + ", y=" + std::to_string(msg.y) +
         ", z=" + std::to_string(msg.z) + ", w=" + std::to_string(msg.w) + ")";
}

/// Rejects what tf2::getYaw() would otherwise accept in silence, starting with the zero quaternion.
ConversionStatus check_quaternion(const geometry_msgs::msg::Quaternion & msg)
{
  if (
    !std::isfinite(msg.x) || !std::isfinite(msg.y) || !std::isfinite(msg.z) ||
    !std::isfinite(msg.w)) {
    return ConversionStatus::failure(
      "eltanin_ros_common: rejected " + describe(msg) + ": components must be finite");
  }
  const double norm = std::sqrt(msg.x * msg.x + msg.y * msg.y + msg.z * msg.z + msg.w * msg.w);
  if (std::abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE) {
    return ConversionStatus::failure(
      "eltanin_ros_common: rejected " + describe(msg) + ": norm is " + std::to_string(norm) +
      ", tolerance around 1 is " + std::to_string(QUATERNION_NORM_TOLERANCE));
  }
  return ConversionStatus::success();
}

}  // namespace

ConversionResult<double> to_yaw(const geometry_msgs::msg::Quaternion & msg)
{
  const ConversionStatus status = check_quaternion(msg);
  if (!status.ok()) {
    return ConversionResult<double>::failure(status.message());
  }
  return ConversionResult<double>::success(eltanin::normalize_angle(tf2::getYaw(to_tf2(msg))));
}

ConversionResult<RollPitch> to_roll_pitch(const geometry_msgs::msg::Quaternion & msg)
{
  const ConversionStatus status = check_quaternion(msg);
  if (!status.ok()) {
    return ConversionResult<RollPitch>::failure(status.message());
  }
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  tf2::getEulerYPR(to_tf2(msg), yaw, pitch, roll);
  return ConversionResult<RollPitch>::success(
    RollPitch{eltanin::normalize_angle(roll), eltanin::normalize_angle(pitch)});
}

geometry_msgs::msg::Quaternion to_quaternion(double yaw)
{
  assert(std::isfinite(yaw));
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  msg.w = quaternion.w();
  return msg;
}

ConversionResult<eltanin::Pose2D> to_pose2d(const geometry_msgs::msg::Pose & msg)
{
  if (!std::isfinite(msg.position.x) || !std::isfinite(msg.position.y)) {
    return ConversionResult<eltanin::Pose2D>::failure(
      "eltanin_ros_common: rejected pose position (x=" + std::to_string(msg.position.x) +
      ", y=" + std::to_string(msg.position.y) + "): both must be finite");
  }
  const ConversionResult<double> yaw = to_yaw(msg.orientation);
  if (!yaw.ok()) {
    return ConversionResult<eltanin::Pose2D>::failure(yaw.error());
  }
  return ConversionResult<eltanin::Pose2D>::success(
    eltanin::Pose2D{Eigen::Vector2d{msg.position.x, msg.position.y}, yaw.value()});
}

geometry_msgs::msg::Pose to_pose_msg(const eltanin::Pose2D & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.position.x();
  msg.position.y = pose.position.y();
  msg.position.z = 0.0;
  msg.orientation = to_quaternion(pose.yaw);
  return msg;
}

ConversionResult<eltanin::Twist2D> to_twist2d(const geometry_msgs::msg::Twist & msg)
{
  if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.angular.z)) {
    return ConversionResult<eltanin::Twist2D>::failure(
      "eltanin_ros_common: rejected twist (linear.x=" + std::to_string(msg.linear.x) +
      ", angular.z=" + std::to_string(msg.angular.z) + "): both must be finite");
  }
  return ConversionResult<eltanin::Twist2D>::success(
    eltanin::Twist2D{Eigen::Vector2d{msg.linear.x, 0.0}, msg.angular.z});
}

geometry_msgs::msg::Twist to_twist_msg(const eltanin::Twist2D & twist)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = twist.linear.x();
  msg.angular.z = twist.angular;
  return msg;
}

ConversionResult<Transform2DConversion> to_transform2d(
  const geometry_msgs::msg::TransformStamped & msg, const PlanarityTolerance & tolerance)
{
  const auto & translation = msg.transform.translation;
  if (
    !std::isfinite(translation.x) || !std::isfinite(translation.y) ||
    !std::isfinite(translation.z)) {
    return ConversionResult<Transform2DConversion>::failure(
      "eltanin_ros_common: rejected transform " + msg.header.frame_id + " to " +
      msg.child_frame_id + " translation (x=" + std::to_string(translation.x) +
      ", y=" + std::to_string(translation.y) + ", z=" + std::to_string(translation.z) +
      "): all three must be finite");
  }
  const ConversionResult<double> yaw = to_yaw(msg.transform.rotation);
  if (!yaw.ok()) {
    return ConversionResult<Transform2DConversion>::failure(yaw.error());
  }
  const ConversionResult<RollPitch> roll_pitch = to_roll_pitch(msg.transform.rotation);
  if (!roll_pitch.ok()) {
    return ConversionResult<Transform2DConversion>::failure(roll_pitch.error());
  }

  PlanarityDeviation deviation;
  deviation.z = translation.z;
  deviation.roll = roll_pitch.value().roll;
  deviation.pitch = roll_pitch.value().pitch;
  deviation.within_tolerance = std::abs(deviation.z) <= tolerance.z &&
                               std::abs(deviation.roll) <= tolerance.roll &&
                               std::abs(deviation.pitch) <= tolerance.pitch;

  return ConversionResult<Transform2DConversion>::success(Transform2DConversion{
    eltanin::Transform2D{Eigen::Vector2d{translation.x, translation.y}, yaw.value()}, deviation});
}

}  // namespace eltanin_ros_common
