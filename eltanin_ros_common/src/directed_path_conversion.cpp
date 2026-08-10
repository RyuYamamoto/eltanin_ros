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

#include "eltanin_ros_common/directed_path_conversion.hpp"

#include "eltanin_ros_common/geometry_conversion.hpp"
#include "src/diagnostic.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace eltanin_ros_common
{

namespace
{

using DirectedPath = eltanin_msgs::msg::DirectedPath;

std::string reject(
  const std::string & frame_id, std::size_t size, std::size_t index, const std::string & violation)
{
  return diagnostic::rejected(
    "DirectedPath (frame_id='" + frame_id + "', " + std::to_string(size) + " poses) at index " +
      std::to_string(index),
    violation);
}

std::string reject_shape(
  const std::string & frame_id, std::size_t poses, const std::string & violation)
{
  return diagnostic::rejected(
    "DirectedPath (frame_id='" + frame_id + "', " + std::to_string(poses) + " poses)", violation);
}

}  // namespace

std::uint8_t to_direction_msg(eltanin::Direction direction) noexcept
{
  switch (direction) {
    case eltanin::Direction::Reverse:
      return DirectedPath::DIRECTION_REVERSE;
    case eltanin::Direction::InPlace:
      return DirectedPath::DIRECTION_IN_PLACE;
    case eltanin::Direction::Forward:
      break;
  }
  return DirectedPath::DIRECTION_FORWARD;
}

ConversionResult<eltanin::Direction> to_direction(std::uint8_t value)
{
  switch (value) {
    case DirectedPath::DIRECTION_FORWARD:
      return ConversionResult<eltanin::Direction>::success(eltanin::Direction::Forward);
    case DirectedPath::DIRECTION_REVERSE:
      return ConversionResult<eltanin::Direction>::success(eltanin::Direction::Reverse);
    case DirectedPath::DIRECTION_IN_PLACE:
      return ConversionResult<eltanin::Direction>::success(eltanin::Direction::InPlace);
    default:
      break;
  }
  return ConversionResult<eltanin::Direction>::failure(diagnostic::rejected(
    "direction " + std::to_string(static_cast<int>(value)),
    "must be one of DIRECTION_FORWARD, DIRECTION_REVERSE, DIRECTION_IN_PLACE"));
}

ConversionResult<DirectedPath> to_directed_path_msg(
  const eltanin::Path & path, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  DirectedPath msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.poses.resize(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    const eltanin::Pose2D & pose = path[i];
    if (
      !std::isfinite(pose.position.x()) || !std::isfinite(pose.position.y()) ||
      !std::isfinite(pose.yaw)) {
      return ConversionResult<DirectedPath>::failure(reject(
        frame_id, path.size(), i,
        "pose (x=" + std::to_string(pose.position.x()) +
          ", y=" + std::to_string(pose.position.y()) + ", yaw=" + std::to_string(pose.yaw) +
          ") must be finite"));
    }
    msg.poses[i] = to_pose_msg(pose);
  }
  msg.segment_directions.reserve(path.directions().size());
  for (const eltanin::Direction direction : path.directions()) {
    msg.segment_directions.push_back(to_direction_msg(direction));
  }
  return ConversionResult<DirectedPath>::success(std::move(msg));
}

ConversionResult<eltanin::Path> to_path(const DirectedPath & msg)
{
  if (!msg.segment_directions.empty() && msg.segment_directions.size() + 1 != msg.poses.size()) {
    return ConversionResult<eltanin::Path>::failure(reject_shape(
      msg.header.frame_id, msg.poses.size(),
      "segment_directions holds " + std::to_string(msg.segment_directions.size()) +
        " entries, which is neither 0 nor one fewer than the poses"));
  }

  std::vector<eltanin::Pose2D> poses;
  poses.reserve(msg.poses.size());
  for (std::size_t i = 0; i < msg.poses.size(); ++i) {
    const ConversionResult<eltanin::Pose2D> pose = to_pose2d(msg.poses[i]);
    if (!pose.ok()) {
      return ConversionResult<eltanin::Path>::failure(
        reject(msg.header.frame_id, msg.poses.size(), i, diagnostic::nested(pose.error())));
    }
    poses.push_back(pose.value());
  }

  std::vector<eltanin::Direction> directions;
  directions.reserve(msg.segment_directions.size());
  for (std::size_t i = 0; i < msg.segment_directions.size(); ++i) {
    const ConversionResult<eltanin::Direction> direction = to_direction(msg.segment_directions[i]);
    if (!direction.ok()) {
      return ConversionResult<eltanin::Path>::failure(
        reject(msg.header.frame_id, msg.poses.size(), i, diagnostic::nested(direction.error())));
    }
    directions.push_back(direction.value());
  }
  return ConversionResult<eltanin::Path>::success(
    eltanin::Path(std::move(poses), std::move(directions)));
}

}  // namespace eltanin_ros_common
