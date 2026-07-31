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

#include "eltanin_ros_common/path_conversion.hpp"

#include "eltanin_ros_common/geometry_conversion.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace eltanin_ros_common
{

namespace
{

std::string reject(
  const std::string & frame_id, std::size_t size, std::size_t index, const std::string & violation)
{
  return "eltanin_ros_common: rejected Path (frame_id='" + frame_id + "', " + std::to_string(size) +
         " poses) at index " + std::to_string(index) + ": " + violation;
}

}  // namespace

ConversionResult<nav_msgs::msg::Path> to_path_msg(
  const eltanin::Path & path, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.poses.resize(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    const eltanin::Pose2D & pose = path[i];
    if (
      !std::isfinite(pose.position.x()) || !std::isfinite(pose.position.y()) ||
      !std::isfinite(pose.yaw)) {
      return ConversionResult<nav_msgs::msg::Path>::failure(reject(
        frame_id, path.size(), i,
        "pose (x=" + std::to_string(pose.position.x()) +
          ", y=" + std::to_string(pose.position.y()) + ", yaw=" + std::to_string(pose.yaw) +
          ") must be finite"));
    }
    msg.poses[i].header = msg.header;
    msg.poses[i].pose = to_pose_msg(pose);
  }
  return ConversionResult<nav_msgs::msg::Path>::success(std::move(msg));
}

ConversionResult<eltanin::Path> to_path(const nav_msgs::msg::Path & msg)
{
  std::vector<eltanin::Pose2D> poses;
  poses.reserve(msg.poses.size());
  for (std::size_t i = 0; i < msg.poses.size(); ++i) {
    const geometry_msgs::msg::PoseStamped & stamped = msg.poses[i];
    if (!stamped.header.frame_id.empty() && stamped.header.frame_id != msg.header.frame_id) {
      return ConversionResult<eltanin::Path>::failure(reject(
        msg.header.frame_id, msg.poses.size(), i,
        "frame_id is '" + stamped.header.frame_id +
          "', which is neither empty nor the path frame"));
    }
    const ConversionResult<eltanin::Pose2D> pose = to_pose2d(stamped.pose);
    if (!pose.ok()) {
      return ConversionResult<eltanin::Path>::failure(
        reject(msg.header.frame_id, msg.poses.size(), i, pose.error()));
    }
    poses.push_back(pose.value());
  }
  return ConversionResult<eltanin::Path>::success(eltanin::Path(std::move(poses)));
}

}  // namespace eltanin_ros_common
