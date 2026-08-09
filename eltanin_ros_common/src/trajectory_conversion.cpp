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

#include "eltanin_ros_common/trajectory_conversion.hpp"

#include "src/diagnostic.hpp"

#include <eltanin/core/angle.hpp>

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
  return diagnostic::rejected(
    "Trajectory2D (frame_id='" + frame_id + "', " + std::to_string(size) + " points) at index " +
      std::to_string(index),
    violation);
}

}  // namespace

ConversionResult<eltanin::Path> to_path(const eltanin_msgs::msg::Trajectory2D & msg)
{
  std::vector<eltanin::Pose2D> poses;
  poses.reserve(msg.points.size());
  for (std::size_t i = 0; i < msg.points.size(); ++i) {
    const eltanin_msgs::msg::TrajectoryPoint2D & point = msg.points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.theta)) {
      return ConversionResult<eltanin::Path>::failure(reject(
        msg.header.frame_id, msg.points.size(), i,
        "point (x=" + std::to_string(point.x) + ", y=" + std::to_string(point.y) +
          ", theta=" + std::to_string(point.theta) + ") must be finite"));
    }
    poses.push_back(
      eltanin::Pose2D{Eigen::Vector2d{point.x, point.y}, eltanin::normalize_angle(point.theta)});
  }
  return ConversionResult<eltanin::Path>::success(eltanin::Path(std::move(poses)));
}

}  // namespace eltanin_ros_common
