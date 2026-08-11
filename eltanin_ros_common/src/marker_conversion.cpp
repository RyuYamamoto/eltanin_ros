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

#include "eltanin_ros_common/marker_conversion.hpp"

#include "src/diagnostic.hpp"

#include <eltanin/core/types.hpp>

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace eltanin_ros_common
{

namespace
{

using visualization_msgs::msg::Marker;

/// The smallest polygon that has an inside; anything less cannot be a footprint.
constexpr std::size_t MINIMUM_VERTICES = 3;

std::string reject(const std::string & violation)
{
  return diagnostic::rejected("visualization_msgs/MarkerArray (footprint)", violation);
}

geometry_msgs::msg::Point to_point(const Eigen::Vector2d & position)
{
  geometry_msgs::msg::Point point;
  point.x = position.x();
  point.y = position.y();
  return point;
}

/// Cleans out the previous array, so a shorter path never leaves the old outlines behind.
Marker delete_all(const std::string & ns)
{
  Marker marker;
  marker.ns = ns;
  marker.action = Marker::DELETEALL;
  return marker;
}

bool is_finite(const eltanin::Pose2D & pose)
{
  return std::isfinite(pose.position.x()) && std::isfinite(pose.position.y()) &&
         std::isfinite(pose.yaw);
}

/// One closed outline of the footprint placed at pose, drawn as a LINE_STRIP in frame_id.
Marker outline_at(
  const eltanin::Pose2D & pose, const eltanin::Polygon2D & footprint, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp, const std::string & ns, int id, double line_width)
{
  const eltanin::Transform2D placement = eltanin::Transform2D::from_pose(pose);
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;
  marker.points.reserve(footprint.size() + 1);
  for (const Eigen::Vector2d & vertex : footprint) {
    marker.points.push_back(to_point(placement * vertex));
  }
  // Polygon2D is implicitly closed; a LINE_STRIP is not, so the first vertex is repeated.
  marker.points.push_back(to_point(placement * footprint[0]));
  return marker;
}

}  // namespace

ConversionResult<visualization_msgs::msg::MarkerArray> to_footprint_markers(
  const eltanin::Path & path, const eltanin::Polygon2D & footprint, int stride,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp, const std::string & ns,
  const FootprintMarkerStyle & style)
{
  using Result = ConversionResult<visualization_msgs::msg::MarkerArray>;
  if (footprint.size() < MINIMUM_VERTICES) {
    return Result::failure(reject(
      "the footprint has " + std::to_string(footprint.size()) + " vertices, at least " +
      std::to_string(MINIMUM_VERTICES) + " are needed"));
  }
  for (const Eigen::Vector2d & vertex : footprint) {
    if (!std::isfinite(vertex.x()) || !std::isfinite(vertex.y())) {
      return Result::failure(reject("the footprint has a vertex that is not finite"));
    }
  }
  if (stride < 1) {
    return Result::failure(
      reject("stride is " + std::to_string(stride) + ", which must be at least 1"));
  }
  if (!std::isfinite(style.line_width) || style.line_width <= 0.0) {
    return Result::failure(reject(
      "line_width is " + std::to_string(style.line_width) +
      ", which must be finite and greater than 0"));
  }

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(delete_all(ns));
  if (path.empty()) {
    return Result::success(std::move(markers));
  }

  const std::size_t last = path.size() - 1;
  const auto step = static_cast<std::size_t>(stride);
  std::vector<std::size_t> sampled;
  for (std::size_t at = 0; at < last; at += step) {
    sampled.push_back(at);
  }
  // The pose the robot is asked to end in is the one worth looking at, whatever the stride.
  sampled.push_back(last);

  for (std::size_t index = 0; index < sampled.size(); ++index) {
    const eltanin::Pose2D & pose = path[sampled[index]];
    if (!is_finite(pose)) {
      return Result::failure(
        reject("pose " + std::to_string(sampled[index]) + " of the path is not finite"));
    }
    const eltanin::Transform2D placement = eltanin::Transform2D::from_pose(pose);

    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = static_cast<int>(index);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    // The points are already in frame_id, so the marker pose has to be the identity.
    marker.pose.orientation.w = 1.0;
    marker.scale.x = style.line_width;
    marker.color.r = style.red;
    marker.color.g = style.green;
    marker.color.b = style.blue;
    marker.color.a = style.alpha;
    marker.points.reserve(footprint.size() + 1);
    for (const Eigen::Vector2d & vertex : footprint) {
      marker.points.push_back(to_point(placement * vertex));
    }
    // Polygon2D is implicitly closed; a LINE_STRIP is not, so the first vertex is repeated.
    marker.points.push_back(to_point(placement * footprint[0]));
    markers.markers.push_back(std::move(marker));
  }
  return Result::success(std::move(markers));
}

ConversionResult<visualization_msgs::msg::MarkerArray> to_predicted_footprint_markers(
  const eltanin::Path & path, const eltanin::Polygon2D & footprint,
  const std::vector<std_msgs::msg::ColorRGBA> & colors, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp, bool mark_contact, const std::string & ns,
  double line_width)
{
  using Result = ConversionResult<visualization_msgs::msg::MarkerArray>;
  if (footprint.size() < MINIMUM_VERTICES) {
    return Result::failure(reject(
      "the footprint has " + std::to_string(footprint.size()) + " vertices, at least " +
      std::to_string(MINIMUM_VERTICES) + " are needed"));
  }
  if (colors.size() != path.size()) {
    return Result::failure(reject(
      "there are " + std::to_string(colors.size()) + " colours for " + std::to_string(path.size()) +
      " poses; one per pose is needed"));
  }
  if (!std::isfinite(line_width) || line_width <= 0.0) {
    return Result::failure(reject(
      "line_width is " + std::to_string(line_width) + ", which must be finite and greater than 0"));
  }

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(delete_all(ns));
  if (path.empty()) {
    return Result::success(std::move(markers));
  }

  for (std::size_t index = 0; index < path.size(); ++index) {
    if (!is_finite(path[index])) {
      return Result::failure(
        reject("pose " + std::to_string(index) + " of the path is not finite"));
    }
    Marker marker =
      outline_at(path[index], footprint, frame_id, stamp, ns, static_cast<int>(index), line_width);
    marker.color = colors[index];
    markers.markers.push_back(std::move(marker));
  }

  // The last pose is where the rollout stopped, so a colliding one ends there; mark that point.
  if (mark_contact) {
    Marker contact;
    contact.header.frame_id = frame_id;
    contact.header.stamp = stamp;
    contact.ns = ns + "_contact";
    contact.id = 0;
    contact.type = Marker::SPHERE;
    contact.action = Marker::ADD;
    contact.pose.position = to_point(path[path.size() - 1].position);
    contact.pose.orientation.w = 1.0;
    contact.scale.x = 0.08;
    contact.scale.y = 0.08;
    contact.scale.z = 0.08;
    contact.color = colors[path.size() - 1];
    markers.markers.push_back(std::move(contact));
  } else {
    markers.markers.push_back(delete_all(ns + "_contact"));
  }
  return Result::success(std::move(markers));
}

}  // namespace eltanin_ros_common
