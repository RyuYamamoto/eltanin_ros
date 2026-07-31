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

#include "eltanin_ros_common/map_conversion.hpp"

#include "eltanin_ros_common/geometry_conversion.hpp"
#include "src/diagnostic.hpp"

#include <eltanin/map/map_geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

namespace eltanin_ros_common
{

namespace
{

/// Origin rotation this library cannot represent; not a knob, because MapGeometry has no rotation.
constexpr double ORIGIN_ROTATION_TOLERANCE = 1e-6;

/// Every rejection names the map it rejected, so the log line identifies which publisher was wrong.
std::string describe(
  std::string_view type, std::string_view frame_id, double resolution, std::size_t width,
  std::size_t height)
{
  return std::string(type) + " (frame_id='" + std::string(frame_id) +
         "', resolution=" + std::to_string(resolution) + ", " + std::to_string(width) + "x" +
         std::to_string(height) + ")";
}

std::string reject(const std::string & context, const std::string & violation)
{
  return diagnostic::rejected(context, violation);
}

std::string tolerance_suffix(double value, double tolerance)
{
  return std::to_string(value) + " rad, tolerance is " + std::to_string(tolerance) + " rad";
}

/// The checks every map message shares, ordered so that the message can be described before use.
ConversionStatus check_grid(
  const std::string & context, double resolution, std::size_t width, std::size_t height,
  std::size_t data_size, double origin_x, double origin_y, const MapLimits & limits)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return ConversionStatus::failure(
      reject(context, "resolution must be finite and greater than 0"));
  }
  if (width == 0 || height == 0) {
    return ConversionStatus::failure(reject(context, "width and height must both be non-zero"));
  }
  const auto int_max = static_cast<std::size_t>(std::numeric_limits<int>::max());
  if (width > int_max || height > int_max) {
    return ConversionStatus::failure(reject(
      context, "width and height must each fit in an int, at most " + std::to_string(int_max)));
  }
  const std::size_t cells = width * height;
  if (cells > limits.max_cells) {
    return ConversionStatus::failure(reject(
      context,
      std::to_string(cells) + " cells exceeds max_cells " + std::to_string(limits.max_cells)));
  }
  if (data_size != cells) {
    return ConversionStatus::failure(reject(
      context, "data has " + std::to_string(data_size) + " entries but the size implies " +
                 std::to_string(cells)));
  }
  if (!std::isfinite(origin_x) || !std::isfinite(origin_y)) {
    return ConversionStatus::failure(reject(
      context, "origin (x=" + std::to_string(origin_x) + ", y=" + std::to_string(origin_y) +
                 ") must be finite"));
  }
  return ConversionStatus::success();
}

/// A pose whose rotation is not representable is refused rather than silently flattened.
ConversionStatus check_origin_rotation(
  const std::string & context, const geometry_msgs::msg::Quaternion & rotation)
{
  const ConversionResult<RollPitch> roll_pitch = to_roll_pitch(rotation);
  if (!roll_pitch.ok()) {
    return ConversionStatus::failure(
      reject(context, "origin " + diagnostic::nested(roll_pitch.error())));
  }
  if (std::abs(roll_pitch.value().roll) > ORIGIN_ROTATION_TOLERANCE) {
    return ConversionStatus::failure(reject(
      context,
      "origin roll is " + tolerance_suffix(roll_pitch.value().roll, ORIGIN_ROTATION_TOLERANCE)));
  }
  if (std::abs(roll_pitch.value().pitch) > ORIGIN_ROTATION_TOLERANCE) {
    return ConversionStatus::failure(reject(
      context,
      "origin pitch is " + tolerance_suffix(roll_pitch.value().pitch, ORIGIN_ROTATION_TOLERANCE)));
  }
  const ConversionResult<double> yaw = to_yaw(rotation);
  if (!yaw.ok()) {
    return ConversionStatus::failure(reject(context, "origin " + diagnostic::nested(yaw.error())));
  }
  if (std::abs(yaw.value()) > ORIGIN_ROTATION_TOLERANCE) {
    return ConversionStatus::failure(
      reject(context, "origin yaw is " + tolerance_suffix(yaw.value(), ORIGIN_ROTATION_TOLERANCE)));
  }
  return ConversionStatus::success();
}

/// Rejects the map the caller is about to publish, so a broken costmap does not leave the process.
ConversionStatus check_costmap(const std::string & context, const eltanin::map::Costmap & costmap)
{
  const eltanin::map::MapGeometry & geometry = costmap.geometry();
  if (!std::isfinite(geometry.resolution()) || geometry.resolution() <= 0.0) {
    return ConversionStatus::failure(
      reject(context, "resolution must be finite and greater than 0"));
  }
  if (geometry.size_x() <= 0 || geometry.size_y() <= 0) {
    return ConversionStatus::failure(reject(context, "size_x and size_y must both be positive"));
  }
  if (costmap.cell_count() == 0) {
    return ConversionStatus::failure(reject(context, "the map has no cells"));
  }
  if (costmap.data().size() != geometry.cell_count()) {
    return ConversionStatus::failure(reject(
      context, "the cell vector holds " + std::to_string(costmap.data().size()) +
                 " entries but the geometry implies " + std::to_string(geometry.cell_count())));
  }
  if (!std::isfinite(geometry.origin().x()) || !std::isfinite(geometry.origin().y())) {
    return ConversionStatus::failure(reject(context, "origin must be finite"));
  }
  return ConversionStatus::success();
}

eltanin::map::MapGeometry make_geometry(
  std::size_t width, std::size_t height, double resolution, double origin_x, double origin_y)
{
  return eltanin::map::MapGeometry(
    static_cast<int>(width), static_cast<int>(height), resolution,
    Eigen::Vector2d{origin_x, origin_y});
}

}  // namespace

ConversionResult<eltanin::map::Costmap> to_costmap(
  const nav_msgs::msg::OccupancyGrid & msg, const OccupancyThresholds & thresholds,
  const MapLimits & limits)
{
  const std::string context = describe(
    "OccupancyGrid", msg.header.frame_id, msg.info.resolution, msg.info.width, msg.info.height);
  const ConversionStatus thresholds_status = validate(thresholds);
  if (!thresholds_status.ok()) {
    return ConversionResult<eltanin::map::Costmap>::failure(
      reject(context, diagnostic::nested(thresholds_status.message())));
  }
  const ConversionStatus grid = check_grid(
    context, msg.info.resolution, msg.info.width, msg.info.height, msg.data.size(),
    msg.info.origin.position.x, msg.info.origin.position.y, limits);
  if (!grid.ok()) {
    return ConversionResult<eltanin::map::Costmap>::failure(grid.message());
  }
  const ConversionStatus rotation = check_origin_rotation(context, msg.info.origin.orientation);
  if (!rotation.ok()) {
    return ConversionResult<eltanin::map::Costmap>::failure(rotation.message());
  }

  eltanin::map::Costmap costmap(make_geometry(
    msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin.position.x,
    msg.info.origin.position.y));
  occupancy_to_costs(msg.data, thresholds, costmap.data());
  return ConversionResult<eltanin::map::Costmap>::success(std::move(costmap));
}

ConversionResult<eltanin::map::Costmap> to_costmap(
  const eltanin_msgs::msg::Costmap & msg, const MapLimits & limits)
{
  const std::string context = describe(
    "eltanin_msgs/Costmap", msg.header.frame_id, msg.info.resolution, msg.info.width,
    msg.info.height);
  const ConversionStatus grid = check_grid(
    context, msg.info.resolution, msg.info.width, msg.info.height, msg.data.size(),
    msg.info.origin_x, msg.info.origin_y, limits);
  if (!grid.ok()) {
    return ConversionResult<eltanin::map::Costmap>::failure(grid.message());
  }

  eltanin::map::Costmap costmap(make_geometry(
    msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin_x, msg.info.origin_y));
  costmap.data() = msg.data;
  return ConversionResult<eltanin::map::Costmap>::success(std::move(costmap));
}

ConversionResult<eltanin_msgs::msg::Costmap> to_costmap_msg(
  const eltanin::map::Costmap & costmap, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  const eltanin::map::MapGeometry & geometry = costmap.geometry();
  const std::string context = describe(
    "eltanin_msgs/Costmap", frame_id, geometry.resolution(),
    static_cast<std::size_t>(std::max(0, geometry.size_x())),
    static_cast<std::size_t>(std::max(0, geometry.size_y())));
  const ConversionStatus status = check_costmap(context, costmap);
  if (!status.ok()) {
    return ConversionResult<eltanin_msgs::msg::Costmap>::failure(status.message());
  }

  eltanin_msgs::msg::Costmap msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.info.resolution = geometry.resolution();
  msg.info.width = static_cast<std::uint32_t>(geometry.size_x());
  msg.info.height = static_cast<std::uint32_t>(geometry.size_y());
  msg.info.origin_x = geometry.origin().x();
  msg.info.origin_y = geometry.origin().y();
  msg.data = costmap.data();
  return ConversionResult<eltanin_msgs::msg::Costmap>::success(std::move(msg));
}

ConversionResult<nav_msgs::msg::OccupancyGrid> to_occupancy_grid(
  const eltanin::map::Costmap & costmap, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  const eltanin::map::MapGeometry & geometry = costmap.geometry();
  const std::string context = describe(
    "OccupancyGrid", frame_id, geometry.resolution(),
    static_cast<std::size_t>(std::max(0, geometry.size_x())),
    static_cast<std::size_t>(std::max(0, geometry.size_y())));
  const ConversionStatus status = check_costmap(context, costmap);
  if (!status.ok()) {
    return ConversionResult<nav_msgs::msg::OccupancyGrid>::failure(status.message());
  }

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.info.resolution = geometry.resolution();
  msg.info.width = static_cast<std::uint32_t>(geometry.size_x());
  msg.info.height = static_cast<std::uint32_t>(geometry.size_y());
  msg.info.origin = to_pose_msg(eltanin::Pose2D{geometry.origin(), 0.0});
  costs_to_occupancy(costmap.data(), msg.data);
  return ConversionResult<nav_msgs::msg::OccupancyGrid>::success(std::move(msg));
}

}  // namespace eltanin_ros_common
