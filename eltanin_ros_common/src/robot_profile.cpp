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

#include "eltanin_ros_common/robot_profile.hpp"

#include "src/diagnostic.hpp"

#include <eltanin/collision/velocity_limiter.hpp>
#include <eltanin/core/polygon.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace eltanin_ros_common
{

RobotProfile::RobotProfile(
  eltanin::Polygon2D footprint, eltanin::map::InflationCostModel inflation,
  double cost_scaling_factor, VelocityLimits limits, FrameIds frames)
: footprint_(std::move(footprint)),
  inflation_(std::move(inflation)),
  cost_scaling_factor_(cost_scaling_factor),
  limits_(limits),
  frames_(std::move(frames))
{
}

namespace
{

constexpr const char * KEY_FOOTPRINT = "robot.footprint";
constexpr const char * KEY_INFLATION_RADIUS = "robot.inflation_radius";
constexpr const char * KEY_COST_SCALING_FACTOR = "robot.cost_scaling_factor";
constexpr const char * KEY_MAX_LINEAR_VEL = "robot.max_linear_vel";
constexpr const char * KEY_MAX_ANGULAR_VEL = "robot.max_angular_vel";
constexpr const char * KEY_MAX_ACCEL = "robot.max_accel";
constexpr const char * KEY_MAX_DECEL = "robot.max_decel";
constexpr const char * KEY_FRAME_MAP = "frames.map";
constexpr const char * KEY_FRAME_ODOM = "frames.odom";
constexpr const char * KEY_FRAME_BASE = "frames.base";

/// The kachaka measurements of design section 7; robot/kachaka.yaml repeats them and is authority.
constexpr double DEFAULT_INFLATION_RADIUS = 0.55;
constexpr double DEFAULT_COST_SCALING_FACTOR = 10.0;
constexpr double DEFAULT_MAX_LINEAR_VEL = 0.30;
constexpr double DEFAULT_MAX_ANGULAR_VEL = 1.57;
constexpr double DEFAULT_MAX_ACCEL = 0.5;
constexpr double DEFAULT_MAX_DECEL = 0.5;

const std::vector<double> & default_footprint()
{
  static const std::vector<double> FOOTPRINT{-0.150, -0.120, 0.237,  -0.120,
                                             0.237,  0.120,  -0.150, 0.120};
  return FOOTPRINT;
}

struct RawParameters
{
  std::vector<double> footprint;
  double inflation_radius{0.0};
  double cost_scaling_factor{0.0};
  double max_linear_vel{0.0};
  double max_angular_vel{0.0};
  double max_accel{0.0};
  double max_decel{0.0};
  std::string frame_map;
  std::string frame_odom;
  std::string frame_base;
};

/// what() is normally one line, but the single-line rule must not depend on that.
std::string flatten(std::string text)
{
  std::replace(text.begin(), text.end(), '\n', ' ');
  std::replace(text.begin(), text.end(), '\r', ' ');
  return text;
}

std::string number(double value)
{
  return std::to_string(value);
}

/// Declares the key when it is new, then reads it; every parameter exception becomes one line.
template <class T>
std::optional<std::string> read(rclcpp::Node & node, const char * key, const T & fallback, T & out)
{
  try {
    if (!node.has_parameter(key)) {
      node.declare_parameter(key, fallback);
    }
    out = node.get_parameter(key).get_value<T>();
  } catch (const std::runtime_error & error) {
    return diagnostic::rejected(key, flatten(error.what()));
  }
  return std::nullopt;
}

std::optional<std::string> read_all(rclcpp::Node & node, RawParameters & raw)
{
  if (auto error = read(node, KEY_FOOTPRINT, default_footprint(), raw.footprint)) {
    return error;
  }
  if (
    auto error = read(node, KEY_INFLATION_RADIUS, DEFAULT_INFLATION_RADIUS, raw.inflation_radius)) {
    return error;
  }
  if (
    auto error =
      read(node, KEY_COST_SCALING_FACTOR, DEFAULT_COST_SCALING_FACTOR, raw.cost_scaling_factor)) {
    return error;
  }
  if (auto error = read(node, KEY_MAX_LINEAR_VEL, DEFAULT_MAX_LINEAR_VEL, raw.max_linear_vel)) {
    return error;
  }
  if (auto error = read(node, KEY_MAX_ANGULAR_VEL, DEFAULT_MAX_ANGULAR_VEL, raw.max_angular_vel)) {
    return error;
  }
  if (auto error = read(node, KEY_MAX_ACCEL, DEFAULT_MAX_ACCEL, raw.max_accel)) {
    return error;
  }
  if (auto error = read(node, KEY_MAX_DECEL, DEFAULT_MAX_DECEL, raw.max_decel)) {
    return error;
  }
  if (auto error = read(node, KEY_FRAME_MAP, std::string("map"), raw.frame_map)) {
    return error;
  }
  if (auto error = read(node, KEY_FRAME_ODOM, std::string("odom"), raw.frame_odom)) {
    return error;
  }
  return read(node, KEY_FRAME_BASE, std::string("base_footprint"), raw.frame_base);
}

/// Length and finiteness come first: eltanin's predicates read NaN vertices as an ordinary shape.
std::optional<std::string> validate_footprint_values(const std::vector<double> & values)
{
  if (values.size() < 6 || values.size() % 2 != 0) {
    return diagnostic::rejected(
      KEY_FOOTPRINT, "has " + std::to_string(values.size()) +
                       " values: x,y pairs for at least 3 vertices are required");
  }
  for (std::size_t at = 0; at < values.size(); ++at) {
    if (!std::isfinite(values[at])) {
      return diagnostic::rejected(
        KEY_FOOTPRINT,
        "value " + std::to_string(at) + " is " + number(values[at]) + ", which is not finite");
    }
  }
  return std::nullopt;
}

eltanin::Polygon2D to_polygon(const std::vector<double> & values)
{
  std::vector<Eigen::Vector2d> vertices;
  vertices.reserve(values.size() / 2);
  for (std::size_t at = 0; at + 1 < values.size(); at += 2) {
    vertices.emplace_back(values[at], values[at + 1]);
  }
  return eltanin::Polygon2D(std::move(vertices));
}

/// Every verdict comes from eltanin's own predicate, so the two libraries cannot disagree (A-28).
std::optional<std::string> validate_footprint_shape(const eltanin::Polygon2D & polygon)
{
  if (eltanin::winding(polygon) == eltanin::Winding::Degenerate) {
    return diagnostic::rejected(
      KEY_FOOTPRINT, "has signed area " + number(eltanin::signed_area(polygon)) +
                       " m^2, which eltanin reports as degenerate");
  }
  if (!eltanin::is_convex(polygon)) {
    return diagnostic::rejected(
      KEY_FOOTPRINT, "with " + std::to_string(polygon.size()) + " vertices is not convex");
  }
  if (!eltanin::contains(polygon, Eigen::Vector2d::Zero())) {
    const auto [min, max] = eltanin::bounding_box(polygon);
    return diagnostic::rejected(
      KEY_FOOTPRINT, "does not contain the base frame origin; its bounds are x [" +
                       number(min.x()) + ", " + number(max.x()) + "] y [" + number(min.y()) + ", " +
                       number(max.y()) + "]");
  }
  return std::nullopt;
}

/// The condition of DistanceTraversabilityModel::from_radii that is easiest to miss: inflation
/// below the circle.
std::optional<std::string> validate_inflation_radius(
  const eltanin::Polygon2D & polygon, double inflation_radius)
{
  if (!std::isfinite(inflation_radius)) {
    return diagnostic::rejected(
      KEY_INFLATION_RADIUS, "is " + number(inflation_radius) + ", which is not finite");
  }
  const std::optional<double> circumscribed = eltanin::circumscribed_radius(polygon);
  if (!circumscribed.has_value()) {
    return diagnostic::rejected(KEY_FOOTPRINT, "has no circumscribed radius");
  }
  if (inflation_radius < *circumscribed) {
    return diagnostic::rejected(
      KEY_INFLATION_RADIUS, "is " + number(inflation_radius) +
                              " m, below the circumscribed radius " + number(*circumscribed) +
                              " m of robot.footprint");
  }
  return std::nullopt;
}

std::optional<std::string> validate_not_negative(const char * key, double value)
{
  if (!std::isfinite(value) || value < 0.0) {
    return diagnostic::rejected(
      key, "is " + number(value) + ", which must be finite and not negative");
  }
  return std::nullopt;
}

std::optional<std::string> validate_positive(const char * key, double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return diagnostic::rejected(
      key, "is " + number(value) + ", which must be finite and greater than zero");
  }
  return std::nullopt;
}

std::optional<std::string> validate_frame(const char * key, const std::string & value)
{
  if (value.empty()) {
    return diagnostic::rejected(key, "is empty, and tf2 rejects an empty frame id");
  }
  if (value.front() == '/') {
    return diagnostic::rejected(
      key, "is '" + value + "', and tf2 rejects a frame id starting with '/'");
  }
  return std::nullopt;
}

std::optional<std::string> validate_all(const RawParameters & raw, const eltanin::Polygon2D & shape)
{
  if (auto error = validate_footprint_shape(shape)) {
    return error;
  }
  if (auto error = validate_inflation_radius(shape, raw.inflation_radius)) {
    return error;
  }
  if (auto error = validate_not_negative(KEY_COST_SCALING_FACTOR, raw.cost_scaling_factor)) {
    return error;
  }
  if (auto error = validate_positive(KEY_MAX_LINEAR_VEL, raw.max_linear_vel)) {
    return error;
  }
  if (auto error = validate_positive(KEY_MAX_ANGULAR_VEL, raw.max_angular_vel)) {
    return error;
  }
  if (auto error = validate_positive(KEY_MAX_ACCEL, raw.max_accel)) {
    return error;
  }
  if (auto error = validate_positive(KEY_MAX_DECEL, raw.max_decel)) {
    return error;
  }
  if (auto error = validate_frame(KEY_FRAME_MAP, raw.frame_map)) {
    return error;
  }
  if (auto error = validate_frame(KEY_FRAME_ODOM, raw.frame_odom)) {
    return error;
  }
  return validate_frame(KEY_FRAME_BASE, raw.frame_base);
}

/// Reached only if the list above missed a condition; refusing beats starting on an unknown shape.
std::string unexplained(const char * factory)
{
  return diagnostic::rejected(
    KEY_FOOTPRINT, std::string("was not accepted by eltanin's ") + factory +
                     ", and the parameter checks did not identify why");
}

}  // namespace

ConversionResult<RobotProfile> declare_robot_profile(rclcpp::Node & node)
{
  using Result = ConversionResult<RobotProfile>;

  RawParameters raw;
  if (auto error = read_all(node, raw)) {
    return Result::failure(*error);
  }
  if (auto error = validate_footprint_values(raw.footprint)) {
    return Result::failure(*error);
  }

  const eltanin::Polygon2D shape = to_polygon(raw.footprint);
  if (auto error = validate_all(raw, shape)) {
    return Result::failure(*error);
  }

  const std::optional<eltanin::DistanceTraversabilityModel> distance_model =
    eltanin::DistanceTraversabilityModel::from_footprint(shape, raw.inflation_radius);
  if (!distance_model.has_value()) {
    return Result::failure(unexplained("DistanceTraversabilityModel::from_footprint"));
  }
  const std::optional<eltanin::map::InflationCostModel> inflation =
    eltanin::map::InflationCostModel::create(*distance_model, raw.cost_scaling_factor);
  if (!inflation.has_value()) {
    return Result::failure(unexplained("InflationCostModel::create"));
  }
  eltanin::collision::VelocityLimiterParams limiter_params;
  limiter_params.footprint = shape;
  if (!eltanin::collision::VelocityLimiter::create(limiter_params).has_value()) {
    return Result::failure(unexplained("VelocityLimiter::create"));
  }

  const VelocityLimits limits{
    raw.max_linear_vel, raw.max_angular_vel, raw.max_accel, raw.max_decel};
  FrameIds frames{raw.frame_map, raw.frame_odom, raw.frame_base};
  return Result::success(RobotProfile(
    eltanin::to_counter_clockwise(shape), *inflation, raw.cost_scaling_factor, limits,
    std::move(frames)));
}

}  // namespace eltanin_ros_common
