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

#include "eltanin_planner/global_path_planner.hpp"

#include "src/diagnostic.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace eltanin_planner
{

namespace
{

/// A node that cannot be configured must not start; there is nowhere to return a failure to.
[[noreturn]] void refuse_to_start(rclcpp::Node & node, const std::string & line)
{
  RCLCPP_ERROR(node.get_logger(), "%s", line.c_str());
  throw std::runtime_error(line);
}

template <class T>
T require_parameter(rclcpp::Node & node, const char * key, const T & fallback)
{
  try {
    if (!node.has_parameter(key)) {
      node.declare_parameter(key, fallback);
    }
    return node.get_parameter(key).get_value<T>();
  } catch (const std::runtime_error & error) {
    refuse_to_start(node, diagnostic::rejected(key, diagnostic::flatten(error.what())));
  }
}

/// declare_parameter hands back an int64; only the mechanical range is checked here.
int require_int(rclcpp::Node & node, const char * key, int fallback)
{
  const auto value = require_parameter<std::int64_t>(node, key, fallback);
  if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
    refuse_to_start(
      node, diagnostic::rejected(key, "is " + std::to_string(value) + ", not an int"));
  }
  return static_cast<int>(value);
}

/// Read once at construction and never again; the defaults come from eltanin, not from literals.
PlannerParameters require_parameters(rclcpp::Node & node)
{
  const PlannerParameters defaults;
  PlannerParameters parameters;
  parameters.astar.start_search_radius_cells =
    require_int(node, KEY_START_SEARCH_RADIUS_CELLS, defaults.astar.start_search_radius_cells);
  parameters.smoother.weight_data =
    require_parameter(node, KEY_WEIGHT_DATA, defaults.smoother.weight_data);
  parameters.smoother.weight_smooth =
    require_parameter(node, KEY_WEIGHT_SMOOTH, defaults.smoother.weight_smooth);
  parameters.smoother.tolerance =
    require_parameter(node, KEY_SMOOTHER_TOLERANCE, defaults.smoother.tolerance);
  parameters.smoother.max_iterations =
    require_int(node, KEY_SMOOTHER_MAX_ITERATIONS, defaults.smoother.max_iterations);
  parameters.publish_raw_path =
    require_parameter(node, KEY_PUBLISH_RAW_PATH, defaults.publish_raw_path);
  parameters.unknown_is_free =
    require_parameter(node, KEY_UNKNOWN_IS_FREE, defaults.unknown_is_free);
  parameters.tf_lookup_timeout =
    require_parameter(node, KEY_TF_LOOKUP_TIMEOUT, defaults.tf_lookup_timeout);

  const eltanin_ros_common::ConversionStatus status = validate(parameters);
  if (!status.ok()) {
    refuse_to_start(node, status.message());
  }
  return parameters;
}

/// RobotProfile has no default constructor precisely so that "not validated yet" cannot be a state.
eltanin_ros_common::RobotProfile require_robot_profile(rclcpp::Node & node)
{
  eltanin_ros_common::ConversionResult<eltanin_ros_common::RobotProfile> profile =
    eltanin_ros_common::declare_robot_profile(node);
  if (!profile.ok()) {
    refuse_to_start(node, profile.error());
  }
  return std::move(profile.value());
}

}  // namespace

GlobalPathPlanner::GlobalPathPlanner(const rclcpp::NodeOptions & options)
: rclcpp::Node("global_path_planner", options),
  profile_(require_robot_profile(*this)),
  parameters_(require_parameters(*this)),
  model_(profile_.inflation_cost_model().circumscribed_cost(), parameters_.unknown_is_free)
{
}

}  // namespace eltanin_planner

RCLCPP_COMPONENTS_REGISTER_NODE(eltanin_planner::GlobalPathPlanner)
