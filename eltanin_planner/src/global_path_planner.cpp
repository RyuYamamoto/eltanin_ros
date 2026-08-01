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

#include <eltanin_ros_common/map_conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
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

rclcpp::QoS whole_map_qos()
{
  // Matches global_costmap's publisher; a mismatch here is a subscription that never connects.
  return rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
}

rclcpp::QoS patch_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
}

std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}

}  // namespace

GlobalPathPlanner::GlobalPathPlanner(const rclcpp::NodeOptions & options)
: rclcpp::Node("global_path_planner", options),
  profile_(require_robot_profile(*this)),
  parameters_(require_parameters(*this)),
  model_(profile_.inflation_cost_model().circumscribed_cost(), parameters_.unknown_is_free)
{
  belief_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions belief_options;
  belief_options.callback_group = belief_group_;
  costmap_subscription_ = create_subscription<eltanin_msgs::msg::Costmap>(
    "global_costmap/global_costmap", whole_map_qos(),
    [this](eltanin_msgs::msg::Costmap::ConstSharedPtr msg) { on_costmap(msg); }, belief_options);
  update_subscription_ = create_subscription<eltanin_msgs::msg::CostmapUpdate>(
    "global_costmap/global_costmap_updates", patch_qos(),
    [this](eltanin_msgs::msg::CostmapUpdate::ConstSharedPtr msg) { on_costmap_update(msg); },
    belief_options);

  RCLCPP_INFO(
    get_logger(), "waiting for %s before the first plan can run",
    costmap_subscription_->get_topic_name());
}

std::shared_ptr<const eltanin::map::Costmap> GlobalPathPlanner::snapshot() const
{
  const std::lock_guard<std::mutex> lock(belief_mutex_);
  return costmap_;
}

void GlobalPathPlanner::on_costmap(eltanin_msgs::msg::Costmap::ConstSharedPtr msg)
{
  if (msg->header.frame_id != profile_.frames().map) {
    if (whole_frame_latch_.should_warn()) {
      RCLCPP_WARN(
        get_logger(), "%s is in frame '%s', not frames.map '%s'; the costmap is dropped",
        costmap_subscription_->get_topic_name(), msg->header.frame_id.c_str(),
        profile_.frames().map.c_str());
    }
    return;
  }
  eltanin_ros_common::ConversionResult<eltanin::map::Costmap> converted =
    eltanin_ros_common::to_costmap(*msg);
  if (!converted.ok()) {
    if (whole_conversion_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", converted.error().c_str());
    }
    return;
  }

  auto replacement = std::make_shared<const eltanin::map::Costmap>(std::move(converted.value()));
  const std::lock_guard<std::mutex> lock(belief_mutex_);
  costmap_ = std::move(replacement);
  costmap_stamp_ns_ = to_nanoseconds(msg->header.stamp);
}

void GlobalPathPlanner::on_costmap_update(eltanin_msgs::msg::CostmapUpdate::ConstSharedPtr msg)
{
  std::shared_ptr<const eltanin::map::Costmap> current;
  std::int64_t current_stamp_ns = 0;
  {
    const std::lock_guard<std::mutex> lock(belief_mutex_);
    current = costmap_;
    current_stamp_ns = costmap_stamp_ns_;
  }

  if (current == nullptr) {
    if (patch_no_map_latch_.should_warn()) {
      RCLCPP_WARN(
        get_logger(), "%s arrived before %s; the patch is dropped",
        update_subscription_->get_topic_name(), costmap_subscription_->get_topic_name());
    }
    return;
  }
  if (msg->header.frame_id != profile_.frames().map) {
    if (patch_frame_latch_.should_warn()) {
      RCLCPP_WARN(
        get_logger(), "%s is in frame '%s', not frames.map '%s'; the patch is dropped",
        update_subscription_->get_topic_name(), msg->header.frame_id.c_str(),
        profile_.frames().map.c_str());
    }
    return;
  }
  const std::int64_t stamp_ns = to_nanoseconds(msg->header.stamp);
  if (stamp_ns <= current_stamp_ns) {
    // Not a warning: the whole area carries the same stamp today and is already applied (D-T7-2).
    RCLCPP_DEBUG(
      get_logger(), "%s at %ld ns is not newer than the costmap at %ld ns; it is dropped",
      update_subscription_->get_topic_name(), stamp_ns, current_stamp_ns);
    return;
  }

  // Copying unlocked is safe: belief_group_ is MutuallyExclusive, so nothing else writes costmap_.
  auto patched = std::make_shared<eltanin::map::Costmap>(*current);
  const eltanin_ros_common::ConversionStatus status =
    eltanin_ros_common::apply_costmap_update(*msg, *patched);
  if (!status.ok()) {
    if (patch_reject_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", status.message().c_str());
    }
    return;
  }

  const std::lock_guard<std::mutex> lock(belief_mutex_);
  costmap_ = std::move(patched);
  costmap_stamp_ns_ = stamp_ns;
}

}  // namespace eltanin_planner

RCLCPP_COMPONENTS_REGISTER_NODE(eltanin_planner::GlobalPathPlanner)
