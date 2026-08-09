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

#include "eltanin_costmap/global_costmap.hpp"

#include <eltanin_ros_common/map_conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace eltanin_costmap
{

namespace
{

constexpr const char * KEY_INFLATE_UNKNOWN = "inflate_unknown";
constexpr const char * KEY_OCCUPIED_THRESHOLD = "occupied_threshold";
constexpr const char * KEY_FREE_THRESHOLD = "free_threshold";
constexpr const char * KEY_PUBLISH_VISUALIZATION = "publish_visualization";

/// Visualization is on by default, so the default configuration is one RViz can already read.

std::string reject_parameter(const char * key, const std::string & violation)
{
  return std::string("eltanin_costmap: rejected ") + key + ": " + violation;
}

/// what() is normally one line, but the single-line rule must not depend on that.
std::string flatten(std::string text)
{
  std::replace(text.begin(), text.end(), '\n', ' ');
  std::replace(text.begin(), text.end(), '\r', ' ');
  return text;
}

/// A node that cannot be configured must not start; there is nowhere to return a failure to.
[[noreturn]] void refuse_to_start(rclcpp::Node & node, const std::string & line)
{
  RCLCPP_ERROR(node.get_logger(), "%s", line.c_str());
  throw std::runtime_error(line);
}

/// Declared without a fallback: a key missing from the configuration stops the node naming it,
/// rather than running on a value nobody chose.
template <class T>
T require_parameter(rclcpp::Node & node, const char * key)
{
  try {
    if (!node.has_parameter(key)) {
      node.declare_parameter<T>(key);
    }
    return node.get_parameter(key).get_value<T>();
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
    refuse_to_start(node, reject_parameter(key, "is not set; every key has to come from a config"));
  } catch (const std::runtime_error & error) {
    refuse_to_start(node, reject_parameter(key, flatten(error.what())));
  }
}

/// Only the mechanical int range is checked here; 0 <= free < occupied <= 100 belongs to D-20.
int require_threshold(rclcpp::Node & node, const char * key)
{
  const auto value = require_parameter<std::int64_t>(node, key);
  if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
    refuse_to_start(node, reject_parameter(key, "is " + std::to_string(value) + ", not an int"));
  }
  return static_cast<int>(value);
}

eltanin_ros_common::OccupancyThresholds require_thresholds(rclcpp::Node & node)
{
  eltanin_ros_common::OccupancyThresholds thresholds;
  thresholds.occupied_threshold = require_threshold(node, KEY_OCCUPIED_THRESHOLD);
  thresholds.free_threshold = require_threshold(node, KEY_FREE_THRESHOLD);

  const eltanin_ros_common::ConversionStatus status = eltanin_ros_common::validate(thresholds);
  if (!status.ok()) {
    refuse_to_start(node, status.message());
  }
  return thresholds;
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
  // transient_local so a planner that starts later still gets the belief it has to plan against.
  return rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
}

rclcpp::QoS window_qos()
{
  // Depth 1 and volatile: a window that is late is worth less than the next one (R-P6-1).
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
}

std::string describe_patch(const std::optional<eltanin::map::CellRect> & patch)
{
  if (!patch.has_value()) {
    return "no patch";
  }
  return "patch " + std::to_string(patch->max_x - patch->min_x + 1) + "x" +
         std::to_string(patch->max_y - patch->min_y + 1) + " at (" + std::to_string(patch->min_x) +
         ", " + std::to_string(patch->min_y) + ")";
}

/// Elapsed computation time must not follow use_sim_time, where it would read as zero (D-P6-8).
std::string milliseconds_between(
  const std::chrono::steady_clock::time_point & from,
  const std::chrono::steady_clock::time_point & to)
{
  const std::chrono::duration<double, std::milli> elapsed = to - from;
  return std::to_string(elapsed.count());
}

}  // namespace

GlobalCostmap::GlobalCostmap(const rclcpp::NodeOptions & options)
: rclcpp::Node("global_costmap", options),
  profile_(require_robot_profile(*this)),
  thresholds_(require_thresholds(*this)),
  inflate_unknown_(require_parameter<bool>(*this, KEY_INFLATE_UNKNOWN)),
  publish_visualization_(require_parameter<bool>(*this, KEY_PUBLISH_VISUALIZATION))
{
  map_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  observation_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Both services change the state from outside and both take the one mutex, so they share a group.
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  costmap_publisher_ =
    create_publisher<eltanin_msgs::msg::Costmap>("~/global_costmap", whole_map_qos());
  update_publisher_ =
    create_publisher<eltanin_msgs::msg::CostmapUpdate>("~/global_costmap_updates", window_qos());
  if (publish_visualization_) {
    visual_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("~/global_costmap_visual", whole_map_qos());
  }

  rclcpp::SubscriptionOptions map_options;
  map_options.callback_group = map_group_;
  map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", whole_map_qos(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { on_map(msg); }, map_options);

  rclcpp::SubscriptionOptions observation_options;
  observation_options.callback_group = observation_group_;
  local_map_subscription_ = create_subscription<eltanin_msgs::msg::Costmap>(
    "local_map/local_map", window_qos(),
    [this](eltanin_msgs::msg::Costmap::ConstSharedPtr msg) { on_local_map(msg); },
    observation_options);

  update_service_ = create_service<Trigger>(
    "~/update",
    [this](const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response) {
      on_update(request, response);
    },
    rclcpp::ServicesQoS(), service_group_);
  clear_service_ = create_service<Trigger>(
    "~/clear_observations",
    [this](const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response) {
      on_clear_observations(request, response);
    },
    rclcpp::ServicesQoS(), service_group_);

  RCLCPP_INFO(
    get_logger(), "waiting for %s; %s is updated on that and on %s only",
    map_subscription_->get_topic_name(), costmap_publisher_->get_topic_name(),
    update_service_->get_service_name());
}

void GlobalCostmap::on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  eltanin_ros_common::ConversionResult<eltanin::map::Costmap> accepted =
    accept_static_map(*msg, profile_.frames().map, thresholds_);
  if (!accepted.ok()) {
    // The rejected map reaches no state at all, so ~/update keeps failing rather than half working.
    RCLCPP_ERROR(get_logger(), "%s", accepted.error().c_str());
    return;
  }
  if (state_.has_value()) {
    RCLCPP_INFO(
      get_logger(), "a second static map replaces the first; dropping %zu observation cells",
      state_->observation_count());
  }
  state_.emplace(std::move(accepted.value()), profile_.inflation_cost_model(), inflate_unknown_);
  reset_latches();

  const GlobalCostmapState::UpdateOutcome outcome = state_->update();
  // No patch on the first build: the whole area carries the same content and is transient_local.
  publish_all(*state_, std::nullopt, now());
  RCLCPP_INFO(
    get_logger(), "built a %dx%d costmap at %f m from %s, %zu observation cells",
    state_->geometry().size_x(), state_->geometry().size_y(), state_->geometry().resolution(),
    map_subscription_->get_topic_name(), outcome.observation_count);
}

void GlobalCostmap::on_local_map(eltanin_msgs::msg::Costmap::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.has_value()) {
    if (no_map_latch_.should_warn()) {
      RCLCPP_WARN(
        get_logger(), "%s arrived before %s; the window is dropped",
        local_map_subscription_->get_topic_name(), map_subscription_->get_topic_name());
    }
    return;
  }
  if (msg->header.frame_id != profile_.frames().map) {
    if (local_frame_latch_.should_warn()) {
      RCLCPP_WARN(
        get_logger(), "%s is in frame '%s', not frames.map '%s'; the window is dropped",
        local_map_subscription_->get_topic_name(), msg->header.frame_id.c_str(),
        profile_.frames().map.c_str());
    }
    return;
  }
  // The message stays const: what the store keeps is a copy in eltanin's own type (AC-24).
  const eltanin_ros_common::ConversionResult<eltanin::map::Costmap> window =
    eltanin_ros_common::to_costmap(*msg);
  if (!window.ok()) {
    if (conversion_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", window.error().c_str());
    }
    return;
  }

  // Only accumulation happens here: update() is 0.285 s at 4000x4000 and cannot run at 10 Hz (C-3).
  const ObservationStore::AbsorbOutcome outcome = state_->absorb(window.value());
  if (!outcome.overlaps && overlap_latch_.should_warn()) {
    RCLCPP_WARN(
      get_logger(), "%s does not overlap the static map; the window is dropped",
      local_map_subscription_->get_topic_name());
  }
  if (outcome.resolution_differs && resolution_latch_.should_warn()) {
    RCLCPP_WARN(
      get_logger(), "%s is at %f m but the static map is at %f m; obstacles may be missed",
      local_map_subscription_->get_topic_name(), msg->info.resolution,
      state_->geometry().resolution());
  }
}

void GlobalCostmap::on_update(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.has_value()) {
    response->success = false;
    response->message = std::string(map_subscription_->get_topic_name()) + " has not been received";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  const auto started = std::chrono::steady_clock::now();
  const GlobalCostmapState::UpdateOutcome outcome = state_->update();
  const auto updated = std::chrono::steady_clock::now();
  // Publishing inside the lock costs a held mutex; publishing outside it costs a 16 MB copy.
  const bool published = publish_all(*state_, outcome.publish_rect, now());
  const auto finished = std::chrono::steady_clock::now();

  response->success = published;
  response->message = "updated in " + milliseconds_between(started, updated) +
                      " ms, published in " + milliseconds_between(updated, finished) + " ms, " +
                      std::to_string(outcome.observation_count) + " observation cells, " +
                      describe_patch(outcome.publish_rect);
  if (published) {
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }
}

void GlobalCostmap::on_clear_observations(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  response->success = true;
  if (!state_.has_value()) {
    // The contract is "nothing is accumulated", and with no map that already holds (D-P6-7).
    response->message = "no static map yet, nothing has been accumulated";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return;
  }

  const std::size_t dropped = state_->observation_count();
  state_->clear_observations();
  response->message = "dropped " + std::to_string(dropped) + " observation cells; call " +
                      update_service_->get_service_name() + " to publish the result";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

bool GlobalCostmap::publish_all(
  const GlobalCostmapState & state, const std::optional<eltanin::map::CellRect> & patch,
  const rclcpp::Time & stamp)
{
  const std::string & frame_id = profile_.frames().map;
  bool published = true;

  // The whole area goes first, so a late subscriber never sees a patch without its ground.
  eltanin_ros_common::ConversionResult<eltanin_msgs::msg::Costmap> whole =
    eltanin_ros_common::to_costmap_msg(state.costmap(), frame_id, stamp);
  if (whole.ok()) {
    costmap_publisher_->publish(std::move(whole.value()));
  } else {
    RCLCPP_ERROR(get_logger(), "%s", whole.error().c_str());
    published = false;
  }

  if (visual_publisher_) {
    eltanin_ros_common::ConversionResult<nav_msgs::msg::OccupancyGrid> visual =
      eltanin_ros_common::to_occupancy_grid(state.costmap(), frame_id, stamp);
    if (visual.ok()) {
      visual_publisher_->publish(std::move(visual.value()));
    } else {
      RCLCPP_ERROR(get_logger(), "%s", visual.error().c_str());
      published = false;
    }
  }

  if (patch.has_value()) {
    // Same stamp as the whole area, which is how a consumer knows what this patch applies to.
    eltanin_ros_common::ConversionResult<eltanin_msgs::msg::CostmapUpdate> update =
      eltanin_ros_common::to_costmap_update_msg(state.costmap(), *patch, frame_id, stamp);
    if (update.ok()) {
      update_publisher_->publish(std::move(update.value()));
    } else {
      RCLCPP_ERROR(get_logger(), "%s", update.error().c_str());
      published = false;
    }
  }
  return published;
}

void GlobalCostmap::reset_latches() noexcept
{
  no_map_latch_ = eltanin_ros_common::WarnOnceLatch{};
  local_frame_latch_ = eltanin_ros_common::WarnOnceLatch{};
  conversion_latch_ = eltanin_ros_common::WarnOnceLatch{};
  resolution_latch_ = eltanin_ros_common::WarnOnceLatch{};
  overlap_latch_ = eltanin_ros_common::WarnOnceLatch{};
}

}  // namespace eltanin_costmap

RCLCPP_COMPONENTS_REGISTER_NODE(eltanin_costmap::GlobalCostmap)
