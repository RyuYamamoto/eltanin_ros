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

#include "eltanin_planner/plan_attempt.hpp"
#include "src/diagnostic.hpp"

#include <eltanin/planner/path_smoother.hpp>
#include <eltanin_ros_common/geometry_conversion.hpp>
#include <eltanin_ros_common/map_conversion.hpp>
#include <eltanin_ros_common/path_conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.hpp>

#include <eltanin_msgs/msg/navigation_state.hpp>

#include <tf2_ros/create_timer_ros.h>

#include <chrono>
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

/// tf2 reads a zero time as "the latest available", which is what a plan without a stamp wants.
const rclcpp::Time LATEST_AVAILABLE(0, 0, RCL_ROS_TIME);

std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}

rclcpp::QoS path_qos()
{
  // volatile: a late subscriber must not be handed the path of a goal that was already discarded.
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
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

GlobalPathPlanner::GlobalPathPlanner(const rclcpp::NodeOptions & options)
: rclcpp::Node("global_path_planner", options),
  profile_(require_robot_profile(*this)),
  parameters_(require_parameters(*this)),
  model_(profile_.inflation_cost_model().circumscribed_cost(), parameters_.unknown_is_free),
  tf_timeout_(rclcpp::Duration::from_seconds(parameters_.tf_lookup_timeout)),
  tf_buffer_(get_clock())
{
  tf_buffer_.setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface()));
  // spin_thread: a blocking lookup on the worker must not depend on a free executor thread (P-2).
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, true);

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

  action_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  path_publisher_ = create_publisher<nav_msgs::msg::Path>("~/global_path", path_qos());
  if (parameters_.publish_raw_path) {
    raw_path_publisher_ = create_publisher<nav_msgs::msg::Path>("~/global_path_raw", path_qos());
  }

  action_server_ = rclcpp_action::create_server<Action>(
    this, "~/compute_path_to_pose",
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal) {
      return handle_goal(uuid, std::move(goal));
    },
    [this](const std::shared_ptr<GoalHandle> handle) { return handle_cancel(handle); },
    [this](const std::shared_ptr<GoalHandle> handle) { handle_accepted(handle); },
    rcl_action_server_get_default_options(), action_group_);

  worker_ = std::thread([this]() { run(); });

  RCLCPP_INFO(
    get_logger(), "waiting for %s before the first plan can run",
    costmap_subscription_->get_topic_name());
}

GlobalPathPlanner::~GlobalPathPlanner()
{
  {
    const std::lock_guard<std::mutex> lock(work_mutex_);
    stopping_ = true;
  }
  work_cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }

  std::shared_ptr<GoalHandle> left_waiting;
  {
    const std::lock_guard<std::mutex> lock(work_mutex_);
    left_waiting = std::exchange(pending_, nullptr);
  }
  if (left_waiting) {
    terminate_preempted(left_waiting);
  }
}

rclcpp_action::GoalResponse GlobalPathPlanner::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPathPlanner::handle_cancel(const std::shared_ptr<GoalHandle> &)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPathPlanner::handle_accepted(const std::shared_ptr<GoalHandle> & handle)
{
  std::shared_ptr<GoalHandle> displaced;
  {
    const std::lock_guard<std::mutex> lock(work_mutex_);
    // At most one goal waits; a newer one displaces the waiting goal before it ever ran (P-3).
    displaced = std::exchange(pending_, handle);
    if (active_) {
      preempt_active_ = true;
    }
  }
  if (displaced) {
    terminate_preempted(displaced);
  }
  work_cv_.notify_one();
}

void GlobalPathPlanner::run()
{
  for (;;) {
    std::shared_ptr<GoalHandle> handle;
    {
      std::unique_lock<std::mutex> lock(work_mutex_);
      work_cv_.wait(lock, [this]() { return stopping_ || pending_ != nullptr; });
      if (stopping_) {
        return;
      }
      handle = std::exchange(pending_, nullptr);
      active_ = handle;
      preempt_active_ = false;
    }

    execute(handle);
    {
      const std::lock_guard<std::mutex> lock(work_mutex_);
      active_.reset();
      preempt_active_ = false;
    }
  }
}

void GlobalPathPlanner::abort_with(
  const std::shared_ptr<GoalHandle> & handle, std::uint8_t outcome, const std::string & message)
{
  auto result = std::make_shared<Action::Result>();
  result->outcome = outcome;
  result->message = diagnostic::flatten(message);
  handle->abort(result);
  RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
}

void GlobalPathPlanner::terminate_preempted(const std::shared_ptr<GoalHandle> & handle)
{
  auto result = std::make_shared<Action::Result>();
  result->outcome = eltanin_msgs::msg::NavigationState::OUTCOME_CANCELED;
  result->message = diagnostic::line("the goal was preempted by a newer goal");
  handle->abort(result);
}

bool GlobalPathPlanner::interrupted(const std::shared_ptr<GoalHandle> & handle)
{
  if (handle->is_canceling()) {
    auto result = std::make_shared<Action::Result>();
    result->outcome = eltanin_msgs::msg::NavigationState::OUTCOME_CANCELED;
    result->message = diagnostic::line("the goal was canceled; no path is published");
    handle->canceled(result);
    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
    return true;
  }

  bool preempted = false;
  {
    const std::lock_guard<std::mutex> lock(work_mutex_);
    preempted = preempt_active_;
  }
  if (!preempted) {
    return false;
  }
  terminate_preempted(handle);
  return true;
}

void GlobalPathPlanner::execute(const std::shared_ptr<GoalHandle> & handle)
{
  using eltanin_msgs::msg::NavigationState;
  const std::shared_ptr<const Action::Goal> goal = handle->get_goal();
  if (interrupted(handle)) {
    return;
  }

  const std::shared_ptr<const eltanin::map::Costmap> costmap = snapshot();
  if (costmap == nullptr) {
    abort_with(
      handle, NavigationState::OUTCOME_INPUT_STALE,
      diagnostic::line(
        std::string("no costmap has arrived on ") + costmap_subscription_->get_topic_name() +
        " yet; there is nothing to plan against"));
    return;
  }

  const eltanin_ros_common::ConversionResult<eltanin::Pose2D> goal_pose =
    resolve_pose(goal->goal, "goal");
  if (!goal_pose.ok()) {
    abort_with(handle, NavigationState::OUTCOME_START_GOAL_FAILED, goal_pose.error());
    return;
  }
  const eltanin_ros_common::ConversionResult<eltanin::Pose2D> start_pose =
    goal->use_start ? resolve_pose(goal->start, "start") : robot_pose();
  if (!start_pose.ok()) {
    abort_with(handle, NavigationState::OUTCOME_START_GOAL_FAILED, start_pose.error());
    return;
  }

  if (interrupted(handle)) {
    return;
  }

  const auto search_started = std::chrono::steady_clock::now();
  const PlanAttempt attempt =
    attempt_plan(*costmap, model_, start_pose.value(), goal_pose.value(), parameters_);
  const auto search_finished = std::chrono::steady_clock::now();
  if (!attempt.ok()) {
    abort_with(handle, to_outcome(attempt.failure), attempt.message);
    return;
  }

  if (interrupted(handle)) {
    return;
  }

  const eltanin::Path smoothed =
    eltanin::planner::smooth(attempt.path, *costmap, model_, parameters_.smoother);
  const auto smoothing_finished = std::chrono::steady_clock::now();

  if (interrupted(handle)) {
    return;
  }

  const rclcpp::Time stamp = now();
  const std::string & map_frame = profile_.frames().map;
  const eltanin_ros_common::ConversionResult<nav_msgs::msg::Path> path_msg =
    eltanin_ros_common::to_path_msg(smoothed, map_frame, stamp);
  if (!path_msg.ok()) {
    abort_with(handle, NavigationState::OUTCOME_PLAN_FAILED, path_msg.error());
    return;
  }

  if (raw_path_publisher_) {
    const eltanin_ros_common::ConversionResult<nav_msgs::msg::Path> raw_msg =
      eltanin_ros_common::to_path_msg(attempt.path, map_frame, stamp);
    if (raw_msg.ok()) {
      raw_path_publisher_->publish(raw_msg.value());
    } else {
      RCLCPP_WARN(get_logger(), "%s", raw_msg.error().c_str());
    }
  }

  // Published before succeed() so the topic already carries the path the result announces (P-6).
  path_publisher_->publish(path_msg.value());

  auto result = std::make_shared<Action::Result>();
  result->path = path_msg.value();
  result->outcome = NavigationState::OUTCOME_REACHED;
  result->message = diagnostic::line(
    "planned " + std::to_string(smoothed.size()) + " poses, " +
    std::to_string(eltanin::path_length(smoothed)) + " m, searched in " +
    milliseconds_between(search_started, search_finished) + " ms, smoothed in " +
    milliseconds_between(search_finished, smoothing_finished) + " ms");
  handle->succeed(result);
  RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
}

std::shared_ptr<const eltanin::map::Costmap> GlobalPathPlanner::snapshot() const
{
  const std::lock_guard<std::mutex> lock(belief_mutex_);
  return costmap_;
}

eltanin_ros_common::ConversionResult<eltanin::Pose2D> GlobalPathPlanner::resolve_pose(
  const geometry_msgs::msg::PoseStamped & msg, const char * what)
{
  using Result = eltanin_ros_common::ConversionResult<eltanin::Pose2D>;
  const std::string & map_frame = profile_.frames().map;
  if (msg.header.frame_id.empty()) {
    return Result::failure(diagnostic::rejected(
      what, "its header.frame_id is empty; frames.map '" + map_frame + "' is not assumed"));
  }

  eltanin_ros_common::ConversionResult<eltanin::Pose2D> pose =
    eltanin_ros_common::to_pose2d(msg.pose);
  if (!pose.ok()) {
    return Result::failure(diagnostic::rejected(what, diagnostic::flatten(pose.error())));
  }
  if (msg.header.frame_id == map_frame) {
    return pose;
  }

  const rclcpp::Time at(msg.header.stamp);
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(map_frame, msg.header.frame_id, at, tf_timeout_);
  } catch (const tf2::TransformException & error) {
    return Result::failure(diagnostic::rejected(
      what, "'" + msg.header.frame_id + "' to frames.map '" + map_frame + "' at " +
              std::to_string(at.nanoseconds()) + " ns is not available within " +
              std::to_string(tf_timeout_.seconds()) + " s: " + diagnostic::flatten(error.what())));
  }

  const eltanin_ros_common::ConversionResult<eltanin_ros_common::Transform2DConversion> converted =
    eltanin_ros_common::to_transform2d(transform);
  if (!converted.ok()) {
    return Result::failure(diagnostic::rejected(what, diagnostic::flatten(converted.error())));
  }
  if (!converted.value().deviation.within_tolerance && planarity_latch_.should_warn()) {
    RCLCPP_WARN(
      get_logger(), "the transform from '%s' to '%s' is out of plane by z %f m, roll %f, pitch %f",
      msg.header.frame_id.c_str(), map_frame.c_str(), converted.value().deviation.z,
      converted.value().deviation.roll, converted.value().deviation.pitch);
  }
  return Result::success(converted.value().transform * pose.value());
}

eltanin_ros_common::ConversionResult<eltanin::Pose2D> GlobalPathPlanner::robot_pose()
{
  using Result = eltanin_ros_common::ConversionResult<eltanin::Pose2D>;
  const std::string & map_frame = profile_.frames().map;
  const std::string & base_frame = profile_.frames().base;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(map_frame, base_frame, LATEST_AVAILABLE, tf_timeout_);
  } catch (const tf2::TransformException & error) {
    return Result::failure(diagnostic::rejected(
      "start", "frames.base '" + base_frame + "' in frames.map '" + map_frame +
                 "' is not available within " + std::to_string(tf_timeout_.seconds()) +
                 " s: " + diagnostic::flatten(error.what())));
  }

  const eltanin_ros_common::ConversionResult<eltanin_ros_common::Transform2DConversion> converted =
    eltanin_ros_common::to_transform2d(transform);
  if (!converted.ok()) {
    return Result::failure(diagnostic::rejected("start", diagnostic::flatten(converted.error())));
  }
  if (!converted.value().deviation.within_tolerance && planarity_latch_.should_warn()) {
    RCLCPP_WARN(
      get_logger(), "the transform from '%s' to '%s' is out of plane by z %f m, roll %f, pitch %f",
      base_frame.c_str(), map_frame.c_str(), converted.value().deviation.z,
      converted.value().deviation.roll, converted.value().deviation.pitch);
  }
  return Result::success(converted.value().transform.to_pose());
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
