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

#include "eltanin_controller/path_follower.hpp"

#include "src/diagnostic.hpp"

#include <eltanin_ros_common/geometry_conversion.hpp>
#include <eltanin_ros_common/path_conversion.hpp>
#include <eltanin_ros_common/trajectory_conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.hpp>

#include <tf2_ros/create_timer_ros.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace eltanin_controller
{

namespace
{

using Diagnostic = eltanin_msgs::msg::FollowerDiagnostic;

/// tf2 reads a zero time as "the latest available", which is what a control cycle wants.
const rclcpp::Time LATEST_AVAILABLE(0, 0, RCL_ROS_TIME);

/// A periodic consumer never blocks inside its own cycle waiting for a transform to turn up.
const rclcpp::Duration NO_TF_WAIT = rclcpp::Duration::from_nanoseconds(0);

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

PathSource require_path_source(rclcpp::Node & node, PathSource fallback)
{
  const auto name = require_parameter<std::string>(node, KEY_PATH_SOURCE, name_of(fallback));
  const std::optional<PathSource> source = to_path_source(name);
  if (!source.has_value()) {
    refuse_to_start(
      node, diagnostic::rejected(
              KEY_PATH_SOURCE, "is '" + name + "', not '" + name_of(PathSource::Path) + "' or '" +
                                 name_of(PathSource::Trajectory) + "'"));
  }
  return *source;
}

/// Read once at construction and never again; the defaults come from eltanin, not from literals.
FollowerParameters require_parameters(
  rclcpp::Node & node, const eltanin_ros_common::VelocityLimits & limits)
{
  const FollowerParameters defaults;
  FollowerParameters parameters;
  parameters.update_frequency =
    require_parameter(node, KEY_UPDATE_FREQUENCY, defaults.update_frequency);
  parameters.path_source = require_path_source(node, defaults.path_source);
  parameters.pursuit.desired_linear_vel =
    require_parameter(node, KEY_DESIRED_LINEAR_VEL, defaults.pursuit.desired_linear_vel);
  parameters.pursuit.yaw_tolerance =
    require_parameter(node, KEY_YAW_TOLERANCE, defaults.pursuit.yaw_tolerance);
  parameters.pursuit.lookahead_time =
    require_parameter(node, KEY_LOOKAHEAD_TIME, defaults.pursuit.lookahead_time);
  parameters.pursuit.min_lookahead_dist =
    require_parameter(node, KEY_MIN_LOOKAHEAD_DIST, defaults.pursuit.min_lookahead_dist);
  parameters.approach.xy_goal_tolerance =
    require_parameter(node, KEY_XY_GOAL_TOLERANCE, defaults.approach.xy_goal_tolerance);
  parameters.approach.yaw_goal_tolerance =
    require_parameter(node, KEY_YAW_GOAL_TOLERANCE, defaults.approach.yaw_goal_tolerance);
  parameters.approach.approach_distance =
    require_parameter(node, KEY_APPROACH_DISTANCE, defaults.approach.approach_distance);
  parameters.approach.approach_decel =
    require_parameter(node, KEY_APPROACH_DECEL, defaults.approach.approach_decel);
  parameters.approach.yaw_align_timeout =
    require_parameter(node, KEY_YAW_ALIGN_TIMEOUT, defaults.approach.yaw_align_timeout);
  parameters.trajectory_timeout =
    require_parameter(node, KEY_TRAJECTORY_TIMEOUT, defaults.trajectory_timeout);
  parameters.path_timeout = require_parameter(node, KEY_PATH_TIMEOUT, defaults.path_timeout);

  // Before validate(), which checks the values the two create() calls are actually handed.
  const VelocityClamp clamp = apply_velocity_limits(parameters, limits);
  if (clamp.clamped) {
    RCLCPP_WARN(
      node.get_logger(),
      "%s %f m/s is above robot.max_linear_vel %f m/s; the cruise speed is that limit",
      KEY_DESIRED_LINEAR_VEL, clamp.requested, clamp.applied);
  }

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

/// validate() has already named the offending value; create() itself carries no reason.
eltanin::control::PurePursuit require_pursuit(
  rclcpp::Node & node, const eltanin::control::PurePursuitParams & params)
{
  std::optional<eltanin::control::PurePursuit> pursuit =
    eltanin::control::PurePursuit::create(params);
  if (!pursuit.has_value()) {
    refuse_to_start(node, diagnostic::rejected("the pure pursuit parameters", "eltanin refused"));
  }
  return std::move(*pursuit);
}

/// The same, for the approach; the two rejections stay distinguishable in the log.
eltanin::control::GoalApproach require_approach(
  rclcpp::Node & node, const eltanin::control::GoalApproachParams & params)
{
  std::optional<eltanin::control::GoalApproach> approach =
    eltanin::control::GoalApproach::create(params);
  if (!approach.has_value()) {
    refuse_to_start(node, diagnostic::rejected("the goal approach parameters", "eltanin refused"));
  }
  return std::move(*approach);
}

/// nullopt is "no deadline", which a global path published once per replan needs.
std::optional<double> deadline_of(const FollowerParameters & parameters)
{
  if (parameters.path_source == PathSource::Trajectory) {
    return parameters.trajectory_timeout;
  }
  return parameters.path_timeout > 0.0 ? std::optional<double>(parameters.path_timeout)
                                       : std::nullopt;
}

rclcpp::QoS control_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
}

}  // namespace

PathFollower::PathFollower(const rclcpp::NodeOptions & options)
: rclcpp::Node("path_follower", options),
  profile_(require_robot_profile(*this)),
  parameters_(require_parameters(*this, profile_.limits())),
  clock_(get_clock()),
  pursuit_(require_pursuit(*this, parameters_.pursuit)),
  approach_(require_approach(*this, parameters_.approach)),
  input_(deadline_of(parameters_)),
  tf_buffer_(get_clock())
{
  tf_buffer_.setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface()));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, true);

  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  command_publisher_ =
    create_publisher<geometry_msgs::msg::TwistStamped>("~/cmd_vel_raw", control_qos());
  diagnostic_publisher_ = create_publisher<Diagnostic>("~/follower_state", control_qos());
  lookahead_publisher_ =
    create_publisher<geometry_msgs::msg::PointStamped>("~/lookahead_point", control_qos());

  rclcpp::SubscriptionOptions input_options;
  input_options.callback_group = input_group_;
  // One subscription only, so which input is followed is settled at startup rather than per
  // message.
  if (parameters_.path_source == PathSource::Trajectory) {
    trajectory_subscription_ = create_subscription<eltanin_msgs::msg::Trajectory2D>(
      "local_path_planner/local_trajectory", control_qos(),
      [this](eltanin_msgs::msg::Trajectory2D::ConstSharedPtr msg) { on_trajectory(msg); },
      input_options);
  } else {
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "global_path_planner/global_path", control_qos(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) { on_path(msg); }, input_options);
  }

  reset_service_ = create_service<std_srvs::srv::Trigger>(
    "~/reset",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) { on_reset(request, response); },
    rclcpp::ServicesQoS(), service_group_);

  timer_ = create_timer(
    std::chrono::duration<double>(1.0 / parameters_.update_frequency), [this]() { on_timer(); },
    control_group_);

  RCLCPP_INFO(
    get_logger(), "following %s at %f Hz, cruising at %f m/s",
    parameters_.path_source == PathSource::Trajectory ? trajectory_subscription_->get_topic_name()
                                                      : path_subscription_->get_topic_name(),
    parameters_.update_frequency, parameters_.pursuit.desired_linear_vel);
}

void PathFollower::on_timer()
{
  const rclcpp::Time now = this->now();
  const CycleOutcome cycle = run_cycle(now);
  publish_cycle(cycle, now);
}

PathFollower::CycleOutcome PathFollower::run_cycle(const rclcpp::Time & now)
{
  // The PeriodicClock is not reset with them; that would make the next cycle a first tick.
  if (reset_requested_.exchange(false)) {
    pursuit_.reset();
    approach_.reset();
  }

  const eltanin_ros_common::PeriodicClock::Tick tick = clock_.tick();
  if (!tick.usable()) {
    CycleOutcome cycle;
    cycle.outcome = composition::input_failure(Diagnostic::REASON_NO_DT);
    cycle.message =
      diagnostic::line("no usable elapsed time this cycle: " + std::to_string(tick.seconds) + " s");
    if (no_dt_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", cycle.message.c_str());
    }
    return cycle;
  }

  PathInput::Reading reading;
  std::string rejection;
  {
    const std::lock_guard<std::mutex> lock(input_mutex_);
    reading = input_.read(now);
    if (reading.state == PathInput::State::Rejected) {
      rejection = input_.rejection_detail();
    }
  }

  CycleOutcome cycle;
  switch (reading.state) {
    case PathInput::State::NeverReceived:
      cycle.outcome = composition::input_failure(Diagnostic::REASON_NO_INPUT);
      cycle.message = diagnostic::line("no path has arrived yet");
      if (no_input_latch_.should_warn()) {
        RCLCPP_WARN(get_logger(), "%s", cycle.message.c_str());
      }
      return cycle;
    case PathInput::State::Rejected:
      cycle.outcome = composition::input_failure(Diagnostic::REASON_INPUT_REJECTED);
      cycle.message = rejection;
      return cycle;
    case PathInput::State::Stale:
      cycle.outcome = composition::input_failure(Diagnostic::REASON_INPUT_STALE);
      cycle.message = diagnostic::line(
        "the path is " + std::to_string(reading.elapsed_seconds) + " s old, past the deadline");
      if (stale_latch_.should_warn()) {
        RCLCPP_WARN(get_logger(), "%s", cycle.message.c_str());
      }
      return cycle;
    case PathInput::State::Available:
      break;
  }

  if (reading.snapshot.path->empty()) {
    cycle.outcome = composition::input_failure(Diagnostic::REASON_INPUT_EMPTY);
    cycle.message = diagnostic::line("the path carries no poses");
    if (empty_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", cycle.message.c_str());
    }
    return cycle;
  }

  const std::optional<eltanin::Pose2D> robot = robot_pose();
  if (!robot.has_value()) {
    cycle.outcome = composition::input_failure(Diagnostic::REASON_NO_TRANSFORM);
    cycle.message = diagnostic::line(
      "frames.base '" + profile_.frames().base + "' in frames.map '" + profile_.frames().map +
      "' is not available");
    return cycle;
  }

  // GoalApproach first, and PurePursuit only where the approach still leaves something to track.
  const eltanin::Path & path = *reading.snapshot.path;
  const eltanin::control::GoalApproach::Result approach =
    approach_.compute(*robot, path, tick.seconds);
  std::optional<eltanin::control::PurePursuit::Result> tracking;
  if (composition::tracking_required(approach.state)) {
    tracking = pursuit_.compute(*robot, path, tick.seconds);
  }

  cycle.outcome = composition::compose(approach, tracking);
  cycle.remaining_arc = approach.remaining_arc;
  cycle.position_error = approach.position_error;
  cycle.yaw_error = approach.yaw_error;
  cycle.align_elapsed = approach.align_elapsed;
  cycle.control_dt = tick.seconds;
  if (cycle.outcome.reason != Diagnostic::REASON_NONE) {
    cycle.message = diagnostic::line(
      "eltanin returned status " + std::to_string(cycle.outcome.status) + " and approach state " +
      std::to_string(cycle.outcome.approach_state));
    if (!cycle.outcome.ok && controller_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", cycle.message.c_str());
    }
  }
  return cycle;
}

void PathFollower::publish_cycle(const CycleOutcome & cycle, const rclcpp::Time & now)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now;
  command.header.frame_id = profile_.frames().base;
  command.twist = eltanin_ros_common::to_twist_msg(cycle.outcome.command);
  command_publisher_->publish(command);

  Diagnostic diagnostic;
  diagnostic.header.stamp = now;
  diagnostic.status = cycle.outcome.status;
  diagnostic.approach_state = cycle.outcome.approach_state;
  diagnostic.reason = cycle.outcome.reason;
  diagnostic.ok = cycle.outcome.ok;
  diagnostic.message = cycle.message;
  diagnostic.remaining_arc = cycle.remaining_arc;
  diagnostic.position_error = cycle.position_error;
  diagnostic.yaw_error = cycle.yaw_error;
  diagnostic.lookahead_index = static_cast<std::uint32_t>(cycle.outcome.lookahead_index);
  diagnostic.control_dt = cycle.control_dt;
  diagnostic.align_elapsed = cycle.align_elapsed;
  diagnostic_publisher_->publish(diagnostic);

  if (!cycle.outcome.has_lookahead) {
    return;
  }
  geometry_msgs::msg::PointStamped point;
  point.header.stamp = now;
  point.header.frame_id = profile_.frames().map;
  point.point.x = cycle.outcome.lookahead_point.x();
  point.point.y = cycle.outcome.lookahead_point.y();
  lookahead_publisher_->publish(point);
}

void PathFollower::on_path(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  accept_path(msg->header, eltanin_ros_common::to_path(*msg));
}

void PathFollower::on_trajectory(eltanin_msgs::msg::Trajectory2D::ConstSharedPtr msg)
{
  accept_path(msg->header, eltanin_ros_common::to_path(*msg));
}

void PathFollower::accept_path(
  const std_msgs::msg::Header & header,
  const eltanin_ros_common::ConversionResult<eltanin::Path> & converted)
{
  // Upstream publishes in frames.map by contract; transforming here would add a third way to fail.
  if (header.frame_id != profile_.frames().map) {
    const std::string detail = diagnostic::rejected(
      "the path",
      "is in frame '" + header.frame_id + "', not frames.map '" + profile_.frames().map + "'");
    {
      const std::lock_guard<std::mutex> lock(input_mutex_);
      input_.reject(detail);
    }
    if (rejected_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    }
    return;
  }
  if (!converted.ok()) {
    const std::string detail = diagnostic::flatten(converted.error());
    {
      const std::lock_guard<std::mutex> lock(input_mutex_);
      input_.reject(detail);
    }
    if (rejected_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    }
    return;
  }

  PathSnapshot snapshot;
  snapshot.path = std::make_shared<const eltanin::Path>(converted.value());
  snapshot.stamp = rclcpp::Time(header.stamp, RCL_ROS_TIME);
  const std::lock_guard<std::mutex> lock(input_mutex_);
  input_.accept(std::move(snapshot));
}

void PathFollower::on_reset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  reset_requested_.store(true);
  response->success = true;
  response->message =
    diagnostic::line("the pursuit and the approach are reset before the next command");
}

std::optional<eltanin::Pose2D> PathFollower::robot_pose()
{
  const std::string & map_frame = profile_.frames().map;
  const std::string & base_frame = profile_.frames().base;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(map_frame, base_frame, LATEST_AVAILABLE, NO_TF_WAIT);
  } catch (const tf2::TransformException & error) {
    if (transform_latch_.should_warn()) {
      RCLCPP_WARN(
        get_logger(), "frames.base '%s' in frames.map '%s' is not available: %s",
        base_frame.c_str(), map_frame.c_str(), diagnostic::flatten(error.what()).c_str());
    }
    return std::nullopt;
  }

  const eltanin_ros_common::ConversionResult<eltanin_ros_common::Transform2DConversion> converted =
    eltanin_ros_common::to_transform2d(transform);
  if (!converted.ok()) {
    if (transform_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", converted.error().c_str());
    }
    return std::nullopt;
  }
  if (!converted.value().deviation.within_tolerance && planarity_latch_.should_warn()) {
    RCLCPP_WARN(
      get_logger(), "the transform from '%s' to '%s' is out of plane by z %f m, roll %f, pitch %f",
      base_frame.c_str(), map_frame.c_str(), converted.value().deviation.z,
      converted.value().deviation.roll, converted.value().deviation.pitch);
  }
  return converted.value().transform.to_pose();
}

}  // namespace eltanin_controller

RCLCPP_COMPONENTS_REGISTER_NODE(eltanin_controller::PathFollower)
