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

#include "eltanin_controller/collision_predictor.hpp"

#include "src/diagnostic.hpp"

#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/distance_map.hpp>
#include <eltanin_ros_common/diagnostic_conversion.hpp>
#include <eltanin_ros_common/geometry_conversion.hpp>
#include <eltanin_ros_common/map_conversion.hpp>
#include <eltanin_ros_common/marker_conversion.hpp>
#include <eltanin_ros_common/path_conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.hpp>

#include <tf2_ros/create_timer_ros.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace eltanin_controller
{

namespace
{

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

rclcpp::QoS control_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
}

/// Green at slow_down_clearance and above, red at zero, so the colour changes where the ramp does.
std_msgs::msg::ColorRGBA clearance_color(double clearance, double slow_down_clearance)
{
  const double green = slow_down_clearance > 0.0
                         ? std::clamp(clearance / slow_down_clearance, 0.0, 1.0)
                         : (clearance > 0.0 ? 1.0 : 0.0);
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(1.0 - green);
  color.g = static_cast<float>(green);
  color.b = 0.0F;
  color.a = 0.8F;
  return color;
}

/// The obstacle model the distance transform reads; unknown is an obstacle, never free space.
eltanin::map::CostTraversabilityModel obstacle_model()
{
  return eltanin::map::CostTraversabilityModel(eltanin::map::INSCRIBED_INFLATED_OBSTACLE, false);
}

/// Declared with defaults so the node starts with no configuration at all; the safe one is off.
limiter::LimiterParameters declare_limiter_parameters(
  rclcpp::Node & node, const eltanin_ros_common::RobotProfile & profile)
{
  limiter::LimiterParameters parameters;
  eltanin::collision::VelocityLimiterParams & limits = parameters.governor.limiter;

  const auto declare = [&node](const char * key, auto fallback) {
    return node.has_parameter(key) ? node.get_parameter(key).get_value<decltype(fallback)>()
                                   : node.declare_parameter(key, fallback);
  };

  parameters.update_frequency = declare(limiter::KEY_UPDATE_FREQUENCY, parameters.update_frequency);
  parameters.cmd_timeout = declare(limiter::KEY_CMD_TIMEOUT, parameters.cmd_timeout);
  parameters.map_timeout = declare(limiter::KEY_MAP_TIMEOUT, parameters.map_timeout);
  parameters.output_enabled_on_startup =
    declare(limiter::KEY_OUTPUT_ENABLED_ON_STARTUP, parameters.output_enabled_on_startup);
  const auto steps =
    declare(limiter::KEY_PREDICTION_STEPS, static_cast<std::int64_t>(limits.prediction_steps));
  if (steps < std::numeric_limits<int>::min() || steps > std::numeric_limits<int>::max()) {
    refuse_to_start(
      node, diagnostic::rejected(
              limiter::KEY_PREDICTION_STEPS, "is " + std::to_string(steps) + ", not an int"));
  }
  limits.prediction_steps = static_cast<int>(steps);
  limits.reaction_time = declare(limiter::KEY_REACTION_TIME, limits.reaction_time);
  limits.collision_margin = declare(limiter::KEY_COLLISION_MARGIN, limits.collision_margin);
  limits.exact_footprint_check =
    declare(limiter::KEY_EXACT_FOOTPRINT_CHECK, limits.exact_footprint_check);
  limits.stop_clearance = declare(limiter::KEY_STOP_CLEARANCE, limits.stop_clearance);
  limits.slow_down_clearance =
    declare(limiter::KEY_SLOW_DOWN_CLEARANCE, limits.slow_down_clearance);
  limits.min_proximity_scale =
    declare(limiter::KEY_MIN_PROXIMITY_SCALE, limits.min_proximity_scale);
  parameters.governor.release_time =
    declare(limiter::KEY_RELEASE_TIME, parameters.governor.release_time);
  parameters.distance_map.max_distance =
    declare(limiter::KEY_CLEARANCE_MAX_DISTANCE, parameters.distance_map.max_distance);

  // The braking law and the footprint are machine values; this node declares neither (C-c / U-3).
  limits.footprint = profile.footprint();
  limits.max_deceleration = profile.limits().max_decel;

  const eltanin_ros_common::ConversionStatus status =
    limiter::validate(parameters, profile.distance_model());
  if (!status.ok()) {
    refuse_to_start(node, status.message());
  }
  for (const std::string & warning : limiter::warnings(parameters, profile)) {
    RCLCPP_WARN(node.get_logger(), "%s", warning.c_str());
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

/// validate() has already named every value; this is the backstop for anything it does not cover.
eltanin::collision::VelocityGovernor require_governor(
  rclcpp::Node & node, const limiter::LimiterParameters & parameters)
{
  std::optional<eltanin::collision::VelocityGovernor> governor =
    eltanin::collision::VelocityGovernor::create(parameters.governor);
  if (!governor.has_value()) {
    refuse_to_start(node, diagnostic::rejected("the limiter parameters", "eltanin refused"));
  }
  return std::move(*governor);
}

}  // namespace

CollisionPredictor::CollisionPredictor(const rclcpp::NodeOptions & options)
: rclcpp::Node("collision_predictor", options),
  profile_(require_robot_profile(*this)),
  parameters_(declare_limiter_parameters(*this, profile_)),
  clock_(get_clock()),
  governor_(require_governor(*this, parameters_)),
  inputs_(parameters_.cmd_timeout, parameters_.map_timeout),
  output_enabled_(parameters_.output_enabled_on_startup),
  tf_buffer_(get_clock())
{
  tf_buffer_.setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface()));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, true);

  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // A relative name, not ~/: the final stage owns /cmd_vel and a remap is how it is moved.
  command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", control_qos());
  predicted_poses_publisher_ =
    create_publisher<nav_msgs::msg::Path>("~/predicted_poses", control_qos());
  footprint_publisher_ =
    create_publisher<geometry_msgs::msg::PolygonStamped>("~/footprint", control_qos());
  swept_footprint_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/swept_footprint", control_qos());
  diagnostic_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", control_qos());

  rclcpp::SubscriptionOptions command_options;
  command_options.callback_group = command_group_;
  command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "path_follower/cmd_vel_raw", control_qos(),
    [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) { on_command(msg); },
    command_options);

  rclcpp::SubscriptionOptions map_options;
  map_options.callback_group = map_group_;
  map_subscription_ = create_subscription<eltanin_msgs::msg::Costmap>(
    "local_map/local_map", control_qos(),
    [this](eltanin_msgs::msg::Costmap::ConstSharedPtr msg) { on_map(msg); }, map_options);

  enable_output_service_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_output",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      on_enable_output(request, response);
    },
    rclcpp::ServicesQoS(), service_group_);

  timer_ = create_timer(
    std::chrono::duration<double>(1.0 / parameters_.update_frequency), [this]() { on_timer(); },
    control_group_);

  RCLCPP_INFO(
    get_logger(), "owning %s at %f Hz with the output %s", command_publisher_->get_topic_name(),
    parameters_.update_frequency, output_enabled_.load() ? "enabled" : "disabled");
}

void CollisionPredictor::on_timer()
{
  const rclcpp::Time now = this->now();
  CycleOutcome cycle = run_cycle(now);
  publish_cycle(cycle, now);
}

CollisionPredictor::CycleOutcome CollisionPredictor::run_cycle(const rclcpp::Time & now)
{
  CycleOutcome cycle;
  // dt is never a gate; it only feeds the hysteresis, which treats a non-positive step as a hold.
  cycle.cycle_dt = clock_.tick().seconds;

  const bool enabled = output_enabled_.load();
  cycle.publish_command = enabled || flush_zero_once_.exchange(false);

  evaluate_cycle(now, cycle);

  // Being disabled suppresses the command, not the checking: the cycle above ran either way, so
  // the transform, the clearance and the predicted poses can be read before anything can move.
  if (!enabled) {
    cycle.command = eltanin::Twist2D{};
    cycle.reason = LimiterReason::OutputDisabled;
    cycle.message = diagnostic::line("the output is disabled; ~/enable_output turns it on");
  }
  return cycle;
}

void CollisionPredictor::evaluate_cycle(const rclcpp::Time & now, CycleOutcome & cycle)
{
  const CommandReading command = inputs_.read_command(now);
  const MapReading map = inputs_.read_map(now);
  cycle.requested = command.value;
  cycle.command_age = command.age_seconds;
  cycle.command_has_age = command.has_age;
  cycle.map_age = map.age_seconds;
  cycle.map_has_age = map.has_age;

  if (command.reason != LimiterReason::None) {
    cycle.reason = command.reason;
    cycle.message =
      command.reason == LimiterReason::CommandRejected
        ? inputs_.command_rejection()
        : diagnostic::line(std::string("the requested command is ") + name_of(command.reason));
    return;
  }
  if (map.reason != LimiterReason::None) {
    cycle.reason = map.reason;
    cycle.message = map.reason == LimiterReason::MapRejected
                      ? inputs_.map_rejection()
                      : diagnostic::line(std::string("the local map is ") + name_of(map.reason));
    return;
  }

  const std::optional<eltanin::Pose2D> robot = robot_pose();
  if (!robot.has_value()) {
    cycle.reason = LimiterReason::NoTransform;
    cycle.message = diagnostic::line(
      "frames.base '" + profile_.frames().base + "' in frames.map '" + profile_.frames().map +
      "' is not available");
    return;
  }
  cycle.transform_ok = true;

  const eltanin::collision::VelocityLimiter::Result result =
    governor_.update(*map.value, profile_.distance_model(), *robot, command.value, cycle.cycle_dt);
  cycle.has_collision = result.has_collision;
  cycle.collision_distance = result.collision_distance;
  cycle.time_to_collision = result.time_to_collision;
  cycle.horizon = result.horizon;
  cycle.clearance = result.clearance;
  cycle.proximity_scale = result.proximity_scale;
  cycle.prediction_truncated =
    result.predicted_poses.size() <
      static_cast<std::size_t>(parameters_.governor.limiter.prediction_steps) + 1 &&
    !result.has_collision;
  cycle.predicted_poses = std::move(result.predicted_poses);

  // Room beside the body at each pose, read per pose for the markers rather than swept to a
  // minimum.
  const double clearance_radius = governor_.limiter().clearance_radius();
  cycle.pose_clearances.reserve(cycle.predicted_poses.size());
  for (const eltanin::Pose2D & pose : cycle.predicted_poses) {
    const std::optional<eltanin::map::MapIndex> index =
      map.value->geometry().world_to_map(pose.position);
    const std::optional<float> distance =
      index.has_value() ? map.value->get(index->x, index->y) : std::nullopt;
    cycle.pose_clearances.push_back(
      distance.has_value() ? static_cast<double>(*distance) - clearance_radius
                           : std::numeric_limits<double>::infinity());
  }

  // A rollout that left the map was never checked, so the command it produced says nothing (U-7).
  if (cycle.prediction_truncated) {
    cycle.reason = LimiterReason::OutsideMap;
    cycle.message = diagnostic::line("the prediction left the local map before it was checked");
    if (outside_map_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", cycle.message.c_str());
    }
    return;
  }

  cycle.command = result.command;
  cycle.reason = (result.has_collision || result.proximity_scale < 1.0) ? LimiterReason::Limited
                                                                        : LimiterReason::None;
}

std::vector<std_msgs::msg::ColorRGBA> CollisionPredictor::clearance_colors(
  const CycleOutcome & cycle) const
{
  const double slow_down = parameters_.governor.limiter.slow_down_clearance;
  std::vector<std_msgs::msg::ColorRGBA> colors;
  colors.reserve(cycle.pose_clearances.size());
  for (const double clearance : cycle.pose_clearances) {
    colors.push_back(clearance_color(clearance, slow_down));
  }
  return colors;
}

void CollisionPredictor::publish_cycle(CycleOutcome & cycle, const rclcpp::Time & now)
{
  if (cycle.publish_command) {
    command_publisher_->publish(eltanin_ros_common::to_twist_msg(cycle.command));
  }

  // Coloured before the poses are moved into the Path below; contact sphere on the last pose.
  const eltanin_ros_common::ConversionResult<visualization_msgs::msg::MarkerArray> swept =
    eltanin_ros_common::to_swept_footprint_markers(
      eltanin::Path(cycle.predicted_poses), governor_.limiter().footprint(),
      clearance_colors(cycle), profile_.frames().map, now, cycle.has_collision);
  if (swept.ok()) {
    swept_footprint_publisher_->publish(swept.value());
  }

  // An empty path rather than the previous one: keeping a stale prediction on screen would lie.
  const eltanin_ros_common::ConversionResult<nav_msgs::msg::Path> path =
    eltanin_ros_common::to_path_msg(
      eltanin::Path(std::move(cycle.predicted_poses)), profile_.frames().map, now);
  if (path.ok()) {
    predicted_poses_publisher_->publish(path.value());
  }
  footprint_publisher_->publish(eltanin_ros_common::to_polygon_msg(
    governor_.limiter().footprint(), profile_.frames().base, now));

  eltanin_ros_common::DiagnosticStatusBuilder builder(
    get_fully_qualified_name(), level_of(cycle.reason), cycle.message);
  builder.add("reason", name_of(cycle.reason))
    .add("output_enabled", output_enabled_.load())
    .add("published", cycle.publish_command)
    .add(
      "command_age",
      cycle.command_has_age ? cycle.command_age : std::numeric_limits<double>::infinity())
    .add("map_age", cycle.map_has_age ? cycle.map_age : std::numeric_limits<double>::infinity())
    .add("transform_ok", cycle.transform_ok)
    .add("requested_linear", cycle.requested.linear.x())
    .add("requested_angular", cycle.requested.angular)
    .add("command_linear", cycle.command.linear.x())
    .add("command_angular", cycle.command.angular)
    .add("has_collision", cycle.has_collision)
    .add("collision_distance", cycle.collision_distance)
    .add("time_to_collision", cycle.time_to_collision)
    .add("horizon", cycle.horizon)
    .add("clearance", cycle.clearance.value_or(std::numeric_limits<double>::quiet_NaN()))
    .add("clearance_available", cycle.clearance.has_value())
    .add("proximity_scale", cycle.proximity_scale)
    .add("predicted_poses", path.ok() ? path.value().poses.size() : std::size_t{0})
    .add("prediction_truncated", cycle.prediction_truncated)
    .add("exact_footprint_check", parameters_.governor.limiter.exact_footprint_check)
    .add("cycle_dt", cycle.cycle_dt);

  diagnostic_msgs::msg::DiagnosticArray diagnostic;
  diagnostic.header.stamp = now;
  diagnostic.status.push_back(builder.take());
  diagnostic_publisher_->publish(diagnostic);
}

void CollisionPredictor::on_command(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  if (msg->header.frame_id != profile_.frames().base) {
    const std::string detail = diagnostic::rejected(
      "the requested command", "is in frame '" + msg->header.frame_id + "', not frames.base '" +
                                 profile_.frames().base + "'");
    inputs_.reject_command(detail);
    if (command_rejected_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    }
    return;
  }

  const eltanin_ros_common::ConversionResult<eltanin::Twist2D> converted =
    eltanin_ros_common::to_twist2d(msg->twist);
  if (!converted.ok()) {
    const std::string detail = diagnostic::flatten(converted.error());
    inputs_.reject_command(detail);
    if (command_rejected_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    }
    return;
  }

  // The received value, unchanged: no path in this node writes a limited command back here (N-2).
  inputs_.accept_command(converted.value(), rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));
}

void CollisionPredictor::on_map(eltanin_msgs::msg::Costmap::ConstSharedPtr msg)
{
  const auto reject = [this](const std::string & detail) {
    inputs_.reject_map(detail);
    if (map_rejected_latch_.should_warn()) {
      RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    }
  };

  if (msg->header.frame_id != profile_.frames().map) {
    reject(diagnostic::rejected(
      "the local map", "is in frame '" + msg->header.frame_id + "', not frames.map '" +
                         profile_.frames().map + "'"));
    return;
  }
  const eltanin_ros_common::ConversionResult<eltanin::map::Costmap> costmap =
    eltanin_ros_common::to_costmap(*msg);
  if (!costmap.ok()) {
    reject(diagnostic::flatten(costmap.error()));
    return;
  }

  // Converted and transformed once here; the timer only ever copies the pointer that comes out.
  std::optional<eltanin::map::DistanceMap> distances =
    eltanin::map::build_distance_map(costmap.value(), obstacle_model(), parameters_.distance_map);
  if (!distances.has_value()) {
    reject(diagnostic::rejected("the local map", "carries no usable distance field"));
    return;
  }

  inputs_.accept_map(
    std::make_shared<const eltanin::map::DistanceMap>(std::move(*distances)),
    rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));
}

void CollisionPredictor::on_enable_output(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  const bool previous = output_enabled_.exchange(request->data);
  if (previous && !request->data) {
    flush_zero_once_.store(true);
  }
  response->success = true;
  response->message = diagnostic::line(
    request->data ? "the output is enabled; the robot moves as soon as both inputs are fresh"
                  : "the output is disabled after one last zero command");
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

std::optional<eltanin::Pose2D> CollisionPredictor::robot_pose()
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
  return converted.value().transform.to_pose();
}

}  // namespace eltanin_controller

RCLCPP_COMPONENTS_REGISTER_NODE(eltanin_controller::CollisionPredictor)
