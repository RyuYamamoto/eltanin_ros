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

#ifndef ELTANIN_CONTROLLER__PATH_FOLLOWER_HPP_
#define ELTANIN_CONTROLLER__PATH_FOLLOWER_HPP_

#include "eltanin_controller/command_composition.hpp"
#include "eltanin_controller/follower_parameters.hpp"
#include "eltanin_controller/path_input.hpp"

#include <eltanin/control/goal_approach.hpp>
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin_ros_common/conversion_result.hpp>
#include <eltanin_ros_common/robot_profile.hpp>
#include <eltanin_ros_common/timing.hpp>
#include <eltanin_ros_common/warn_once.hpp>
#include <rclcpp/rclcpp.hpp>

#include <eltanin_msgs/msg/follower_diagnostic.hpp>
#include <eltanin_msgs/msg/trajectory2_d.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace eltanin_controller
{

/// Turns a path into a requested velocity, and publishes one every cycle whether it has one or not.
class PathFollower : public rclcpp::Node
{
public:
  explicit PathFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// What one cycle produced; run_cycle() returns it and nothing here returns without one.
  struct CycleOutcome
  {
    composition::Outcome outcome{};
    /// These defaults are what GoalApproach reports for an empty path, so the vocabulary is one.
    double remaining_arc{std::numeric_limits<double>::infinity()};
    double position_error{std::numeric_limits<double>::infinity()};
    double yaw_error{0.0};
    double align_elapsed{0.0};
    double control_dt{0.0};
    /// Built only when the reason is not REASON_NONE, so a tracking cycle allocates no string.
    std::string message;
  };

  void on_timer();

  /// Failures leave here as values, never as a return from on_timer(); the publish cannot be lost.
  CycleOutcome run_cycle(const rclcpp::Time & now);

  /// The single publish path: the command and the diagnostic always, the point when there is one.
  void publish_cycle(const CycleOutcome & cycle, const rclcpp::Time & now);

  void on_path(nav_msgs::msg::Path::ConstSharedPtr msg);
  void on_trajectory(eltanin_msgs::msg::Trajectory2D::ConstSharedPtr msg);

  /// Accepts what either subscription produced; the frame check and the bookkeeping are shared.
  void accept_path(
    const std_msgs::msg::Header & header,
    const eltanin_ros_common::ConversionResult<eltanin::Path> & converted);

  void on_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// frames.map to frames.base at the latest available time, with a timeout of 0.
  std::optional<eltanin::Pose2D> robot_pose();

  const eltanin_ros_common::RobotProfile profile_;
  const FollowerParameters parameters_;

  eltanin_ros_common::PeriodicClock clock_;
  eltanin::control::PurePursuit pursuit_;
  eltanin::control::GoalApproach approach_;

  /// Set by ~/reset alone; the path subscription never touches the two controllers.
  std::atomic<bool> reset_requested_{false};

  mutable std::mutex input_mutex_;
  PathInput input_;

  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<eltanin_msgs::msg::Trajectory2D>::SharedPtr trajectory_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<eltanin_msgs::msg::FollowerDiagnostic>::SharedPtr diagnostic_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  eltanin_ros_common::WarnOnceLatch no_input_latch_;
  eltanin_ros_common::WarnOnceLatch stale_latch_;
  eltanin_ros_common::WarnOnceLatch empty_latch_;
  eltanin_ros_common::WarnOnceLatch rejected_latch_;
  eltanin_ros_common::WarnOnceLatch transform_latch_;
  eltanin_ros_common::WarnOnceLatch no_dt_latch_;
  eltanin_ros_common::WarnOnceLatch planarity_latch_;
  eltanin_ros_common::WarnOnceLatch controller_latch_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace eltanin_controller

#endif  // ELTANIN_CONTROLLER__PATH_FOLLOWER_HPP_
