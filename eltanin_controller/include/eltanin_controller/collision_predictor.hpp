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

#ifndef ELTANIN_CONTROLLER__COLLISION_PREDICTOR_HPP_
#define ELTANIN_CONTROLLER__COLLISION_PREDICTOR_HPP_

#include "eltanin_controller/limiter_diagnostic.hpp"
#include "eltanin_controller/limiter_inputs.hpp"
#include "eltanin_controller/limiter_parameters.hpp"

#include <eltanin/collision/velocity_governor.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin_ros_common/robot_profile.hpp>
#include <eltanin_ros_common/timing.hpp>
#include <eltanin_ros_common/warn_once.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <eltanin_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace eltanin_controller
{

/// The single owner of /cmd_vel; it runs outside any action and stops on its own when an input
/// dies.
class CollisionPredictor : public rclcpp::Node
{
public:
  explicit CollisionPredictor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// What one cycle decided; every failure is a value here, never a return from on_timer().
  struct CycleOutcome
  {
    /// False only while the output is disabled and no transition zero is owed (D-22 / U-8).
    bool publish_command{false};
    /// Zero unless the governor ran; nothing else ever writes it.
    eltanin::Twist2D command{};
    /// What cmd_vel_raw asked for, for the diagnostic; never written back into the input.
    eltanin::Twist2D requested{};
    LimiterReason reason{LimiterReason::None};
    bool has_collision{false};
    double collision_distance{std::numeric_limits<double>::infinity()};
    double time_to_collision{std::numeric_limits<double>::infinity()};
    double horizon{0.0};
    std::optional<double> clearance{};
    double proximity_scale{1.0};
    std::vector<eltanin::Pose2D> predicted_poses{};
    bool prediction_truncated{false};
    double cycle_dt{0.0};
    double command_age{0.0};
    bool command_has_age{false};
    double map_age{0.0};
    bool map_has_age{false};
    bool transform_ok{false};
    std::string message;
  };

  void on_timer();

  CycleOutcome run_cycle(const rclcpp::Time & now);

  /// The whole pipeline, run whether or not the output is enabled; failures stay in the outcome.
  void evaluate_cycle(const rclcpp::Time & now, CycleOutcome & cycle);

  /// The single publish path; the only branch in it is whether /cmd_vel goes out at all.
  void publish_cycle(CycleOutcome & cycle, const rclcpp::Time & now);

  void on_command(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  void on_map(eltanin_msgs::msg::Costmap::ConstSharedPtr msg);

  void on_enable_output(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /// frames.map to frames.base at the latest available time, with a timeout of 0.
  std::optional<eltanin::Pose2D> robot_pose();

  const eltanin_ros_common::RobotProfile profile_;
  const limiter::LimiterParameters parameters_;

  eltanin_ros_common::PeriodicClock clock_;
  eltanin::collision::VelocityGovernor governor_;
  LimiterInputs inputs_;

  /// Read by the timer, written by the service; false at startup on the robot (D-22 / R-5).
  std::atomic<bool> output_enabled_;
  /// Set when the output is switched off while running, so disabling means stopping (U-8).
  std::atomic<bool> flush_zero_once_{false};

  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr map_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  rclcpp::Subscription<eltanin_msgs::msg::Costmap>::SharedPtr map_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_poses_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_output_service_;

  eltanin_ros_common::WarnOnceLatch command_rejected_latch_;
  eltanin_ros_common::WarnOnceLatch map_rejected_latch_;
  eltanin_ros_common::WarnOnceLatch transform_latch_;
  eltanin_ros_common::WarnOnceLatch outside_map_latch_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace eltanin_controller

#endif  // ELTANIN_CONTROLLER__COLLISION_PREDICTOR_HPP_
