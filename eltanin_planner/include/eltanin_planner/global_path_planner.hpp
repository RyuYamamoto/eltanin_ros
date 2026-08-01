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

#ifndef ELTANIN_PLANNER__GLOBAL_PATH_PLANNER_HPP_
#define ELTANIN_PLANNER__GLOBAL_PATH_PLANNER_HPP_

#include "eltanin_planner/planner_parameters.hpp"

#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin_ros_common/robot_profile.hpp>
#include <eltanin_ros_common/warn_once.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <eltanin_msgs/action/compute_path_to_pose.hpp>
#include <eltanin_msgs/msg/costmap.hpp>
#include <eltanin_msgs/msg/costmap_update.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace eltanin_planner
{

/// Answers ~/compute_path_to_pose with an A* path, and with the reason when there is no path.
class GlobalPathPlanner : public rclcpp::Node
{
public:
  explicit GlobalPathPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Stops the worker and terminates whatever goal it held; a destroyed handle never answers.
  ~GlobalPathPlanner() override;

private:
  using Action = eltanin_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  void on_costmap(eltanin_msgs::msg::Costmap::ConstSharedPtr msg);
  void on_costmap_update(eltanin_msgs::msg::CostmapUpdate::ConstSharedPtr msg);

  /// Never inspects the goal: a rejection carries no result, so no outcome could be reported.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> & handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> & handle);

  void run();
  void execute(const std::shared_ptr<GoalHandle> & handle);

  /// The only place a cancellation or a preemption is observed; the search cannot be interrupted.
  bool interrupted(const std::shared_ptr<GoalHandle> & handle);
  void terminate_preempted(const std::shared_ptr<GoalHandle> & handle);
  void abort_with(
    const std::shared_ptr<GoalHandle> & handle, std::uint8_t outcome, const std::string & message);

  /// The pointer a plan runs against; taken under the lock and then read without it.
  std::shared_ptr<const eltanin::map::Costmap> snapshot() const;

  /// Resolves one PoseStamped into frames.map; an empty frame_id is a rejection, never an
  /// assumption.
  eltanin_ros_common::ConversionResult<eltanin::Pose2D> resolve_pose(
    const geometry_msgs::msg::PoseStamped & msg, const char * what);

  /// frames.map to frames.base at the latest available time; the plan starts where the robot is.
  eltanin_ros_common::ConversionResult<eltanin::Pose2D> robot_pose();

  const eltanin_ros_common::RobotProfile profile_;
  const PlannerParameters parameters_;
  /// The threshold global_costmap inflated with; the same robot.* yaml has to reach both nodes.
  const eltanin::map::CostTraversabilityModel model_;
  /// A deliberate departure from the design's timeout of 0, which is for periodic work (D-T7-7).
  const rclcpp::Duration tf_timeout_;

  /// Both costmap subscriptions, so that replacing and patching the belief cannot interleave.
  rclcpp::CallbackGroup::SharedPtr belief_group_;
  rclcpp::Subscription<eltanin_msgs::msg::Costmap>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<eltanin_msgs::msg::CostmapUpdate>::SharedPtr update_subscription_;

  mutable std::mutex belief_mutex_;
  std::shared_ptr<const eltanin::map::Costmap> costmap_;
  /// Raw nanoseconds rather than rclcpp::Time, whose comparison throws on a clock_type mismatch.
  std::int64_t costmap_stamp_ns_{0};

  eltanin_ros_common::WarnOnceLatch whole_frame_latch_;
  eltanin_ros_common::WarnOnceLatch whole_conversion_latch_;
  eltanin_ros_common::WarnOnceLatch patch_no_map_latch_;
  eltanin_ros_common::WarnOnceLatch patch_frame_latch_;
  eltanin_ros_common::WarnOnceLatch patch_reject_latch_;
  eltanin_ros_common::WarnOnceLatch planarity_latch_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// The action server alone; handle_goal, handle_cancel and handle_accepted all return at once.
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  /// Only created when publish_raw_path is set; a topic nobody publishes is not created.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_publisher_;
  /// Same rule, for publish_footprint_path: the footprint laid along the path that was published.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_publisher_;

  /// One worker, so that "at most one plan runs" is a property of the code and not of the executor.
  std::thread worker_;
  std::mutex work_mutex_;
  std::condition_variable work_cv_;
  std::shared_ptr<GoalHandle> pending_;
  std::shared_ptr<GoalHandle> active_;
  bool preempt_active_{false};
  bool stopping_{false};
};

}  // namespace eltanin_planner

#endif  // ELTANIN_PLANNER__GLOBAL_PATH_PLANNER_HPP_
