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

#include <eltanin_msgs/msg/costmap.hpp>
#include <eltanin_msgs/msg/costmap_update.hpp>

#include <cstdint>
#include <memory>
#include <mutex>

namespace eltanin_planner
{

/// Answers ~/compute_path_to_pose with an A* path, and with the reason when there is no path.
class GlobalPathPlanner : public rclcpp::Node
{
public:
  explicit GlobalPathPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_costmap(eltanin_msgs::msg::Costmap::ConstSharedPtr msg);
  void on_costmap_update(eltanin_msgs::msg::CostmapUpdate::ConstSharedPtr msg);

  /// The pointer a plan runs against; taken under the lock and then read without it.
  std::shared_ptr<const eltanin::map::Costmap> snapshot() const;

  const eltanin_ros_common::RobotProfile profile_;
  const PlannerParameters parameters_;
  /// The threshold global_costmap inflated with; the same robot.* yaml has to reach both nodes.
  const eltanin::map::CostTraversabilityModel model_;

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
};

}  // namespace eltanin_planner

#endif  // ELTANIN_PLANNER__GLOBAL_PATH_PLANNER_HPP_
