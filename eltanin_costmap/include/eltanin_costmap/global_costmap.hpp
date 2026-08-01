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

#ifndef ELTANIN_COSTMAP__GLOBAL_COSTMAP_HPP_
#define ELTANIN_COSTMAP__GLOBAL_COSTMAP_HPP_

#include "eltanin_costmap/global_costmap_state.hpp"

#include <eltanin_ros_common/cost_conversion.hpp>
#include <eltanin_ros_common/robot_profile.hpp>
#include <eltanin_ros_common/warn_once.hpp>
#include <rclcpp/rclcpp.hpp>

#include <eltanin_msgs/msg/costmap.hpp>
#include <eltanin_msgs/msg/costmap_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mutex>
#include <optional>

namespace eltanin_costmap
{

/// Owns the inflated whole-area costmap; rebuilt when /map arrives and when ~/update is called.
class GlobalCostmap : public rclcpp::Node
{
public:
  explicit GlobalCostmap(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Trigger = std_srvs::srv::Trigger;

  void on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  void on_local_map(eltanin_msgs::msg::Costmap::ConstSharedPtr msg);

  void on_update(const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);

  void on_clear_observations(
    const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);

  /// Whole area, then visualization, then the patch, all under one stamp; false if any was dropped.
  bool publish_all(
    const GlobalCostmapState & state, const std::optional<eltanin::map::CellRect> & patch,
    const rclcpp::Time & stamp);

  /// A new static map is a new set of conditions, so the warnings start over with it.
  void reset_latches() noexcept;

  /// Read once at construction and never again, the same as declare_robot_profile() behaves.
  const eltanin_ros_common::RobotProfile profile_;
  const eltanin_ros_common::OccupancyThresholds thresholds_;
  const bool inflate_unknown_;
  const bool publish_visualization_;

  rclcpp::CallbackGroup::SharedPtr map_group_;
  rclcpp::CallbackGroup::SharedPtr observation_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<eltanin_msgs::msg::Costmap>::SharedPtr local_map_subscription_;
  rclcpp::Publisher<eltanin_msgs::msg::Costmap>::SharedPtr costmap_publisher_;
  rclcpp::Publisher<eltanin_msgs::msg::CostmapUpdate>::SharedPtr update_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr visual_publisher_;
  rclcpp::Service<Trigger>::SharedPtr update_service_;
  rclcpp::Service<Trigger>::SharedPtr clear_service_;

  /// One mutex around everything below it; callbacks take it and leave (design 3.3).
  std::mutex mutex_;
  std::optional<GlobalCostmapState> state_;
  eltanin_ros_common::WarnOnceLatch no_map_latch_;
  eltanin_ros_common::WarnOnceLatch local_frame_latch_;
  eltanin_ros_common::WarnOnceLatch conversion_latch_;
  eltanin_ros_common::WarnOnceLatch resolution_latch_;
  eltanin_ros_common::WarnOnceLatch overlap_latch_;
};

}  // namespace eltanin_costmap

#endif  // ELTANIN_COSTMAP__GLOBAL_COSTMAP_HPP_
