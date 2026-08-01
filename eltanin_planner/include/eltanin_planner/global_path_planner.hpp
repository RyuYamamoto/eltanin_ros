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
#include <eltanin_ros_common/robot_profile.hpp>
#include <rclcpp/rclcpp.hpp>

namespace eltanin_planner
{

/// Answers ~/compute_path_to_pose with an A* path, and with the reason when there is no path.
class GlobalPathPlanner : public rclcpp::Node
{
public:
  explicit GlobalPathPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  const eltanin_ros_common::RobotProfile profile_;
  const PlannerParameters parameters_;
  /// The threshold global_costmap inflated with; the same robot.* yaml has to reach both nodes.
  const eltanin::map::CostTraversabilityModel model_;
};

}  // namespace eltanin_planner

#endif  // ELTANIN_PLANNER__GLOBAL_PATH_PLANNER_HPP_
