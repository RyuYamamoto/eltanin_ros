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

#ifndef ELTANIN_ROS_COMMON__TRAJECTORY_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__TRAJECTORY_CONVERSION_HPP_

#include "eltanin_ros_common/conversion_result.hpp"

#include <eltanin/core/path.hpp>

#include <eltanin_msgs/msg/trajectory2_d.hpp>

namespace eltanin_ros_common
{

/// Drops the speed no eltanin type can carry, but keeps its sign as the segment direction; an
/// empty trajectory is not an error.
ConversionResult<eltanin::Path> to_path(const eltanin_msgs::msg::Trajectory2D & msg);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__TRAJECTORY_CONVERSION_HPP_
