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

#ifndef ELTANIN_ROS_COMMON__PATH_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__PATH_CONVERSION_HPP_

#include "eltanin_ros_common/conversion_result.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <eltanin/core/path.hpp>

#include <nav_msgs/msg/path.hpp>

#include <string>

namespace eltanin_ros_common
{

/// Every PoseStamped repeats the path header, as nav2 and navyu do. An empty path is not an error.
ConversionResult<nav_msgs::msg::Path> to_path_msg(
  const eltanin::Path & path, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

/// A per-pose frame_id is accepted only when empty or equal to the path's; never assumed to be map.
ConversionResult<eltanin::Path> to_path(const nav_msgs::msg::Path & msg);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__PATH_CONVERSION_HPP_
