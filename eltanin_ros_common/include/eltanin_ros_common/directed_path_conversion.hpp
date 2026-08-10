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

#ifndef ELTANIN_ROS_COMMON__DIRECTED_PATH_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__DIRECTED_PATH_CONVERSION_HPP_

#include "eltanin_ros_common/conversion_result.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <eltanin/core/path.hpp>

#include <eltanin_msgs/msg/directed_path.hpp>

#include <cstdint>
#include <string>

namespace eltanin_ros_common
{

/// The wire value of one eltanin::Direction; the enum's declaration order is the wire format.
std::uint8_t to_direction_msg(eltanin::Direction direction) noexcept;

/// nullopt for a value no eltanin::Direction has.
ConversionResult<eltanin::Direction> to_direction(std::uint8_t value);

/// An empty direction array means the whole path is forward, exactly as eltanin::Path reads it.
ConversionResult<eltanin_msgs::msg::DirectedPath> to_directed_path_msg(
  const eltanin::Path & path, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

/// Refuses a direction array that is neither empty nor one shorter than the pose array.
ConversionResult<eltanin::Path> to_path(const eltanin_msgs::msg::DirectedPath & msg);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__DIRECTED_PATH_CONVERSION_HPP_
