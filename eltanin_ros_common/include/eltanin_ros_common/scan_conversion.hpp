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

#ifndef ELTANIN_ROS_COMMON__SCAN_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__SCAN_CONVERSION_HPP_

#include "eltanin_ros_common/conversion_result.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <eltanin/sensor/scan_projection.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>

namespace eltanin_ros_common
{

/// ScanData carries neither frame nor time, so they travel with it and cannot be filled in by hand.
struct StampedScan
{
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
  eltanin::sensor::ScanData scan;
};

/// Copies the ranges unchanged; whether inf and NaN mean anything is the projection's call.
ConversionResult<StampedScan> to_scan(const sensor_msgs::msg::LaserScan & msg);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__SCAN_CONVERSION_HPP_
