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

#ifndef ELTANIN_ROS_COMMON__DIAGNOSTIC_CONVERSION_HPP_
#define ELTANIN_ROS_COMMON__DIAGNOSTIC_CONVERSION_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstdint>
#include <string>
#include <utility>

namespace eltanin_ros_common
{

/// The one place a key name is spelled and a value is formatted; every node builds its status here.
class DiagnosticStatusBuilder
{
public:
  DiagnosticStatusBuilder(std::string name, std::uint8_t level, std::string message);

  /// Trailing zeros are trimmed, so 0.5 reads as "0.5"; inf and nan keep the C++ spelling.
  DiagnosticStatusBuilder & add(const std::string & key, double value);

  DiagnosticStatusBuilder & add(const std::string & key, int value);

  DiagnosticStatusBuilder & add(const std::string & key, std::size_t value);

  /// "true" or "false"; never 1 or 0, which would read like a count.
  DiagnosticStatusBuilder & add(const std::string & key, bool value);

  DiagnosticStatusBuilder & add(const std::string & key, const std::string & value);

  DiagnosticStatusBuilder & add(const std::string & key, const char * value);

  const diagnostic_msgs::msg::DiagnosticStatus & status() const noexcept { return status_; }

  diagnostic_msgs::msg::DiagnosticStatus take() { return std::move(status_); }

private:
  diagnostic_msgs::msg::DiagnosticStatus status_;
};

/// The spelling every double in a diagnostic value gets; exposed so tests can pin it in one place.
std::string to_diagnostic_value(double value);

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__DIAGNOSTIC_CONVERSION_HPP_
