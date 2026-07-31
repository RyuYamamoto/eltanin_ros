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

#ifndef ELTANIN_ROS_COMMON__DIAGNOSTIC_HPP_
#define ELTANIN_ROS_COMMON__DIAGNOSTIC_HPP_

#include <string>
#include <string_view>

namespace eltanin_ros_common::diagnostic
{

inline constexpr std::string_view PREFIX = "eltanin_ros_common: ";
inline constexpr std::string_view REJECTED = "rejected ";

/// The single shape every rejection has: who rejected it, what was rejected, and what was wrong.
inline std::string rejected(std::string_view subject, std::string_view violation)
{
  return std::string(PREFIX) + std::string(REJECTED) + std::string(subject) + ": " +
         std::string(violation);
}

/// A message from another conversion, with the prefix removed so one line never repeats it.
inline std::string nested(const std::string & message)
{
  std::string_view view{message};
  if (view.starts_with(PREFIX)) {
    view.remove_prefix(PREFIX.size());
  }
  if (view.starts_with(REJECTED)) {
    view.remove_prefix(REJECTED.size());
  }
  return std::string(view);
}

}  // namespace eltanin_ros_common::diagnostic

#endif  // ELTANIN_ROS_COMMON__DIAGNOSTIC_HPP_
