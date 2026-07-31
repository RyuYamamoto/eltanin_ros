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

#ifndef ELTANIN_ROS_COMMON__WARN_ONCE_HPP_
#define ELTANIN_ROS_COMMON__WARN_ONCE_HPP_

namespace eltanin_ros_common
{

/// Answers "should this be logged now"; the node owns one per condition and logs it itself.
class WarnOnceLatch
{
public:
  /// True on the first call only, so a per-cycle condition is reported once and not every cycle.
  bool should_warn() noexcept
  {
    if (warned_) {
      return false;
    }
    warned_ = true;
    return true;
  }

private:
  bool warned_{false};
};

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__WARN_ONCE_HPP_
