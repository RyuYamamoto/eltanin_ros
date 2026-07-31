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

#ifndef ELTANIN_ROS_COMMON__CONVERSION_RESULT_HPP_
#define ELTANIN_ROS_COMMON__CONVERSION_RESULT_HPP_

#include <cassert>
#include <optional>
#include <string>
#include <utility>

namespace eltanin_ros_common
{

/// Outcome of a check that produces no value; failure is returned, never thrown.
class ConversionStatus
{
public:
  static ConversionStatus success() { return ConversionStatus(std::string{}); }

  /// Precondition: a non-empty single line. An empty message would read as success.
  static ConversionStatus failure(std::string message)
  {
    assert(!message.empty());
    return ConversionStatus(std::move(message));
  }

  bool ok() const noexcept { return message_.empty(); }

  explicit operator bool() const noexcept { return ok(); }

  /// Empty on success; one line with no newline on failure.
  const std::string & message() const noexcept { return message_; }

private:
  explicit ConversionStatus(std::string message) : message_(std::move(message)) {}

  std::string message_;
};

/// Either a converted value or one line saying why the input was rejected.
template <class T>
class ConversionResult
{
public:
  static ConversionResult success(T value) { return ConversionResult(std::move(value), {}); }

  /// Precondition: a non-empty single line.
  static ConversionResult failure(std::string message)
  {
    assert(!message.empty());
    return ConversionResult(std::nullopt, std::move(message));
  }

  bool ok() const noexcept { return value_.has_value(); }

  explicit operator bool() const noexcept { return ok(); }

  /// Precondition: ok(), by assert only. Move it out with std::move(result.value()).
  const T & value() const noexcept
  {
    assert(ok());
    return *value_;
  }

  T & value() noexcept
  {
    assert(ok());
    return *value_;
  }

  /// Empty on success; one line with no newline on failure.
  const std::string & error() const noexcept { return error_; }

private:
  ConversionResult(std::optional<T> value, std::string error)
  : value_(std::move(value)), error_(std::move(error))
  {
  }

  std::optional<T> value_;
  std::string error_;
};

}  // namespace eltanin_ros_common

#endif  // ELTANIN_ROS_COMMON__CONVERSION_RESULT_HPP_
