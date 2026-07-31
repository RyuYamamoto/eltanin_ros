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

#ifndef CONVERSION_TEST_HELPERS_HPP_
#define CONVERSION_TEST_HELPERS_HPP_

#include <gtest/gtest.h>

#include <cstddef>
#include <string>

namespace eltanin_ros_common::test
{

/// Error text is checked by substring, never equality, so rewording is not a breakage.
inline ::testing::AssertionResult contains(const std::string & text, const std::string & fragment)
{
  if (text.find(fragment) != std::string::npos) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "'" << text << "' does not contain '" << fragment << "'";
}

/// Diagnostics have to stay greppable as one line.
inline ::testing::AssertionResult is_one_line(const std::string & text)
{
  if (text.find('\n') == std::string::npos) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "'" << text << "' contains a newline";
}

/// A nested diagnostic must not repeat the prefix; one rejection reads as one sentence.
inline ::testing::AssertionResult names_the_package_once(const std::string & text)
{
  std::size_t count = 0;
  for (std::size_t at = text.find("eltanin_ros_common:"); at != std::string::npos;
       at = text.find("eltanin_ros_common:", at + 1)) {
    ++count;
  }
  if (count == 1) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "'" << text << "' names the package " << count << " times, expected once";
}

}  // namespace eltanin_ros_common::test

#endif  // CONVERSION_TEST_HELPERS_HPP_
