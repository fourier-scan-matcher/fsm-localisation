// Copyright 2026 Alexandros Filotheou
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @brief The two bounds that stop the grid path spinning forever on an
 * estimate standing in unmapped or blocked space.
 * @details
 * `Utils::generatePose`'s grid branch gives up after a fixed number of draws
 * and `Match::l2recovery`'s grid branch gives up after a fixed number of
 * attempts at that same draw, each reporting through the diagnostic sink
 * before it does. Neither bound has ever been reached by a test: every grid
 * built elsewhere in this package has free space somewhere in it. The map
 * below has none, so both bounds are reached on every run, which is what
 * lets this file assert that they are reached at all rather than trust that
 * they would be.
 * The ring path is not tested here. It has no such bound, on purpose, and
 * that is unchanged by any of this.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "fsm_localisation/fsm_core.hpp"

namespace
{
class EntirelyBlockedGrid : public FSM::GridReference
{
 public:

  bool isFree(const double, const double) const noexcept override
  {
    return false;
  }

  std::vector<double> ranges(
    const FSM::Pose&, const unsigned int, const double) const override
  {
    return {};
  }
};

}

TEST(LivenessGuards, TheInnerDrawBoundIsReachedAndReportedRatherThanSpinning)
{
  std::vector<std::string> reports;
  FSM::Diagnostics::setSink([&reports](const std::string& message)
  {
    reports.push_back(message);
  });

  const EntirelyBlockedGrid grid;
  const FSM::Pose base_pose{.x = 0.0, .y = 0.0, .t = 0.0};

  const std::optional<FSM::Pose> drawn = FSM::Utils::generatePose(
    base_pose, FSM::Reference(grid, 10.0), 1.0, 1.0, 0.0,
    FSM::RaySearch::angular, 12345U);

  FSM::Diagnostics::setSink(nullptr);

  EXPECT_FALSE(drawn.has_value())
    << "no position is free in this grid, so the draw loop must exhaust its "
       "bound and report failure rather than return a candidate";

  const bool reported = std::find_if(
    reports.begin(), reports.end(),
    [](const std::string& message)
    {
      return message.find("generatePose") != std::string::npos &&
        message.find("exhausted") != std::string::npos;
    }) != reports.end();

  EXPECT_TRUE(reported)
    << "the draw bound was exhausted, so it must reach the diagnostic sink; "
       "the sink instead received " << reports.size() << " message(s)";
}

TEST(LivenessGuards, TheOuterRetryBoundIsReachedAndReturnsTheInputPoseUnchanged)
{
  std::vector<std::string> reports;
  FSM::Diagnostics::setSink([&reports](const std::string& message)
  {
    reports.push_back(message);
  });

  const EntirelyBlockedGrid grid;
  const FSM::Pose input_pose{.x = 1.5, .y = -2.5, .t = 0.7};

  const FSM::Pose output_pose = FSM::Match::l2recovery(
    input_pose, FSM::Reference(grid, 10.0), 1.0, 1.0,
    FSM::RaySearch::angular, 54321U);

  FSM::Diagnostics::setSink(nullptr);

  EXPECT_EQ(output_pose.x, input_pose.x);
  EXPECT_EQ(output_pose.y, input_pose.y);
  EXPECT_EQ(output_pose.t, input_pose.t);

  const bool inner_reported = std::find_if(
    reports.begin(), reports.end(),
    [](const std::string& message)
    {
      return message.find("generatePose") != std::string::npos &&
        message.find("exhausted") != std::string::npos;
    }) != reports.end();

  const bool outer_reported = std::find_if(
    reports.begin(), reports.end(),
    [](const std::string& message)
    {
      return message.find("l2recovery") != std::string::npos &&
        message.find("exhausted") != std::string::npos;
    }) != reports.end();

  EXPECT_TRUE(inner_reported)
    << "the outer bound retries the inner draw loop, so the inner loop's own "
       "report must reach the sink at least once along the way";

  EXPECT_TRUE(outer_reported)
    << "the attempt bound was exhausted, so it must reach the diagnostic "
       "sink in its own right, not only through the inner loop's report; the "
       "sink received " << reports.size() << " message(s)";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
