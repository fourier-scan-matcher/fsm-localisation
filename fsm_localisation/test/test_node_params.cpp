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
 * @brief What the node refuses to start with, and what it starts with anyway.
 * @details
 * The refusals that can be decided before any message arrives are decided in
 * the constructor, so the node either comes up usable or does not come up. This
 * file constructs the node directly with parameter overrides rather than
 * launching it, which is what makes each case one line of setup and its
 * outcome a thrown exception rather than a log line somebody has to read.
 * **The load bearing property here is that only the running mode's settings are
 * read.** One configuration file is meant to serve both modes, so it will
 * always carry settings for the mode that is not running, and those must not be
 * able to stop the node. Every refusal below therefore comes with its mirror:
 * the same nonsense value, under the other mode, starting cleanly.
 */

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fsm_localisation/fsm_localisation_interface.hpp"

namespace
{
using fsm_localisation::Interface;

rclcpp::NodeOptions optionsWith(
  const std::vector<rclcpp::Parameter>& overrides)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return options;
}

rclcpp::NodeOptions inOdometryModeWith(
  const std::vector<rclcpp::Parameter>& overrides)
{
  std::vector<rclcpp::Parameter> all{rclcpp::Parameter("mode", "odometry")};
  all.insert(all.end(), overrides.begin(), overrides.end());
  return optionsWith(all);
}

rclcpp::NodeOptions inAmeliorationModeWith(
  const std::vector<rclcpp::Parameter>& overrides)
{
  std::vector<rclcpp::Parameter> all{rclcpp::Parameter("mode", "amelioration")};
  all.insert(all.end(), overrides.begin(), overrides.end());
  return optionsWith(all);
}

}

TEST(NodeParameters, AnAbsentModeIsRefused)
{
  EXPECT_THROW(Interface{optionsWith({})}, std::invalid_argument);
}

TEST(NodeParameters, AnUnrecognisedModeIsRefused)
{
  EXPECT_THROW(
    Interface{optionsWith({rclcpp::Parameter("mode", "localisation")})},
    std::invalid_argument);
}

TEST(NodeParameters, EitherModeStartsFromItsDefaults)
{
  EXPECT_NO_THROW(Interface{inOdometryModeWith({})});
  EXPECT_NO_THROW(Interface{inAmeliorationModeWith({})});
}

TEST(NodeParameters, TheIdleModesSettingsCannotStopTheNode)
{
  EXPECT_NO_THROW(Interface{inOdometryModeWith({
        rclcpp::Parameter("map_scan_method", "nonsense"),
        rclcpp::Parameter("occupancy_threshold", 900),
        rclcpp::Parameter("tf_lookup_timeout", -1.0),
        rclcpp::Parameter("map_topic", ""),
        rclcpp::Parameter("robot_frame_id", "/leading_slash")})});

  EXPECT_NO_THROW(Interface{inAmeliorationModeWith({
        rclcpp::Parameter("lo_topic", ""),
        rclcpp::Parameter("path_estimate_topic", ""),
        rclcpp::Parameter("initial_pose_topic", "")})});
}

TEST(NodeParameters, AnOccupancyThresholdOutsideAPercentageIsRefused)
{
  EXPECT_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("occupancy_threshold", 101)})},
    std::invalid_argument);

  EXPECT_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("occupancy_threshold", -1)})},
    std::invalid_argument);
}

TEST(NodeParameters, AnUnrecognisedCastingMethodIsRefused)
{
  EXPECT_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("map_scan_method", "raymarching")})},
    std::invalid_argument);
}

TEST(NodeParameters, ANegativeTransformTimeoutIsRefused)
{
  EXPECT_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("tf_lookup_timeout", -0.1)})},
    std::invalid_argument);

  EXPECT_NO_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("tf_lookup_timeout", 0.0)})});
}

TEST(NodeParameters, AnEmptyRequiredTopicIsRefused)
{
  EXPECT_THROW(
    Interface{inOdometryModeWith({rclcpp::Parameter("lo_topic", "")})},
    std::invalid_argument);

  EXPECT_THROW(
    Interface{inAmeliorationModeWith({rclcpp::Parameter("map_topic", "")})},
    std::invalid_argument);

  EXPECT_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("pose_estimate_input_topic", "")})},
    std::invalid_argument);
}

TEST(NodeParameters, AnEmptyMapScanTopicMeansNoMapScan)
{
  EXPECT_NO_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("map_scan_topic", "")})});
}

TEST(NodeParameters, AFrameNameBeginningWithASlashIsRefused)
{
  EXPECT_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("robot_frame_id", "/base_footprint")})},
    std::invalid_argument);

  EXPECT_THROW(
    Interface{inAmeliorationModeWith({
        rclcpp::Parameter("estimate_frame_id", "")})},
    std::invalid_argument);
}

TEST(NodeParameters, TheMatchersOwnRefusalsStillHoldInBothModes)
{
  EXPECT_THROW(
    Interface{inOdometryModeWith({rclcpp::Parameter("num_iterations", 0)})},
    std::invalid_argument);

  EXPECT_THROW(
    Interface{inAmeliorationModeWith({rclcpp::Parameter("num_iterations", 0)})},
    std::invalid_argument);

  EXPECT_THROW(
    Interface{inAmeliorationModeWith({rclcpp::Parameter("ray_search", "near")})},
    std::invalid_argument);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
