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
 * @brief `Corrector`: an estimate displaced from where the sensor really was,
 *        put back by matching its scan against a map.
 *
 * @details
 * **How a case is built.** A room is drawn as an occupancy grid, a pose inside
 * it is chosen as the truth, and the scan that pose would see is cast from the
 * map itself. That scan is then handed to the corrector along with an estimate
 * displaced from the truth by a known amount. What is measured is whether the
 * corrector puts the estimate back where the scan came from. Casting the scan
 * from the same map the correction reads means this file measures the matcher
 * and not the map: whether the map is a faithful rendering of a room is
 * `test_reference_agreement.cpp`'s question, and it answers it.
 *
 * **The room is not symmetric under any rotation.** A bare rectangle maps onto
 * itself under a half turn, so a heading error of pi could produce a scan
 * indistinguishable from the right one and pass. A pillar off the centre lines
 * removes every such coincidence.
 *
 * **What the answer is held to, and where the bound comes from.** A cast ray
 * stops at a cell, so a map cannot support a position better than one cell
 * however good the match is. That is the bound on position and it belongs to
 * the map rather than to the matcher, which is stated in the specification
 * rather than discovered here.
 *
 * The heading needs the same cell expressed as an angle. A heading wrong by
 * `dt` moves what the sensor would see at range `r` sideways by `r * dt`, so a
 * heading is resolved no better than one cell seen at the ranges the scan
 * actually carries. The bound is therefore `atan(resolution / mean_range)`
 * with the mean taken over the scan under test, which makes it a property of
 * the case rather than a number chosen to pass.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "fsm_localisation/fsm_localisation.hpp"
#include "fsm_localisation/occupancy_grid_map.hpp"

namespace
{

using fsm_localisation::CastingMethod;
using fsm_localisation::CorrectionError;
using fsm_localisation::CorrectionResult;
using fsm_localisation::Corrector;
using fsm_localisation::OccupancyGridMap;
using fsm_localisation::Parameters;
using fsm_localisation::Pose;

const std::int8_t kBlocked = 100;
const std::int8_t kFree = 0;

const int kOccupancyThreshold = 50;

/* 6.05 m by 4.55 m at five centimetres a cell. Odd in both directions so the
 * room has a centre cell rather than a centre corner, and wider than it is
 * tall so a quarter turn cannot be mistaken for no turn. */
const unsigned int kWidth = 121U;
const unsigned int kHeight = 91U;
const double kResolution = 0.05;

/* Beyond the room's diagonal of 7.57 m, so a ray meeting nothing is
 * distinguishable from one meeting the far corner. */
const double kRangeMax = 12.0;

const unsigned int kNumRays = 360U;

/* Off both centre lines and off both diagonals, so the scan from it is not
 * symmetric in any way the room might otherwise have been. */
const Pose kTruth{1.53, 1.07, 0.4};

/*******************************************************************************
 * A closed room with one pillar in it, as an occupancy grid message.
 */
nav_msgs::msg::OccupancyGrid roomMessage()
{
  nav_msgs::msg::OccupancyGrid message;

  message.info.width = kWidth;
  message.info.height = kHeight;
  message.info.resolution = static_cast<float>(kResolution);
  message.info.origin.orientation.w = 1.0;

  message.data.assign(
    static_cast<std::size_t>(kWidth) * kHeight, kFree);

  const auto block = [&message](const unsigned int column, const unsigned int row)
    {
      message.data[static_cast<std::size_t>(row) * kWidth + column] = kBlocked;
    };

  for (unsigned int column = 0U; column < kWidth; column++)
  {
    block(column, 0U);
    block(column, kHeight - 1U);
  }

  for (unsigned int row = 0U; row < kHeight; row++)
  {
    block(0U, row);
    block(kWidth - 1U, row);
  }

  /* The pillar: nine cells, off both centre lines. */
  for (unsigned int column = 80U; column <= 82U; column++)
    for (unsigned int row = 60U; row <= 62U; row++)
      block(column, row);

  return message;
}

/*******************************************************************************
 * A grid with nothing in it at all, so every ray escapes.
 */
nav_msgs::msg::OccupancyGrid emptyMessage()
{
  nav_msgs::msg::OccupancyGrid message;

  message.info.width = kWidth;
  message.info.height = kHeight;
  message.info.resolution = static_cast<float>(kResolution);
  message.info.origin.orientation.w = 1.0;

  message.data.assign(
    static_cast<std::size_t>(kWidth) * kHeight, kFree);

  return message;
}

/*******************************************************************************
 * A map standing behind the interface the corrector reaches a map through, in
 * which exactly one point is free.
 *
 * It exists for one case: the search that recovers a bad match draws positions
 * around the estimate until one is free, and here none ever is. The estimate
 * itself passes the corrector's entry guard, so the match starts; the first
 * step of the translation stage leaves the free point, which is what the core
 * reports as having left the map, and every recovery from there is a search
 * with nothing to find. That is the shape of an estimate that has wandered
 * into unmapped space, reduced to the smallest thing that produces it every
 * time rather than usually.
 */
class OnePointOfFreeSpace : public FSM::GridReference
{
 public:

  explicit OnePointOfFreeSpace(const Pose& free_point)
  : free_point_(free_point)
  {
  }

  bool isFree(const double x, const double y) const noexcept override
  {
    return x == free_point_.x && y == free_point_.y;
  }

  /*
   * Two things are needed of these ranges and they pull in opposite
   * directions. Not every ray may be at the maximum, or the cast before the
   * match refuses the case for the wrong reason. And they must not be a
   * constant either: a scan of one range is a circle centred on the sensor,
   * and a circle held against a circle gives a correction of exactly zero,
   * so the pose never leaves the one free point and the match converges on
   * the spot.
   *
   * The shape below carries one full turn of a sine around the ray fan, and
   * that is the part that matters rather than the amplitude: the translation
   * stage reads the first harmonic of the difference between the scan it was
   * given and the scan it would see, so a shape built from any higher
   * harmonic alone gives a correction of exactly zero just as a constant
   * does. Because the shape does not depend on the pose it is asked from,
   * the correction neither vanishes nor shrinks, so the first step of the
   * translation stage carries the pose off the free point and every step
   * after it keeps the pose off.
   */
  std::vector<double> ranges(
    const Pose&, const unsigned int num_rays,
    const double max_range) const override
  {
    std::vector<double> out(num_rays);

    for (unsigned int i = 0U; i < num_rays; i++)
      out[i] = max_range *
        (0.3 + 0.2 * std::sin(i * 2.0 * M_PI / num_rays));

    return out;
  }

 private:

  Pose free_point_;
};

/*******************************************************************************
 * The parameters every case starts from: the shipping defaults with the
 * recovery search pinned, so a case that recovers gives the same answer twice.
 */
Parameters defaultParameters()
{
  Parameters parameters;
  parameters.rng_seed = 1234U;
  return parameters;
}

/*******************************************************************************
*/
std::shared_ptr<const OccupancyGridMap> mapOf(
  const nav_msgs::msg::OccupancyGrid& message)
{
  const std::expected<std::shared_ptr<const OccupancyGridMap>, std::string> map =
    OccupancyGridMap::build(message, kOccupancyThreshold, CastingMethod::ray_marching);

  return map.has_value() ? *map : nullptr;
}

/*******************************************************************************
 * One cell seen at the ranges this scan actually carries, in radians. See the
 * file header for where this comes from.
 */
double headingBound(const std::vector<double>& scan)
{
  const double mean =
    std::accumulate(scan.begin(), scan.end(), 0.0) / scan.size();

  return std::atan(kResolution / mean);
}

/*******************************************************************************
 * The scan the truth pose sees, cast from the map itself.
 */
std::vector<double> scanFromTruth(const OccupancyGridMap& map)
{
  return map.ranges(kTruth, kNumRays, kRangeMax);
}

}  // namespace

/*******************************************************************************
*/
TEST(Corrector, AnEstimateDisplacedInPositionIsPutBack)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax);

  ASSERT_TRUE(corrected.has_value());

  EXPECT_NEAR(corrected->corrected.x, kTruth.x, kResolution);
  EXPECT_NEAR(corrected->corrected.y, kTruth.y, kResolution);
  EXPECT_NEAR(corrected->corrected.t, kTruth.t, headingBound(scan));
}

/*******************************************************************************
*/
TEST(Corrector, AnEstimateDisplacedInHeadingIsPutBack)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x, kTruth.y, kTruth.t + 0.12};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax);

  ASSERT_TRUE(corrected.has_value());

  EXPECT_NEAR(corrected->corrected.x, kTruth.x, kResolution);
  EXPECT_NEAR(corrected->corrected.y, kTruth.y, kResolution);
  EXPECT_NEAR(corrected->corrected.t, kTruth.t, headingBound(scan));
}

/*******************************************************************************
*/
TEST(Corrector, AnEstimateDisplacedInBothIsPutBack)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t + 0.12};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax);

  ASSERT_TRUE(corrected.has_value());

  EXPECT_NEAR(corrected->corrected.x, kTruth.x, kResolution);
  EXPECT_NEAR(corrected->corrected.y, kTruth.y, kResolution);
  EXPECT_NEAR(corrected->corrected.t, kTruth.t, headingBound(scan));
}

/*******************************************************************************
 * Nothing survives a correction, so the same one asked twice is the same
 * question twice and must give the same answer to the last bit rather than to
 * a tolerance.
 */
TEST(Corrector, TheSameCorrectionTwiceGivesTheSameAnswerExactly)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t + 0.12};

  const std::expected<CorrectionResult, CorrectionError> first =
    corrector.correct(scan, estimate, kRangeMax);
  const std::expected<CorrectionResult, CorrectionError> second =
    corrector.correct(scan, estimate, kRangeMax);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->corrected.x, second->corrected.x);
  EXPECT_EQ(first->corrected.y, second->corrected.y);
  EXPECT_EQ(first->corrected.t, second->corrected.t);
  EXPECT_EQ(first->num_recoveries, second->num_recoveries);
}

/*******************************************************************************
 * The correction before this one cannot reach the one after it, which is the
 * whole of what "holds no state" means from outside.
 */
TEST(Corrector, ACorrectionIsNotAffectedByTheOneBeforeIt)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t};

  const std::expected<CorrectionResult, CorrectionError> alone =
    corrector.correct(scan, estimate, kRangeMax);

  const Corrector second_corrector(defaultParameters(), map);

  ASSERT_TRUE(second_corrector.correct(
      scan, Pose{kTruth.x - 0.13, kTruth.y + 0.07, kTruth.t + 0.2},
      kRangeMax).has_value());

  const std::expected<CorrectionResult, CorrectionError> after =
    second_corrector.correct(scan, estimate, kRangeMax);

  ASSERT_TRUE(alone.has_value());
  ASSERT_TRUE(after.has_value());

  EXPECT_EQ(alone->corrected.x, after->corrected.x);
  EXPECT_EQ(alone->corrected.y, after->corrected.y);
  EXPECT_EQ(alone->corrected.t, after->corrected.t);
}

/*******************************************************************************
*/
TEST(Corrector, AScanShorterThanTheConfiguredSizeIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  Parameters parameters = defaultParameters();
  parameters.size_scan = 180U;

  const Corrector corrector(parameters, map);

  const std::vector<double> scan(179U, 1.0);

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, kTruth, kRangeMax);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::scan_too_short);
}

/*******************************************************************************
 * All three ways a driver says "no reading" at once, so the refusal cannot be
 * passing because of only one of them.
 */
TEST(Corrector, AScanWithNoUsableRangeIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const Corrector corrector(defaultParameters(), map);

  std::vector<double> scan(kNumRays, 0.0);

  for (std::size_t i = 0; i < scan.size(); i += 3)
    scan[i] = std::numeric_limits<double>::infinity();

  for (std::size_t i = 1; i < scan.size(); i += 3)
    scan[i] = std::numeric_limits<double>::quiet_NaN();

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, kTruth, kRangeMax);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::scan_entirely_invalid);
}

/*******************************************************************************
*/
TEST(Corrector, AnEstimateOutsideTheMapIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose outside{-5.0, -5.0, 0.0};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, outside, kRangeMax);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::estimate_not_in_free_space);
}

/*******************************************************************************
*/
TEST(Corrector, AnEstimateStandingInAWallIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  /* The middle of the pillar, which spans columns 80 to 82 and rows 60 to 62. */
  const Pose in_the_pillar{81.5 * kResolution, 61.5 * kResolution, 0.0};

  ASSERT_FALSE(map->isFree(in_the_pillar.x, in_the_pillar.y));

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, in_the_pillar, kRangeMax);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::estimate_not_in_free_space);
}

/*******************************************************************************
 * An empty grid has nothing for a ray to meet, so every ray leaves it and the
 * correction is refused before a match is spent reaching the same conclusion.
 */
TEST(Corrector, AnEstimateSeeingNothingOnTheMapIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(emptyMessage());
  ASSERT_NE(map, nullptr);

  const Corrector corrector(defaultParameters(), map);

  const std::vector<double> scan(kNumRays, 1.0);

  ASSERT_TRUE(map->isFree(kTruth.x, kTruth.y));

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, kTruth, kRangeMax);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(),
    CorrectionError::first_cast_entirely_at_max_range);
}

/*******************************************************************************
 * The refusal that stands between an estimate in unmapped space and a node
 * that stops answering. A bound on wall clock time is on the target rather
 * than in this case, because what a regression here produces is a hang.
 */
TEST(Corrector, AMatchThatSpendsEveryRecoveryIsRefused)
{
  const Pose estimate{1.0, 1.0, 0.0};

  const std::shared_ptr<const FSM::GridReference> map =
    std::make_shared<const OnePointOfFreeSpace>(estimate);

  Parameters parameters = defaultParameters();
  parameters.max_recoveries = 1U;

  const Corrector corrector(parameters, map);

  const std::vector<double> scan(90U, 2.0);

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::recovery_budget_exhausted);
}
