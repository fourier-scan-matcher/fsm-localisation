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
 * put back by matching its scan against a map.
 * @details
 * **How a case is built.** A room is drawn as an occupancy grid, a pose inside
 * it is chosen as the truth, and the scan that pose would see is cast from the
 * map itself. That scan is then handed to the corrector along with an estimate
 * displaced from the truth by a known amount. What is measured is whether the
 * corrector puts the estimate back where the scan came from. Casting the scan
 * from the same map the correction reads means this file measures the matcher
 * and not the map: whether the map is a faithful rendering of a room is
 * `test_reference_agreement.cpp`'s question, and it answers it.
 * **The room is not symmetric under any rotation.** A bare rectangle maps onto
 * itself under a half turn, so a heading error of pi could produce a scan
 * indistinguishable from the right one and pass. A pillar off the centre lines
 * removes every such coincidence.
 * **What the answer is held to, and where the bound comes from.** A cast ray
 * stops at a cell, so a map cannot support a position better than one cell
 * however good the match is. That is the bound on position and it belongs to
 * the map rather than to the matcher, which is stated in the specification
 * rather than discovered here.
 * The heading needs the same cell expressed as an angle. A heading wrong by
 * `dt` moves what the sensor would see at range `r` sideways by `r * dt`, so a
 * heading is resolved no better than one cell seen at the ranges the scan
 * actually carries. The bound is therefore `atan(resolution / mean_range)`
 * with the mean taken over the scan under test, which makes it a property of
 * the case rather than a number chosen to pass.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
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

const unsigned int kWidth = 121U;
const unsigned int kHeight = 91U;
const double kResolution = 0.05;

const double kRangeMax = 12.0;
const double kSymmetricHeadingBound = 0.004;
const double kFirstRayAtMinusPi = -M_PI;
const double kFullTurn = 2.0 * M_PI;

const unsigned int kNumRays = 360U;

const double kRayAngleBound = 1e-6;

const std::vector<std::size_t> kMagnifications = {1U, 2U, 4U, 8U};

const unsigned int kNarrowRays = 271U;

const unsigned int kRb1Rays = 1073U;
const double kRb1AngleMin = -2.3387413;
const double kRb1AngleMax = 2.3387413;

const Pose kTruth{1.53, 1.07, 0.4};

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

  for (unsigned int column = 80U; column <= 82U; column++)
    for (unsigned int row = 60U; row <= 62U; row++)
      block(column, row);

  return message;
}

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

Parameters defaultParameters()
{
  Parameters parameters;
  parameters.rng_seed = 1234U;
  return parameters;
}

std::shared_ptr<const OccupancyGridMap> mapOf(
  const nav_msgs::msg::OccupancyGrid& message)
{
  const std::expected<std::shared_ptr<const OccupancyGridMap>, std::string> map =
    OccupancyGridMap::build(message, kOccupancyThreshold, CastingMethod::ray_marching);

  return map.has_value() ? *map : nullptr;
}

double headingBound(const std::vector<double>& scan)
{
  const double mean =
    std::accumulate(scan.begin(), scan.end(), 0.0) / scan.size();

  return std::atan(kResolution / mean);
}

std::vector<double> scanFromTruth(const OccupancyGridMap& map)
{
  return map.ranges(kTruth, kNumRays, kRangeMax);
}

/**
 * @brief What a sensor covering `aperture` would have seen from the truth.
 * @details The middle window of the panoramic cast, rather than a second cast
 * at a narrower spacing. A narrower sensor at the same pose sees the same rays
 * the panoramic one saw, fewer of them, so taking the window is what such a
 * sensor really reports and casting again would be a different measurement.
 */
std::vector<double> narrowScanFromTruth(const OccupancyGridMap& map,
  const double aperture)
{
  const std::vector<double> panoramic = scanFromTruth(map);

  const std::size_t rays = static_cast<std::size_t>(
    std::llround(kNumRays * aperture / (2.0 * M_PI)));

  const std::size_t prefix = FSM::windowPrefixOf(panoramic.size(), rays);

  return std::vector<double>(panoramic.begin() + prefix,
    panoramic.begin() + prefix + rays);
}

}

/**
 * @brief A sensor covering three quarters of a turn is corrected as well as a
 * panoramic one.
 * @details This is the case the corrector answered confidently and wrongly
 * before it was told what the sensor covers. It read the map over a whole turn
 * and matched it against three quarters of one, which converges rather than
 * failing, so nothing anywhere reported a problem: the caller got a pose, the
 * pose was wrong by metres, and only a comparison against a known truth showed
 * it. Reinstate the fault and this case fails while every panoramic case above
 * still passes, which is why the panoramic ones could not have caught it.
 */
TEST(Corrector, ASensorCoveringThreeQuartersOfATurnIsPutBack)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const double aperture = 1.5 * M_PI;

  const std::vector<double> scan = narrowScanFromTruth(*map, aperture);

  ASSERT_EQ(scan.size(), 270U);

  Parameters parameters = defaultParameters();
  parameters.size_scan = scan.size();

  const Corrector corrector(parameters, map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t + 0.08};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax, aperture, -0.5 * aperture);

  ASSERT_TRUE(corrected.has_value());

  EXPECT_NEAR(corrected->corrected.x, kTruth.x, kResolution);
  EXPECT_NEAR(corrected->corrected.y, kTruth.y, kResolution);
  EXPECT_NEAR(corrected->corrected.t, kTruth.t, headingBound(scan));
}

/**
 * @brief A whole turn beginning half a ray after minus pi is put back as well
 * as one beginning at minus pi.
 * @details The scan is what a sensor symmetric about its heading reports, so
 * its first ray lies half a ray past where the matcher lays a whole turn out
 * from. Told where the first ray sits, the answer must not depend on it. Told
 * it sits at minus pi, it comes out turned. Measured, the answer is 0.0007
 * radians from the truth when told and 0.0080 when not. The bound is between
 * the two.
 */
TEST(Corrector, AScanSymmetricAboutTheHeadingIsPutBack)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const double half_a_ray = M_PI / kNumRays;

  const std::vector<double> scan = map->ranges(
    Pose{kTruth.x, kTruth.y, kTruth.t + half_a_ray}, kNumRays, kRangeMax);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t + 0.08};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax, kFullTurn,
      kFirstRayAtMinusPi + half_a_ray);

  ASSERT_TRUE(corrected.has_value());

  EXPECT_NEAR(corrected->corrected.x, kTruth.x, kResolution);
  EXPECT_NEAR(corrected->corrected.y, kTruth.y, kResolution);
  EXPECT_NEAR(corrected->corrected.t, kTruth.t, kSymmetricHeadingBound);
}

/**
 * @brief Every ray of the map's scan lies along the ray of the scan it is
 * matched against, wherever that scan's first ray sits.
 * @details The reference is told the angle of the scan's first ray and cast
 * at the scan's own count of rays and at 2, 4 and 8 times it, as the rotation
 * step casts it, and ray `k * i` of a cast of `k` times the rays has to lie
 * along ray `i` of the scan. Each layout is one a sensor reports: a
 * whole turn from minus pi, a whole turn symmetric about the heading, three
 * quarters of a turn, 271 rays symmetric about the heading, where the turn
 * less the rays is odd, and the 1073 rays of the rb1 recording, where it is
 * odd too. A point on each ray gives the ray's angle exactly, whatever the
 * range. Measured, at every count of rays the worst ray is 3.6e-15 radians
 * out on every layout but rb1's and 2.0e-7 on rb1's, whose recorded spacing
 * exceeds a turn's 1440th by 1.9e-10 radians, 2.0e-7 over its 1072 steps.
 * Turned for the scan's own count alone, the casts at 2, 4 and 8 times the
 * rays of the 271 and 1073 ray layouts are half a ray out, 0.0087 and 0.0022
 * radians. Not turned at all, the whole turn symmetric about the heading is
 * 0.0087 out at every count and the other two as before. The bound is
 * between the two.
 */
TEST(Corrector, EveryRayOfTheMapScanLiesAlongTheRayOfTheScan)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  struct Layout
  {
    std::size_t rays;
    double spacing;
    double first_ray_angle;
  };

  const double turn_spacing = kFullTurn / kNumRays;
  const double rb1_spacing = (kRb1AngleMax - kRb1AngleMin) / (kRb1Rays - 1);

  const std::vector<Layout> layouts = {
    {kNumRays, turn_spacing, kFirstRayAtMinusPi},
    {kNumRays, turn_spacing, kFirstRayAtMinusPi + 0.5 * turn_spacing},
    {3 * kNumRays / 4, turn_spacing, -0.375 * kFullTurn},
    {kNarrowRays, turn_spacing, -0.5 * (kNarrowRays - 1) * turn_spacing},
    {kRb1Rays, rb1_spacing, kRb1AngleMin},
  };

  for (const Layout& layout : layouts)
  {
    const double aperture = layout.rays * layout.spacing;

    const FSM::Reference reference(*map, kRangeMax, aperture,
      layout.first_ray_angle);

    for (const std::size_t magnification : kMagnifications)
    {
      const std::size_t rays = magnification * layout.rays;
      const double spacing = layout.spacing / magnification;

      const std::vector<std::pair<double, double>> points = FSM::X::find(
        kTruth, reference, static_cast<unsigned int>(rays),
        FSM::RaySearch::angular);

      ASSERT_EQ(points.size(), rays);

      double worst = 0.0;
      for (std::size_t i = 0; i < points.size(); ++i)
      {
        const double along = std::atan2(points[i].second - kTruth.y,
          points[i].first - kTruth.x);
        const double expected = kTruth.t + layout.first_ray_angle + i * spacing;

        worst = std::max(worst,
          std::abs(std::remainder(along - expected, kFullTurn)));
      }

      EXPECT_LE(worst, kRayAngleBound) << rays << " rays from "
        << layout.first_ray_angle << " radians";
    }
  }
}

/**
 * @brief An aperture no sensor can cover is refused rather than answered.
 */
TEST(Corrector, AnApertureNoSensorCanCoverIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  for (const double aperture : {0.0, -1.0, 4.0 * M_PI})
  {
    const std::expected<CorrectionResult, CorrectionError> corrected =
      corrector.correct(scan, kTruth, kRangeMax, aperture, -0.5 * aperture);

    ASSERT_FALSE(corrected.has_value()) << "aperture " << aperture;
    EXPECT_EQ(corrected.error(), CorrectionError::aperture_not_a_sensor)
      << "aperture " << aperture;
  }
}

/**
 * @brief The window arithmetic, pinned at the two apertures that matter.
 * @details Every place that takes this window has to agree on where it starts
 * or the rays they align are not the same rays. These are the numbers the sift
 * of the package above and the harness that shapes a narrower scan also hold.
 */
TEST(Corrector, TheWindowArithmeticIsPinned)
{
  EXPECT_EQ(FSM::panoramicRaysOf(360U, 2.0 * M_PI), 360U);
  EXPECT_EQ(FSM::windowPrefixOf(360U, 360U), 0U);

  EXPECT_EQ(FSM::panoramicRaysOf(270U, 1.5 * M_PI), 360U);
  EXPECT_EQ(FSM::windowPrefixOf(360U, 270U), 45U);

  EXPECT_EQ(FSM::panoramicRaysOf(271U, 1.5 * M_PI), 361U);
  EXPECT_EQ(FSM::windowPrefixOf(361U, 271U), 45U);

  EXPECT_EQ(FSM::panoramicRaysOf(0U, 1.5 * M_PI), 0U);
  EXPECT_EQ(FSM::panoramicRaysOf(360U, 0.0), 360U);
}

TEST(Corrector, AnEstimateDisplacedInPositionIsPutBack)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_TRUE(corrected.has_value());

  EXPECT_NEAR(corrected->corrected.x, kTruth.x, kResolution);
  EXPECT_NEAR(corrected->corrected.y, kTruth.y, kResolution);
  EXPECT_NEAR(corrected->corrected.t, kTruth.t, headingBound(scan));
}

TEST(Corrector, AnEstimateDisplacedInHeadingIsPutBack)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x, kTruth.y, kTruth.t + 0.12};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_TRUE(corrected.has_value());

  EXPECT_NEAR(corrected->corrected.x, kTruth.x, kResolution);
  EXPECT_NEAR(corrected->corrected.y, kTruth.y, kResolution);
  EXPECT_NEAR(corrected->corrected.t, kTruth.t, headingBound(scan));
}

TEST(Corrector, AnEstimateDisplacedInBothIsPutBack)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t + 0.12};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, estimate, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_TRUE(corrected.has_value());

  EXPECT_NEAR(corrected->corrected.x, kTruth.x, kResolution);
  EXPECT_NEAR(corrected->corrected.y, kTruth.y, kResolution);
  EXPECT_NEAR(corrected->corrected.t, kTruth.t, headingBound(scan));
}

TEST(Corrector, TheSameCorrectionTwiceGivesTheSameAnswerExactly)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t + 0.12};

  const std::expected<CorrectionResult, CorrectionError> first =
    corrector.correct(scan, estimate, kRangeMax, kFullTurn, kFirstRayAtMinusPi);
  const std::expected<CorrectionResult, CorrectionError> second =
    corrector.correct(scan, estimate, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->corrected.x, second->corrected.x);
  EXPECT_EQ(first->corrected.y, second->corrected.y);
  EXPECT_EQ(first->corrected.t, second->corrected.t);
  EXPECT_EQ(first->num_recoveries, second->num_recoveries);
}

TEST(Corrector, ACorrectionIsNotAffectedByTheOneBeforeIt)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose estimate{kTruth.x + 0.11, kTruth.y - 0.09, kTruth.t};

  const std::expected<CorrectionResult, CorrectionError> alone =
    corrector.correct(scan, estimate, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  const Corrector second_corrector(defaultParameters(), map);

  ASSERT_TRUE(second_corrector.correct(
      scan, Pose{kTruth.x - 0.13, kTruth.y + 0.07, kTruth.t + 0.2},
      kRangeMax, kFullTurn, kFirstRayAtMinusPi).has_value());

  const std::expected<CorrectionResult, CorrectionError> after =
    second_corrector.correct(scan, estimate, kRangeMax, kFullTurn,
      kFirstRayAtMinusPi);

  ASSERT_TRUE(alone.has_value());
  ASSERT_TRUE(after.has_value());

  EXPECT_EQ(alone->corrected.x, after->corrected.x);
  EXPECT_EQ(alone->corrected.y, after->corrected.y);
  EXPECT_EQ(alone->corrected.t, after->corrected.t);
}

TEST(Corrector, AScanShorterThanTheConfiguredSizeIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  Parameters parameters = defaultParameters();
  parameters.size_scan = 180U;

  const Corrector corrector(parameters, map);

  const std::vector<double> scan(179U, 1.0);

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, kTruth, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::scan_too_short);
}

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
    corrector.correct(scan, kTruth, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::scan_entirely_invalid);
}

TEST(Corrector, AnEstimateOutsideTheMapIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose outside{-5.0, -5.0, 0.0};

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, outside, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::estimate_not_in_free_space);
}

TEST(Corrector, AnEstimateStandingInAWallIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(roomMessage());
  ASSERT_NE(map, nullptr);

  const std::vector<double> scan = scanFromTruth(*map);

  const Corrector corrector(defaultParameters(), map);

  const Pose in_the_pillar{81.5 * kResolution, 61.5 * kResolution, 0.0};

  ASSERT_FALSE(map->isFree(in_the_pillar.x, in_the_pillar.y));

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, in_the_pillar, kRangeMax, kFullTurn,
      kFirstRayAtMinusPi);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::estimate_not_in_free_space);
}

TEST(Corrector, AnEstimateSeeingNothingOnTheMapIsRefused)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(emptyMessage());
  ASSERT_NE(map, nullptr);

  const Corrector corrector(defaultParameters(), map);

  const std::vector<double> scan(kNumRays, 1.0);

  ASSERT_TRUE(map->isFree(kTruth.x, kTruth.y));

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector.correct(scan, kTruth, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(),
    CorrectionError::first_cast_entirely_at_max_range);
}

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
    corrector.correct(scan, estimate, kRangeMax, kFullTurn, kFirstRayAtMinusPi);

  ASSERT_FALSE(corrected.has_value());
  EXPECT_EQ(corrected.error(), CorrectionError::recovery_budget_exhausted);
}
