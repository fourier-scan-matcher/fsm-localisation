// Copyright 2022 Alexandros Filotheou
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

/*
 * The two things done to a scan before anything is matched against it: rays
 * the sensor could not measure are filled in from their neighbours, and the
 * scan is reduced to the configured number of rays.
 *
 * Gap filling does not interpolate along the run despite its name. Every ray
 * in a run of missing readings takes the same value, the mean of the two
 * readings either side of the run. A scan is a ring, so a run that reaches the
 * end of the array continues at the start of it, and the two readings either
 * side are found by wrapping.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{

const double kExact = 1e-12;

/*
 * Awkward values on purpose. Round numbers survive being stored at single
 * precision, so a test built on them would pass against arithmetic that had
 * quietly lost half its digits.
 */
const double kA = 5.317;
const double kB = 3.041;
const double kC = 7.628;

std::vector<double> uniformScan(const std::size_t size, const double range)
{
  return std::vector<double>(size, range);
}

}  // namespace

/*
 * A run of missing readings in the middle of the scan takes the mean of the
 * reading before it and the reading after it, and nothing else moves.
 */
TEST(ScanHandling, AnInteriorRunTakesTheMeanOfItsNeighbours)
{
  const std::vector<double> scan{kA, 0.0, 0.0, kB, kC, kC};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());

  const double expected = (kA + kB) / 2;
  EXPECT_NEAR(filled[1], expected, kExact);
  EXPECT_NEAR(filled[2], expected, kExact);

  EXPECT_NEAR(filled[0], kA, kExact);
  EXPECT_NEAR(filled[3], kB, kExact);
  EXPECT_NEAR(filled[4], kC, kExact);
  EXPECT_NEAR(filled[5], kC, kExact);
}

/*
 * A scan is a ring. A run that starts near the end of the array and continues
 * past it is one run, not two, and its neighbours are the last reading before
 * it and the first reading after it, found by wrapping round.
 */
TEST(ScanHandling, ARunThatWrapsTheEndOfTheArrayIsOneRun)
{
  const std::vector<double> scan{0.0, 0.0, kB, kC, kC, kC, kA, 0.0};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());

  /* The run is indices 7, 0 and 1. Its neighbours are index 6 and index 2. */
  const double expected = (kA + kB) / 2;
  EXPECT_NEAR(filled[7], expected, kExact) << "before the wrap";
  EXPECT_NEAR(filled[0], expected, kExact) << "after the wrap";
  EXPECT_NEAR(filled[1], expected, kExact) << "after the wrap";

  EXPECT_NEAR(filled[2], kB, kExact);
  EXPECT_NEAR(filled[6], kA, kExact);
}

/*
 * Two runs are filled from their own neighbours rather than from each other.
 */
TEST(ScanHandling, TwoRunsAreFilledIndependently)
{
  const std::vector<double> scan{kA, 0.0, kB, kC, 0.0, 0.0, kA, kB};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());
  EXPECT_NEAR(filled[1], (kA + kB) / 2, kExact) << "first run";
  EXPECT_NEAR(filled[4], (kC + kA) / 2, kExact) << "second run";
  EXPECT_NEAR(filled[5], (kC + kA) / 2, kExact) << "second run";
}

/*
 * A scan with nothing missing comes back exactly as it went in. Real sensors
 * nearly always return at least one bad ray, which is why this case went
 * unexercised long enough to crash on an empty list of runs.
 */
TEST(ScanHandling, AScanWithNothingMissingIsUnchanged)
{
  const std::vector<double> scan{kA, kB, kC, kA, kB, kC};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());
  for (std::size_t i = 0; i < scan.size(); i++)
    EXPECT_NEAR(filled[i], scan[i], kExact) << "ray " << i;
}

/*
 * A single missing ray between two readings is a run of one.
 */
TEST(ScanHandling, ASingleMissingRayIsFilled)
{
  const std::vector<double> scan{kA, kB, 0.0, kC, kA};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());
  EXPECT_NEAR(filled[2], (kB + kC) / 2, kExact);
}

/*
 * A scan in which nothing at all was measured is refused before it reaches the
 * gap filling. That guard is not a nicety. Gap filling given a scan that is
 * entirely missing readings does not return: it appends a list of indices to
 * itself while walking it, and allocates until the process dies. Recorded as
 * known and unfixed, since repairing it would move the numbers; the guard is
 * what makes it unreachable, and this is the case that holds the guard in
 * place.
 */
TEST(ScanHandling, AScanWithNothingMeasuredNeverReachesGapFilling)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 8;

  fsm_lidar_odometry::Matcher matcher(parameters);

  const std::vector<double> nothing(8, 0.0);
  const auto result = matcher.process(nothing);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), fsm_lidar_odometry::MatchError::scan_entirely_invalid);
}

/*
 * Nothing is discarded unless discarding was asked for. Where no size is
 * configured the scan is matched at whatever resolution it arrives with, so a
 * scan far shorter than the old fixed default of 360 is ordinary rather than
 * refused.
 */
TEST(ScanHandling, AScanIsMatchedAtTheSizeItArrivesWith)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 0;

  fsm_lidar_odometry::Matcher matcher(parameters);

  EXPECT_EQ(matcher.matchSize(), 0u) << "nothing has arrived to settle it";

  const std::vector<double> first = uniformScan(90, kC);
  const auto reference = matcher.process(first);

  ASSERT_FALSE(reference.has_value());
  EXPECT_EQ(reference.error(), fsm_lidar_odometry::MatchError::no_reference_yet);
  EXPECT_EQ(matcher.matchSize(), 90u);

  const auto matched = matcher.process(uniformScan(90, kC));
  EXPECT_TRUE(matched.has_value());
}

/*
 * The size the first scan settles holds for the session, because a match
 * compares two scans of one size or nothing. A later scan of a different
 * length is resampled to it rather than thrown away: a driver that
 * occasionally truncates a scan should cost one coarser match, not a gap in
 * the odometry.
 */
TEST(ScanHandling, ALaterScanOfADifferentLengthIsResampledRatherThanRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 0;

  fsm_lidar_odometry::Matcher matcher(parameters);

  matcher.process(uniformScan(180, kC));
  ASSERT_EQ(matcher.matchSize(), 180u);

  const auto longer = matcher.process(uniformScan(720, kC));
  EXPECT_TRUE(longer.has_value());
  EXPECT_EQ(matcher.matchSize(), 180u) << "the size must not follow the scan";

  const auto shorter = matcher.process(uniformScan(45, kC));
  EXPECT_TRUE(shorter.has_value());
  EXPECT_EQ(matcher.matchSize(), 180u);
}

/*
 * Asking for a size is asking for scans of at least that size. A scan too
 * short to be reduced to it is refused, as it always was, because filling in
 * rays that were never measured is not the same as discarding rays that were.
 */
TEST(ScanHandling, AScanShorterThanTheSizeAskedForIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 360;

  fsm_lidar_odometry::Matcher matcher(parameters);

  const auto result = matcher.process(uniformScan(90, kC));

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), fsm_lidar_odometry::MatchError::scan_too_short);
  EXPECT_EQ(matcher.matchSize(), 360u) << "asked for, not settled by a scan";
}

/*
 * Subsampling reduces a scan to the configured number of rays by turning it
 * into points and casting fewer rays at them.
 */
TEST(ScanHandling, SubsamplingProducesTheRequestedNumberOfRays)
{
  const std::vector<double> scan = uniformScan(360, kC);

  const FSM::RaySearch angular = FSM::RaySearch::angular;

  EXPECT_EQ(FSM::Utils::subsampleScan(scan, 360, angular).size(), 360u);
  EXPECT_EQ(FSM::Utils::subsampleScan(scan, 180, angular).size(), 180u);
  EXPECT_EQ(FSM::Utils::subsampleScan(scan, 90, angular).size(), 90u);
}

/*
 * When the new ray count divides the old one, every new ray leaves along the
 * direction of one of the old readings, so it meets the points exactly where
 * that reading put them. A scan of one constant range therefore subsamples to
 * the same constant range, with no loss at all.
 */
TEST(ScanHandling, SubsamplingByAWholeFactorKeepsTheRangesExactly)
{
  const std::vector<double> scan = uniformScan(360, kC);
  const std::vector<double> smaller =
    FSM::Utils::subsampleScan(scan, 90, FSM::RaySearch::angular);

  ASSERT_EQ(smaller.size(), 90u);
  for (std::size_t i = 0; i < smaller.size(); i++)
    EXPECT_NEAR(smaller[i], kC, 1e-8) << "ray " << i;
}

/*
 * When it does not divide, the new rays fall between the old readings, where
 * the points are joined by straight lines rather than by the arc they came
 * from. Every such ray is therefore a little short, never long, and by less
 * than the sagitta of one step of the original scan.
 */
TEST(ScanHandling, SubsamplingByAFractionFallsShortAndNeverLong)
{
  const std::vector<double> scan = uniformScan(360, kC);
  const std::vector<double> smaller =
    FSM::Utils::subsampleScan(scan, 100, FSM::RaySearch::angular);

  ASSERT_EQ(smaller.size(), 100u);

  const double step = 2 * M_PI / 360;
  const double deepest = kC * (1 - std::cos(step / 2));

  for (std::size_t i = 0; i < smaller.size(); i++)
  {
    EXPECT_LE(smaller[i], kC + 1e-8) << "ray " << i << " reaches too far";
    EXPECT_GE(smaller[i], kC - deepest - 1e-8) << "ray " << i << " falls short";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
