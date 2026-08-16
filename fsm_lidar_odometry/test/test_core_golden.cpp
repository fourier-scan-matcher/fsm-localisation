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
 * The gate for the modernisation work.
 *
 * Drives the matcher over the same scans the ROS 1 reference build was driven
 * over, and compares against the numbers that build produced. The reference
 * files hold nothing but numbers, so this needs no ROS, no container and no
 * network: it can be run after every single change to the core.
 *
 * A modernisation pass that moves any of these numbers has changed behaviour,
 * whatever it claimed to do.
 *
 *
 * WHAT THE RUNTIME PATH REACHES, AND WHAT IT DOES NOT
 *
 * This file drives the wrapper, which is the only way the node reaches the
 * core, so every core function the runtime path can reach runs here at least
 * once. Several also have tests of their own elsewhere, against answers worked
 * out by hand rather than recorded.
 *
 * The core header declares seventy three functions. Forty five of them are
 * reachable. The twenty eight below are not: nothing on the path from a scan
 * arriving to a pose being published calls them, directly or at any depth.
 *
 * The list is arrived at rather than guessed. A single translation unit that
 * asks the core for exactly what the wrapper asks it for is compiled with
 * inlining off:
 *
 *   g++ -std=c++23 -O0 -fno-inline -ffunction-sections -c probe.cpp
 *   nm -C --defined-only probe.o | grep FSM::
 *
 * A function is emitted only where something calls it, so what the object file
 * holds is what the runtime path reaches, and the rest of the header is what
 * it does not. Re-run it after any change that adds or removes a call.
 *
 * Recording helpers. They write points, scans, maps and polygons to files for
 * offline inspection, and the node inspects nothing offline:
 *
 *   Dump::convexHulls, Dump::map, Dump::points, Dump::polygon,
 *   Dump::polygons, Dump::rangeScan, Dump::scan, DatasetUtils::printDataset
 *
 * Dataset reading. These belong to the offline driver the algorithm was
 * written around, which reads scans from a file rather than from a sensor:
 *
 *   DatasetUtils::dataset2points, DatasetUtils::dataset2rangesAndPose,
 *   DatasetUtils::readDataset, DatasetUtils::readDatasetScan
 *
 * Scan completion. Six variations on filling in a scan from a hypothesised
 * pose. The matcher completes scans by another route entirely and calls none
 * of them:
 *
 *   ScanCompletion::completeScan, ScanCompletion::completeScan1 through
 *   ScanCompletion::completeScan5
 *
 * Small helpers left behind, each called by nothing:
 *
 *   DFTUtils::fftshift, Rotation::angleById, Utils::generatePoseWithinMap,
 *   Utils::innerProduct, Utils::isPositionFartherThan,
 *   Utils::multiplyWithRotationMatrix, Utils::pairDiff, Utils::sgn,
 *   Utils::vectorDiff
 *
 * One is unreachable on purpose. X::findExact walks every wall for every ray
 * and is the slow, obvious answer the two selectable ray searches are held to
 * in test_geometry.cpp. It is reference material, not dead weight.
 *
 * Of the other twenty seven, only DFTUtils::fftshift has a test. The
 * remaining twenty six are shipped, compiled, and exercised by nothing at all.
 * Deleting them is a decision about how much of the original algorithm this
 * package is obliged to carry, which is why they are listed here rather than
 * removed.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{

const double kTolerance = 1e-9;

struct Reference
{
  std::vector<fsm_lidar_odometry::Pose> increments;
  std::vector<fsm_lidar_odometry::Pose> accumulated;
  std::vector<unsigned int> recoveries;
};

std::string fixture(const std::string& name)
{
  return std::string(FSM_LIDAR_ODOMETRY_TEST_FIXTURES) + "/" + name;
}

std::vector<std::vector<double>> readScans(const std::string& path)
{
  std::ifstream file(fixture(path));
  EXPECT_TRUE(file.good()) << "cannot open " << fixture(path);

  std::size_t count = 0;
  std::size_t size = 0;
  file >> count >> size;

  std::vector<std::vector<double>> scans(count, std::vector<double>(size));
  for (std::size_t s = 0; s < count; s++)
    for (std::size_t i = 0; i < size; i++)
      file >> scans[s][i];

  return scans;
}

Reference readReference(const std::string& path)
{
  std::ifstream file(fixture(path));
  EXPECT_TRUE(file.good()) << "cannot open " << fixture(path);

  Reference reference;

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line))
  {
    if (line.empty())
      continue;

    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);

    unsigned int index = 0;
    fsm_lidar_odometry::Pose increment;
    fsm_lidar_odometry::Pose accumulated;
    unsigned int recoveries = 0;

    values >> index
           >> increment.x >> increment.y >> increment.t
           >> accumulated.x >> accumulated.y >> accumulated.t
           >> recoveries;

    reference.increments.push_back(increment);
    reference.accumulated.push_back(accumulated);
    reference.recoveries.push_back(recoveries);
  }

  return reference;
}

void expectAgreement(const std::string& scenario)
{
  const std::vector<std::vector<double>> scans =
    readScans(scenario + "_scans.csv");
  const Reference reference = readReference(scenario + "_core.csv");

  ASSERT_GE(scans.size(), 2u) << scenario;
  ASSERT_EQ(reference.increments.size(), scans.size() - 1) << scenario;

  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = scans[0].size();

  fsm_lidar_odometry::Matcher matcher(parameters);

  const std::expected<fsm_lidar_odometry::MatchResult, fsm_lidar_odometry::MatchError> first =
    matcher.process(scans[0]);
  ASSERT_FALSE(first.has_value()) << "the first scan must yield no match";
  EXPECT_EQ(first.error(), fsm_lidar_odometry::MatchError::no_reference_yet);

  for (std::size_t s = 1; s < scans.size(); s++)
  {
    const std::expected<fsm_lidar_odometry::MatchResult, fsm_lidar_odometry::MatchError> result =
      matcher.process(scans[s]);

    ASSERT_TRUE(result.has_value()) << scenario << " step " << s;

    const std::size_t step = s - 1;

    EXPECT_NEAR(result->increment.x, reference.increments[step].x, kTolerance)
      << scenario << " step " << s << ", increment x";
    EXPECT_NEAR(result->increment.y, reference.increments[step].y, kTolerance)
      << scenario << " step " << s << ", increment y";
    EXPECT_NEAR(result->increment.t, reference.increments[step].t, kTolerance)
      << scenario << " step " << s << ", increment orientation";

    EXPECT_NEAR(result->accumulated.x, reference.accumulated[step].x, kTolerance)
      << scenario << " step " << s << ", accumulated x";
    EXPECT_NEAR(result->accumulated.y, reference.accumulated[step].y, kTolerance)
      << scenario << " step " << s << ", accumulated y";
    EXPECT_NEAR(result->accumulated.t, reference.accumulated[step].t, kTolerance)
      << scenario << " step " << s << ", accumulated orientation";

    EXPECT_EQ(result->num_recoveries, reference.recoveries[step])
      << scenario << " step " << s << ", recovery count";
    EXPECT_EQ(result->num_recoveries, 0u)
      << scenario << " step " << s
      << ": the recovery path fired, which makes this scenario unusable for "
         "cross version comparison";
  }
}

}  // namespace

TEST(CoreGolden, PureRotation)
{
  expectAgreement("pure_rotation");
}

TEST(CoreGolden, PureTranslation)
{
  expectAgreement("pure_translation");
}

TEST(CoreGolden, RectangularRoom)
{
  expectAgreement("rect_room_short");
}
