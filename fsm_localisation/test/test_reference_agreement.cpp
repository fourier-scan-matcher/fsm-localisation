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
 * @brief One room expressed twice, as a ring of points and as an occupancy
 * grid, and the ranges the matcher gets from each held against one
 * another.
 * @details
 * This is the file that says the two kinds of reference mean the same thing,
 * rather than that each computes something. Both sides are reached the way the
 * matcher reaches them, through `FSM::X::find` and `FSM::Utils::points2scan`,
 * so what is compared is what a match would see. The grid path additionally
 * carries `X::findGrid`'s round trip through a point on each ray, which is part
 * of what has to agree.
 * **The room.** A rectangle with one corner cut away, given as the union of two
 * rectangles. It has no rotational symmetry of any kind, so a ray fan cast one
 * index out of step, mirrored, or turned by half a turn cannot match by luck.
 * That matters here because the ray angle convention is written twice, once in
 * `X::findExactAngular` and once in `OccupancyGridMap::ranges`, on opposite
 * sides of a module boundary.
 * **Why the rasterisation is exact, and why that is the whole design of this
 * file.** Every wall sits at a multiple of the coarser of the two cell sizes,
 * and the coarser is a whole multiple of the finer, so no cell of either grid
 * straddles a wall. A cell is wholly inside the room or wholly outside it, the
 * grid *is* the room rather than an approximation of it, and what is left to
 * disagree is only what the cell size forces. A room whose walls fell between
 * cells would rasterise into a staircase, its rasterised face would stand up to
 * a cell from its true one, and the measurement below would be of the fixture
 * rather than of the two references.
 * **What a range is measured to.** The ring answers with the exact distance to
 * the wall's near face. A caster cannot: it answers with a distance to a cell.
 * The rest of this work measures its analytic ranges to the centre of the first
 * cell a ray enters and says so; the bounds below are built from the same
 * convention, as the distance between what the two references can each say
 * about the same crossing.
 * **The bounds, read off each caster rather than off a run.** Both are numbers
 * of cells, which is what makes the disagreement scale with the cell size, and
 * both are reached by asking two questions of a caster: how far along the ray
 * its samples are spaced, and how far its answer can sit from the sample it
 * stopped at.
 * *Bresenham*, `RangeLib.h:543`. It steps one whole cell at a time along
 * whichever axis the ray runs faster in, so its samples are
 * `1 / max(|cos|,|sin|)` cells apart along the ray, between one and `sqrt(2)`.
 * It reports the first sample whose cell is blocked, and since no cell straddles
 * a wall that sample lies in blocked space, so it cannot be more than one
 * spacing past the crossing. But the line it walks is an integer approximation
 * of the true one and stands up to half a cell to the side of it, and half a
 * cell to the side of a wall met at an angle `a` is `0.5 / sin a` cells along
 * the ray. That second term is the shallow angle term, it is a genuine property
 * of walking a line on a lattice rather than a defect, and it is what makes a
 * flat one cell bound wrong: a ray meeting a wall at six degrees can be five
 * cells out and still be right.
 * bresenham, in cells = 1 / max(|cos t|, |sin t|) + 0.5 / |sin a|
 * *Ray marching*, `RangeLib.h:626`. It follows the true line, so it has no
 * shallow angle term at all, and one other term instead. It advances by
 * `max(0.999 * d, 1)` cells where `d` is the distance transform read at the
 * cell the sample stands in, so the step is never shorter than a cell; and `d`
 * is a distance between cells, while the sample can stand `hypot(0.5,0.5)` from
 * its own cell's centre and blocked space can begin `hypot(0.5,0.5)` before the
 * blocked cell's centre, so a step can carry the sample up to `sqrt(2)` cells
 * past the crossing. It then reports the distance to that sample itself, not
 * to the corner of the cell it truncates into, which is what the ray marching
 * correction in `RangeLib.h` keeps of cbgl's own answer rather than the
 * published original's: measuring to the corner would add a second `sqrt(2)`
 * cells on top, correlated with heading rather than random, and is the
 * defect `range_libc/README.md` records rather than a property of this file's
 * geometry.
 * ray marching, in cells = sqrt(2)
 * **CDDT is not exercised here.** It quantises the cast heading as well as
 * space, and that term is `range * pi / 720` whatever the cell size is. A
 * method whose error does not shrink with the cells cannot take part in a
 * measurement of how the error shrinks with the cells. It has its own bound and
 * its own cases in `test_ray_casting.cpp`.
 * **Rays that graze a corner are set aside, counted, and reported.** Which side
 * of a corner a ray passes decides which wall it meets, and the two references
 * decide that from positions that differ by up to a cell. A ray passing within
 * one cell of a corner can therefore be answered honestly by both and differ by
 * metres. The share set aside grows with the cell size, which is the same cell
 * size showing through in another place.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fsm_localisation/occupancy_grid_map.hpp"

namespace
{
using fsm_localisation::CastingMethod;
using fsm_localisation::OccupancyGridMap;

const std::int8_t kBlocked = 100;
const std::int8_t kFree = 0;

const int kOccupancyThreshold = 50;

const double kRoomRight = 3.20;
const double kRoomTop = 2.40;
const double kNotchLeft = 2.00;
const double kNotchBottom = 1.60;

const double kFineResolution = 0.05;
const double kCoarseResolution = 0.20;

const double kMargin = 0.40;

const double kMaxRange = 20.0;

const unsigned int kNumRays = 181U;

const double kCornerGuardInCells = 1.0;

const double kMinComparedShare = 0.5;

const double kSquareEnoughSine = 0.5;

struct NamedPose
{
  const char* what{""};
  FSM::Pose pose;
};

struct RayComparison
{
  double bound{1.0};
  double difference{0.0};
  double ratio{-1.0};
  double from_grid{0.0};
  double from_ring{0.0};
  double heading{0.0};
  double sine_to_wall{0.0};
  unsigned int ray{0U};
  const char* what{""};
};

struct Disagreement
{
  RayComparison largest;

  RayComparison tightest;

  unsigned int compared{0U};
  unsigned int grazing{0U};
  unsigned int shallow{0U};
};

double rayHeading(const unsigned int i, const unsigned int num_rays, const double t);

std::vector<double> rangesFromTheGrid(
  const OccupancyGridMap& map, const FSM::Pose& pose, const unsigned int num_rays);

std::vector<double> rangesFromTheRing(const FSM::Pose& pose, const unsigned int num_rays);

nav_msgs::msg::OccupancyGrid roomMessage(const double resolution);

std::vector<std::pair<double, double>> roomRing();

double sineOfTheAngleToTheWall(
  const double heading, const std::pair<double, double>& meeting);

double boundInCells(
  const CastingMethod method, const double heading, const double sine_to_wall)
{
  if (method == CastingMethod::ray_marching)
    return M_SQRT2;

  const double sample_spacing =
    1.0 / std::max(std::abs(std::cos(heading)), std::abs(std::sin(heading)));

  return sample_spacing + 0.5 / sine_to_wall;
}

double cornerMiss(
  const FSM::Pose& pose, const unsigned int i, const unsigned int num_rays)
{
  const double heading = rayHeading(i, num_rays, pose.t);
  const double ux = std::cos(heading);
  const double uy = std::sin(heading);

  double nearest = std::numeric_limits<double>::max();

  for (const std::pair<double, double>& corner : roomRing())
  {
    const double dx = corner.first - pose.x;
    const double dy = corner.second - pose.y;

    if (dx * ux + dy * uy <= 0.0)
      continue;

    nearest = std::min(nearest, std::abs(dx * uy - dy * ux));
  }

  return nearest;
}

bool isInsideTheRoom(const double x, const double y)
{
  const bool in_the_low_wide_part =
    x > 0.0 && x < kRoomRight && y > 0.0 && y < kNotchBottom;

  const bool in_the_tall_narrow_part =
    x > 0.0 && x < kNotchLeft && y >= kNotchBottom && y < kRoomTop;

  return in_the_low_wide_part || in_the_tall_narrow_part;
}

Disagreement measureDisagreement(
  const OccupancyGridMap& map, const CastingMethod method,
  const std::vector<NamedPose>& poses, const bool square_only)
{
  Disagreement measured;

  const std::vector<std::pair<double, double>> ring = roomRing();

  for (const NamedPose& named : poses)
  {
    const std::vector<double> from_ring = rangesFromTheRing(named.pose, kNumRays);
    const std::vector<double> from_grid =
      rangesFromTheGrid(map, named.pose, kNumRays);

    const std::vector<std::pair<double, double>> meetings =
      FSM::X::find(named.pose, ring, kNumRays, FSM::RaySearch::angular);

    EXPECT_EQ(from_ring.size(), kNumRays);
    EXPECT_EQ(from_grid.size(), kNumRays);

    for (unsigned int i = 0; i < kNumRays; ++i)
    {
      if (cornerMiss(named.pose, i, kNumRays) <
          kCornerGuardInCells * map.resolution())
      {
        measured.grazing++;
        continue;
      }

      const double heading = rayHeading(i, kNumRays, named.pose.t);
      const double sine = sineOfTheAngleToTheWall(heading, meetings[i]);

      if (square_only && sine < kSquareEnoughSine)
      {
        measured.shallow++;
        continue;
      }

      measured.compared++;

      const double bound = boundInCells(method, heading, sine) * map.resolution();
      const double difference = std::abs(from_grid[i] - from_ring[i]);

      const RayComparison comparison{
        .bound = bound,
        .difference = difference,
        .ratio = difference / bound,
        .from_grid = from_grid[i],
        .from_ring = from_ring[i],
        .heading = heading,
        .sine_to_wall = sine,
        .ray = i,
        .what = named.what};

      if (comparison.difference > measured.largest.difference)
        measured.largest = comparison;

      if (comparison.ratio > measured.tightest.ratio)
        measured.tightest = comparison;
    }
  }

  return measured;
}

std::string methodSuffix(const testing::TestParamInfo<CastingMethod>& info)
{
  return std::string(fsm_localisation::nameOfCastingMethod(info.param));
}

std::vector<NamedPose> posesInTheRoom()
{
  return {
    {"the middle of the room, far from every wall",
      FSM::Pose{.x = 1.51, .y = 0.83, .t = 0.37}},
    {"close to the left wall, within one coarse cell of it",
      FSM::Pose{.x = 0.13, .y = 1.27, .t = 2.11}},
    {"in the corner where the left and bottom walls meet",
      FSM::Pose{.x = 0.17, .y = 0.19, .t = 1.13}},
    {"facing the cut away corner along the diagonal",
      FSM::Pose{.x = 1.17, .y = 0.77, .t = M_PI / 4}}};
}

std::vector<double> rangesFromTheGrid(
  const OccupancyGridMap& map, const FSM::Pose& pose, const unsigned int num_rays)
{
  return FSM::Utils::points2scan(
    FSM::X::find(pose, FSM::Reference(map, kMaxRange), num_rays,
      FSM::RaySearch::angular),
    pose);
}

std::vector<double> rangesFromTheRing(const FSM::Pose& pose, const unsigned int num_rays)
{
  const std::vector<std::pair<double, double>> ring = roomRing();

  return FSM::Utils::points2scan(
    FSM::X::find(pose, ring, num_rays, FSM::RaySearch::angular), pose);
}

double rayHeading(const unsigned int i, const unsigned int num_rays, const double t)
{
  const double heading = i * 2 * M_PI / num_rays + t - M_PI;
  return std::fmod(heading + 5 * M_PI, 2 * M_PI) - M_PI;
}

void report(
  const char* which, const OccupancyGridMap& map, const Disagreement& measured)
{
  std::cout
    << "[ MEASURED ] " << which << ", " << map.resolution()
    << " m a cell: largest disagreement " << measured.largest.difference
    << " m (" << measured.largest.difference / map.resolution() << " cells) on "
    << measured.largest.what << ", ray " << measured.largest.ray << " at "
    << measured.largest.heading << " rad meeting its wall at sine "
    << measured.largest.sine_to_wall << ", ring " << measured.largest.from_ring
    << " m against grid " << measured.largest.from_grid
    << " m. Tightest against its own bound: " << measured.tightest.difference
    << " m of " << measured.tightest.bound << " m allowed, ray "
    << measured.tightest.ray << " at sine " << measured.tightest.sine_to_wall
    << ". " << measured.compared << " rays compared, " << measured.grazing
    << " grazing a corner, " << measured.shallow << " too shallow."
    << std::endl;
}

std::shared_ptr<const OccupancyGridMap> roomGrid(
  const double resolution, const CastingMethod method)
{
  const auto built =
    OccupancyGridMap::build(roomMessage(resolution), kOccupancyThreshold, method);

  EXPECT_TRUE(built.has_value())
    << "the room could not be expressed as a grid at " << resolution
    << " m a cell";

  return built.value_or(nullptr);
}

nav_msgs::msg::OccupancyGrid roomMessage(const double resolution)
{
  const unsigned int width = static_cast<unsigned int>(
    std::llround((kRoomRight + 2 * kMargin) / resolution));
  const unsigned int height = static_cast<unsigned int>(
    std::llround((kRoomTop + 2 * kMargin) / resolution));

  nav_msgs::msg::OccupancyGrid message;

  message.info.width = width;
  message.info.height = height;
  message.info.resolution = static_cast<float>(resolution);
  message.info.origin.position.x = -kMargin;
  message.info.origin.position.y = -kMargin;
  message.info.origin.orientation.w = 1.0;

  message.data.assign(static_cast<std::size_t>(width) * height, kBlocked);

  for (unsigned int row = 0; row < height; ++row)
  {
    for (unsigned int column = 0; column < width; ++column)
    {
      const double centre_x = -kMargin + (column + 0.5) * resolution;
      const double centre_y = -kMargin + (row + 0.5) * resolution;

      if (isInsideTheRoom(centre_x, centre_y))
        message.data[static_cast<std::size_t>(row) * width + column] = kFree;
    }
  }

  return message;
}

std::vector<std::pair<double, double>> roomRing()
{
  return {
    {0.0, 0.0},
    {kRoomRight, 0.0},
    {kRoomRight, kNotchBottom},
    {kNotchLeft, kNotchBottom},
    {kNotchLeft, kRoomTop},
    {0.0, kRoomTop}};
}

double sineOfTheAngleToTheWall(
  const double heading, const std::pair<double, double>& meeting)
{
  const std::vector<std::pair<double, double>> ring = roomRing();

  const double ux = std::cos(heading);
  const double uy = std::sin(heading);

  double nearest = std::numeric_limits<double>::max();
  double sine = 1.0;

  for (std::size_t l = 0; l < ring.size(); ++l)
  {
    const std::pair<double, double>& from = ring[l];
    const std::pair<double, double>& to = ring[(l + 1) % ring.size()];

    const double wx = to.first - from.first;
    const double wy = to.second - from.second;
    const double length = std::hypot(wx, wy);

    const double along = std::clamp(
      ((meeting.first - from.first) * wx + (meeting.second - from.second) * wy) /
        (length * length),
      0.0, 1.0);

    const double distance = std::hypot(
      meeting.first - (from.first + along * wx),
      meeting.second - (from.second + along * wy));

    if (distance >= nearest)
      continue;

    nearest = distance;
    sine = std::abs(ux * wy - uy * wx) / length;
  }

  return sine;
}

class ReferenceAgreement : public testing::TestWithParam<CastingMethod>
{
};

}

TEST_P(ReferenceAgreement, TheRingAndTheGridAnswerTheSameRoomAtEveryPose)
{
  const std::shared_ptr<const OccupancyGridMap> map =
    roomGrid(kFineResolution, GetParam());
  ASSERT_NE(map, nullptr);

  for (const NamedPose& named : posesInTheRoom())
  {
    ASSERT_TRUE(isInsideTheRoom(named.pose.x, named.pose.y))
      << named.what << " is not in the room, so neither reference is being "
         "asked a question it can answer";

    const Disagreement measured =
      measureDisagreement(*map, GetParam(), {named}, false);

    report(named.what, *map, measured);

    ASSERT_GT(measured.compared, kMinComparedShare * kNumRays)
      << named.what << ": " << measured.grazing << " of " << kNumRays
      << " rays pass within one cell of a corner, which leaves too little of "
         "the room for what is left to be a measurement of anything";

    EXPECT_LE(measured.tightest.difference, measured.tightest.bound)
      << named.what << ": ray " << measured.tightest.ray << " at "
      << measured.tightest.heading << " rad meets its wall at a sine of "
      << measured.tightest.sine_to_wall << ", where a caster answering to a "
         "cell can be "
      << measured.tightest.bound / map->resolution() << " cells out, and the "
         "two references differ by "
      << measured.tightest.difference / map->resolution()
      << " cells: ring " << measured.tightest.from_ring << " m against grid "
      << measured.tightest.from_grid << " m. Either they are not describing "
         "the same room, or the ray angle convention has drifted between "
         "X::findExactAngular and OccupancyGridMap::ranges";
  }
}

TEST_P(ReferenceAgreement, TheDisagreementScalesWithTheCellSize)
{
  const std::shared_ptr<const OccupancyGridMap> fine =
    roomGrid(kFineResolution, GetParam());
  const std::shared_ptr<const OccupancyGridMap> coarse =
    roomGrid(kCoarseResolution, GetParam());
  ASSERT_NE(fine, nullptr);
  ASSERT_NE(coarse, nullptr);

  const Disagreement at_fine =
    measureDisagreement(*fine, GetParam(), posesInTheRoom(), true);
  const Disagreement at_coarse =
    measureDisagreement(*coarse, GetParam(), posesInTheRoom(), true);

  report("fine grid", *fine, at_fine);
  report("coarse grid", *coarse, at_coarse);

  ASSERT_GT(at_fine.compared, 0U);
  ASSERT_GT(at_coarse.compared, 0U);

  EXPECT_LE(at_fine.tightest.difference, at_fine.tightest.bound)
    << "at " << fine->resolution() << " m a cell, ray " << at_fine.tightest.ray
    << " disagrees by " << at_fine.tightest.difference / fine->resolution()
    << " cells where its geometry allows "
    << at_fine.tightest.bound / fine->resolution();

  EXPECT_LE(at_coarse.tightest.difference, at_coarse.tightest.bound)
    << "at " << coarse->resolution() << " m a cell, ray "
    << at_coarse.tightest.ray << " disagrees by "
    << at_coarse.tightest.difference / coarse->resolution()
    << " cells where its geometry allows "
    << at_coarse.tightest.bound / coarse->resolution();

  EXPECT_GT(at_coarse.largest.difference, at_fine.tightest.bound)
    << "the coarse grid disagrees by " << at_coarse.largest.difference
    << " m, which the fine grid was allowed to as well: the bound is not "
       "tracking the cell size at all";

  const double cell_ratio = coarse->resolution() / fine->resolution();

  EXPECT_GT(at_coarse.largest.difference, 0.5 * cell_ratio * at_fine.largest.difference)
    << "multiplying the cell size by " << cell_ratio
    << " moved the largest disagreement from " << at_fine.largest.difference
    << " m to " << at_coarse.largest.difference
    << " m, which is not the cell size showing through";

  EXPECT_LT(at_coarse.largest.difference, 2.0 * cell_ratio * at_fine.largest.difference)
    << "multiplying the cell size by " << cell_ratio
    << " multiplied the largest disagreement by more than "
    << 2.0 * cell_ratio << ", so something other than the cells is growing";
}

TEST_P(ReferenceAgreement, TheTwoReferencesAgreeOnWhatIsFreeSpace)
{
  const std::shared_ptr<const OccupancyGridMap> map =
    roomGrid(kFineResolution, GetParam());
  ASSERT_NE(map, nullptr);

  const std::vector<std::pair<double, double>> ring = roomRing();

  const double step = kFineResolution / 3.0;
  unsigned int probed = 0U;
  unsigned int inside = 0U;

  for (double x = -kMargin + step / 2; x < kRoomRight + kMargin; x += step)
  {
    for (double y = -kMargin + step / 2; y < kRoomTop + kMargin; y += step)
    {
      const FSM::Pose probe{.x = x, .y = y, .t = 0.0};

      const bool ring_says = FSM::Utils::isPositionInMap(probe, ring);
      const bool grid_says =
        FSM::Utils::isPositionInMap(probe, FSM::Reference(*map, kMaxRange));

      probed++;
      inside += ring_says ? 1U : 0U;

      ASSERT_EQ(ring_says, grid_says)
        << "at (" << x << ", " << y << ") the ring says "
        << (ring_says ? "free" : "not free") << " and the grid says "
        << (grid_says ? "free" : "not free") << ". No cell of this grid "
           "straddles a wall, so the two are answering the same question and "
           "there is no cell size to hide behind";
    }
  }

  EXPECT_GT(inside, 0U);
  EXPECT_LT(inside, probed);
}

INSTANTIATE_TEST_SUITE_P(
  SpatialCasters, ReferenceAgreement,
  testing::Values(CastingMethod::bresenham, CastingMethod::ray_marching),
  methodSuffix);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
