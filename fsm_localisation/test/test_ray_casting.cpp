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
 * @brief Ray marching, Bresenham's line and CDDT over grids drawn by hand,
 * held against the range geometry says rather than against each other.
 * @details
 * Every room below is small enough to draw on paper and every expected range
 * is a count of cells worked out inside the case from the room's own
 * dimensions. Nothing here was recorded from a previous run, so a case that
 * fails is saying a caster walks the cells wrongly and not that a number has
 * moved.
 * **What a range is measured to.** A caster stops at a cell, never at the wall
 * inside it, so the finest answer any of this can give is a cell. The analytic
 * range below is therefore the distance from the sensor to the *centre* of the
 * first cell the ray enters. The sensor stands at the centre of its own cell in
 * every case, so that distance is a whole number of cells along an axis and a
 * whole number times the square root of two along a diagonal, and a reader can
 * check it by counting squares. The wall's near face is half a cell nearer than
 * its centre, which is inside the same one cell bound, and measuring to the
 * near face instead would put ray marching's own quantisation exactly on the
 * bound rather than half way inside it.
 * **The three bounds, and why they differ.** `toleranceOf` below carries them
 * with their derivations. In short: ray marching and Bresenham quantise
 * position to a cell and nothing else, so one cell covers them; CDDT quantises
 * twice, once in heading and once in the width of the lookup table bin it finds
 * obstacles through, and one cell does not cover the second of those for a ray
 * along a diagonal.
 * **Two cases hold exactly and are asserted with no tolerance at all.** An
 * entirely free grid returns the maximum range for every ray of every method,
 * and a ray that leaves the grid does the same. A sensor standing on a blocked
 * cell returns exactly zero for Bresenham and for CDDT, both of which test the
 * cell underneath the query point before doing anything else; ray marching has
 * no such test and returns the distance from the sensor to the corner of the
 * cell it is standing in, which is up to `hypot(0.5, 0.5)` of a cell. That last
 * behaviour is inherited from the published `range_libc` and is the reason
 * `ASensorInsideAnObstacleSeesNothingBeyondTheCellItStandsIn` is held to a cell
 * rather than to zero for all three.
 * Ray `i` of `num_rays` leaves at `i * 2*M_PI / num_rays + pose.t - M_PI`,
 * wrapped, read out of `FSM::X::findExactAngular`. A pose heading of `M_PI` and
 * a ray count of eight therefore put ray zero along positive x, ray one along
 * the positive diagonal, ray two along positive y and so on round, which is how
 * the axis and diagonal cases below place their rays. The convention itself is
 * held down in `test_occupancy_grid_map.cpp`; here it is only used.
 * **Every case above `RayMarchingHoldsItsOneCellBoundAtOffAxisHeadings` leaves
 * along a multiple of forty five degrees.** That covers whether a caster walks
 * the right cells, but it is blind to an error that only shows itself away
 * from those headings: measuring ray marching's answer to the point it
 * actually reached, rather than to the corner of the cell that point
 * truncates into, discards a fraction of a cell correlated with the ray's own
 * heading. At an axis or a diagonal that fraction happens to stay small
 * enough that every case above still holds to one cell whichever of the two
 * points is used, so a defect that measured to the corner instead would pass
 * every one of them. `RayMarchingHoldsItsOneCellBoundAtOffAxisHeadings` casts
 * at headings chosen for the opposite property, and is the case that catches
 * it.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "fsm_localisation/occupancy_grid_map.hpp"

namespace
{
using fsm_localisation::CastingMethod;
using fsm_localisation::CellCoordinates;
using fsm_localisation::OccupancyGridMap;
using fsm_localisation::WorldPoint;

const std::int8_t kBlocked = 100;
const std::int8_t kFree = 0;

const int kOccupancyThreshold = 50;

const unsigned int kWidth = 61U;
const unsigned int kHeight = 91U;
const double kResolution = 0.05;

const unsigned int kSensorColumn = 30U;
const unsigned int kSensorRow = 45U;

const double kMaxRange = 10.0;

const double kCddtThetaDiscretization = 720.0;

struct CellValue
{
  unsigned int column{0U};
  unsigned int row{0U};
  std::int8_t value{kFree};
};

struct GridGeometry
{
  unsigned int width{kWidth};
  unsigned int height{kHeight};
  double resolution{kResolution};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
};

std::vector<CellValue> blockedRectangle(
  const unsigned int first_column, const unsigned int first_row,
  const unsigned int last_column, const unsigned int last_row)
{
  std::vector<CellValue> cells;
  for (unsigned int column = first_column; column <= last_column; ++column)
    for (unsigned int row = first_row; row <= last_row; ++row)
      cells.push_back(CellValue{.column = column, .row = row, .value = kBlocked});

  return cells;
}

std::vector<CellValue> blockedUnion(
  const std::vector<std::vector<CellValue>>& parts)
{
  std::vector<CellValue> cells;
  for (const std::vector<CellValue>& part : parts)
    cells.insert(cells.end(), part.begin(), part.end());

  return cells;
}

nav_msgs::msg::OccupancyGrid gridMessage(
  const GridGeometry& geometry, const std::vector<CellValue>& blocked)
{
  nav_msgs::msg::OccupancyGrid message;

  message.info.width = geometry.width;
  message.info.height = geometry.height;
  message.info.resolution = static_cast<float>(geometry.resolution);
  message.info.origin.position.x = geometry.origin_x;
  message.info.origin.position.y = geometry.origin_y;
  message.info.origin.orientation.z = std::sin(geometry.origin_yaw / 2.0);
  message.info.origin.orientation.w = std::cos(geometry.origin_yaw / 2.0);

  message.data.assign(
    static_cast<std::size_t>(geometry.width) * geometry.height, kFree);

  for (const CellValue& cell : blocked)
    message.data[static_cast<std::size_t>(cell.row) * geometry.width + cell.column] =
      cell.value;

  return message;
}

std::string methodSuffix(const testing::TestParamInfo<CastingMethod>& info)
{
  return std::string(fsm_localisation::nameOfCastingMethod(info.param));
}

double toleranceOf(
  const CastingMethod method, const double heading, const double range,
  const double resolution)
{
  if (method != CastingMethod::cddt)
    return resolution;

  const double bin_width_in_cells =
    std::abs(std::sin(heading)) + std::abs(std::cos(heading));

  return bin_width_in_cells * resolution +
    range * M_PI / kCddtThetaDiscretization;
}

class RayCasting : public testing::TestWithParam<CastingMethod>
{
 protected:

  CellCoordinates sensorCell() const
  {
    return CellCoordinates{.column = kSensorColumn + 0.5, .row = kSensorRow + 0.5};
  }
};

class RayCastingCellUnderneath : public testing::TestWithParam<CastingMethod>
{
};

}

TEST_P(RayCasting, ARectangularRoomIsMetAlongTheAxesAndTheDiagonals)
{
  const unsigned int left_wall = 15U;
  const unsigned int right_wall = 55U;
  const unsigned int bottom_wall = 25U;
  const unsigned int top_wall = 75U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{},
      blockedUnion({
        blockedRectangle(left_wall, 0U, left_wall, kHeight - 1U),
        blockedRectangle(right_wall, 0U, right_wall, kHeight - 1U),
        blockedRectangle(0U, bottom_wall, kWidth - 1U, bottom_wall),
        blockedRectangle(0U, top_wall, kWidth - 1U, top_wall)})),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(sensorCell());

  const unsigned int num_rays = 8U;
  const std::vector<double> ranges =
    map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = M_PI}, num_rays, kMaxRange);

  ASSERT_EQ(ranges.size(), num_rays);

  const double to_right = right_wall - kSensorColumn;
  const double to_left = kSensorColumn - left_wall;
  const double to_top = top_wall - kSensorRow;
  const double to_bottom = kSensorRow - bottom_wall;

  const double expected[8] = {
    to_right,
    std::hypot(std::min(to_right, to_top), std::min(to_right, to_top)),
    to_top,
    std::hypot(std::min(to_left, to_top), std::min(to_left, to_top)),
    to_left,
    std::hypot(std::min(to_left, to_bottom), std::min(to_left, to_bottom)),
    to_bottom,
    std::hypot(std::min(to_right, to_bottom), std::min(to_right, to_bottom))};

  const char* along[8] = {
    "positive x", "the positive x, positive y diagonal", "positive y",
    "the negative x, positive y diagonal", "negative x",
    "the negative x, negative y diagonal", "negative y",
    "the positive x, negative y diagonal"};

  for (unsigned int i = 0; i < num_rays; ++i)
  {
    const double heading = i * 2*M_PI / num_rays;
    const double analytic = expected[i] * map->resolution();
    const double tolerance =
      toleranceOf(GetParam(), heading, analytic, map->resolution());

    EXPECT_NEAR(ranges[i], analytic, tolerance)
      << "ray " << i << " leaves along " << along[i] << " and meets a wall "
      << expected[i] << " cells away, so it must report " << analytic
      << " m to within " << tolerance / map->resolution() << " cells. It "
      << "reported " << ranges[i] << " m, which is "
      << (ranges[i] - analytic) / map->resolution() << " cells out. The four "
      << "gaps in this room are " << to_left << ", " << to_bottom << ", "
      << to_right << " and " << to_top << " cells, all different, so a range "
      << "matching one of the others means the fan is pointing somewhere else "
      << "rather than that the traversal is inaccurate.";
  }
}

TEST_P(RayCasting, ARightAngledCornerIsMetAlongTheDiagonalIntoIt)
{
  const unsigned int vertical_wall = 55U;
  const unsigned int horizontal_wall = 75U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{},
      blockedUnion({
        blockedRectangle(vertical_wall, kSensorRow - 5U, vertical_wall, kHeight - 1U),
        blockedRectangle(kSensorColumn - 5U, horizontal_wall, kWidth - 1U, horizontal_wall)})),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(sensorCell());

  const unsigned int num_rays = 8U;
  const std::vector<double> ranges =
    map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = M_PI}, num_rays, kMaxRange);

  ASSERT_EQ(ranges.size(), num_rays);

  const double to_vertical = vertical_wall - kSensorColumn;
  const double to_horizontal = horizontal_wall - kSensorRow;

  const double corner_cells =
    std::hypot(std::min(to_vertical, to_horizontal), std::min(to_vertical, to_horizontal));

  const double heading = 2*M_PI / num_rays;
  const double analytic = corner_cells * map->resolution();
  const double tolerance =
    toleranceOf(GetParam(), heading, analytic, map->resolution());

  EXPECT_NEAR(ranges[1], analytic, tolerance)
    << "ray one leaves along the diagonal into the corner. The vertical wall "
    << "stands " << to_vertical << " cells away and the horizontal one "
    << to_horizontal << ", so the diagonal meets the nearer of them after "
    << std::min(to_vertical, to_horizontal) << " cells in each direction, "
    << corner_cells << " cells along the ray, which is " << analytic << " m. "
    << "It reported " << ranges[1] << " m, "
    << (ranges[1] - analytic) / map->resolution() << " cells out. A range near "
    << std::max(to_vertical, to_horizontal) * std::sqrt(2.0) * map->resolution()
    << " m is the further wall, which means the ray walked past the nearer one.";

  EXPECT_NEAR(ranges[0], to_vertical * map->resolution(),
    toleranceOf(GetParam(), 0.0, to_vertical * map->resolution(), map->resolution()))
    << "ray zero leaves along positive x, straight at the vertical wall "
    << to_vertical << " cells away.";

  EXPECT_NEAR(ranges[2], to_horizontal * map->resolution(),
    toleranceOf(GetParam(), M_PI/2, to_horizontal * map->resolution(), map->resolution()))
    << "ray two leaves along positive y, straight at the horizontal wall "
    << to_horizontal << " cells away.";

  for (const unsigned int i : {3U, 4U, 5U, 6U, 7U})
    EXPECT_EQ(ranges[i], kMaxRange)
      << "ray " << i << " leaves away from the corner, and both walls stop "
      << "short of reaching it, so it leaves the grid having met nothing and "
      << "must report the maximum range of " << kMaxRange << " m exactly. It "
      << "reported " << ranges[i] << " m.";
}

TEST_P(RayCasting, AnEntirelyFreeGridReturnsTheMaximumRangeForEveryRayExactly)
{
  const auto built = OccupancyGridMap::build(
    gridMessage(GridGeometry{}, {}), kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const double diagonal = std::hypot(
    static_cast<double>(map->width()), static_cast<double>(map->height())) *
    map->resolution();

  const WorldPoint sensor = map->worldFromCell(sensorCell());
  const unsigned int num_rays = 23U;

  for (const double max_range : {diagonal / 2.0, diagonal * 2.0})
  {
    const std::vector<double> ranges = map->ranges(
      FSM::Pose{.x = sensor.x, .y = sensor.y, .t = 0.37}, num_rays, max_range);

    ASSERT_EQ(ranges.size(), num_rays);

    for (unsigned int i = 0; i < num_rays; ++i)
      EXPECT_EQ(ranges[i], max_range)
        << "ray " << i << " of " << num_rays << " crosses a grid with nothing "
        << "in it, so it meets nothing and must report the maximum range of "
        << max_range << " m exactly, with no tolerance: there is no cell for "
        << "any of the three methods to stop at, so this is one of the two "
        << "answers all three give identically. It reported " << ranges[i]
        << " m. A report of " << diagonal << " m is the grid's own diagonal, "
        << "which is the maximum the caster itself was built with.";
  }
}

TEST_P(RayCasting, AnIsolatedBlockedCellIsMetHeadOnAndMissedEitherSide)
{
  const unsigned int obstacle_column = 52U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{},
      blockedRectangle(obstacle_column, kSensorRow, obstacle_column, kSensorRow)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(sensorCell());

  const unsigned int num_rays = 4U;
  const std::vector<double> ranges =
    map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = M_PI}, num_rays, kMaxRange);

  ASSERT_EQ(ranges.size(), num_rays);

  const double gap = obstacle_column - kSensorColumn;
  const double analytic = gap * map->resolution();

  EXPECT_NEAR(ranges[0], analytic,
    toleranceOf(GetParam(), 0.0, analytic, map->resolution()))
    << "ray zero leaves along positive x, in the sensor's own row, and the one "
    << "blocked cell of this grid stands " << gap << " cells along it, so it "
    << "must report " << analytic << " m. It reported " << ranges[0] << " m, "
    << (ranges[0] - analytic) / map->resolution() << " cells out.";

  for (const unsigned int i : {1U, 2U, 3U})
    EXPECT_EQ(ranges[i], kMaxRange)
      << "ray " << i << " leaves at right angles to the one blocked cell or "
      << "away from it altogether, so it meets nothing and must report the "
      << "maximum range of " << kMaxRange << " m exactly. It reported "
      << ranges[i] << " m, which means an obstacle one cell across is being "
      << "reported to rays that do not point at it.";
}

TEST_P(RayCasting, ARayLeavingTheGridReportsTheMaximumRangeExactly)
{
  const unsigned int wall_column = 15U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{},
      blockedRectangle(wall_column, 0U, wall_column, kHeight - 1U)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const double diagonal = std::hypot(
    static_cast<double>(map->width()), static_cast<double>(map->height())) *
    map->resolution();

  const WorldPoint sensor = map->worldFromCell(sensorCell());

  for (const double max_range : {diagonal / 2.0, diagonal * 2.0})
  {
    const std::vector<double> ranges =
      map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = M_PI}, 4U, max_range);

    ASSERT_EQ(ranges.size(), std::size_t{4});

    for (const unsigned int i : {0U, 1U, 3U})
      EXPECT_EQ(ranges[i], max_range)
        << "ray " << i << " leaves along positive x, positive y or negative y, "
        << "and the room's one wall stands to negative x, so this ray runs off "
        << "the edge of the grid having met nothing and must report the "
        << "maximum range of " << max_range << " m exactly. It reported "
        << ranges[i] << " m. A report of " << diagonal << " m is the grid's "
        << "diagonal, the maximum the caster was built with, being mistaken "
        << "for an obstacle.";

    const double analytic = (kSensorColumn - wall_column) * map->resolution();
    if (analytic < max_range)
    {
      EXPECT_NEAR(ranges[2], analytic,
        toleranceOf(GetParam(), M_PI, analytic, map->resolution()))
        << "ray two leaves along negative x, straight at the wall "
        << kSensorColumn - wall_column << " cells away, so the case is testing "
        << "a ray that escapes rather than a grid nothing can be found in.";
    }
  }
}

TEST_P(RayCasting, ASensorInsideAnObstacleSeesNothingBeyondTheCellItStandsIn)
{
  const std::vector<CellValue> geometry[2] = {
    blockedRectangle(kSensorColumn, kSensorRow, kSensorColumn, kSensorRow),
    blockedRectangle(
      kSensorColumn - 3U, kSensorRow - 3U, kSensorColumn + 3U, kSensorRow + 3U)};
  const char* description[2] = {
    "one blocked cell standing alone", "the middle of a seven by seven block"};

  for (unsigned int g = 0; g < 2; ++g)
  {
    const auto built = OccupancyGridMap::build(
      gridMessage(GridGeometry{}, geometry[g]), kOccupancyThreshold, GetParam());
    ASSERT_TRUE(built.has_value()) << built.error();
    const std::shared_ptr<const OccupancyGridMap> map = built.value();

    const WorldPoint sensor = map->worldFromCell(sensorCell());
    const unsigned int num_rays = 23U;
    const std::vector<double> ranges = map->ranges(
      FSM::Pose{.x = sensor.x, .y = sensor.y, .t = 0.37}, num_rays, kMaxRange);

    ASSERT_EQ(ranges.size(), num_rays);

    for (unsigned int i = 0; i < num_rays; ++i)
      EXPECT_LT(ranges[i], map->resolution())
        << "the sensor stands at the centre of a blocked cell, " << description[g]
        << ", so ray " << i << " is inside an obstacle before it has gone "
        << "anywhere and cannot see past the cell it starts in. The furthest "
        << "corner of that cell is " << std::hypot(0.5, 0.5) << " of a cell "
        << "away, so no range here may reach one cell, "
        << map->resolution() << " m. It reported " << ranges[i] << " m, which "
        << "is " << ranges[i] / map->resolution() << " cells and means the ray "
        << "was cast out of the obstacle rather than stopped inside it.";
  }
}

TEST_P(RayCasting, ASensorOneCellFromAWallReportsOneCell)
{
  const unsigned int wall_column = kSensorColumn + 1U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{},
      blockedRectangle(wall_column, 0U, wall_column, kHeight - 1U)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(sensorCell());

  const std::vector<double> ranges =
    map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = M_PI}, 4U, kMaxRange);

  ASSERT_EQ(ranges.size(), std::size_t{4});

  const double analytic = (wall_column - kSensorColumn) * map->resolution();

  EXPECT_NEAR(ranges[0], analytic,
    toleranceOf(GetParam(), 0.0, analytic, map->resolution()))
    << "the wall stands in the very next column, so ray zero must report one "
    << "cell, " << analytic << " m. It reported " << ranges[0] << " m, "
    << (ranges[0] - analytic) / map->resolution() << " cells out. A range of "
    << "zero would mean the sensor's own cell is being read as blocked; a "
    << "range of " << 2.0 * analytic << " m would mean the first step of the "
    << "traversal steps over the cell in front of it.";
}

TEST_P(RayCasting, AStraightCorridorIsMetAcrossItAndAlongIt)
{
  const unsigned int left_wall = 25U;
  const unsigned int right_wall = 36U;
  const unsigned int near_end = 25U;
  const unsigned int far_end = 75U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{},
      blockedUnion({
        blockedRectangle(left_wall, 0U, left_wall, kHeight - 1U),
        blockedRectangle(right_wall, 0U, right_wall, kHeight - 1U),
        blockedRectangle(0U, near_end, kWidth - 1U, near_end),
        blockedRectangle(0U, far_end, kWidth - 1U, far_end)})),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(sensorCell());

  const unsigned int num_rays = 4U;
  const std::vector<double> ranges =
    map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = M_PI}, num_rays, kMaxRange);

  ASSERT_EQ(ranges.size(), num_rays);

  const double gap[4] = {
    static_cast<double>(right_wall - kSensorColumn),
    static_cast<double>(far_end - kSensorRow),
    static_cast<double>(kSensorColumn - left_wall),
    static_cast<double>(kSensorRow - near_end)};
  const char* along[4] = {
    "across the corridor, to the wall on its positive x side",
    "along the corridor, to its far end",
    "across the corridor, to the wall on its negative x side",
    "along the corridor, to its near end"};

  for (unsigned int i = 0; i < num_rays; ++i)
  {
    const double heading = i * 2*M_PI / num_rays;
    const double analytic = gap[i] * map->resolution();
    const double tolerance =
      toleranceOf(GetParam(), heading, analytic, map->resolution());

    EXPECT_NEAR(ranges[i], analytic, tolerance)
      << "ray " << i << " leaves " << along[i] << ", which is " << gap[i]
      << " cells, so it must report " << analytic << " m to within "
      << tolerance / map->resolution() << " cells. It reported " << ranges[i]
      << " m. The corridor is " << gap[0] + gap[2] + 1.0 << " cells wide and "
      << gap[1] + gap[3] + 1.0 << " cells long, so a range across it where a "
      << "range along it belongs is a fan cast a quarter turn out rather than "
      << "a traversal that is inaccurate.";
  }
}

INSTANTIATE_TEST_SUITE_P(
  AllThreeMethods, RayCasting,
  testing::Values(
    CastingMethod::bresenham, CastingMethod::cddt, CastingMethod::ray_marching),
  methodSuffix);

TEST_P(RayCastingCellUnderneath, ASensorOnABlockedCellReturnsZeroForEveryRay)
{
  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{},
      blockedRectangle(kSensorColumn, kSensorRow, kSensorColumn, kSensorRow)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(
    CellCoordinates{.column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  const unsigned int num_rays = 23U;
  const std::vector<double> ranges = map->ranges(
    FSM::Pose{.x = sensor.x, .y = sensor.y, .t = 0.37}, num_rays, kMaxRange);

  ASSERT_EQ(ranges.size(), num_rays);

  for (unsigned int i = 0; i < num_rays; ++i)
    EXPECT_EQ(ranges[i], 0.0)
      << "the sensor stands on a blocked cell, so ray " << i << " has met an "
      << "obstacle before it has travelled anywhere and must report exactly "
      << "zero, with no tolerance: this method tests the cell underneath the "
      << "query point and returns from there. It reported " << ranges[i]
      << " m, which is " << ranges[i] / map->resolution() << " cells, so that "
      << "test is either not being reached or is being reached after a "
      << "distance has already been computed.";
}

INSTANTIATE_TEST_SUITE_P(
  TheTwoMethodsThatTestTheCellUnderneath, RayCastingCellUnderneath,
  testing::Values(CastingMethod::bresenham, CastingMethod::cddt),
  methodSuffix);

TEST(RayCastingAgreement, CddtAgreesWithBresenhamWithinItsOwnQuantisation)
{
  const unsigned int left_wall = 15U;
  const unsigned int right_wall = 55U;
  const unsigned int bottom_wall = 25U;
  const unsigned int top_wall = 75U;

  const nav_msgs::msg::OccupancyGrid message = gridMessage(
    GridGeometry{},
    blockedUnion({
      blockedRectangle(left_wall, 0U, left_wall, kHeight - 1U),
      blockedRectangle(right_wall, 0U, right_wall, kHeight - 1U),
      blockedRectangle(0U, bottom_wall, kWidth - 1U, bottom_wall),
      blockedRectangle(0U, top_wall, kWidth - 1U, top_wall)}));

  const auto bresenham = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::bresenham);
  ASSERT_TRUE(bresenham.has_value()) << bresenham.error();

  const auto cddt = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::cddt);
  ASSERT_TRUE(cddt.has_value()) << cddt.error();

  const double resolution = bresenham.value()->resolution();
  const WorldPoint sensor = bresenham.value()->worldFromCell(
    CellCoordinates{.column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});
  const FSM::Pose pose{.x = sensor.x, .y = sensor.y, .t = M_PI};

  const unsigned int num_rays = 8U;
  const std::vector<double> straight =
    bresenham.value()->ranges(pose, num_rays, kMaxRange);
  const std::vector<double> binned =
    cddt.value()->ranges(pose, num_rays, kMaxRange);

  ASSERT_EQ(straight.size(), binned.size());

  for (unsigned int i = 0; i < num_rays; ++i)
  {
    const double heading = i * 2*M_PI / num_rays;
    const double tolerance =
      toleranceOf(CastingMethod::cddt, heading, straight[i], resolution) +
      toleranceOf(CastingMethod::bresenham, heading, straight[i], resolution);

    EXPECT_NEAR(binned[i], straight[i], tolerance)
      << "ray " << i << " of " << num_rays << ", at a heading of " << heading
      << " rad: Bresenham reported " << straight[i] << " m and CDDT "
      << binned[i] << " m, a difference of "
      << (binned[i] - straight[i]) / resolution << " cells against a bound of "
      << tolerance / resolution << " cells. That bound is CDDT's lookup table "
      << "bin, " << std::abs(std::sin(heading)) + std::abs(std::cos(heading))
      << " cells wide at this heading, plus its half degree heading bin at "
      << "this range, plus Bresenham's own cell. A larger disagreement than "
      << "that is not CDDT's quantisation and needs another explanation.";
  }
}

TEST(RayCastingAgreement, RayMarchingAndBresenhamAgreeToWithinOneCell)
{
  const unsigned int left_wall = 15U;
  const unsigned int right_wall = 55U;
  const unsigned int bottom_wall = 25U;
  const unsigned int top_wall = 75U;

  const nav_msgs::msg::OccupancyGrid message = gridMessage(
    GridGeometry{},
    blockedUnion({
      blockedRectangle(left_wall, 0U, left_wall, kHeight - 1U),
      blockedRectangle(right_wall, 0U, right_wall, kHeight - 1U),
      blockedRectangle(0U, bottom_wall, kWidth - 1U, bottom_wall),
      blockedRectangle(0U, top_wall, kWidth - 1U, top_wall)}));

  const auto bresenham = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::bresenham);
  ASSERT_TRUE(bresenham.has_value()) << bresenham.error();

  const auto marching = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(marching.has_value()) << marching.error();

  const double resolution = bresenham.value()->resolution();
  const WorldPoint sensor = bresenham.value()->worldFromCell(
    CellCoordinates{.column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});
  const FSM::Pose pose{.x = sensor.x, .y = sensor.y, .t = M_PI};

  const unsigned int num_rays = 8U;
  const std::vector<double> stepped =
    bresenham.value()->ranges(pose, num_rays, kMaxRange);
  const std::vector<double> marched =
    marching.value()->ranges(pose, num_rays, kMaxRange);

  ASSERT_EQ(stepped.size(), marched.size());

  for (unsigned int i = 0; i < num_rays; ++i)
    EXPECT_NEAR(marched[i], stepped[i], resolution)
      << "ray " << i << " of " << num_rays << ": Bresenham reported "
      << stepped[i] << " m and ray marching " << marched[i] << " m, a "
      << "difference of " << (marched[i] - stepped[i]) / resolution
      << " cells. Both walk the same cells of the same grid and quantise "
      << "nothing but position, so a cell of " << resolution << " m is the "
      << "whole of the difference either can have from the other. A larger one "
      << "means the two are walking different cells, which is what the fourth "
      << "cbgl defect did by rounding one axis of the marching point up and "
      << "the other down.";
}

TEST(RayCastingAgreement, RayMarchingHoldsItsOneCellBoundAtOffAxisHeadings)
{
  const unsigned int width = 200U;
  const unsigned int height = 200U;
  const unsigned int left_wall = 2U;
  const unsigned int right_wall = 197U;
  const unsigned int bottom_wall = 2U;
  const unsigned int top_wall = 197U;
  const unsigned int sensor_column = 70U;
  const unsigned int sensor_row = 55U;
  const double max_range = 20.0;

  const GridGeometry geometry{.width = width, .height = height};

  const auto built = OccupancyGridMap::build(
    gridMessage(
      geometry,
      blockedUnion({
        blockedRectangle(left_wall, 0U, left_wall, height - 1U),
        blockedRectangle(right_wall, 0U, right_wall, height - 1U),
        blockedRectangle(0U, bottom_wall, width - 1U, bottom_wall),
        blockedRectangle(0U, top_wall, width - 1U, top_wall)})),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(
    CellCoordinates{.column = sensor_column + 0.5, .row = sensor_row + 0.5});

  const double headings[2] = {5.817531, 2.074324};

  for (const double heading : headings)
  {
    const double ux = std::cos(heading);
    const double uy = std::sin(heading);

    const double t_vertical =
      (ux > 0.0 ? right_wall - (sensor_column + 0.5) : left_wall - (sensor_column + 0.5)) / ux;
    const double t_horizontal =
      (uy > 0.0 ? top_wall - (sensor_row + 0.5) : bottom_wall - (sensor_row + 0.5)) / uy;

    double analytic_cells = 0.0;
    if (t_vertical < t_horizontal)
    {
      const double wall = ux > 0.0 ? right_wall : left_wall;
      const double row_cross = sensor_row + 0.5 + uy * t_vertical;
      analytic_cells = std::hypot(
        wall + 0.5 - (sensor_column + 0.5), std::floor(row_cross) + 0.5 - (sensor_row + 0.5));
    }
    else
    {
      const double wall = uy > 0.0 ? top_wall : bottom_wall;
      const double column_cross = sensor_column + 0.5 + ux * t_horizontal;
      analytic_cells = std::hypot(
        std::floor(column_cross) + 0.5 - (sensor_column + 0.5), wall + 0.5 - (sensor_row + 0.5));
    }

    const double analytic = analytic_cells * map->resolution();
    const double tolerance =
      toleranceOf(CastingMethod::ray_marching, heading, analytic, map->resolution());

    const std::vector<double> ranges =
      map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = heading + M_PI}, 1U, max_range);

    ASSERT_EQ(ranges.size(), std::size_t{1});

    EXPECT_NEAR(ranges[0], analytic, tolerance)
      << "a ray at " << heading << " rad, neither along an axis nor a "
      << "diagonal, must report " << analytic << " m to within "
      << tolerance / map->resolution() << " cell. It reported " << ranges[0]
      << " m, " << (ranges[0] - analytic) / map->resolution() << " cells out. "
      << "A defect that measured to the corner of the truncated cell rather "
      << "than to the point the ray reached would miss this bound rather than "
      << "sit inside it.";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
