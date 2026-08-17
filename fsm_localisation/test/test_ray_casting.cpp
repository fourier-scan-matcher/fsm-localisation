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
 *        held against the range geometry says rather than against each other.
 *
 * @details
 * Every room below is small enough to draw on paper and every expected range
 * is a count of cells worked out inside the case from the room's own
 * dimensions. Nothing here was recorded from a previous run, so a case that
 * fails is saying a caster walks the cells wrongly and not that a number has
 * moved.
 *
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
 *
 * **The three bounds, and why they differ.** `toleranceOf` below carries them
 * with their derivations. In short: ray marching and Bresenham quantise
 * position to a cell and nothing else, so one cell covers them; CDDT quantises
 * twice, once in heading and once in the width of the lookup table bin it finds
 * obstacles through, and one cell does not cover the second of those for a ray
 * along a diagonal.
 *
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
 *
 * Ray `i` of `num_rays` leaves at `i * 2*M_PI / num_rays + pose.t - M_PI`,
 * wrapped, read out of `FSM::X::findExactAngular`. A pose heading of `M_PI` and
 * a ray count of eight therefore put ray zero along positive x, ray one along
 * the positive diagonal, ray two along positive y and so on round, which is how
 * the axis and diagonal cases below place their rays. The convention itself is
 * held down in `test_occupancy_grid_map.cpp`; here it is only used.
 *
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

/* Non square, so a case that reads a cell the wrong way round is wrong rather
 * than merely lucky, and odd in both directions so that there is a cell at the
 * centre for the sensor to stand in. */
const unsigned int kWidth = 61U;
const unsigned int kHeight = 91U;
const double kResolution = 0.05;

const unsigned int kSensorColumn = 30U;
const unsigned int kSensorRow = 45U;

/* Beyond the grid's diagonal of `hypot(61, 91) * 0.05` metres, so a ray that
 * meets nothing reports this and cannot be mistaken for a ray that met the far
 * corner. */
const double kMaxRange = 10.0;

/* CDDT rounds a cast heading into one of this many bins. It is fixed at 720 in
 * `occupancy_grid_map.cpp` to match cbgl, and it is named here because it sets
 * the tolerance of every CDDT range below, not because this file may choose
 * it. */
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

/*
 * A rectangle of blocked cells, inclusive at both ends. Every wall, corner and
 * isolated obstacle below is one or more of these.
 */
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

/*
 * The union of several rectangles, so a room can be described as the list of
 * its walls.
 */
std::vector<CellValue> blockedUnion(
  const std::vector<std::vector<CellValue>>& parts)
{
  std::vector<CellValue> cells;
  for (const std::vector<CellValue>& part : parts)
    cells.insert(cells.end(), part.begin(), part.end());

  return cells;
}

/*
 * A `nav_msgs::msg::OccupancyGrid` of the given geometry with `blocked` set and
 * every other cell free. The only place in this file that computes a cell's
 * index into the message's flat `data`, in the message's own row major order.
 */
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

/*
 * How far a method's reported range may sit from the analytic range, in metres,
 * for a ray of heading `heading` whose analytic range is `range`.
 *
 * **Ray marching and Bresenham: one cell, for the headings this file casts at.**
 * Every case in this file leaves along an axis or a diagonal, and at those
 * headings ray marching answers with the distance from the sensor to the
 * integer corner of the cell it first samples as occupied, which is up to
 * `hypot(0.5, 0.5)` of a cell from that cell's centre; Bresenham steps a whole
 * cell at a time from the sensor's own position and answers with a whole
 * number of steps. Neither has any other source of error at these headings,
 * and the measurements in this file sit at 0.71 of a cell.
 *
 * One cell is not a general bound on ray marching, only what these particular
 * headings measure to: at an axis or a diagonal, the step it advances by,
 * `max(0.999 * d, 1)` cells, and the cell it truncates into line up in a way
 * that keeps the two comfortably apart. At an arbitrary heading they need not,
 * and `test_reference_agreement.cpp` derives the fuller figure, `sqrt(2)`
 * cells, from the same algorithm without that alignment to rely on.
 * `RayMarchingHoldsItsOneCellBoundAtOffAxisHeadings` below casts at two such
 * headings and still holds to one cell, but that is this file's own room
 * being kind rather than a property ray marching offers everywhere.
 *
 * **CDDT: the width of a lookup table bin, plus its heading quantisation.** It
 * rounds the cast heading into one of `kCddtThetaDiscretization` bins, which at
 * a range of `range` displaces the far end of the ray by
 * `range * M_PI / kCddtThetaDiscretization`. And it finds obstacles not by
 * walking cells but by projecting them onto an axis along the ray and looking
 * in the bin the sensor projects into; `half_lut_space_width` in `RangeLib.h`
 * makes each cell occupy `(|sin heading| + |cos heading|)` bins, so a cell that
 * far to one side of the ray can be the one it answers with, and such a cell
 * can be nearer along the ray than the cell the ray really meets.
 *
 * That second term is exactly one cell for a ray along an axis, which is the
 * bound the amended step names, and `sqrt(2)` cells for a ray along a diagonal,
 * which is where a single cell is not enough: the diagonal cases below are
 * measured at 1.41 cells short, and no room in this file needs more.
 */
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

/* ========================================================================== */
class RayCasting : public testing::TestWithParam<CastingMethod>
{
 protected:

  /* Every case casts from the centre of the middle cell, so that a range along
   * an axis is a difference of whole cell indices. */
  CellCoordinates sensorCell() const
  {
    return CellCoordinates{.column = kSensorColumn + 0.5, .row = kSensorRow + 0.5};
  }
};

class RayCastingCellUnderneath : public testing::TestWithParam<CastingMethod>
{
};

}  // namespace

/*
 * Rays along the axes and along the diagonals of a rectangular room, which is
 * where a cell traversal fault hides: an axis aligned ray never changes one of
 * its two indices, and a diagonal ray changes both at every step and passes
 * exactly through the corners between cells.
 *
 * Eight rays with a pose heading of `M_PI` put ray zero along positive x and
 * every ray after it a further eighth of a turn round. The four gaps to the
 * walls are fifteen, twenty, twenty five and thirty cells, all different, so a
 * fan cast in the wrong direction or turned by a quarter cannot match by
 * symmetry.
 */
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

  /* The gap in cells from the sensor's cell to each wall. */
  const double to_right = right_wall - kSensorColumn;
  const double to_left = kSensorColumn - left_wall;
  const double to_top = top_wall - kSensorRow;
  const double to_bottom = kSensorRow - bottom_wall;

  /* A ray along an axis meets that wall after the gap. A ray along a diagonal
   * advances one cell in each direction at a time, so it meets whichever of the
   * two walls is the fewer cells away, after that many cells in each direction,
   * which is `hypot` of the two. */
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

/*
 * A right angled corner, with the two walls finite so that only the three rays
 * that ought to meet them can.
 *
 * The diagonal into the corner is the case worth having: it advances both
 * indices at once and crosses cell corners exactly, which is where a traversal
 * that rounds one axis differently from the other gives itself away, and it was
 * the fourth cbgl defect's own symptom.
 */
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

  /* Into the corner: the ray advances one cell in each direction at a time, so
   * it meets the nearer of the two walls after that many cells each way. */
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

  /* The two walls stop short of the sensor's own row and column, so the five
   * rays that leave away from the corner have nothing at all to meet. */
  for (const unsigned int i : {3U, 4U, 5U, 6U, 7U})
    EXPECT_EQ(ranges[i], kMaxRange)
      << "ray " << i << " leaves away from the corner, and both walls stop "
      << "short of reaching it, so it leaves the grid having met nothing and "
      << "must report the maximum range of " << kMaxRange << " m exactly. It "
      << "reported " << ranges[i] << " m.";
}

/*
 * An entirely free grid. Every ray of every method reports the maximum range,
 * exactly, and this is one of the two places in this file where the three
 * agree with no tolerance at all: there is nothing for any of them to quantise.
 *
 * Both sides of the maximum range rule are covered here, since the caster's own
 * maximum is the grid's diagonal in cells and the scan's may be either side of
 * it.
 */
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

/*
 * A single blocked cell standing alone in open space: the ray at it meets it,
 * and the three rays that do not point at it meet nothing whatever.
 *
 * The second half is the more interesting one. An isolated cell is the whole of
 * the geometry, so a caster that smears an obstacle across the rays either side
 * of it has nothing else to hide behind.
 */
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

/*
 * A ray that leaves the grid entirely, with only one wall in the room so that
 * three of the four rays have nothing in front of them.
 *
 * A ray that met nothing and a ray that met something at the maximum range are
 * different answers, and the caster cannot tell them apart on its own: it is
 * built with the grid's diagonal as its maximum, and a range at that maximum
 * has to come back as the scan's maximum rather than as an obstacle at the far
 * corner. Exact, both sides of the diagonal.
 */
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

    /* And the one ray that does have the wall in front of it still finds it. */
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

/*
 * A sensor standing on a blocked cell, at the centre of that cell.
 *
 * No ray can see past the cell the sensor is inside, so no range can exceed the
 * distance from the sensor to the furthest corner of its own cell, which is
 * `hypot(0.5, 0.5)` of a cell and therefore less than a cell. Bresenham and
 * CDDT return zero here, which the case beside this one asserts exactly; ray
 * marching returns the distance to its own cell's integer corner, because it
 * finds the cell it is standing in by truncating its position rather than by
 * testing it. All three are covered by the one cell bound.
 *
 * Both an isolated blocked cell and the middle of a solid block are tried,
 * because CDDT builds its lookup table from the edge of geometry rather than
 * from its interior, and a cell deep inside a block is not in that table.
 */
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

/*
 * A sensor one cell from a wall, which is the shortest range a grid can report
 * that is not zero and the place a caster's first step is most likely to
 * overshoot.
 */
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

/*
 * A straight corridor, cast across it and along it in one fan of four rays.
 *
 * Across is a short range in both directions and along is a long one in both,
 * and the four are all different, so a fan turned by a quarter turn reports
 * across where along belongs and fails rather than passing on symmetry.
 */
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

/*
 * A sensor standing on a blocked cell returns exactly zero.
 *
 * The second of the two answers in this file that need no tolerance, and it
 * holds for the two methods that test the cell underneath the query point
 * before they do anything else: `BresenhamsLine::calc_range` calls
 * `map.isOccupied` on its first line, and `CDDTCast::calc_range` reaches its own
 * `map.grid[x][y]` test before it can return a distance. Ray marching has no
 * such test, which is why it is not instantiated here and is covered by the one
 * cell bound instead.
 */
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

/*
 * CDDT against the same rectangular room, held to its own quantisation.
 *
 * The bound is `toleranceOf`'s: the width of a lookup table bin in cells plus
 * the heading quantisation as a distance. Along an axis that first term is one
 * cell, which is the bound the amended step names and which CDDT meets with
 * nothing to spare needed; along a diagonal it is `sqrt(2)` cells, and the
 * measurement sits at 1.41 cells, so the amended step's single cell would fail
 * here for a reason that is not a defect.
 *
 * Bresenham is the reference rather than ray marching, since Bresenham reports
 * the whole number of cells between the two cell centres and so is the analytic
 * answer itself in a room of walls like this one.
 */
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

/*
 * Ray marching and Bresenham against each other, to within one cell.
 *
 * Neither quantises anything but position, so a cell is the whole of the
 * difference between them and this is the pair the amended step's one cell
 * bound was written for. It is measured at 0.71 of a cell across every ray of
 * this room, which is the half cell each of them can differ from the cell
 * centre in each of two directions.
 */
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

/*
 * Ray marching cast at two headings that are neither a multiple of forty five
 * degrees nor close enough to one to inherit its cancellation, held to the
 * same one cell bound every case above uses.
 *
 * The room is larger than the rest of this file's, on purpose: the corner
 * truncation fraction these headings are chosen to expose needs a longer ray
 * to grow past what ray marching's own step size already costs before it is
 * visible at all, and the smaller room shared by the cases above never gives
 * it that room. Both headings were found by sweeping this room's own geometry
 * for cases where the corrected caster, measuring to the point the ray
 * actually reached, sits inside the bound with room to spare, and the caster
 * it replaced, measuring to the corner of the cell that point truncates into,
 * does not. Restoring `xd = px - x0` and `yd = py - y0` in
 * `RangeLib.h:RayMarching::calc_range` fails this case; the tolerance itself
 * is untouched.
 */
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

  /* Neither a multiple of forty five degrees nor within eight degrees of one,
   * found by sweeping this room for headings where the fix holds with room to
   * spare and the defect it replaced does not. */
  const double headings[2] = {5.817531, 2.074324};

  for (const double heading : headings)
  {
    const double ux = std::cos(heading);
    const double uy = std::sin(heading);

    /* The wall this heading meets, by simple ray against axis aligned box
     * crossing, and the row or column of the first cell entered along the
     * other axis at that crossing. */
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
