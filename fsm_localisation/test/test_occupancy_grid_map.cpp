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
 * @brief `fsm_localisation::OccupancyGridMap`: what it refuses, what it calls
 * free, how it converts between the world and its own cells, and where
 * each of a pose's rays meets it.
 * @details
 * Every expected value below is computed inside the case it is used in, from
 * the grid's own geometry, and every tolerance is read from the map's own
 * `resolution()`. Nothing here is a number recorded from a previous run, so a
 * case that fails is saying the geometry is wrong rather than that it has
 * changed.
 * Four conventions decide the answers and none of them is obvious.
 * **The ray angle.** Ray `i` of `num_rays` leaves a pose at
 * ```
 * t_ray = i * 2*M_PI / num_rays + pose.t - M_PI
 * t_ray = fmod(t_ray + 5*M_PI, 2*M_PI) - M_PI
 * ```
 * read out of `FSM::X::findExactAngular` in `fsm_core.hpp`, which is the
 * definition of record, and carried in the comment above
 * `FSM::GridReference::ranges` beside the interface this type satisfies. Ray
 * zero therefore points *behind* the pose and not along it. That is the part a
 * reader will helpfully correct, and it is why `bearingOfRay` below is written
 * as those same two lines rather than as a sentence about them. The formula now
 * exists on both sides of a module boundary, so `ASpikeBehindThePoseLandsOnRayZero`
 * and the two cases beside it are what stop the two copies drifting apart: they
 * assert a ray *index* as an integer, with no tolerance to argue about, so that
 * an off by one, a mirrored sign and a half turn each fail rather than quietly
 * rotating every grid match.
 * **The cell layout.** A cell is addressed by column and row, `column` bounded
 * by the width and `row` by the height, and the message's `data` is row major:
 * `data[row * width + column]`. `gridMessage` below is the only place in this
 * file that computes that index. Every fixture is non square, because a square
 * grid cannot catch a transposition, and `ABlockedCellWhoseTransposedCellIsOutsideTheGrid`
 * puts a blocked cell where a transposed read would be out of bounds
 * altogether.
 * **What a range is measured to.** A caster stops at a cell, so the finest
 * answer a grid can give is a cell. Where a case has an analytic answer it is
 * measured from the sensor's position to the *centre* of the first cell the ray
 * enters, which for a sensor standing at a cell centre is a whole number of
 * cells and can be checked by counting. The wall's near face is half a cell
 * nearer than that, which is inside the same one cell bound.
 * **The ray heading CDDT is given.** CDDT rounds a cast heading to one of 720
 * bins, so it carries an error of `range * M_PI / 720` on top of the cell. 720
 * is a multiple of four, so a ray along an axis lands exactly on a bin and that
 * term is zero; the diagonal cases where it is not are in
 * `test_ray_casting.cpp`, which is also where the three methods are held
 * against each other.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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
const std::int8_t kUnknown = -1;

const int kOccupancyThreshold = 50;

const unsigned int kWidth = 61U;
const unsigned int kHeight = 91U;
const double kResolution = 0.05;

const unsigned int kSensorColumn = 30U;
const unsigned int kSensorRow = 45U;

const double kOriginX = -3.2;
const double kOriginY = 7.4;
const double kOriginYaw = 0.7;

const double kMaxRange = 10.0;

const unsigned int kRayCount = 23U;

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

double bearingOfRay(
  const unsigned int index, const unsigned int num_rays, const double pose_t)
{
  double t_ray = index * 2*M_PI / num_rays + pose_t - M_PI;
  t_ray = fmod(t_ray + 5*M_PI, 2*M_PI) - M_PI;
  return t_ray;
}

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

std::vector<CellValue> blockedSpikeAt(const unsigned int column, const unsigned int row)
{
  return blockedRectangle(column - 1U, row - 1U, column + 1U, row + 1U);
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

unsigned int rayIndexOfBearing(
  const double bearing, const double pose_t, const unsigned int num_rays)
{
  const double rays = (bearing - pose_t + M_PI) * num_rays / (2*M_PI);
  const long count = static_cast<long>(num_rays);
  const long index = ((std::lround(rays) % count) + count) % count;

  return static_cast<unsigned int>(index);
}

std::vector<unsigned int> shortRays(
  const std::vector<double>& ranges, const double max_range)
{
  std::vector<unsigned int> indices;
  for (unsigned int i = 0; i < ranges.size(); ++i)
    if (ranges[i] < max_range)
      indices.push_back(i);

  return indices;
}

double roundTripToleranceInCells(const WorldPoint& point, const double resolution)
{
  const double magnitude = std::abs(point.x) + std::abs(point.y);
  return 16.0 * std::numeric_limits<double>::epsilon() * magnitude / resolution;
}

class OccupancyGridMapBadResolution : public testing::TestWithParam<double>
{
};

class OccupancyGridMapCasting : public testing::TestWithParam<CastingMethod>
{
};

struct ThresholdCase
{
  std::int8_t value{kFree};
  bool free{true};
};

class OccupancyGridMapThreshold : public testing::TestWithParam<ThresholdCase>
{
};

}

TEST(OccupancyGridMapBuild, AGoodMessageBuildsAMapThatReportsTheGeometryItWasGiven)
{
  const nav_msgs::msg::OccupancyGrid message = gridMessage(
    GridGeometry{
      .width = kWidth, .height = kHeight, .resolution = kResolution,
      .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
    {});

  const auto built = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);

  ASSERT_TRUE(built.has_value())
    << "a message with a positive width and height, a data buffer of exactly "
    << "width times height cells, a finite positive resolution and a "
    << "normalisable orientation is well formed and must build. It was "
    << "refused with: " << built.error();

  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  ASSERT_NE(map, nullptr) << "a successful build cannot hand back a null pointer.";

  EXPECT_EQ(map->width(), kWidth)
    << "the map must report the width the message gave it, so that a caller "
    << "sizing anything against it agrees with the message.";

  EXPECT_EQ(map->height(), kHeight)
    << "the map must report the height the message gave it. A width and a "
    << "height reported the other way round is the transposition this whole "
    << "file is built to catch.";

  EXPECT_DOUBLE_EQ(map->resolution(), static_cast<double>(static_cast<float>(kResolution)))
    << "the map must report the resolution the message gave it, widened from "
    << "the message's float and not otherwise altered: every tolerance in "
    << "every test of this package is read from this number, so a map that "
    << "reports a resolution it is not using would make all of them "
    << "meaningless.";
}

TEST(OccupancyGridMapBuild, DataOneCellLongIsRefused)
{
  nav_msgs::msg::OccupancyGrid message = gridMessage(GridGeometry{}, {});
  message.data.push_back(kFree);

  const auto built = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);

  ASSERT_FALSE(built.has_value())
    << "a data buffer of " << message.data.size() << " cells cannot describe a "
    << kWidth << " by " << kHeight << " grid, which needs "
    << static_cast<std::size_t>(kWidth) * kHeight << ". It must be refused "
    << "rather than read as far as it goes.";

  EXPECT_NE(built.error().find(std::to_string(message.data.size())), std::string::npos)
    << "the refusal must name the length it was given, " << message.data.size()
    << ", so an operator reading the log knows what to correct. It said: "
    << built.error();
}

TEST(OccupancyGridMapBuild, DataOneCellShortIsRefused)
{
  nav_msgs::msg::OccupancyGrid message = gridMessage(GridGeometry{}, {});
  message.data.pop_back();

  const auto built = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);

  ASSERT_FALSE(built.has_value())
    << "a data buffer of " << message.data.size() << " cells is one short of "
    << "the " << static_cast<std::size_t>(kWidth) * kHeight << " a " << kWidth
    << " by " << kHeight << " grid needs. Building from it would read past the "
    << "end of the buffer for the last cell.";

  EXPECT_NE(
    built.error().find(std::to_string(static_cast<std::size_t>(kWidth) * kHeight)),
    std::string::npos)
    << "the refusal must name the length the geometry requires. It said: "
    << built.error();
}

TEST(OccupancyGridMapBuild, HeightZeroIsRefused)
{
  nav_msgs::msg::OccupancyGrid message = gridMessage(GridGeometry{}, {});
  message.info.height = 0U;
  message.data.clear();

  const auto built = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);

  ASSERT_FALSE(built.has_value())
    << "a grid of zero height holds no cell, so there is nothing for a ray to "
    << "meet and nothing for a pose to stand in. It must be refused rather "
    << "than built into an object every query of which is meaningless.";

  EXPECT_NE(built.error().find("height"), std::string::npos)
    << "the refusal must name the field that was wrong, so that a zero height "
    << "is not reported as a zero width. It said: " << built.error();
}

TEST(OccupancyGridMapBuild, OriginTranslationThatIsNotFiniteIsRefused)
{
  for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()})
  {
    nav_msgs::msg::OccupancyGrid message = gridMessage(GridGeometry{}, {});
    message.info.origin.position.x = bad;

    const auto built = OccupancyGridMap::build(
      message, kOccupancyThreshold, CastingMethod::ray_marching);

    EXPECT_FALSE(built.has_value())
      << "an origin translation of " << bad << " makes every world to cell "
      << "conversion NaN, and a NaN cell is reported as outside the grid "
      << "rather than as an error, so every ray would report the maximum "
      << "range and nothing would say why. It must be refused at the "
      << "boundary. It was accepted instead.";
  }
}

TEST(OccupancyGridMapBuild, WidthZeroIsRefused)
{
  nav_msgs::msg::OccupancyGrid message = gridMessage(GridGeometry{}, {});
  message.info.width = 0U;
  message.data.clear();

  const auto built = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);

  ASSERT_FALSE(built.has_value())
    << "a grid of zero width holds no cell. It must be refused rather than "
    << "built, since `row * width + column` is zero for every cell of it and "
    << "every read would be out of bounds.";

  EXPECT_NE(built.error().find("width"), std::string::npos)
    << "the refusal must name the field that was wrong. It said: " << built.error();
}

TEST(OccupancyGridMapBuild, ZeroQuaternionIsRefused)
{
  nav_msgs::msg::OccupancyGrid message = gridMessage(GridGeometry{}, {});
  message.info.origin.orientation.x = 0.0;
  message.info.origin.orientation.y = 0.0;
  message.info.origin.orientation.z = 0.0;
  message.info.origin.orientation.w = 0.0;

  const auto built = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);

  ASSERT_FALSE(built.has_value())
    << "an orientation of all zeros has norm zero and no yaw to extract. "
    << "Normalising it divides by zero, so it must be refused rather than "
    << "turned into a rotation nobody chose. This is the orientation a "
    << "default constructed message carries, so it is the one a publisher "
    << "that never set the origin will send.";

  EXPECT_NE(built.error().find("orientation"), std::string::npos)
    << "the refusal must name the orientation rather than the translation "
    << "beside it. It said: " << built.error();
}

TEST_P(OccupancyGridMapBadResolution, IsRefused)
{
  nav_msgs::msg::OccupancyGrid message = gridMessage(GridGeometry{}, {});
  message.info.resolution = static_cast<float>(GetParam());

  const auto built = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);

  ASSERT_FALSE(built.has_value())
    << "a resolution of " << GetParam() << " metres a cell is not a length. "
    << "It divides every world to cell conversion and multiplies every range, "
    << "so it must be refused at the boundary rather than propagated into "
    << "answers that look like distances. It was accepted instead.";

  EXPECT_NE(built.error().find("resolution"), std::string::npos)
    << "the refusal must name the resolution. It said: " << built.error();
}

INSTANTIATE_TEST_SUITE_P(
  ZeroNegativeInfiniteAndNotANumber, OccupancyGridMapBadResolution,
  testing::Values(
    0.0, -kResolution,
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()));

TEST_P(OccupancyGridMapCasting, AnAxisAlignedWallGivesTheCellGapTimesTheResolution)
{
  const unsigned int left_wall = 15U;
  const unsigned int right_wall = 55U;
  const unsigned int bottom_wall = 25U;
  const unsigned int top_wall = 75U;

  std::vector<CellValue> cells;
  for (const std::vector<CellValue>& wall : {
         blockedRectangle(left_wall, 0U, left_wall, kHeight - 1U),
         blockedRectangle(right_wall, 0U, right_wall, kHeight - 1U),
         blockedRectangle(0U, bottom_wall, kWidth - 1U, bottom_wall),
         blockedRectangle(0U, top_wall, kWidth - 1U, top_wall)})
    cells.insert(cells.end(), wall.begin(), wall.end());

  const auto built = OccupancyGridMap::build(
    gridMessage(GridGeometry{}, cells), kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(CellCoordinates{
    .column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  const unsigned int num_rays = 8U;
  const std::vector<double> ranges =
    map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = M_PI}, num_rays, kMaxRange);

  ASSERT_EQ(ranges.size(), num_rays);

  const double expected[4] = {
    (right_wall - kSensorColumn) * map->resolution(),
    (top_wall - kSensorRow) * map->resolution(),
    (kSensorColumn - left_wall) * map->resolution(),
    (kSensorRow - bottom_wall) * map->resolution()};

  const unsigned int ray_of[4] = {0U, 2U, 4U, 6U};
  const char* direction[4] = {"positive x", "positive y", "negative x", "negative y"};

  for (unsigned int k = 0; k < 4; ++k)
  {
    EXPECT_NEAR(ranges[ray_of[k]], expected[k], map->resolution())
      << "ray " << ray_of[k] << " leaves along " << direction[k]
      << " and meets the wall " << expected[k] / map->resolution()
      << " cells away, so it must report " << expected[k] << " m to within one "
      << "cell of " << map->resolution() << " m. It reported "
      << ranges[ray_of[k]] << " m, which is "
      << (ranges[ray_of[k]] - expected[k]) / map->resolution() << " cells out. "
      << "A range near one of the other three gaps, "
      << expected[0] / map->resolution() << ", " << expected[1] / map->resolution()
      << ", " << expected[2] / map->resolution() << " or "
      << expected[3] / map->resolution() << " cells, means the rays are being "
      << "cast in the wrong direction rather than the ranges being inaccurate.";
  }
}

TEST_P(OccupancyGridMapCasting, AMaximumRangeBeyondTheGridDiagonalIsReportedRatherThanTheDiagonal)
{
  const auto built = OccupancyGridMap::build(
    gridMessage(GridGeometry{}, {}), kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const double diagonal = std::hypot(
    static_cast<double>(map->width()), static_cast<double>(map->height())) *
    map->resolution();

  const double max_range = 2.0 * diagonal;

  const WorldPoint sensor = map->worldFromCell(CellCoordinates{
    .column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  const std::vector<double> ranges = map->ranges(
    FSM::Pose{.x = sensor.x, .y = sensor.y, .t = 0.37}, kRayCount, max_range);

  ASSERT_EQ(ranges.size(), kRayCount);

  for (unsigned int i = 0; i < kRayCount; ++i)
    EXPECT_EQ(ranges[i], max_range)
      << "ray " << i << " crosses an entirely free grid and meets nothing, so "
      << "it must report the maximum range it was asked for, " << max_range
      << " m, exactly. It reported " << ranges[i] << " m. A report of "
      << diagonal << " m is the grid's own diagonal, which is the maximum the "
      << "caster was built with, and means a ray that met nothing is being "
      << "read as a ray that met the far corner.";
}

TEST_P(OccupancyGridMapCasting, AMaximumRangeShorterThanTheTrueRangeIsReported)
{
  const unsigned int wall_column = kSensorColumn + 20U;

  const auto built = OccupancyGridMap::build(
    gridMessage(GridGeometry{}, blockedRectangle(wall_column, 0U, wall_column, kHeight - 1U)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const double true_range = (wall_column - kSensorColumn) * map->resolution();
  const double max_range = true_range / 2.0;

  const WorldPoint sensor = map->worldFromCell(CellCoordinates{
    .column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  const std::vector<double> ranges =
    map->ranges(FSM::Pose{.x = sensor.x, .y = sensor.y, .t = M_PI}, 8U, max_range);

  ASSERT_EQ(ranges.size(), std::size_t{8});

  EXPECT_EQ(ranges[0], max_range)
    << "ray zero leaves along positive x and meets a wall " << true_range
    << " m away, which is beyond the " << max_range << " m the scan can "
    << "report. A range the sensor could not have measured must be reported as "
    << "the maximum exactly, not as the range the grid happens to know about. "
    << "It reported " << ranges[0] << " m.";
}

TEST_P(OccupancyGridMapCasting, APoseOutsideTheGridReturnsTheMaximumRangeForEveryRay)
{
  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = kWidth, .height = kHeight, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      blockedSpikeAt(kSensorColumn, kSensorRow)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const double outside = 7.5;

  const CellCoordinates poses[5] = {
    {.column = -outside, .row = kSensorRow + 0.5},
    {.column = kWidth + outside, .row = kSensorRow + 0.5},
    {.column = kSensorColumn + 0.5, .row = -outside},
    {.column = kSensorColumn + 0.5, .row = kHeight + outside},
    {.column = -outside, .row = kHeight + outside}};

  for (const CellCoordinates& cell : poses)
  {
    ASSERT_FALSE(map->isInside(cell))
      << "the fixture must place this pose outside a " << map->width() << " by "
      << map->height() << " grid for the case to test anything; cell ("
      << cell.column << ", " << cell.row << ") is inside it.";

    const WorldPoint pose = map->worldFromCell(cell);
    const std::vector<double> ranges = map->ranges(
      FSM::Pose{.x = pose.x, .y = pose.y, .t = 0.37}, kRayCount, kMaxRange);

    ASSERT_EQ(ranges.size(), kRayCount);

    for (unsigned int i = 0; i < kRayCount; ++i)
      EXPECT_EQ(ranges[i], kMaxRange)
        << "the pose stands at cell (" << cell.column << ", " << cell.row
        << "), outside the grid, so there is no cell for a ray to leave from "
        << "and every ray must report the maximum range, " << kMaxRange
        << " m, exactly. Ray " << i << " reported " << ranges[i] << " m, so "
        << "the caster was reached with a pose it has no bounds test for.";
  }
}

TEST_P(OccupancyGridMapCasting, APoseWithANonFiniteHeadingReturnsTheMaximumRangeForEveryRay)
{
  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = kWidth, .height = kHeight, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      blockedSpikeAt(kSensorColumn, kSensorRow)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(
    CellCoordinates{.column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  const std::vector<double> ranges = map->ranges(
    FSM::Pose{.x = sensor.x, .y = sensor.y,
      .t = std::numeric_limits<double>::quiet_NaN()},
    kRayCount, kMaxRange);

  ASSERT_EQ(ranges.size(), kRayCount);

  for (unsigned int i = 0; i < kRayCount; ++i)
    EXPECT_EQ(ranges[i], kMaxRange)
      << "the pose's heading is not a number, so no ray has a direction to "
      << "leave from and every ray must report the maximum range, "
      << kMaxRange << " m, exactly. Ray " << i << " reported " << ranges[i]
      << " m, so the caster was reached with a heading it has no bounds "
      << "test for.";
}

TEST_P(OccupancyGridMapCasting, RangesAreInvariantUnderReplacingTheMap)
{
  const unsigned int left_wall = 15U;
  const unsigned int right_wall = 55U;
  const unsigned int bottom_wall = 25U;
  const unsigned int top_wall = 75U;

  std::vector<CellValue> cells;
  for (const std::vector<CellValue>& part : {
         blockedRectangle(left_wall, 0U, left_wall, kHeight - 1U),
         blockedRectangle(right_wall, 0U, right_wall, kHeight - 1U),
         blockedRectangle(0U, bottom_wall, kWidth - 1U, bottom_wall),
         blockedRectangle(0U, top_wall, kWidth - 1U, top_wall),

         blockedSpikeAt(kSensorColumn + 12U, kSensorRow + 19U)})
    cells.insert(cells.end(), part.begin(), part.end());

  const auto at_origin = OccupancyGridMap::build(
    gridMessage(GridGeometry{}, cells), kOccupancyThreshold, GetParam());
  ASSERT_TRUE(at_origin.has_value()) << at_origin.error();

  const auto moved = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = kWidth, .height = kHeight, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      cells),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(moved.has_value()) << moved.error();

  const WorldPoint sensor = at_origin.value()->worldFromCell(
    CellCoordinates{.column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});
  const double heading = 0.37;

  const FSM::Pose here{.x = sensor.x, .y = sensor.y, .t = heading};
  const FSM::Pose there{
    .x = kOriginX + std::cos(kOriginYaw) * sensor.x - std::sin(kOriginYaw) * sensor.y,
    .y = kOriginY + std::sin(kOriginYaw) * sensor.x + std::cos(kOriginYaw) * sensor.y,
    .t = heading + kOriginYaw};

  const std::vector<double> before =
    at_origin.value()->ranges(here, kRayCount, kMaxRange);
  const std::vector<double> after =
    moved.value()->ranges(there, kRayCount, kMaxRange);

  ASSERT_EQ(before.size(), after.size());

  for (unsigned int i = 0; i < kRayCount; ++i)
    EXPECT_NEAR(after[i], before[i], at_origin.value()->resolution())
      << "ray " << i << " looks at the same wall of the same room from the "
      << "same place, so the two maps must answer within a cell of each other. "
      << "The map at the origin said " << before[i] << " m and the map "
      << "translated by (" << kOriginX << ", " << kOriginY << ") and rotated "
      << "by " << kOriginYaw << " rad said " << after[i] << " m, a difference "
      << "of " << (after[i] - before[i]) / at_origin.value()->resolution()
      << " cells. A disagreement across the whole fan is a sign error in the "
      << "rotation; one in a single ray is the heading being turned into the "
      << "grid's frame the wrong way.";
}

TEST_P(OccupancyGridMapCasting, RangesAreIdenticalWhenTheMapMovesByAWholeNumberOfCells)
{
  const double resolution = 0.125;
  const double shift_x = 32.0 * resolution;
  const double shift_y = -64.0 * resolution;

  std::vector<CellValue> cells;
  for (const std::vector<CellValue>& part : {
         blockedRectangle(15U, 0U, 15U, kHeight - 1U),
         blockedRectangle(55U, 0U, 55U, kHeight - 1U),
         blockedRectangle(0U, 25U, kWidth - 1U, 25U),
         blockedRectangle(0U, 75U, kWidth - 1U, 75U),
         blockedSpikeAt(kSensorColumn + 12U, kSensorRow + 19U)})
    cells.insert(cells.end(), part.begin(), part.end());

  const GridGeometry here{
    .width = kWidth, .height = kHeight, .resolution = resolution,
    .origin_x = 0.0, .origin_y = 0.0, .origin_yaw = 0.0};
  const GridGeometry there{
    .width = kWidth, .height = kHeight, .resolution = resolution,
    .origin_x = shift_x, .origin_y = shift_y, .origin_yaw = 0.0};

  const auto first = OccupancyGridMap::build(
    gridMessage(here, cells), kOccupancyThreshold, GetParam());
  ASSERT_TRUE(first.has_value()) << first.error();

  const auto second = OccupancyGridMap::build(
    gridMessage(there, cells), kOccupancyThreshold, GetParam());
  ASSERT_TRUE(second.has_value()) << second.error();

  const WorldPoint sensor = first.value()->worldFromCell(
    CellCoordinates{.column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  const std::vector<double> before = first.value()->ranges(
    FSM::Pose{.x = sensor.x, .y = sensor.y, .t = 0.37}, kRayCount, kMaxRange);
  const std::vector<double> after = second.value()->ranges(
    FSM::Pose{.x = sensor.x + shift_x, .y = sensor.y + shift_y, .t = 0.37},
    kRayCount, kMaxRange);

  for (unsigned int i = 0; i < kRayCount; ++i)
    EXPECT_EQ(after[i], before[i])
      << "ray " << i << ": at " << resolution << " m a cell and a translation "
      << "of a whole number of cells, every step of the world to cell "
      << "conversion is exact in binary, so the two maps hand the caster the "
      << "same cell coordinates and must return the same range bit for bit. "
      << "They returned " << before[i] << " m and " << after[i] << " m.";
}

TEST_P(OccupancyGridMapCasting, ASingleSpikeLandsOnTheIndexTheConventionNames)
{
  const double heading = 0.37;
  const unsigned int aimed_at = 3U;
  const double reach = 20.0;

  const double bearing = bearingOfRay(aimed_at, kRayCount, heading);
  const unsigned int spike_column = static_cast<unsigned int>(
    std::lround(kSensorColumn + 0.5 + reach * std::cos(bearing)));
  const unsigned int spike_row = static_cast<unsigned int>(
    std::lround(kSensorRow + 0.5 + reach * std::sin(bearing)));

  const auto built = OccupancyGridMap::build(
    gridMessage(GridGeometry{}, blockedSpikeAt(spike_column, spike_row)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(CellCoordinates{
    .column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});
  const WorldPoint spike = map->worldFromCell(CellCoordinates{
    .column = spike_column + 0.5, .row = spike_row + 0.5});

  const unsigned int expected = rayIndexOfBearing(
    std::atan2(spike.y - sensor.y, spike.x - sensor.x), heading, kRayCount);

  const std::vector<double> ranges = map->ranges(
    FSM::Pose{.x = sensor.x, .y = sensor.y, .t = heading}, kRayCount, kMaxRange);

  const std::vector<unsigned int> met = shortRays(ranges, kMaxRange);

  ASSERT_EQ(met.size(), std::size_t{1})
    << "one spike stands in an otherwise free grid, " << reach << " cells from "
    << "the sensor, and at " << kRayCount << " rays the fan is "
    << 2*M_PI / kRayCount * reach << " cells wide there, wider than the spike. "
    << "So exactly one ray can meet it and " << met.size() << " did.";

  EXPECT_EQ(met[0], expected)
    << "the spike stands at cell (" << spike_column << ", " << spike_row
    << "), on a bearing of "
    << std::atan2(spike.y - sensor.y, spike.x - sensor.x) << " rad from a pose "
    << "of heading " << heading << " rad. Ray i leaves at i * 2*M_PI / "
    << kRayCount << " + " << heading << " - M_PI, so the ray that meets it is "
    << "ray " << expected << ". Ray " << met[0] << " met it instead. A "
    << "difference of one is an off by one in the ray angle; a difference that "
    << "reflects " << expected << " about ray zero, to "
    << (kRayCount - expected) % kRayCount << ", is a mirrored sign; and a "
    << "difference of about " << kRayCount / 2 << " is a half turn, which is "
    << "ray zero being pointed along the pose instead of behind it.";
}

TEST_P(OccupancyGridMapCasting, ASpikeBehindThePoseLandsOnRayZero)
{
  const double heading = 0.37;
  const double reach = 20.0;

  const double bearing = heading + M_PI;
  const unsigned int spike_column = static_cast<unsigned int>(
    std::lround(kSensorColumn + 0.5 + reach * std::cos(bearing)));
  const unsigned int spike_row = static_cast<unsigned int>(
    std::lround(kSensorRow + 0.5 + reach * std::sin(bearing)));

  const auto built = OccupancyGridMap::build(
    gridMessage(GridGeometry{}, blockedSpikeAt(spike_column, spike_row)),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(CellCoordinates{
    .column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  const std::vector<double> ranges = map->ranges(
    FSM::Pose{.x = sensor.x, .y = sensor.y, .t = heading}, kRayCount, kMaxRange);

  const std::vector<unsigned int> met = shortRays(ranges, kMaxRange);

  ASSERT_EQ(met.size(), std::size_t{1})
    << "one spike, placed where only one ray can reach it. " << met.size()
    << " rays met something.";

  EXPECT_EQ(met[0], 0U)
    << "the spike stands directly behind the pose, on bearing " << bearing
    << " rad from a heading of " << heading << " rad. Ray zero leaves at "
    << "0 * 2*M_PI / " << kRayCount << " + " << heading << " - M_PI, which is "
    << "that bearing, so ray zero and no other must meet it. Ray " << met[0]
    << " met it. Ray " << kRayCount / 2 << " or " << (kRayCount + 1) / 2
    << " meeting it means ray zero has been pointed along the pose instead of "
    << "behind it, which is the correction a reader of the formula will want "
    << "to make and which `FSM::X::findExactAngular` does not make.";
}

TEST_P(OccupancyGridMapCasting, TwoSpikesLandOnTheIndicesTheConventionNames)
{
  const double heading_in_grid = 0.37;
  const double heading = heading_in_grid + kOriginYaw;

  const unsigned int aimed_at[2] = {3U, 11U};
  const double reach[2] = {12.0, 24.0};

  std::vector<CellValue> cells;
  unsigned int spike_column[2] = {0U, 0U};
  unsigned int spike_row[2] = {0U, 0U};

  for (unsigned int s = 0; s < 2; ++s)
  {
    const double bearing = bearingOfRay(aimed_at[s], kRayCount, heading_in_grid);
    spike_column[s] = static_cast<unsigned int>(
      std::lround(kSensorColumn + 0.5 + reach[s] * std::cos(bearing)));
    spike_row[s] = static_cast<unsigned int>(
      std::lround(kSensorRow + 0.5 + reach[s] * std::sin(bearing)));

    const std::vector<CellValue> spike = blockedSpikeAt(spike_column[s], spike_row[s]);
    cells.insert(cells.end(), spike.begin(), spike.end());
  }

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = kWidth, .height = kHeight, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      cells),
    kOccupancyThreshold, GetParam());
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint sensor = map->worldFromCell(CellCoordinates{
    .column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  const std::vector<double> ranges = map->ranges(
    FSM::Pose{.x = sensor.x, .y = sensor.y, .t = heading}, kRayCount, kMaxRange);

  const std::vector<unsigned int> met = shortRays(ranges, kMaxRange);

  ASSERT_EQ(met.size(), std::size_t{2})
    << "two spikes stand in an otherwise free grid, at " << reach[0] << " and "
    << reach[1] << " cells, far enough apart in bearing that one ray each can "
    << "reach them. " << met.size() << " rays met something.";

  for (unsigned int s = 0; s < 2; ++s)
  {
    const WorldPoint spike = map->worldFromCell(CellCoordinates{
      .column = spike_column[s] + 0.5, .row = spike_row[s] + 0.5});
    const unsigned int expected = rayIndexOfBearing(
      std::atan2(spike.y - sensor.y, spike.x - sensor.x), heading, kRayCount);

    EXPECT_EQ(met[s], expected)
      << "the spike at cell (" << spike_column[s] << ", " << spike_row[s]
      << ") stands " << reach[s] << " cells out on the bearing ray " << expected
      << " of " << kRayCount << " leaves along, from a pose of heading "
      << heading << " rad on a map rotated by " << kOriginYaw << " rad. Ray "
      << met[s] << " met it. The two spikes must be met by rays {"
      << aimed_at[0] << ", " << aimed_at[1] << "}: an off by one would give {"
      << aimed_at[0] - 1U << ", " << aimed_at[1] - 1U << "}, a mirrored sign {"
      << (kRayCount - aimed_at[1]) % kRayCount << ", "
      << (kRayCount - aimed_at[0]) % kRayCount << "}, and a half turn {"
      << (aimed_at[0] + kRayCount / 2) % kRayCount << ", "
      << (aimed_at[1] + kRayCount / 2) % kRayCount << "}. A failure here with "
      << "the unrotated cases passing means the ray heading is being turned "
      << "into the grid's frame with the wrong sign.";

    const double centre = reach[s] * map->resolution();
    EXPECT_NEAR(ranges[met[s]], centre, 2.5 * map->resolution())
      << "the spike at cell (" << spike_column[s] << ", " << spike_row[s]
      << ") has its centre " << reach[s] << " cells from the sensor, so the "
      << "range must be " << centre << " m to within the spike's own half "
      << "width of one and a half cells plus a cell of the caster's. It was "
      << ranges[met[s]] << " m. The two spikes stand at " << reach[0]
      << " and " << reach[1] << " cells, so a range near "
      << reach[1 - s] * map->resolution() << " m means the two indices have "
      << "been swapped rather than each being wrong.";
  }
}

INSTANTIATE_TEST_SUITE_P(
  AllThreeMethods, OccupancyGridMapCasting,
  testing::Values(
    CastingMethod::bresenham, CastingMethod::cddt, CastingMethod::ray_marching),
  methodSuffix);

TEST(OccupancyGridMapConversion, AMapWithNoYawReducesToADivisionByTheResolution)
{
  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = 20U, .height = 40U, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = 0.0},
      {}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const double points[3][2] = {
    {kOriginX + 0.35, kOriginY + 0.85},
    {kOriginX - 1.10, kOriginY + 0.05},
    {kOriginX, kOriginY - 2.20}};

  for (const auto& point : points)
  {
    const CellCoordinates cell = map->cellFromWorld(point[0], point[1]);

    const double expected_column = (point[0] - kOriginX) / map->resolution();
    const double expected_row = (point[1] - kOriginY) / map->resolution();

    const double tolerance = 16.0 * std::numeric_limits<double>::epsilon() *
      (std::abs(point[0]) + std::abs(point[1])) / map->resolution();

    EXPECT_NEAR(cell.column, expected_column, tolerance)
      << "with no yaw the column of a world point is (x - origin x) divided by "
      << "the resolution, which for x = " << point[0] << " is "
      << expected_column << ". It was " << cell.column << ". A column of "
      << expected_row << " is the row, which means the two axes have been "
      << "swapped.";

    EXPECT_NEAR(cell.row, expected_row, tolerance)
      << "with no yaw the row of a world point is (y - origin y) divided by "
      << "the resolution, which for y = " << point[1] << " is " << expected_row
      << ". It was " << cell.row << ".";
  }
}

TEST(OccupancyGridMapConversion, TheFarCornerIsTheWidthByTheHeight)
{
  const unsigned int width = 20U;
  const unsigned int height = 40U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = width, .height = height, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      {}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint corner = map->worldFromCell(
    CellCoordinates{.column = width, .row = height});

  const double expected_x = kOriginX + map->resolution() *
    (std::cos(kOriginYaw) * width - std::sin(kOriginYaw) * height);
  const double expected_y = kOriginY + map->resolution() *
    (std::sin(kOriginYaw) * width + std::cos(kOriginYaw) * height);

  const double tolerance = 16.0 * std::numeric_limits<double>::epsilon() *
    (std::abs(expected_x) + std::abs(expected_y));

  EXPECT_NEAR(corner.x, expected_x, tolerance)
    << "cell (" << width << ", " << height << ") stands at the origin plus the "
    << "grid's diagonal rotated by the map's yaw of " << kOriginYaw << " rad, "
    << "which puts its x at " << expected_x << ". It was at " << corner.x
    << ". An x of "
    << kOriginX + map->resolution() *
       (std::cos(kOriginYaw) * width + std::sin(kOriginYaw) * height)
    << " is the rotation applied the other way round.";

  EXPECT_NEAR(corner.y, expected_y, tolerance)
    << "cell (" << width << ", " << height << ") has its y at " << expected_y
    << ". It was at " << corner.y << ".";

  const CellCoordinates back = map->cellFromWorld(corner.x, corner.y);

  EXPECT_NEAR(
    back.column, static_cast<double>(width),
    roundTripToleranceInCells(corner, map->resolution()))
    << "and coming back must give the width, " << width << ", not "
    << back.column << ".";
  EXPECT_NEAR(
    back.row, static_cast<double>(height),
    roundTripToleranceInCells(corner, map->resolution()))
    << "and coming back must give the height, " << height << ", not "
    << back.row << ".";
}

TEST(OccupancyGridMapConversion, TheOriginIsCellZeroZero)
{
  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = 20U, .height = 40U, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      {}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const CellCoordinates cell = map->cellFromWorld(kOriginX, kOriginY);

  EXPECT_EQ(cell.column, 0.0)
    << "the map's origin is the world position of cell zero zero, so the "
    << "conversion subtracts the origin from itself and divides zero by the "
    << "resolution. The column must be exactly zero and was " << cell.column
    << ", which means the origin is not being subtracted, or is being "
    << "subtracted after the rotation rather than before it.";

  EXPECT_EQ(cell.row, 0.0)
    << "the row of the origin must be exactly zero and was " << cell.row << ".";

  const WorldPoint back = map->worldFromCell(CellCoordinates{.column = 0.0, .row = 0.0});

  EXPECT_EQ(back.x, kOriginX)
    << "and cell zero zero must stand back at the origin's x, " << kOriginX
    << ", exactly. It was at " << back.x << ".";
  EXPECT_EQ(back.y, kOriginY)
    << "and cell zero zero must stand back at the origin's y, " << kOriginY
    << ", exactly. It was at " << back.y << ".";
}

TEST(OccupancyGridMapConversion, WorldAndCellRoundTripAtTheCornersTheCentreAndOutside)
{
  const unsigned int width = 20U;
  const unsigned int height = 40U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = width, .height = height, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      {}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const CellCoordinates cells[6] = {
    {.column = 0.0, .row = 0.0},
    {.column = width, .row = 0.0},
    {.column = 0.0, .row = height},
    {.column = width, .row = height},
    {.column = width / 2.0, .row = height / 2.0},
    {.column = -1.5 * width, .row = 2.25 * height}};

  for (const CellCoordinates& cell : cells)
  {
    const WorldPoint point = map->worldFromCell(cell);
    const CellCoordinates back = map->cellFromWorld(point.x, point.y);

    const double tolerance = roundTripToleranceInCells(point, map->resolution());

    EXPECT_NEAR(back.column, cell.column, tolerance)
      << "cell (" << cell.column << ", " << cell.row << ") stands at world ("
      << point.x << ", " << point.y << "), and converting that back must give "
      << "the cell it came from. The column came back as " << back.column
      << ". A column of " << cell.row << " means the two conversions disagree "
      << "about which axis is which; a column of " << -cell.column
      << " means one of them rotates the wrong way.";

    EXPECT_NEAR(back.row, cell.row, tolerance)
      << "cell (" << cell.column << ", " << cell.row << ") came back with row "
      << back.row << ".";
  }
}

TEST(OccupancyGridMapFreeSpace, ABlockedCellAndAnUnknownCellAreBothNotFree)
{
  const unsigned int blocked_column = 3U;
  const unsigned int blocked_row = 7U;
  const unsigned int unknown_column = 11U;
  const unsigned int unknown_row = 23U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = 20U, .height = 40U, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      {CellValue{.column = blocked_column, .row = blocked_row, .value = kBlocked},
       CellValue{.column = unknown_column, .row = unknown_row, .value = kUnknown}}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint blocked = map->worldFromCell(
    CellCoordinates{.column = blocked_column + 0.5, .row = blocked_row + 0.5});
  const WorldPoint unknown = map->worldFromCell(
    CellCoordinates{.column = unknown_column + 0.5, .row = unknown_row + 0.5});

  EXPECT_FALSE(map->isFree(blocked.x, blocked.y))
    << "cell (" << blocked_column << ", " << blocked_row << ") holds "
    << static_cast<int>(kBlocked) << ", which is above the threshold of "
    << kOccupancyThreshold << ", so it is occupied and the world point at its "
    << "centre is not free.";

  EXPECT_FALSE(map->isFree(unknown.x, unknown.y))
    << "cell (" << unknown_column << ", " << unknown_row << ") holds "
    << static_cast<int>(kUnknown) << ", which is unknown. An unmapped cell "
    << "blocks: a pose standing in one has to be refused rather than matched "
    << "against a room nobody has seen.";
}

TEST(OccupancyGridMapFreeSpace, AFreeCellIsFree)
{
  const unsigned int column = 5U;
  const unsigned int row = 9U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = 20U, .height = 40U, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      {CellValue{.column = 3U, .row = 7U, .value = kBlocked}}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint point = map->worldFromCell(
    CellCoordinates{.column = column + 0.5, .row = row + 0.5});

  EXPECT_TRUE(map->isFree(point.x, point.y))
    << "cell (" << column << ", " << row << ") holds " << static_cast<int>(kFree)
    << ", which is below the threshold of " << kOccupancyThreshold << " and is "
    << "not negative, so it is free. It is also inside a 20 by 40 grid, which "
    << "is the other half of what free means here.";
}

TEST(OccupancyGridMapFreeSpace, OutsideTheGridIsNotFreeOnAnySideOrDiagonal)
{
  const unsigned int width = 20U;
  const unsigned int height = 40U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = width, .height = height, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      {}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const double inside_column = width / 2.0;
  const double inside_row = height / 2.0;
  const double before = -0.5;
  const double beyond_column = width + 0.5;
  const double beyond_row = height + 0.5;

  const CellCoordinates outside[8] = {
    {.column = before, .row = inside_row},
    {.column = beyond_column, .row = inside_row},
    {.column = inside_column, .row = before},
    {.column = inside_column, .row = beyond_row},
    {.column = before, .row = before},
    {.column = beyond_column, .row = before},
    {.column = before, .row = beyond_row},
    {.column = beyond_column, .row = beyond_row}};

  for (const CellCoordinates& cell : outside)
  {
    EXPECT_FALSE(map->isInside(cell))
      << "cell (" << cell.column << ", " << cell.row << ") lies outside a "
      << width << " by " << height << " grid, whose columns run from 0 to "
      << width - 1U << " and whose rows run from 0 to " << height - 1U << ".";

    const WorldPoint point = map->worldFromCell(cell);

    EXPECT_FALSE(map->isFree(point.x, point.y))
      << "the world point (" << point.x << ", " << point.y << ") stands at "
      << "cell (" << cell.column << ", " << cell.row << "), outside the grid, "
      << "and a position outside the map is not free space: nothing there has "
      << "been observed. The borrowed `OMap::isOccupied` reports an out of "
      << "bounds cell as unoccupied, so an `isFree` leaning on it would call "
      << "every position outside the map free, and the pose generator would "
      << "hand back a pose in unmapped space as though it had found room to "
      << "search.";
  }
}

TEST(OccupancyGridMapMethodNames, AnUnrecognisedNameIsRefused)
{
  for (const std::string name : {"", "Bresenham", "ray marching", "cddt2", "raymarching"})
    EXPECT_FALSE(fsm_localisation::castingMethodFromName(name).has_value())
      << "\"" << name << "\" is not one of the three methods this package "
      << "carries, so it must be refused here, at start up, where the node can "
      << "say so and stop. There is no enumerator for it and a map built with "
      << "a method nobody chose would cast rays nobody asked for.";
}

TEST(OccupancyGridMapMethodNames, EveryEnumeratorRoundTripsThroughItsName)
{
  for (const CastingMethod method : {
         CastingMethod::bresenham, CastingMethod::cddt, CastingMethod::ray_marching})
  {
    const std::string name(fsm_localisation::nameOfCastingMethod(method));
    const auto parsed = fsm_localisation::castingMethodFromName(name);

    ASSERT_TRUE(parsed.has_value())
      << "the name this package prints for a method, \"" << name << "\", must "
      << "be a name it can also read back, or a log line and a parameter file "
      << "disagree about what the same method is called.";

    EXPECT_EQ(parsed.value(), method)
      << "\"" << name << "\" parsed back as a different method, so two of the "
      << "three enumerators share a name or the parser's branches are in the "
      << "wrong order.";
  }
}

TEST(OccupancyGridMapOwnership, TheMapOutlivesTheFirstPointerToIt)
{
  const unsigned int column = 5U;
  const unsigned int row = 9U;

  std::shared_ptr<const OccupancyGridMap> kept;
  WorldPoint point;

  {
    const auto built = OccupancyGridMap::build(
      gridMessage(GridGeometry{.width = 20U, .height = 40U}, {}),
      kOccupancyThreshold, CastingMethod::ray_marching);
    ASSERT_TRUE(built.has_value()) << built.error();

    kept = built.value();
    point = kept->worldFromCell(
      CellCoordinates{.column = column + 0.5, .row = row + 0.5});

    ASSERT_EQ(kept.use_count(), 2)
      << "the pointer `build` returned and the copy taken here are two owners "
      << "of one object.";
  }

  EXPECT_EQ(kept.use_count(), 1)
    << "the pointer `build` returned has gone out of scope, leaving this copy "
    << "as the only owner. A count above one means something else is holding "
    << "the map and the object's lifetime is not the pointer's.";

  EXPECT_TRUE(kept->isFree(point.x, point.y))
    << "the map must still answer after the pointer it was built through has "
    << "gone. This is what lets a correction in flight finish against the map "
    << "it started with while a newer map replaces the node's pointer.";
}

TEST_P(OccupancyGridMapThreshold, ACellIsFreeExactlyWhenItsValueSaysSo)
{
  const unsigned int column = 5U;
  const unsigned int row = 9U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{.width = 20U, .height = 40U},
      {CellValue{.column = column, .row = row, .value = GetParam().value}}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint point = map->worldFromCell(
    CellCoordinates{.column = column + 0.5, .row = row + 0.5});

  EXPECT_EQ(map->isFree(point.x, point.y), GetParam().free)
    << "a cell holding " << static_cast<int>(GetParam().value)
    << ", against a threshold of " << kOccupancyThreshold << ", must be "
    << (GetParam().free ? "free" : "blocked")
    << ". A cell blocks when its value is at or above the threshold, or when "
    << "it is negative: " << kOccupancyThreshold << " itself blocks and "
    << kOccupancyThreshold - 1 << " does not, and every negative value blocks "
    << "rather than only the minus one the message defines as unknown, which "
    << "is the conservative reading of a value the message has no meaning for.";
}

INSTANTIATE_TEST_SUITE_P(
  AtTheThresholdEitherSideOfItAndBelowZero, OccupancyGridMapThreshold,
  testing::Values(
    ThresholdCase{.value = static_cast<std::int8_t>(kOccupancyThreshold), .free = false},
    ThresholdCase{.value = static_cast<std::int8_t>(kOccupancyThreshold - 1), .free = true},
    ThresholdCase{.value = static_cast<std::int8_t>(kOccupancyThreshold + 1), .free = false},
    ThresholdCase{.value = 0, .free = true},
    ThresholdCase{.value = 100, .free = false},
    ThresholdCase{.value = -1, .free = false},
    ThresholdCase{.value = -50, .free = false},
    ThresholdCase{.value = -128, .free = false}));

TEST(OccupancyGridMapTransposition, ABlockedCellIsNotBlockedAtItsTransposedCell)
{
  const unsigned int width = 20U;
  const unsigned int height = 40U;
  const unsigned int column = 3U;
  const unsigned int row = 7U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = width, .height = height, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      {CellValue{.column = column, .row = row, .value = kBlocked}}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  const WorldPoint here = map->worldFromCell(
    CellCoordinates{.column = column + 0.5, .row = row + 0.5});
  const WorldPoint transposed = map->worldFromCell(
    CellCoordinates{.column = row + 0.5, .row = column + 0.5});

  EXPECT_FALSE(map->isFree(here.x, here.y))
    << "the one blocked cell of this grid was written at column " << column
    << ", row " << row << ", which is `data[" << row << " * " << width << " + "
    << column << "]`, and must be blocked when it is read back. If this passes "
    << "and the assertion below fails, the whole grid is transposed.";

  EXPECT_TRUE(map->isFree(transposed.x, transposed.y))
    << "column " << row << ", row " << column << " is where a transposed read "
    << "would put the blocked cell, and it is inside this " << width << " by "
    << height << " grid, so it has to be asserted free rather than left to a "
    << "square fixture that cannot tell the two apart. It came back blocked, "
    << "which means the column and the row have been exchanged somewhere "
    << "between the message's `data` and the answer.";
}

TEST(OccupancyGridMapTransposition, ABlockedCellWhoseTransposedCellIsOutsideTheGrid)
{
  const unsigned int width = 20U;
  const unsigned int height = 40U;
  const unsigned int column = 3U;
  const unsigned int row = 30U;

  const auto built = OccupancyGridMap::build(
    gridMessage(
      GridGeometry{
        .width = width, .height = height, .resolution = kResolution,
        .origin_x = kOriginX, .origin_y = kOriginY, .origin_yaw = kOriginYaw},
      {CellValue{.column = column, .row = row, .value = kBlocked}}),
    kOccupancyThreshold, CastingMethod::ray_marching);
  ASSERT_TRUE(built.has_value()) << built.error();
  const std::shared_ptr<const OccupancyGridMap> map = built.value();

  ASSERT_TRUE(map->isInside(CellCoordinates{.column = column + 0.5, .row = row + 0.5}))
    << "column " << column << ", row " << row << " must be inside a " << width
    << " by " << height << " grid for this case to mean anything.";

  ASSERT_FALSE(map->isInside(CellCoordinates{.column = row + 0.5, .row = column + 0.5}))
    << "column " << row << ", row " << column << " must be outside a " << width
    << " by " << height << " grid, since " << row << " is beyond a width of "
    << width << ". That is the whole point of this case.";

  const WorldPoint here = map->worldFromCell(
    CellCoordinates{.column = column + 0.5, .row = row + 0.5});
  const WorldPoint transposed = map->worldFromCell(
    CellCoordinates{.column = row + 0.5, .row = column + 0.5});

  EXPECT_FALSE(map->isFree(here.x, here.y))
    << "the blocked cell at column " << column << ", row " << row << " must "
    << "be blocked. A transposed read of it would look for column " << row
    << ", row " << column << ", which is outside the grid, so a transposed "
    << "implementation reports this cell free rather than reporting some other "
    << "cell's occupancy.";

  EXPECT_FALSE(map->isFree(transposed.x, transposed.y))
    << "and the transposed cell is outside the grid, which is not free either.";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
