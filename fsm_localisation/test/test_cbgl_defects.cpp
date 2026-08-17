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
 * @brief Four defects that stood in the borrowed ray casting, each held down
 * by a case that failed while the defect stood and now passes.
 * @details
 * The ray casting in `include/fsm_localisation/range_libc/` is Corey Walsh's
 * `range_libc`, taken by way of cbgl. Upstream is sound; cbgl's additions and
 * edits to it were where the defects sat, and this file gives each one a
 * case, so that the correction can be shown to have corrected something real
 * rather than merely to have moved code about.
 * Three of the four were in one place. Upstream builds its occupancy grid
 * from a PNG file; cbgl added a second constructor that built it from a
 * decoded buffer already in memory, `OMap(unsigned char*, float)`, and that
 * addition never assigned `width` or `height`, yet looped over both to size
 * the grid it was building; read a local `error` that nothing had written and
 * abandoned the grid when that indeterminate value was non-zero; and assigned
 * none of the fields that place the grid in the world, so a pose given in
 * world coordinates was converted against an origin that was never set. That
 * constructor is gone rather than repaired: this package has no colour buffer
 * to give it, and the two cases that once built one now build their grid
 * through `freeGrid` and `gridWithBlockedCell` below, which reach the
 * constructor that survives, `OMap(int, int, const std::vector<bool>&)`.
 * The fourth was in `RayMarching::calc_range`, where cbgl rounded one axis of
 * the marching point down and the other up. The case for it stands first
 * below and says the rest.
 * The first three were reads of indeterminate values, which is undefined
 * behaviour and not merely a wrong answer, so what any one run reported was
 * not fixed by the source. `TheUninitialisedErrorTestLeavesTheGridEmpty` was
 * therefore the one of those three to rely on: it asks for a range through a
 * grid whose contents are known, and an empty grid cannot return that range
 * however the indeterminate values happened to fall. The fourth defect was
 * not undefined behaviour at all, and its case was as deterministic as that
 * one.
 * The cases below are ordered alphabetically, which is the house rule and is
 * not the order the defects are listed in here.
 * Two conventions of the borrowed code are worth stating before the cases use
 * them, because neither is obvious and both decide where a blocked cell has
 * to be put.
 * The grid is indexed `grid[first][second]`, where the first index is bounded
 * by `width` and the second by `height`. `RangeMethod::calc_range(x, y, h)`
 * takes them in that order, in cells, and marches from there along `h`.
 * `OMap`'s three argument constructor reads its flat vector the same way the
 * ROS message it stands in for is laid out, row major: `cells[row * width +
 * column]`.
 * The world to grid conversion, guarded by `ROS_WORLD_TO_GRID_CONVERSION` and
 * defined to be on at the head of `RangeLib.h`, hands `calc_range` its two
 * arguments the other way round. A grid read through it is therefore
 * transposed with respect to the map it represents: the first index runs along
 * the world's y axis and the second along the world's x axis. That is the
 * convention upstream's own callers use, and the third case follows it.
 * Ranges are asserted to within one cell throughout. A marching caster stops at
 * the cell it samples rather than at the wall inside it, so one cell is the
 * finest any of this can be held to, and a tighter bound would be asserting
 * against the sampling pattern rather than against the geometry.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "fsm_localisation/range_libc/RangeLib.h"

namespace
{
const unsigned kCellsPerSide = 40U;

const float kMaxRangeCells = 100.0F;

const float kOneCell = 1.0F;

ranges::OMap freeGrid(const unsigned width, const unsigned height)
{
  const std::vector<bool> cells(static_cast<std::size_t>(width) * height, false);
  return ranges::OMap(static_cast<int>(width), static_cast<int>(height), cells);
}

ranges::OMap gridWithBlockedCell(
  const unsigned width, const unsigned height, const unsigned x, const unsigned y)
{
  ranges::OMap map = freeGrid(width, height);
  map.grid[x][y] = true;
  return map;
}

}

TEST(CbglCasterDefects, RowRoundsUpWhileColumnRoundsDown)
{
  const unsigned sensor_row = 5U;
  const unsigned blocked_column = 25U;
  const unsigned decoy_row = sensor_row + 1U;
  const unsigned decoy_column = 35U;

  ranges::OMap map(static_cast<int>(kCellsPerSide), static_cast<int>(kCellsPerSide));
  map.grid[blocked_column][sensor_row] = true;
  map.grid[decoy_column][decoy_row] = true;

  ranges::RayMarching caster(map, kMaxRangeCells);

  const float sensor_column = 12.5F;
  const float sensor_row_centre = sensor_row + 0.5F;
  const float heading = 0.0F;

  const float range = caster.calc_range(sensor_column, sensor_row_centre, heading);

  const float expected = blocked_column - sensor_column;
  const float row_above = decoy_column - sensor_column;

  EXPECT_NEAR(range, expected, kOneCell)
    << "cbgl defect four, the asymmetric rounding: RayMarching::calc_range "
    << "takes the column of the marching point with floorf and its row with "
    << "ceilf, so a sensor standing in the middle of row " << sensor_row
    << " walks row " << decoy_row << " instead. The sign aware code this "
    << "replaced sits commented out immediately above it. A reported range "
    << "near " << row_above << " cells is the cell standing in row "
    << decoy_row << ", not the one in front of the sensor. The correction "
    << "restores the published original's truncation, which takes both axes "
    << "the same way.";
}

TEST(CbglOMapDefects, OriginIsNotAppliedToAWorldPose)
{
  const float resolution = 0.05F;
  const float origin_x = -3.2F;
  const float origin_y = 7.4F;
  const float origin_yaw = 0.7F;

  const unsigned blocked_cell_x = 25U;
  const unsigned blocked_cell_y = 12U;
  const unsigned sensor_cell_x = 5U;

  ranges::OMap map = gridWithBlockedCell(
    kCellsPerSide, kCellsPerSide, blocked_cell_y, blocked_cell_x);

  map.world_scale = resolution;
  map.world_angle = -origin_yaw;
  map.world_origin_x = origin_x;
  map.world_origin_y = origin_y;
  map.world_sin_angle = std::sin(map.world_angle);
  map.world_cos_angle = std::cos(map.world_angle);

  ranges::RayMarching caster(map, kMaxRangeCells);

  const float sensor_x_cells = sensor_cell_x + 0.5F;
  const float sensor_y_cells = blocked_cell_y + 0.5F;

  const float cos_yaw = std::cos(origin_yaw);
  const float sin_yaw = std::sin(origin_yaw);

  float pose[3] = {
    origin_x + resolution * (cos_yaw * sensor_x_cells - sin_yaw * sensor_y_cells),
    origin_y + resolution * (sin_yaw * sensor_x_cells + cos_yaw * sensor_y_cells),
    origin_yaw};

  float range[1] = {-1.0F};
  caster.numpy_calc_range(pose, range, 1);

  const float expected = (blocked_cell_x - sensor_x_cells) * resolution;

  EXPECT_NEAR(range[0], expected, resolution)
    << "cbgl defect three, the origin ignored: the constructor once under "
    << "test here gave a grid neither its cells nor the fields that place it "
    << "in the world, so a world pose was converted against an origin that "
    << "was never assigned and marched through an empty grid. A reported "
    << "range of " << kMaxRangeCells * resolution << " m is the sensor's "
    << "maximum and means the ray met nothing whatever.";
}

TEST(CbglOMapDefects, TheUninitialisedErrorTestLeavesTheGridEmpty)
{
  const unsigned blocked_first = 12U;
  const unsigned blocked_second = 25U;
  const unsigned sensor_cell_second = 5U;

  ranges::OMap map = gridWithBlockedCell(
    kCellsPerSide, kCellsPerSide, blocked_first, blocked_second);

  ranges::RayMarching caster(map, kMaxRangeCells);

  const float sensor_first = blocked_first + 0.5F;
  const float sensor_second = sensor_cell_second + 0.5F;
  const float heading = static_cast<float>(M_PI) / 2.0F;

  const float range = caster.calc_range(sensor_first, sensor_second, heading);

  const float expected = blocked_second - sensor_second;

  EXPECT_NEAR(range, expected, kOneCell)
    << "cbgl defects one and two: the constructor once under test here "
    << "tested an uninitialised error variable and looped over a width and a "
    << "height its initialiser list never assigned, so the grid it built did "
    << "not hold the room it was given. A reported range of " << kMaxRangeCells
    << " cells is the sensor's maximum and means the blocked cell is not in "
    << "the grid at all.";
}

TEST(CbglOMapDefects, WidthAndHeightAreNeverAssigned)
{
  const unsigned expected_width = 24U;
  const unsigned expected_height = 37U;

  ranges::OMap map = freeGrid(expected_width, expected_height);

  EXPECT_EQ(map.width, expected_width)
    << "cbgl defect one: the constructor once under test here never assigned "
    << "width, so the value read here was never written by anything. The "
    << "grid built here is " << expected_width << " by " << expected_height
    << " cells.";

  EXPECT_EQ(map.height, expected_height)
    << "cbgl defect one: the constructor once under test here never assigned "
    << "height, so the value read here was never written by anything. The "
    << "grid built here is " << expected_width << " by " << expected_height
    << " cells.";

  ASSERT_EQ(map.grid.size(), expected_width)
    << "cbgl defect one: the grid is sized by the same width, so it must "
    << "hold exactly that many columns.";

  EXPECT_EQ(map.grid.front().size(), expected_height)
    << "cbgl defect one: the grid's columns are sized by the same height.";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
