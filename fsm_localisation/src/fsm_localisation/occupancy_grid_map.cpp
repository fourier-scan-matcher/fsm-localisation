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
 * @brief `fsm_localisation::OccupancyGridMap`.
 */
#include "fsm_localisation/occupancy_grid_map.hpp"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>

#include "fsm_localisation/range_libc/RangeLib.h"

/*
 * `OccupancyGridMap` is immutable after construction and is safe to query
 * from several threads at once through its `const` methods. That guarantee
 * rests on something the type system is not enforcing: `ranges()` is `const`
 * but reaches `RangeMethod::calc_range`, which is not, because a
 * `std::unique_ptr` member does not propagate `const` to what it points at.
 * With both switches below at zero none of the three casters writes anything
 * on the path this package calls, and the guarantee is genuine. Turning
 * either on makes `calc_range` write to a shared member of the caster, one a
 * cache and the other a table of collision indexes, and every concurrent
 * query becomes a data race that no compiler diagnostic and no test in this
 * package would report. Neither is a build option: both are plain `#define`s
 * in the borrowed header, so the only way past these assertions is an edit to
 * that header, which is what they exist to stop.
 *
 * These pin the two switches, not the property itself. Anyone re-syncing the
 * borrowed library against a newer upstream must read all three
 * `calc_range` bodies again and confirm none of them writes a member.
 */
static_assert(_USE_LRU_CACHE == 0,
  "turning the caster's cache on makes calc_range write to a shared member, "
  "which ends OccupancyGridMap's concurrent const-method guarantee");

static_assert(_TRACK_COLLISION_INDEXES == 0,
  "tracking collision indexes makes calc_range write to a shared member, "
  "which ends OccupancyGridMap's concurrent const-method guarantee");

namespace fsm_localisation
{

namespace
{

/*
 * Whether `blocked`'s cell at `(column, row)` is set, read through the same
 * row major layout `OMap`'s three argument constructor reads its own flat
 * vector through. This is one of exactly two places in the package that
 * compute `row * width + column`; the other is that constructor itself.
 */
bool blockedAt(
  const std::vector<bool>& blocked, const unsigned int width,
  const unsigned int column, const unsigned int row)
{
  return blocked[static_cast<std::size_t>(row) * width + column];
}

/*
 * Builds the one caster `method` names, over `map`, with a maximum range of
 * `max_range_cells`. One caster is built, not three: `map_scan_method`, or
 * its equivalent here, names one, and cbgl itself builds only the one it
 * names.
 *
 * CDDT is given 720 bins, matching cbgl, so that a comparison against it
 * compares like with like. That count quantises the cast heading to half a
 * degree, which at thirty metres is about thirteen centimetres of lateral
 * error: worse than a five centimetre cell, and worth knowing before this
 * method is chosen for its speed.
 */
std::unique_ptr<ranges::RangeMethod> buildCaster(
  const ranges::OMap& map, const CastingMethod method, const float max_range_cells)
{
  constexpr unsigned int kCddtThetaDiscretization = 720U;

  switch (method)
  {
    case CastingMethod::bresenham:
      return std::make_unique<ranges::BresenhamsLine>(map, max_range_cells);

    case CastingMethod::cddt:
      return std::make_unique<ranges::CDDTCast>(
        map, max_range_cells, kCddtThetaDiscretization);

    case CastingMethod::ray_marching:
      return std::make_unique<ranges::RayMarching>(map, max_range_cells);
  }

  assert(false && "buildCaster: unrecognised CastingMethod");
  return std::make_unique<ranges::RayMarching>(map, max_range_cells);
}

/*
 * Whether an occupancy value blocks a cell: at or above `occupancy_threshold`,
 * or negative. Every negative value blocks, not only -1, which is the
 * conservative reading of a malformed map.
 */
bool isBlocked(const std::int8_t value, const int occupancy_threshold)
{
  return value < 0 || static_cast<int>(value) >= occupancy_threshold;
}

/*
 * Whether `value` is neither infinite nor not-a-number. This package builds
 * at `-Ofast` by default, which implies `-ffinite-math-only`, under which
 * GCC is entitled to assume no infinity or NaN ever exists and folds
 * `std::isfinite` to a constant `true`: the same trap `isValidRange` in
 * `fsm_localisation.cpp` already reads bits to avoid. A malformed map is
 * exactly the input this check exists for, so it cannot be allowed to
 * silently pass. Reading the exponent field directly cannot be assumed away.
 */
bool isFinite(const float value)
{
  const std::uint32_t bits = std::bit_cast<std::uint32_t>(value);
  return ((bits >> 23) & 0xFFU) != 0xFFU;
}

bool isFinite(const double value)
{
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
  return ((bits >> 52) & 0x7FFU) != 0x7FFU;
}

/*
 * The largest range in cells the grid can ever answer with: its own
 * diagonal. A caster built with this maximum cannot meet anything further
 * away than the far corner from any cell inside the grid, which is what lets
 * `OccupancyGridMap::ranges` tell "met nothing" apart from "met the corner".
 */
float maxRangeCells(const unsigned int width, const unsigned int height)
{
  return std::hypot(static_cast<float>(width), static_cast<float>(height));
}

/*
 * The map's yaw, read out of its origin's orientation. `OccupancyGridMap::build`
 * has already refused a quaternion whose norm is not finite and positive, so
 * the normalisation here cannot divide by zero.
 */
double yawFromOrientation(const geometry_msgs::msg::Quaternion& orientation)
{
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);

  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;

  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

/*******************************************************************************
*/
std::optional<CastingMethod> castingMethodFromName(const std::string& name)
{
  if (name == "bresenham")
    return CastingMethod::bresenham;

  if (name == "cddt")
    return CastingMethod::cddt;

  if (name == "ray_marching")
    return CastingMethod::ray_marching;

  return std::nullopt;
}

/*******************************************************************************
*/
std::string_view nameOfCastingMethod(const CastingMethod method)
{
  switch (method)
  {
    case CastingMethod::bresenham: return "bresenham";
    case CastingMethod::cddt: return "cddt";
    case CastingMethod::ray_marching: return "ray_marching";
  }

  assert(false && "nameOfCastingMethod: unrecognised CastingMethod");
  return "";
}

/*******************************************************************************
* @details Checked here, each with its own sentence: width or height zero,
* `data` not matching width times height, a resolution that is not finite and
* positive, an origin translation whose components are not finite, an
* orientation whose components are not finite, and an orientation whose norm
* is not positive. The occupancy threshold is not checked beyond what
* `isBlocked` does with whatever it is given; validating it is the caller's,
* per the specification.
*/
std::expected<std::shared_ptr<const OccupancyGridMap>, std::string> OccupancyGridMap::build(
  const nav_msgs::msg::OccupancyGrid& map,
  const int occupancy_threshold,
  const CastingMethod method)
{
  if (map.info.width == 0)
    return std::unexpected("occupancy grid width must be greater than zero");

  if (map.info.height == 0)
    return std::unexpected("occupancy grid height must be greater than zero");

  const std::size_t expected_size =
    static_cast<std::size_t>(map.info.width) * map.info.height;

  if (map.data.size() != expected_size)
    return std::unexpected(std::format(
      "occupancy grid data holds {} cells, width {} and height {} require {}",
      map.data.size(), map.info.width, map.info.height, expected_size));

  if (!isFinite(map.info.resolution) || map.info.resolution <= 0.0F)
    return std::unexpected(std::format(
      "occupancy grid resolution must be finite and positive, got {}",
      map.info.resolution));

  if (!isFinite(map.info.origin.position.x) || !isFinite(map.info.origin.position.y))
    return std::unexpected(std::format(
      "occupancy grid origin translation must be finite, got ({}, {})",
      map.info.origin.position.x, map.info.origin.position.y));

  const geometry_msgs::msg::Quaternion& orientation = map.info.origin.orientation;

  if (!isFinite(orientation.x) || !isFinite(orientation.y) ||
      !isFinite(orientation.z) || !isFinite(orientation.w))
    return std::unexpected(std::format(
      "occupancy grid origin orientation must be finite, got ({}, {}, {}, {})",
      orientation.x, orientation.y, orientation.z, orientation.w));

  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;

  constexpr double kMinimumQuaternionNormSquared = 1e-9;

  if (norm_squared < kMinimumQuaternionNormSquared)
    return std::unexpected(std::format(
      "occupancy grid origin orientation cannot be normalised, norm squared {}",
      norm_squared));

  std::vector<bool> blocked(expected_size);

  for (std::size_t i = 0; i < expected_size; ++i)
    blocked[i] = isBlocked(map.data[i], occupancy_threshold);

  return std::shared_ptr<const OccupancyGridMap>(
    new OccupancyGridMap(map, blocked, method));
}

/*******************************************************************************
*/
OccupancyGridMap::~OccupancyGridMap() = default;

/*******************************************************************************
* @details `cell = R(-yaw) * (p - o) / r`: undo the map's own rotation, then
* scale by the resolution. `worldFromCell`, immediately below, is the inverse
* of exactly this.
*/
CellCoordinates OccupancyGridMap::cellFromWorld(const double x, const double y) const noexcept
{
  const double dx = x - origin_x_;
  const double dy = y - origin_y_;

  return CellCoordinates{
    .column = (cos_yaw_ * dx + sin_yaw_ * dy) / resolution_,
    .row = (-sin_yaw_ * dx + cos_yaw_ * dy) / resolution_};
}

/*******************************************************************************
*/
unsigned int OccupancyGridMap::height() const noexcept
{
  return height_;
}

/*******************************************************************************
* @details Outside the grid is not free: the borrowed `OMap::isOccupied`
* reports out of bounds as unoccupied, and an `isFree` that leant on it would
* call the whole world outside the map free.
*/
bool OccupancyGridMap::isFree(const double x, const double y) const noexcept
{
  const CellCoordinates cell = cellFromWorld(x, y);

  if (!isInside(cell))
    return false;

  const unsigned int column = static_cast<unsigned int>(std::floor(cell.column));
  const unsigned int row = static_cast<unsigned int>(std::floor(cell.row));

  return !blockedAt(blocked_, width_, column, row);
}

/*******************************************************************************
*/
bool OccupancyGridMap::isInside(const CellCoordinates cell) const noexcept
{
  const double column = std::floor(cell.column);
  const double row = std::floor(cell.row);

  if (column < 0.0 || row < 0.0)
    return false;

  return column < static_cast<double>(width_) && row < static_cast<double>(height_);
}

/*******************************************************************************
* @details Ray `i`'s heading is computed with the same two lines
* `X::findExactAngular` computes, `fsm_core.hpp:675` and `:676`, so that ray
* zero points behind the pose on both sides of the module boundary rather
* than drifting apart from it. The result is then turned from the world
* frame into the grid's own by subtracting `origin_yaw_`, since the caster
* knows nothing of the map's rotation; the range a ray meets is a distance
* and is unchanged by which frame it was cast in.
*
* A pose whose cell lies outside the grid returns `max_range` for every ray
* without calling the caster at all. This is also the guard for a defect in
* the borrowed library: `CDDTCast::calc_range` indexes its grid with no
* bounds test, so a pose outside the grid would otherwise be undefined
* behaviour rather than a maximum range. Load bearing: do not remove this
* guard as redundant.
*
* A pose whose heading is not a number is guarded the same way. Every ray's
* heading is derived from `pose.t`, so a single non finite heading would
* carry through to every ray cast this call makes, each one handed to the
* caster's table lookup as an index it never checks.
*
* The caster's own maximum is the grid's diagonal in cells, fixed at
* construction, which can be shorter than `max_range`. A range at or above
* that maximum means the ray escaped the grid or met nothing, and is reported
* as `max_range`; only a shorter range is a real hit. Reporting
* `min(range * resolution_, max_range)` unconditionally would instead call
* the diagonal an obstacle whenever `max_range` is larger than it, which is
* wrong far more often than it is right.
*/
std::vector<double> OccupancyGridMap::ranges(
  const FSM::Pose& pose, const unsigned int num_rays, const double max_range) const
{
  std::vector<double> result(num_rays, max_range);

  const CellCoordinates cell = cellFromWorld(pose.x, pose.y);

  if (!isInside(cell) || !isFinite(pose.t))
    return result;

  for (unsigned int i = 0; i < num_rays; ++i)
  {
    double t_ray = i * 2*M_PI / num_rays + pose.t - M_PI;
    t_ray = fmod(t_ray + 5*M_PI, 2*M_PI) - M_PI;

    const float heading_in_grid = static_cast<float>(t_ray - origin_yaw_);

    const float range_cells = caster_->calc_range(
      static_cast<float>(cell.column), static_cast<float>(cell.row), heading_in_grid);

    if (range_cells < caster_max_cells_)
      result[i] = std::min(static_cast<double>(range_cells) * resolution_, max_range);
  }

  return result;
}

/*******************************************************************************
*/
double OccupancyGridMap::resolution() const noexcept
{
  return resolution_;
}

/*******************************************************************************
*/
unsigned int OccupancyGridMap::width() const noexcept
{
  return width_;
}

/*******************************************************************************
*/
WorldPoint OccupancyGridMap::worldFromCell(const CellCoordinates cell) const noexcept
{
  return WorldPoint{
    .x = origin_x_ + resolution_ * (cos_yaw_ * cell.column - sin_yaw_ * cell.row),
    .y = origin_y_ + resolution_ * (sin_yaw_ * cell.column + cos_yaw_ * cell.row)};
}

/*******************************************************************************
* @details `origin_yaw_` is the map's yaw exactly as the message gives it,
* never negated; `cos_yaw_` and `sin_yaw_` are read from the same angle. The
* helper that extracts it is called three times over the same orientation
* rather than once into a shared local, because the member order above fixes
* `cos_yaw_` and `sin_yaw_` ahead of `origin_yaw_`, and a member's own
* initialiser may not read a sibling declared after it. The extra calls are a
* handful of trigonometric functions at map construction, which happens once
* per map, and are not worth a second constructor to avoid.
*
* `caster_` is initialised first, ahead of every field its own computation
* touches, for the same reason: its initialiser reads `map.info` directly
* rather than through `width_` or `height_`, neither of which exists yet at
* that point in construction order.
*/
OccupancyGridMap::OccupancyGridMap(
  const nav_msgs::msg::OccupancyGrid& map,
  const std::vector<bool>& blocked,
  const CastingMethod method)
: caster_(buildCaster(
    ranges::OMap(
      static_cast<int>(map.info.width), static_cast<int>(map.info.height), blocked),
    method,
    maxRangeCells(map.info.width, map.info.height))),
  blocked_(blocked),
  cos_yaw_(std::cos(yawFromOrientation(map.info.origin.orientation))),
  origin_x_(map.info.origin.position.x),
  origin_y_(map.info.origin.position.y),
  origin_yaw_(yawFromOrientation(map.info.origin.orientation)),
  resolution_(map.info.resolution),
  sin_yaw_(std::sin(yawFromOrientation(map.info.origin.orientation))),
  caster_max_cells_(maxRangeCells(map.info.width, map.info.height)),
  height_(map.info.height),
  width_(map.info.width)
{
}

}  // namespace fsm_localisation
