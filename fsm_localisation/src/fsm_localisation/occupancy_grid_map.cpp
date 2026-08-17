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
bool blockedAt(
  const std::vector<bool>& blocked, const unsigned int width,
  const unsigned int column, const unsigned int row)
{
  return blocked[static_cast<std::size_t>(row) * width + column];
}

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

bool isBlocked(const std::int8_t value, const int occupancy_threshold)
{
  return value < 0 || static_cast<int>(value) >= occupancy_threshold;
}

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

float maxRangeCells(const unsigned int width, const unsigned int height)
{
  return std::hypot(static_cast<float>(width), static_cast<float>(height));
}

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

}

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

OccupancyGridMap::~OccupancyGridMap() = default;

CellCoordinates OccupancyGridMap::cellFromWorld(const double x, const double y) const noexcept
{
  const double dx = x - origin_x_;
  const double dy = y - origin_y_;

  return CellCoordinates{
    .column = (cos_yaw_ * dx + sin_yaw_ * dy) / resolution_,
    .row = (-sin_yaw_ * dx + cos_yaw_ * dy) / resolution_};
}

unsigned int OccupancyGridMap::height() const noexcept
{
  return height_;
}

bool OccupancyGridMap::isFree(const double x, const double y) const noexcept
{
  const CellCoordinates cell = cellFromWorld(x, y);

  if (!isInside(cell))
    return false;

  const unsigned int column = static_cast<unsigned int>(std::floor(cell.column));
  const unsigned int row = static_cast<unsigned int>(std::floor(cell.row));

  return !blockedAt(blocked_, width_, column, row);
}

bool OccupancyGridMap::isInside(const CellCoordinates cell) const noexcept
{
  const double column = std::floor(cell.column);
  const double row = std::floor(cell.row);

  if (column < 0.0 || row < 0.0)
    return false;

  return column < static_cast<double>(width_) && row < static_cast<double>(height_);
}

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

double OccupancyGridMap::resolution() const noexcept
{
  return resolution_;
}

unsigned int OccupancyGridMap::width() const noexcept
{
  return width_;
}

WorldPoint OccupancyGridMap::worldFromCell(const CellCoordinates cell) const noexcept
{
  return WorldPoint{
    .x = origin_x_ + resolution_ * (cos_yaw_ * cell.column - sin_yaw_ * cell.row),
    .y = origin_y_ + resolution_ * (sin_yaw_ * cell.column + cos_yaw_ * cell.row)};
}

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

}
