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
 * @brief An occupancy grid that satisfies `FSM::GridReference`, built once
 * from a `nav_msgs::msg::OccupancyGrid` and immutable thereafter.
 */
#ifndef FSM_LOCALISATION__OCCUPANCY_GRID_MAP_HPP_
#define FSM_LOCALISATION__OCCUPANCY_GRID_MAP_HPP_

#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "fsm_localisation/fsm_core.hpp"

namespace ranges
{
class RangeMethod;
}

namespace fsm_localisation
{
/**
 * @brief Which of the borrowed library's three algorithms casts a ray into
 * an `OccupancyGridMap`.
 */
enum class CastingMethod
{
  bresenham,
  cddt,
  ray_marching
};

/**
 * @brief A position expressed in continuous grid cells rather than metres.
 * `column` is bounded by the grid's width and `row` by its height. Both are
 * continuous: a sensor standing at the middle of a cell casts from the
 * middle of that cell, not from its rounded index.
 */
struct CellCoordinates
{
  double column{0.0};
  double row{0.0};
};

/**
 * @brief A position expressed in the world frame the map's origin is given
 * in.
 */
struct WorldPoint
{
  double x{0.0};
  double y{0.0};
};

/**
 * @brief Parse a casting method's name.
 * @return the matching enumerator, or `std::nullopt` when `name` is none of
 * `"bresenham"`, `"cddt"` or `"ray_marching"`.
 */
std::optional<CastingMethod> castingMethodFromName(const std::string& name);

/**
 * @brief The name a casting method was parsed from, for a log line or a test
 * failure message.
 */
std::string_view nameOfCastingMethod(const CastingMethod method);

/**
 * @brief An occupancy grid, blocked cells and one caster over them, standing
 * behind `FSM::GridReference` so the core can cast a ray into it
 * without knowing what a ROS occupancy grid is.
 * @details
 * Immutable from the end of construction to destruction: there is no setter,
 * no `update`, and no non `const` member of any kind. A second map is a
 * second object, never an edited one, which is what lets a corrector hold a
 * map across the whole of a correction while a newer one replaces it
 * elsewhere. `build` is the only way to make one, and it hands back a
 * `std::shared_ptr<const OccupancyGridMap>` from a private constructor, so an
 * owner that is neither shared nor `const` cannot exist.
 * This type converts between the world frame and the grid's own cells
 * itself, in double, through the map's full origin: translation and yaw
 * both. The borrowed library's own world-taking entry points are never
 * called from here; see the constructor's own note on why the six fields
 * that would feed them are left alone.
 */
class OccupancyGridMap final : public FSM::GridReference
{
public:

  /**
   * @brief Validate `map` and build an `OccupancyGridMap` from it.
   * @param occupancy_threshold a cell value at or above this, or below zero,
   * is blocked; every other value is free.
   * @param method which of the three borrowed casters this map builds. One
   * is built, not three.
   * @return a complete map, or a sentence naming the field that failed
   * validation and the value it was given.
   */
  static std::expected<std::shared_ptr<const OccupancyGridMap>, std::string> build(
    const nav_msgs::msg::OccupancyGrid& map,
    const int occupancy_threshold,
    const CastingMethod method);

  ~OccupancyGridMap();

  OccupancyGridMap(const OccupancyGridMap&) = delete;
  OccupancyGridMap(OccupancyGridMap&&) = delete;
  OccupancyGridMap& operator=(const OccupancyGridMap&) = delete;
  OccupancyGridMap& operator=(OccupancyGridMap&&) = delete;

  /**
   * @brief The continuous cell a world position falls in.
   */
  CellCoordinates cellFromWorld(const double x, const double y) const noexcept;

  /**
   * @brief The grid's height in cells.
   */
  unsigned int height() const noexcept;

  /**
   * @brief Whether `(x, y)` is inside the grid and on an unblocked cell.
   */
  bool isFree(const double x, const double y) const noexcept override;

  /**
   * @brief Whether `cell`'s floor lies within the grid's width and height.
   */
  bool isInside(const CellCoordinates cell) const noexcept;

  /**
   * @brief Cast `num_rays` rays from `pose` and return the range each meets,
   * in metres.
   * @details See the ray angle contract carried above
   * `FSM::GridReference::ranges` in `fsm_core.hpp`: this override
   * computes each ray's heading with that same expression.
   */
  std::vector<double> ranges(
    const FSM::Pose& pose,
    const unsigned int num_rays,
    const double max_range) const override;

  /**
   * @brief The grid's resolution, in metres a cell.
   */
  double resolution() const noexcept;

  /**
   * @brief The grid's width in cells.
   */
  unsigned int width() const noexcept;

  /**
   * @brief The world point a continuous cell stands at.
   */
  WorldPoint worldFromCell(const CellCoordinates cell) const noexcept;

private:

  OccupancyGridMap(
    const nav_msgs::msg::OccupancyGrid& map,
    const std::vector<bool>& blocked,
    const CastingMethod method);

  const std::unique_ptr<ranges::RangeMethod> caster_;
  const std::vector<bool> blocked_;
  const double cos_yaw_;
  const double origin_x_;
  const double origin_y_;
  const double origin_yaw_;
  const double resolution_;
  const double sin_yaw_;
  const float caster_max_cells_;
  const unsigned int height_;
  const unsigned int width_;
};

}

#endif
