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

#ifndef FSM_LOCALISATION__FSM_LOCALISATION_HPP_
#define FSM_LOCALISATION__FSM_LOCALISATION_HPP_

#include <cstddef>
#include <expected>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "fsm_localisation/fsm_core.hpp"

namespace fsm_localisation
{
/**
 * @brief A planar pose, or a planar displacement between two poses.
 * The core carries the same type, so this is an alias rather than a second
 * declaration: the two layers hand poses to each other unconverted.
 */
using Pose = FSM::Pose;

/**
 * @brief Everything the matcher needs to know, and nothing about ROS.
 */
struct Parameters
{
  std::size_t size_scan{0};
  unsigned int num_iterations{2};
  double xy_bound{0.2};
  double t_bound{M_PI / 4};
  unsigned int max_counter{200};
  unsigned int min_magnification_size{0};
  unsigned int max_magnification_size{3};
  unsigned int max_recoveries{10};

  unsigned int rng_seed{0};

  std::string ray_search{"angular"};

  bool enforce_early_gearup{false};

  bool enforce_translation_bound{false};

  bool recover_on_failed_sift{false};

  int occupancy_threshold{65};

  std::string map_scan_method{"ray_marching"};
};

/**
 * @brief What one successful match yields.
 */
struct MatchResult
{
  Pose increment;

  Pose accumulated;

  double execution_time{0.0};
  unsigned int num_recoveries{0};
};

/**
 * @brief Why a scan produced no match.
 */
enum class MatchError
{
  no_reference_yet,

  scan_too_short,

  scan_entirely_invalid,
};

/**
 * @brief What one successful correction yields.
 * There is no accumulated pose and no trajectory. A correction is answerable
 * from the estimate it was handed and nothing else, so there is nothing to
 * accumulate into.
 */
struct CorrectionResult
{
  Pose corrected;

  double execution_time{0.0};
  unsigned int num_recoveries{0};
};

/**
 * @brief Why an estimate was not corrected.
 */
enum class CorrectionError
{
  scan_too_short,

  scan_entirely_invalid,

  estimate_not_in_free_space,

  first_cast_entirely_at_max_range,

  recovery_budget_exhausted,

  aperture_not_a_sensor,
};

/**
 * @brief Whether a range reading carries a usable distance.
 * A lidar reports a ray that struck nothing, or a reading it could not trust,
 * in one of three ways: zero, infinity, or not-a-number. Which of the three it
 * picks is a matter of driver convention. All three mean the same thing here,
 * and are filled in from the rays around them before matching.
 */
bool isValidRange(double range);

/**
 * @brief Validate a parameter set.
 * @return an empty string when the parameters are usable, otherwise a sentence
 * naming the offending parameter and its value.
 */
std::string validate(const Parameters& parameters);

/**
 * @brief Install a destination for the matching core's diagnostics.
 * The core reports in plain strings and does not grade them, having no way to
 * know what a host considers serious. Where no destination is installed the
 * reports are dropped, which is what a library with nobody listening should
 * do.
 * Nearly everything the core has to say is stage timing, and that is compiled
 * out unless the core is built with FSM_LOCALISATION_TRACE. An ordinary build reports
 * only where it cannot write a file or is handed a mode it does not know.
 * Install before matching starts. Nothing guards a host that swaps the
 * destination while a match is running.
 */
void setDiagnosticSink(std::function<void(const std::string&)> sink);

/**
 * @brief Correction of a pose estimate against a map, by the same
 * correspondenceless scan matching.
 * @details
 * Where the odometry matches a scan against the scan before it and reports how
 * far the sensor moved between them, this matches a scan against a map and
 * reports where the sensor actually was. The matcher walks the estimate around
 * the map until the scan it would see there agrees with the scan it did see,
 * casting into the map at every pose it tries rather than at one scan frozen
 * where it started.
 * Nothing survives a correction. There is no previous scan, no accumulated
 * pose and no trajectory, so two corrections in either order give the same two
 * answers, and a correction can be repeated exactly by pinning `rng_seed`.
 * The map is borrowed through `FSM::GridReference`, which is what keeps this
 * file and its implementation free of any ROS header: a host reads an
 * occupancy grid message into an `OccupancyGridMap` and hands that over as the
 * interface. The shared pointer is held for the life of the corrector, so a
 * host replacing the map mid correction constructs a second corrector rather
 * than editing this one's.
 * The accuracy a correction can reach is bounded by the map and not by the
 * matcher. A ray stops at a cell, so a map of five centimetre cells cannot
 * support an answer better than five centimetres however good the match is.
 */
class Corrector
{
public:
  Corrector(const Parameters& parameters,
    std::shared_ptr<const FSM::GridReference> map);

  Corrector(const Corrector&) = delete;
  Corrector& operator=(const Corrector&) = delete;
  Corrector(Corrector&&) = delete;
  Corrector& operator=(Corrector&&) = delete;

  /**
   * @brief Correct one pose estimate against the map.
   * @param ranges the scan as the sensor reported it, unfilled and unreduced.
   * @param estimate where the sensor is thought to have been when that scan
   * was taken, in the frame the map is given in.
   * @param range_max the scan's own maximum range, which is what a ray meeting
   * nothing reports. The map has never seen it, so it is given per
   * correction rather than held.
   * @param aperture what the sensor covers, in radians. A sensor that sees all
   * the way round passes a whole turn.
   * @param first_ray_angle the angle of the first of `ranges` from the
   * sensor's heading, in radians, which is a laser scan's `angle_min`. It
   * stays the angle of the first range where `size_scan` resamples the scan.
   * @remark The aperture carries no default, deliberately. A default of a
   * whole turn would let a caller with a narrower sensor say nothing and be
   * answered confidently and wrongly, the map being read over a whole turn and
   * matched against a measurement covering less of one. That failure has
   * happened once in this family already. A required parameter turns it into a
   * compile error instead.
   */
  std::expected<CorrectionResult, CorrectionError> correct(
    std::span<const double> ranges,
    const Pose& estimate,
    double range_max,
    double aperture,
    double first_ray_angle) const;

  const Parameters& parameters() const { return parameters_; }

private:
  Parameters parameters_;
  std::shared_ptr<const FSM::GridReference> map_;
};

/**
 * @brief Lidar odometry by correspondenceless scan matching.
 * Holds the odometry state that is not a ROS concept: the cached transform
 * plans, the previous scan, the accumulating pose, and the trajectory. Feed it
 * scans and it yields the displacement between each pair.
 */
class Matcher
{
public:
  explicit Matcher(const Parameters& parameters);

  Matcher(const Matcher&) = delete;
  Matcher& operator=(const Matcher&) = delete;
  Matcher(Matcher&&) = delete;
  Matcher& operator=(Matcher&&) = delete;

  /**
   * @brief Match a scan against the one before it.
   * The first scan after construction or after a reset is kept as the
   * reference and yields no result.
   */
  std::expected<MatchResult, MatchError> process(std::span<const double> ranges);

  /**
   * @brief Forget the trajectory and return the accumulated pose to the origin.
   * The reference scan is kept, so the next scan still produces an increment.
   */
  void clearTrajectory();

  /**
   * @brief Place the accumulated pose at an arbitrary starting point.
   */
  void setInitialPose(const Pose& pose);

  const Parameters& parameters() const { return parameters_; }
  Pose accumulatedPose() const;
  const std::vector<Pose>& trajectory() const { return trajectory_; }
  unsigned int scansSeen() const { return scans_seen_; }

  /**
   * @brief How many rays every scan of this session is matched at.
   * Zero until the first scan settles it, where size_scan asked for no
   * particular size.
   */
  std::size_t matchSize() const { return match_size_; }

private:
  Parameters parameters_;

  std::size_t match_size_{0};

  std::vector<double> reference_scan_;
  std::vector<Pose> trajectory_;
  Eigen::Matrix3d accumulated_{Eigen::Matrix3d::Identity()};
  unsigned int scans_seen_{0};
};

}

#endif
