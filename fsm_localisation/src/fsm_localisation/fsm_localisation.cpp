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

#include "fsm_localisation/fsm_localisation.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <ranges>
#include <utility>

namespace fsm_localisation
{
namespace
{
FSM::input_params asInputParams(const Parameters& parameters)
{
  FSM::input_params ip;
  ip.num_iterations = parameters.num_iterations;
  ip.xy_bound = parameters.xy_bound;
  ip.t_bound = parameters.t_bound;
  ip.max_counter = parameters.max_counter;
  ip.min_magnification_size = parameters.min_magnification_size;
  ip.max_magnification_size = parameters.max_magnification_size;
  ip.max_recoveries = parameters.max_recoveries;
  ip.rng_seed = parameters.rng_seed;
  ip.ray_search = parameters.ray_search == "windowed"
    ? FSM::RaySearch::windowed
    : FSM::RaySearch::angular;
  ip.enforce_early_gearup = parameters.enforce_early_gearup;
  ip.enforce_translation_bound = parameters.enforce_translation_bound;
  ip.recover_on_failed_sift = parameters.recover_on_failed_sift;
  return ip;
}

bool isFinite(const double value)
{
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);

  return ((bits >> 52) & 0x7FFU) != 0x7FFU;
}

std::optional<std::vector<double>> reduceScan(
  const std::span<const double> ranges,
  std::size_t& match_size,
  const FSM::RaySearch ray_search)
{
  std::vector<double> scan(ranges.begin(), ranges.end());
  std::ranges::replace_if(scan,
    [](const double range) { return !isValidRange(range); }, 0.0);

  if (std::ranges::none_of(scan, isValidRange))
    return std::nullopt;

  if (match_size == 0)
    match_size = scan.size();

  scan = FSM::DatasetUtils::interpolateRanges(scan);

  return FSM::Utils::subsampleScan(scan, match_size, ray_search);
}

}

bool isValidRange(const double range)
{
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(range);
  const std::uint64_t exponent = (bits >> 52) & 0x7FFU;

  if (exponent == 0x7FFU)
    return false;

  return range > 0.0;
}

void setDiagnosticSink(std::function<void(const std::string&)> sink)
{
  FSM::Diagnostics::setSink(std::move(sink));
}

std::string validate(const Parameters& parameters)
{
  if (parameters.num_iterations == 0)
    return "num_iterations must be greater than zero";

  if (!isFinite(parameters.xy_bound))
    return "xy_bound must be a finite number, got "
      + std::to_string(parameters.xy_bound);

  if (parameters.xy_bound < 0.0)
    return "xy_bound must not be negative, got "
      + std::to_string(parameters.xy_bound);

  if (!isFinite(parameters.t_bound))
    return "t_bound must be a finite number, got "
      + std::to_string(parameters.t_bound);

  if (parameters.t_bound < 0.0)
    return "t_bound must not be negative, got "
      + std::to_string(parameters.t_bound);

  if (parameters.max_counter == 0)
    return "max_counter must be greater than zero";

  if (parameters.max_magnification_size < parameters.min_magnification_size)
    return "max_magnification_size must not be smaller than "
      "min_magnification_size";

  if (parameters.ray_search != "angular" && parameters.ray_search != "windowed")
    return "ray_search must be \"angular\" or \"windowed\", got \""
      + parameters.ray_search + "\"";

  if (parameters.occupancy_threshold < 0 || parameters.occupancy_threshold > 100)
    return "occupancy_threshold must be between 0 and 100, got "
      + std::to_string(parameters.occupancy_threshold);

  return {};
}

Corrector::Corrector(const Parameters& parameters,
  std::shared_ptr<const FSM::GridReference> map)
: parameters_(parameters),
  map_(std::move(map))
{
}

std::expected<CorrectionResult, CorrectionError>
Corrector::correct(std::span<const double> ranges, const Pose& estimate,
  const double range_max, const double aperture,
  const double first_ray_angle) const
{
  if (!(aperture > 0.0) || aperture > 2.0 * M_PI + 1e-9)
    return std::unexpected(CorrectionError::aperture_not_a_sensor);

  if (ranges.size() < parameters_.size_scan)
    return std::unexpected(CorrectionError::scan_too_short);

  if (!map_->isFree(estimate.x, estimate.y))
    return std::unexpected(CorrectionError::estimate_not_in_free_space);

  const FSM::input_params input_parameters = asInputParams(parameters_);

  std::size_t match_size = parameters_.size_scan;

  const std::optional<std::vector<double>> scan =
    reduceScan(ranges, match_size, input_parameters.ray_search);

  if (!scan)
    return std::unexpected(CorrectionError::scan_entirely_invalid);

  const std::vector<double> first_cast = map_->ranges(estimate,
    static_cast<unsigned int>(FSM::panoramicRaysOf(match_size, aperture)),
    range_max);

  if (std::ranges::all_of(first_cast,
      [range_max](const double range) { return range >= range_max; }))
  {
    return std::unexpected(CorrectionError::first_cast_entirely_at_max_range);
  }

  const FSM::MatchOutput match = FSM::Match::fmt(*scan, estimate,
    FSM::Reference(*map_, range_max, aperture, first_ray_angle),
    FSM::DFTUtils::forwardPlan(match_size),
    FSM::DFTUtils::inversePlan(match_size), input_parameters);

  if (match.op.num_recoveries > parameters_.max_recoveries)
    return std::unexpected(CorrectionError::recovery_budget_exhausted);

  return CorrectionResult{
    match.pose,
    match.op.exec_time,
    match.op.num_recoveries};
}

Matcher::Matcher(const Parameters& parameters)
: parameters_(parameters),
  match_size_(parameters.size_scan)
{
}

Pose Matcher::accumulatedPose() const
{
  return Pose{
    accumulated_(0, 2),
    accumulated_(1, 2),
    std::atan2(accumulated_(1, 0), accumulated_(0, 0))};
}

void Matcher::clearTrajectory()
{
  trajectory_.clear();
  accumulated_ = Eigen::Matrix3d::Identity();
}

std::expected<MatchResult, MatchError>
Matcher::process(std::span<const double> ranges)
{
  if (ranges.size() < parameters_.size_scan)
    return std::unexpected(MatchError::scan_too_short);

  const FSM::input_params input_parameters = asInputParams(parameters_);

  std::optional<std::vector<double>> reduced =
    reduceScan(ranges, match_size_, input_parameters.ray_search);

  if (!reduced)
    return std::unexpected(MatchError::scan_entirely_invalid);

  std::vector<double> scan = std::move(*reduced);

  scans_seen_++;

  if (reference_scan_.empty())
  {
    reference_scan_ = std::move(scan);
    return std::unexpected(MatchError::no_reference_yet);
  }

  const Pose origin;

  const std::vector<std::pair<double, double>> reference_points =
    FSM::Utils::scan2points(reference_scan_, origin);

  const FSM::MatchOutput match = FSM::Match::fmt(scan, origin,
    reference_points, FSM::DFTUtils::forwardPlan(match_size_),
    FSM::DFTUtils::inversePlan(match_size_), input_parameters);

  accumulated_ = FSM::Utils::computeTransform(match.pose, accumulated_);
  trajectory_.push_back(accumulatedPose());

  reference_scan_ = std::move(scan);

  return MatchResult{
    match.pose,
    accumulatedPose(),
    match.op.exec_time,
    match.op.num_recoveries};
}

void Matcher::setInitialPose(const Pose& pose)
{
  accumulated_ =
    FSM::Utils::computeTransform(pose, Eigen::Matrix3d::Identity());
}

}
