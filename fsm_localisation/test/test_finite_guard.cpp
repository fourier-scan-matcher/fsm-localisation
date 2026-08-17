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
 * @brief The guards that keep a value which is not a number out of the
 * matcher, exercised at the arithmetic settings this package ships
 * with rather than at the stricter ones every other suite forces.
 * @details
 * The matcher core is a header, and every function in it is inline. Code a
 * test calls directly is therefore recompiled into that test at the test's
 * own settings, whatever the test happens to link. Every other gtest target
 * in this package is built with `-O2 -fno-fast-math -ffp-contract=off`, so
 * every line of the core those suites touch is a recompilation of it under
 * settings the published binary never sees. That is the right trade for a
 * suite asserting agreement to 1e-9, and the wrong one for a guard, because
 * a guard's whole job is to survive the optimiser the package actually
 * ships.
 * This file exists to close that gap, and nothing else. It is not linked
 * against fsm_localisation_core, so no part of it reaches the core through a
 * compiled symbol, and its build pins `-Ofast -ffast-math
 * -fno-finite-math-only -ffp-contract=fast` on the target itself rather than
 * inheriting them from directory scope. The pinning matters twice over:
 * FSM_LOCALISATION_EXACT_FP=ON is a supported build of this package, and it
 * rewrites directory scope to the exact settings. A target that took its
 * arithmetic from directory scope would silently become a sixteenth copy of
 * the same safe build, and the one file written to defeat the optimiser
 * would stop meeting it. The two preprocessor checks below fail the build
 * rather than let that happen quietly.
 * `-Ofast` implies `-ffast-math`, which implies `-ffinite-math-only`, under
 * which the compiler may assume no infinity and no not-a-number value ever
 * occurs. That assumption erases a guard written as a call to std::isfinite
 * and lets an equality against a literal report true for a value that is not
 * a number. The package answers it in two places at once: the build turns
 * `-ffinite-math-only` back off, and the guards read the exponent bits
 * directly instead of asking the standard library. Each case below protects
 * one of those two answers, and each is meaningless anywhere but here.
 * Results are read back as bit patterns, never as doubles. A comparison
 * written in the same translation unit as the value under test is subject to
 * the same licences the value is, so a plain `== 0.0` cannot be trusted to
 * report honestly on a value that may not be a number. Whether any
 * particular comparison is folded is also not predictable from one
 * translation unit to the next, so nothing here asserts that a fold does or
 * does not happen. Every assertion is on an outcome that the settings this
 * target pins guarantee.
 */

#if __FINITE_MATH_ONLY__
#error "This target must keep -fno-finite-math-only. Built with finite math only, every guard it exercises is folded away and every case below passes vacuously."
#endif

#ifndef __ASSOCIATIVE_MATH__
#error "This target must be pinned to the shipping -Ofast. Built with the exact arithmetic settings, it recompiles the core into a build the published binary never runs and observes nothing about it."
#endif

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <bit>
#include <string>
#include <utility>
#include <vector>

#include "fsm_localisation/fsm_core.hpp"

namespace
{
constexpr std::uint64_t exponent_all_ones = (1ULL << 11) - 1;
constexpr std::uint64_t exponent_shift = 52;
constexpr std::uint64_t sign_shift = 63;

constexpr std::uint64_t largest_finite_bits =
  ((exponent_all_ones - 1) << exponent_shift) | ((1ULL << exponent_shift) - 1);
constexpr std::uint64_t negative_infinity_bits =
  (1ULL << sign_shift) | (exponent_all_ones << exponent_shift);
constexpr std::uint64_t negative_one_bits =
  (1ULL << sign_shift) | (1023ULL << exponent_shift);
constexpr std::uint64_t positive_infinity_bits =
  exponent_all_ones << exponent_shift;
constexpr std::uint64_t positive_one_bits = 1023ULL << exponent_shift;
constexpr std::uint64_t positive_zero_bits = 0ULL;
constexpr std::uint64_t quiet_nan_bits =
  (exponent_all_ones << exponent_shift) | (1ULL << (exponent_shift - 1));
constexpr std::uint64_t signalling_nan_bits =
  (exponent_all_ones << exponent_shift) | 1ULL;
constexpr std::uint64_t smallest_subnormal_bits = 1ULL;

struct FinitePattern
{
  std::uint64_t bits;
  bool finite;
  const char* name;
};

std::uint64_t bitsOf(const double value)
{
  return std::bit_cast<std::uint64_t>(value);
}

std::uint64_t exponentOf(const double value)
{
  return (bitsOf(value) >> exponent_shift) & exponent_all_ones;
}

double opaqueDoubleFromBits(const std::uint64_t bits)
{
  volatile std::uint64_t sink = bits;

  return std::bit_cast<double>(static_cast<std::uint64_t>(sink));
}

class FinitePredicate : public testing::TestWithParam<FinitePattern> {};

}

TEST_P(FinitePredicate, ReadsTheExponentRatherThanAskingTheStandardLibrary)
{
  const FinitePattern pattern = GetParam();
  const double value = opaqueDoubleFromBits(pattern.bits);

  EXPECT_EQ(FSM::DFTUtils::isFinite(value), pattern.finite)
    << "the exponent field of " << pattern.name << " is "
    << exponentOf(value) << " and a value is finite exactly when that field "
       "is not " << exponent_all_ones;
}

INSTANTIATE_TEST_SUITE_P(
  AcrossTheEncodingsOfADouble, FinitePredicate,
  testing::Values(
    FinitePattern{largest_finite_bits, true, "the largest finite double"},
    FinitePattern{negative_infinity_bits, false, "negative infinity"},
    FinitePattern{negative_one_bits, true, "minus one"},
    FinitePattern{positive_infinity_bits, false, "positive infinity"},
    FinitePattern{positive_one_bits, true, "one"},
    FinitePattern{positive_zero_bits, true, "zero"},
    FinitePattern{quiet_nan_bits, false, "a quiet not-a-number"},
    FinitePattern{signalling_nan_bits, false, "a signalling not-a-number"},
    FinitePattern{smallest_subnormal_bits, true, "the smallest subnormal"}),
  [](const testing::TestParamInfo<FinitePattern>& info)
  {
    return std::to_string(info.index);
  });

TEST(FiniteGuard, ANonFiniteCoefficientIsZeroedWithoutACachedPlan)
{
  const std::size_t size = 64;

  std::vector<double> rays_diff(size, 0.3);
  rays_diff[0] = opaqueDoubleFromBits(quiet_nan_bits);

  const std::vector<double> coefficient =
    FSM::DFTUtils::getFirstDFTCoefficient(rays_diff);

  ASSERT_EQ(coefficient.size(), 2u);
  EXPECT_EQ(bitsOf(coefficient[0]), bitsOf(0.0))
    << "the real part of a non finite coefficient must be replaced by zero; "
       "its bits were " << bitsOf(coefficient[0]);
  EXPECT_EQ(bitsOf(coefficient[1]), bitsOf(0.0))
    << "the imaginary part of a non finite coefficient must be replaced by "
       "zero; its bits were " << bitsOf(coefficient[1]);
}

TEST(FiniteGuard, ANonFiniteCoefficientIsZeroedWithACachedPlan)
{
  const std::size_t size = 64;

  std::vector<double> rays_diff(size, 0.3);
  rays_diff[0] = opaqueDoubleFromBits(quiet_nan_bits);

  const fftw_plan r2rp = FSM::DFTUtils::forwardPlan(size);

  const std::vector<double> coefficient =
    FSM::DFTUtils::getFirstDFTCoefficient(rays_diff, r2rp);

  ASSERT_EQ(coefficient.size(), 2u);
  EXPECT_EQ(bitsOf(coefficient[0]), bitsOf(0.0))
    << "the real part of a non finite coefficient must be replaced by zero; "
       "its bits were " << bitsOf(coefficient[0]);
  EXPECT_EQ(bitsOf(coefficient[1]), bitsOf(0.0))
    << "the imaginary part of a non finite coefficient must be replaced by "
       "zero; its bits were " << bitsOf(coefficient[1]);
}

TEST(FiniteGuard, AFiniteCoefficientIsLeftAloneByBothOverloads)
{
  const std::size_t size = 64;
  const double phase = 0.7;

  std::vector<double> rays_diff(size);
  for (std::size_t i = 0; i < size; i++)
    rays_diff[i] = cos(2 * M_PI * i / size + phase);

  const fftw_plan r2rp = FSM::DFTUtils::forwardPlan(size);

  const std::vector<double> without_plan =
    FSM::DFTUtils::getFirstDFTCoefficient(rays_diff);
  const std::vector<double> with_plan =
    FSM::DFTUtils::getFirstDFTCoefficient(rays_diff, r2rp);

  ASSERT_EQ(without_plan.size(), 2u);
  ASSERT_EQ(with_plan.size(), 2u);

  EXPECT_NE(bitsOf(without_plan[0]), bitsOf(0.0))
    << "the transform of a phase shifted cosine has a first coefficient whose "
       "real part is nowhere near zero, so the guard must not have replaced it";
  EXPECT_NE(bitsOf(without_plan[1]), bitsOf(0.0))
    << "the transform of a phase shifted cosine has a first coefficient whose "
       "imaginary part is nowhere near zero, so the guard must not have "
       "replaced it";
  EXPECT_EQ(bitsOf(with_plan[0]), bitsOf(without_plan[0]))
    << "the two overloads differ only in where the plan comes from, so a "
       "finite real part must survive both identically";
  EXPECT_EQ(bitsOf(with_plan[1]), bitsOf(without_plan[1]))
    << "the two overloads differ only in where the plan comes from, so a "
       "finite imaginary part must survive both identically";
}

TEST(FiniteGuard, ANonFiniteCriterionIsNotReadAsAPoseThatLeftTheMap)
{
  const std::vector<std::pair<double, double>> map{
    {-4.0, -4.0}, {4.0, -4.0}, {4.0, 4.0}, {-4.0, 4.0}};

  std::vector<double> real_scan(360, 2.0);
  real_scan[0] = opaqueDoubleFromBits(quiet_nan_bits);

  const FSM::Pose pose;

  const fftw_plan r2rp = FSM::DFTUtils::forwardPlan(real_scan.size());

  const FSM::TranslationOutput out = FSM::Translation::tff(
    real_scan, pose, map, 2, 0.2, false, r2rp, FSM::RaySearch::angular);

  EXPECT_EQ(exponentOf(out.criterion), exponent_all_ones)
    << "one non finite ray must leave the criterion non finite; its bits "
       "were " << bitsOf(out.criterion);
  EXPECT_FALSE(out.out_of_map)
    << "the pose sat at the centre of a square map and never left it, so "
       "out_of_map, read directly rather than recovered by comparing the "
       "criterion against a literal, must say so";
}

TEST(FiniteGuard, ANonFiniteRangeIsNotMistakenForAGapToInterpolateOver)
{
  const double range = 2.0;
  const std::size_t gap_index = 3;

  std::vector<double> ranges(8, range);
  ranges[gap_index] = opaqueDoubleFromBits(quiet_nan_bits);

  const std::vector<double> interpolated =
    FSM::DatasetUtils::interpolateRanges(ranges);

  ASSERT_EQ(interpolated.size(), ranges.size());
  for (std::size_t i = 0; i < ranges.size(); i++)
  {
    EXPECT_EQ(bitsOf(interpolated[i]), bitsOf(ranges[i]))
      << "no range in this scan reads zero, so the scan must come back "
         "untouched; index " << i << " came back as " << bitsOf(interpolated[i]);
  }
}

TEST(FiniteGuard, AZeroRangeStillMarksAGapToInterpolateOver)
{
  const double range = 2.0;
  const std::size_t gap_index = 3;

  std::vector<double> ranges(8, range);
  ranges[gap_index] = 0.0;

  const std::vector<double> interpolated =
    FSM::DatasetUtils::interpolateRanges(ranges);

  const double expected = (range + range) / 2.0;

  ASSERT_EQ(interpolated.size(), ranges.size());
  EXPECT_EQ(bitsOf(interpolated[gap_index]), bitsOf(expected))
    << "the gap is bracketed by two ranges of " << range << ", so it must be "
       "filled with their average";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
