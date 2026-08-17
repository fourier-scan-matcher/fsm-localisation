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
 *        matcher, exercised at the arithmetic settings this package ships
 *        with rather than at the stricter ones every other suite forces.
 *
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
 *
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
 *
 * `-Ofast` implies `-ffast-math`, which implies `-ffinite-math-only`, under
 * which the compiler may assume no infinity and no not-a-number value ever
 * occurs. That assumption erases a guard written as a call to std::isfinite
 * and lets an equality against a literal report true for a value that is not
 * a number. The package answers it in two places at once: the build turns
 * `-ffinite-math-only` back off, and the guards read the exponent bits
 * directly instead of asking the standard library. Each case below protects
 * one of those two answers, and each is meaningless anywhere but here.
 *
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

/* A double assembled at run time from its bits, laundered through a volatile
 * so the optimiser cannot carry what it knows about the pattern into the
 * call. Naming the value with std::numeric_limits instead would hand the
 * compiler a constant it is entitled, at these settings, to reason about as
 * one that never occurs. */
double opaqueDoubleFromBits(const std::uint64_t bits)
{
  volatile std::uint64_t sink = bits;

  return std::bit_cast<double>(static_cast<std::uint64_t>(sink));
}

class FinitePredicate : public testing::TestWithParam<FinitePattern> {};

}  // namespace

/*
 * Protects the predicate every other guard in the core is built on.
 * DFTUtils::isFinite reads the exponent field rather than calling
 * std::isfinite, for the reason isValidRange in fsm_localisation.cpp sets out
 * first: the standard call is folded to a constant true under the arithmetic
 * this package ships, and a guard that always says yes is worse than no guard
 * at all, because it looks like one. Reading eleven bits out of an integer is
 * not arithmetic and cannot be assumed away, but nothing except running the
 * function at the shipping settings can show that. Every other suite in this
 * package recompiles this same function under settings that would keep even
 * the std::isfinite version honest, so none of them can tell the two apart.
 *
 * Both non finite encodings are covered on each side of the sign, along with
 * the two ends of the finite range and the quiet and signalling not-a-number
 * payloads, since the check is on the exponent alone and must not read the
 * mantissa.
 */
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

/*
 * Protects the finite check in the plan free overload of
 * getFirstDFTCoefficient, the one the rotation stage reaches through
 * DFTUtils::dft when no cached plan is handed down. A discrete Fourier
 * transform is a linear combination of every sample, so a single ray that is
 * not a number makes every coefficient of the transform not a number too, and
 * the first coefficient is what the correction vector is built from. Left
 * unreplaced it moves the pose by an amount that is not a number, and the
 * pose never recovers.
 *
 * This has to run at the shipping settings to mean anything. Under the exact
 * settings the rest of the suite forces, a guard written as std::isfinite
 * would pass this case as readily as the bit reading one does, so a green
 * result there says nothing about the binary that ships.
 */
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

/*
 * Protects the finite check in the overload that takes a caller supplied
 * plan. It is a second, separate copy of the same four lines, and it is the
 * copy the translation stage runs on every iteration of every match, so a
 * correction applied to one overload alone would leave the hot path
 * unguarded. Two cases rather than one because a shared guard is not what the
 * header has.
 */
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

/*
 * The other side of both guards, and the reason neither of the two cases
 * above can be satisfied by a stage that returns zero whatever it is handed.
 * A single period of a cosine, given a phase so that neither part of the
 * first coefficient lands on zero honestly, must come back through both
 * overloads with both parts left alone. No tolerance is stated and none is
 * needed: what is asserted is that the replacement did not fire, not what the
 * transform computed, which is what the suites built at the exact settings
 * are for. At these settings a value is free to differ in its last places and
 * the assertion still holds.
 */
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

/*
 * Protects the separation between the translation stage's criterion and its
 * report that a pose left the map. The stage used to signal a departure only
 * by returning a criterion of exactly -2.0, leaving every caller to compare a
 * double against that literal. At the settings this package ships, what such
 * a comparison reports for a criterion that is not a number is not something
 * a caller can rely on either way, and a caller that read it as a departure
 * would throw away a pose that never left. TranslationOutput now carries
 * out_of_map as a plain boolean, set only where a pose genuinely leaves the
 * map, which removes the comparison rather than hoping it behaves.
 *
 * A criterion that is not a number without a departure is reachable without
 * contrivance. One non finite ray in the scan makes every coefficient of its
 * transform non finite, which the guard above replaces with zero before it
 * can move the pose, but the criterion is an average of the raw per ray
 * differences taken before that replacement, so it stays not a number
 * regardless. The exponent is read rather than the value compared, for the
 * same reason the stage stopped comparing it.
 */
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

/*
 * Protects the build flag itself. interpolateRanges finds the gaps to fill by
 * comparing each range against the literal zero, which is the second half of
 * the same defect class the checks above answer: `-ffinite-math-only` lets
 * the compiler assume neither side of a floating point equality is ever a
 * value that is not a number, and an equality compiled on that assumption can
 * report true for one. The package answers it by keeping
 * `-fno-finite-math-only` in the shipping build, and this is the only case in
 * either package that would notice if that flag were ever dropped.
 *
 * A range that is not a number does not reach this function on the shipping
 * path, because the entry guard replaces every invalid range with zero first.
 * That is precisely why the case belongs here and not with the scan handling
 * suite: it asks what the arithmetic does if one ever did, and the answer
 * only counts at the settings that ship. Mistaken for a gap, the ray would be
 * quietly overwritten with the average of its neighbours and the scan would
 * look clean while carrying a reading nothing measured.
 *
 * The companion below runs the same scan with a genuine zero in the same slot
 * so the case cannot pass by the function doing nothing at all.
 */
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

/*
 * The discriminating half of the case above: a genuine zero in the same slot
 * of the same scan must be filled with the average of the two neighbours that
 * bracket it, so the case above cannot be satisfied by a function that never
 * interpolates anything.
 */
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
