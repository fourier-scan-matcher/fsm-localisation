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
 * @brief `FSM::Reference`, the handle every odometry call site was widened to
 * carry so that the ring path could stay untouched.
 * @details
 * Nothing else in this package tests the handle directly. The claim that
 * threading it through ten call sites left the odometry path unchanged rests
 * on it staying a small, trivially copyable value: register pressure in the
 * arithmetic carrying functions is what the golden fixtures indirectly pin,
 * and a handle that grew another member or stopped being trivially copyable
 * would perturb that with nothing here to say so first. This file is that
 * something.
 *
 * It has done its job twice. The handle grew a fourth member, the aperture the
 * grid is to be read through, and then a fifth, the turn that lays the window
 * out from the scan's first ray, and this file is what said so each time. The
 * bound below moved from three members to four, 24 to 32 bytes, and from four
 * to five, 32 to 40 bytes, deliberately, and the golden fixtures were run
 * afterwards to confirm the arithmetic path did not move with it. The bound is
 * not to be raised again to make a change fit. It is there to make growth a
 * decision.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>

#include "fsm_localisation/fsm_core.hpp"

namespace
{
class StubGrid : public FSM::GridReference
{
 public:

  bool isFree(const double, const double) const noexcept override
  {
    return true;
  }

  std::vector<double> ranges(
    const FSM::Pose&, const unsigned int, const double) const override
  {
    return {};
  }
};

}

TEST(ReferenceHandle, IsTriviallyCopyable)
{
  EXPECT_TRUE(std::is_trivially_copyable_v<FSM::Reference>);
}

TEST(ReferenceHandle, IsSmallEnoughToPassCheaplyByValue)
{
  EXPECT_LE(sizeof(FSM::Reference), 40U);
}

TEST(ReferenceHandle, EveryOperationIsNoexcept)
{
  const std::vector<std::pair<double, double>> ring;
  const StubGrid grid;

  static_assert(noexcept(FSM::Reference{}));
  static_assert(noexcept(FSM::Reference(ring)));
  static_assert(noexcept(FSM::Reference(grid, 0.0)));
  static_assert(noexcept(FSM::Reference(grid, 0.0, 2.0 * M_PI)));
  static_assert(noexcept(std::declval<const FSM::Reference&>().aperture()));
  static_assert(noexcept(std::declval<const FSM::Reference&>().isRing()));
  static_assert(noexcept(std::declval<const FSM::Reference&>().isGrid()));
  static_assert(noexcept(std::declval<const FSM::Reference&>().ring()));
  static_assert(noexcept(std::declval<const FSM::Reference&>().grid()));
  static_assert(noexcept(std::declval<const FSM::Reference&>().maxRange()));
  static_assert(std::is_nothrow_copy_constructible_v<FSM::Reference>);
  static_assert(std::is_nothrow_move_constructible_v<FSM::Reference>);
}

TEST(ReferenceHandle, ADefaultConstructedHandleIsNeitherARingNorAGrid)
{
  const FSM::Reference reference;

  EXPECT_FALSE(reference.isRing());
  EXPECT_FALSE(reference.isGrid());
}

TEST(ReferenceHandle, AHandleHoldingARingReportsRingAndReturnsTheSameRing)
{
  const std::vector<std::pair<double, double>> ring{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  const FSM::Reference reference(ring);

  EXPECT_TRUE(reference.isRing());
  EXPECT_FALSE(reference.isGrid());
  EXPECT_EQ(&reference.ring(), &ring)
    << "the handle must borrow the ring it was given rather than a copy of "
       "it, since it is not allowed to outlive what it points at";
}

/**
 * @brief A grid handle carries the aperture it is to be read through, and a
 * caller that says nothing gets a whole turn.
 * @details The whole turn default is what keeps every ring era call site
 * correct without being touched. It is safe here and is not safe one layer up:
 * `Corrector::correct` takes the aperture with no default at all, so a caller
 * with a narrower sensor cannot stay silent and be answered wrongly.
 */
TEST(ReferenceHandle, AGridHandleCarriesItsApertureAndDefaultsToAWholeTurn)
{
  const StubGrid grid;

  EXPECT_DOUBLE_EQ(FSM::Reference(grid, 10.0).aperture(), 2.0 * M_PI);
  EXPECT_DOUBLE_EQ(FSM::Reference(grid, 10.0, 1.5 * M_PI).aperture(),
    1.5 * M_PI);
}

TEST(ReferenceHandle, AHandleHoldingAGridReportsGridAndReturnsTheSameGridAndMaxRange)
{
  const StubGrid grid;
  const double max_range = 12.5;
  const FSM::Reference reference(grid, max_range);

  EXPECT_TRUE(reference.isGrid());
  EXPECT_FALSE(reference.isRing());
  EXPECT_EQ(&reference.grid(), &grid)
    << "the handle must borrow the grid it was given rather than a copy of "
       "it, since it is not allowed to outlive what it points at";
  EXPECT_DOUBLE_EQ(reference.maxRange(), max_range);
}

TEST(ReferenceHandle, TheRingConstructorConvertsImplicitly)
{
  const std::vector<std::pair<double, double>> ring{{0.0, 0.0}, {2.0, 0.0}};

  auto holdsARing = [](const FSM::Reference reference) { return reference.isRing(); };

  EXPECT_TRUE(holdsARing(ring))
    << "the ring constructor is deliberately not explicit, so that every "
       "existing call site taking a ring converts with no edit needed there";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
