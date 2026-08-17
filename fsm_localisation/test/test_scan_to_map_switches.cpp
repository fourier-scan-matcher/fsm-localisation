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
 * @brief The three behaviours ported from the scan-to-map matcher, each in a
 *        case built so that it fires, held against the same case with it off.
 *
 * @details
 * All three default off. A case that merely sets one proves nothing, so every
 * case below carries, beside the comparison, evidence that the switch's own
 * condition was met: the magnification the matcher asked for, the number of
 * positions it put to the map and had refused, and the failure sentinel the
 * translation stage returns. Every one of those is read from what the matcher
 * did, not from a recording of a previous run.
 *
 * **The reference is a seam, not the subject.** Each case hands the matcher a
 * `RecordingRoom`, which satisfies `FSM::GridReference` and answers both of the
 * questions that interface asks by putting them to a ring of points: the same
 * room, the same analytic ranges, and a note of every question asked. It is a
 * grid to the matcher and a polygon to the geometry, which is what makes the
 * evidence available without a grid's cell size getting into the arithmetic.
 * The switches themselves are in `Match::fmt` and `Translation::tff` and know
 * nothing of which kind of reference is in the handle.
 *
 * **What the recorder can see, and why it is enough.**
 *
 * - The rotation stage casts `2^magnification * scan size` rays, read out of
 *   `Rotation::fmt2Batch`, and the translation stage casts the scan's own size.
 *   So the ray counts the recorder is asked for are the magnification schedule
 *   the match walked, which is exactly what `enforce_early_gearup` changes.
 * - `Utils::isPositionInMap` is the only thing that asks whether a position is
 *   free, and the translation stage asks it once per iteration. A refusal is
 *   therefore a position the stage abandoned. Counting refusals separates a
 *   candidate that left the room from a candidate that outran its bound, which
 *   is the difference between the condition already there and the one
 *   `enforce_translation_bound` adds.
 *
 * **Determinism.** The recovery search draws from a thread local engine that
 * `Utils::randomEngine` re-seeds only when it is handed a seed different from
 * the one it currently holds, so two matches run with the same pinned seed in
 * the same process do not draw the same numbers. Every match below is preceded
 * by `pinTheRecoverySearch`, which hands the engine a different seed first, so
 * that the seed each match asks for genuinely re-seeds it. Without that, a case
 * comparing two runs that both recover would be comparing two different random
 * draws and would pass or fail by luck.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "fsm_localisation/fsm_core.hpp"

namespace
{

/* The room every case matches against: a rectangle with one corner cut away,
 * which has no rotational symmetry, so nothing below can match by accident. */
const double kRoomRight = 3.20;
const double kRoomTop = 2.40;
const double kNotchLeft = 2.00;
const double kNotchBottom = 1.60;

const unsigned int kScanSize = 360U;

/* Larger than anything a closed room can return, so no range below is a
 * maximum standing in for a ray that met nothing. */
const double kMaxRange = 100.0;

/* Pinned, so that a recovery draws the same positions on every run. See the
 * note on determinism in the file header. */
const unsigned int kRecoverySeed = 12345U;

/* Handed to the engine before each match so that `kRecoverySeed` is a change of
 * seed and therefore re-seeds. Any value other than `kRecoverySeed` and zero
 * serves; zero is the value that means "draw from hardware entropy". */
const unsigned int kSeedNudge = 999983U;

/* How many trial iterations `Match::fmt` gives each candidate angle during the
 * sift. Read out of `Match::fmt`, where it is `const unsigned int ni = 2`, and
 * named here because one case has to reproduce the sift to show that every
 * candidate in it failed. */
const int kSiftTrialIterations = 2;

/*
 * A room that answers the two questions `FSM::GridReference` asks by putting
 * them to a ring of points, and remembers what it was asked. To the matcher it
 * is a grid; to the geometry it is a polygon.
 *
 * `isFree` is `noexcept`, so what it records is counters rather than a
 * container: a `push_back` there could throw and would take the process with
 * it.
 */
class RecordingRoom : public FSM::GridReference
{
 public:

  explicit RecordingRoom(std::vector<std::pair<double, double>> ring)
  : ring_(std::move(ring))
  {
  }

  unsigned int castCount() const
  {
    return static_cast<unsigned int>(ray_counts_.size());
  }

  /* Every distinct ray count asked for, in increasing order. The rotation stage
   * asks for `2^magnification` times the scan size, so this is the set of
   * magnifications the match visited. */
  std::vector<unsigned int> distinctRayCounts() const
  {
    std::vector<unsigned int> distinct = ray_counts_;
    std::sort(distinct.begin(), distinct.end());
    distinct.erase(std::unique(distinct.begin(), distinct.end()), distinct.end());
    return distinct;
  }

  void forget() const
  {
    ray_counts_.clear();
    refusals_ = 0U;
  }

  bool isFree(const double x, const double y) const noexcept override
  {
    const bool free =
      FSM::Utils::isPositionInMap(FSM::Pose{.x = x, .y = y, .t = 0.0}, ring_);

    if (!free)
      refusals_++;

    return free;
  }

  std::vector<double> ranges(
    const FSM::Pose& pose, const unsigned int num_rays,
    const double max_range) const override
  {
    ray_counts_.push_back(num_rays);

    std::vector<double> found = FSM::Utils::points2scan(
      FSM::X::find(pose, ring_, num_rays, FSM::RaySearch::angular), pose);

    for (double& range : found)
      range = std::min(range, max_range);

    return found;
  }

  /* How many of the positions put to this room were refused as not free. */
  unsigned int refusalCount() const
  {
    return refusals_;
  }

  const std::vector<std::pair<double, double>>& ring() const
  {
    return ring_;
  }

 private:

  const std::vector<std::pair<double, double>> ring_;
  mutable std::vector<unsigned int> ray_counts_;
  mutable unsigned int refusals_{0U};
};

bool areTheSamePose(const FSM::Pose& one, const FSM::Pose& other)
{
  return one.x == other.x && one.y == other.y && one.t == other.t;
}

bool holds(const std::vector<unsigned int>& values, const unsigned int wanted)
{
  return std::find(values.begin(), values.end(), wanted) != values.end();
}

/* Whether every count in `part` also appears in `whole`. */
bool isSubsetOf(
  const std::vector<unsigned int>& part, const std::vector<unsigned int>& whole)
{
  for (const unsigned int value : part)
    if (!holds(whole, value))
      return false;

  return true;
}

/*
 * The matcher's parameters as the package ships them, with the recovery search
 * pinned. The three ported behaviours are deliberately not named here: a case
 * that wants one turns it on itself, and the case that holds the defaults needs
 * this to be a set of parameters that has never mentioned them.
 */
FSM::input_params matcherParameters()
{
  FSM::input_params ip;
  ip.num_iterations = 2;
  ip.xy_bound = 0.2;
  ip.t_bound = M_PI / 4;
  ip.max_counter = 200;
  ip.min_magnification_size = 0;
  ip.max_magnification_size = 3;
  ip.max_recoveries = 10;
  ip.rng_seed = kRecoverySeed;
  ip.ray_search = FSM::RaySearch::angular;
  return ip;
}

void pinTheRecoverySearch()
{
  FSM::Utils::randomEngine(kSeedNudge);
}

std::vector<std::pair<double, double>> roomRing()
{
  return {
    {0.0, 0.0},
    {kRoomRight, 0.0},
    {kRoomRight, kNotchBottom},
    {kNotchLeft, kNotchBottom},
    {kNotchLeft, kRoomTop},
    {0.0, kRoomTop}};
}

FSM::MatchOutput runMatch(
  const RecordingRoom& room, const std::vector<double>& real_scan,
  const FSM::Pose& estimate, const FSM::input_params& ip)
{
  room.forget();
  pinTheRecoverySearch();

  return FSM::Match::fmt(real_scan, estimate, FSM::Reference(room, kMaxRange),
    FSM::DFTUtils::forwardPlan(kScanSize), FSM::DFTUtils::inversePlan(kScanSize),
    ip);
}

/* The scan a sensor standing at `pose` in the room would return. */
std::vector<double> scanFrom(
  const FSM::Pose& pose, const std::vector<std::pair<double, double>>& ring)
{
  return FSM::Utils::points2scan(
    FSM::X::find(pose, ring, kScanSize, FSM::RaySearch::angular), pose);
}

}  // namespace

/*
 * `enforce_early_gearup`. The matcher raises the magnification one level at a
 * time as each level stops paying; with the switch on, the first level-up that
 * happens while the rotation criterion is past 0.99999 goes straight to the
 * maximum instead.
 *
 * What makes it fire is a small displacement the matcher recovers easily, so
 * that the rotation criterion is excellent by the second level. The evidence
 * that it fired is the magnification schedule itself: the rotation stage casts
 * `2^magnification` times the scan's own size, so the set of ray counts the
 * room was asked for is the set of magnifications visited. With the switch on
 * that set must be a proper subset of the set with it off, and both must reach
 * the maximum, which together say the match arrived at the finest level without
 * walking through every level below it.
 */
TEST(ScanToMapSwitches, EarlyGearupSkipsTheMagnificationsInBetween)
{
  const RecordingRoom room(roomRing());

  const FSM::Pose truth{.x = 1.50, .y = 0.80, .t = 0.20};
  const FSM::Pose estimate{.x = 1.53, .y = 0.78, .t = 0.16};

  const std::vector<double> real_scan = scanFrom(truth, room.ring());

  FSM::input_params ip = matcherParameters();

  const FSM::MatchOutput off = runMatch(room, real_scan, estimate, ip);
  const std::vector<unsigned int> counts_off = room.distinctRayCounts();
  const unsigned int casts_off = room.castCount();

  ip.enforce_early_gearup = true;

  const FSM::MatchOutput on = runMatch(room, real_scan, estimate, ip);
  const std::vector<unsigned int> counts_on = room.distinctRayCounts();
  const unsigned int casts_on = room.castCount();

  /* The rotation criterion has to reach the threshold, or nothing can fire. It
   * is reported as `op.rc`, which is the criterion of the last iteration. */
  const double gearup_threshold = 0.99999;
  ASSERT_GT(off.op.rc, gearup_threshold)
    << "this fixture never gets the rotation criterion past " << gearup_threshold
    << ", so the switch has nothing to fire on and the case proves nothing";

  const unsigned int finest =
    (1U << ip.max_magnification_size) * kScanSize;

  ASSERT_TRUE(holds(counts_off, finest))
    << "with the switch off the match must still reach magnification "
    << ip.max_magnification_size << ", which casts " << finest << " rays";

  EXPECT_TRUE(holds(counts_on, finest))
    << "with the switch on the match must arrive at magnification "
    << ip.max_magnification_size << " rather than stop short of it";

  EXPECT_TRUE(isSubsetOf(counts_on, counts_off))
    << "with the switch on the match asked for a ray count it never asked for "
       "with the switch off, which is not what jumping to the maximum does";

  EXPECT_LT(counts_on.size(), counts_off.size())
    << "the match visited the same " << counts_off.size()
    << " magnifications either way, so it did not jump: it climbed";

  EXPECT_LT(casts_on, casts_off)
    << "the point of the jump is arriving sooner, and this match cast "
    << casts_on << " times with the switch on against " << casts_off
    << " with it off";

  EXPECT_FALSE(areTheSamePose(on.pose, off.pose))
    << "skipping the magnifications in between left the answer bit for bit "
       "identical, which means the levels that were skipped were doing nothing";

  /* Both answers are still answers: the switch trades the intermediate levels
   * for speed, not for accuracy, and a jump that lost the pose would be a
   * finding rather than a feature. The bound is the displacement the match was
   * given to recover, which either answer must be well inside. */
  const double displacement = std::hypot(estimate.x - truth.x, estimate.y - truth.y);
  EXPECT_LT(std::hypot(on.pose.x - truth.x, on.pose.y - truth.y), 0.1 * displacement);
  EXPECT_LT(std::hypot(off.pose.x - truth.x, off.pose.y - truth.y), 0.1 * displacement);
}

/*
 * `enforce_translation_bound`. The translation stage already abandons a
 * candidate that has left the room; with the switch on it also abandons one
 * that has moved further than `xy_bound + 0.01` on either axis from where the
 * stage started.
 *
 * What makes it fire is arithmetic rather than luck. The estimate sits 0.03 m
 * from the truth along x, and `xy_bound` is set to 0.005, so the bound is
 * 0.015 m. A candidate cannot reach the truth without moving twice the bound
 * along that axis, so the stage must either be refused or stop short.
 *
 * The evidence that the bound, and not the room, did the refusing is that the
 * room refused nothing: every position either run put to it was free. The room
 * is four metres across and the whole match happens within a few centimetres of
 * its middle, so the out-of-map condition that was there before this switch
 * cannot be what fired.
 */
TEST(ScanToMapSwitches, TheTranslationBoundAbandonsACandidateThatOutrunsIt)
{
  const RecordingRoom room(roomRing());

  const FSM::Pose truth{.x = 1.50, .y = 0.80, .t = 0.20};
  const FSM::Pose estimate{.x = 1.53, .y = 0.78, .t = 0.16};

  const std::vector<double> real_scan = scanFrom(truth, room.ring());

  FSM::input_params ip = matcherParameters();
  ip.xy_bound = 0.005;

  /* The recovery draws an orientation within this of the estimate's. Kept
   * small so that a recovery does not turn the estimate into a different
   * problem, which would make the two runs incomparable. */
  ip.t_bound = 0.02;

  /* The failure recurs at every attempt, so the match ends by exhausting its
   * recoveries. One keeps the case short and makes the count that is expected
   * an arithmetic consequence rather than an observation. */
  ip.max_recoveries = 1;

  const double bound = ip.xy_bound + 0.01;
  const double displacement_along_x = std::abs(estimate.x - truth.x);

  ASSERT_GT(displacement_along_x, bound)
    << "the estimate is " << displacement_along_x
    << " m from the truth along x and the bound is " << bound
    << " m, so the bound has to be crossed for the truth to be reached. It is "
       "not, so this fixture cannot make the switch fire";

  const FSM::MatchOutput off = runMatch(room, real_scan, estimate, ip);
  const unsigned int refusals_off = room.refusalCount();

  ip.enforce_translation_bound = true;

  const FSM::MatchOutput on = runMatch(room, real_scan, estimate, ip);
  const unsigned int refusals_on = room.refusalCount();

  EXPECT_EQ(refusals_off, 0U)
    << "with the switch off the match put " << refusals_off
    << " positions to the room and had them refused, so this fixture is not "
       "isolating the bound from the out-of-map condition that was already there";

  EXPECT_EQ(refusals_on, 0U)
    << "with the switch on the match left the room " << refusals_on
    << " times, so what abandoned the candidate may have been the room rather "
       "than the bound";

  /* With the bound off the stage crosses it and arrives. Well inside a tenth of
   * the displacement it was given to recover. */
  EXPECT_EQ(off.op.num_recoveries, 0U)
    << "with the bound off nothing should have failed, and " << off.op.num_recoveries
    << " recoveries say something did";

  EXPECT_LT(std::hypot(off.pose.x - truth.x, off.pose.y - truth.y),
    0.1 * std::hypot(estimate.x - truth.x, estimate.y - truth.y))
    << "with the bound off this match recovers the displacement, and a case "
       "where it does not has nothing to contrast the bound against";

  /* With the bound on the stage is refused, and the refusal is the sentinel
   * `-2.0` that the recovery reads. It recurs, so the match ends by exhausting
   * its recoveries: one more than it was allowed. */
  EXPECT_DOUBLE_EQ(on.op.tc, -2.0)
    << "the translation criterion the match ends on is " << on.op.tc
    << " rather than the out-of-bounds sentinel, so the bound never refused a "
       "candidate";

  EXPECT_EQ(on.op.num_recoveries, ip.max_recoveries + 1U)
    << "the bound refuses every attempt, so the match must recover until it has "
       "used one more than the " << ip.max_recoveries << " it was allowed";

  EXPECT_FALSE(areTheSamePose(on.pose, off.pose))
    << "the bound refused a candidate and the answer did not move, which cannot "
       "both be true";

  EXPECT_GT(std::hypot(on.pose.x - truth.x, on.pose.y - truth.y),
    std::hypot(off.pose.x - truth.x, off.pose.y - truth.y))
    << "abandoning a candidate that outran the bound left the match nearer the "
       "truth than following it did, which is not what abandoning it means";
}

/*
 * `recover_on_failed_sift`. Every candidate angle is given a trial translation
 * of two iterations and the best is adopted. When every one of them fails, the
 * best is a failure, and with the switch on the matcher recovers instead of
 * adopting a failed candidate's angle and running a full translation stage from
 * it.
 *
 * What makes it fire is an estimate standing outside the room, which is the
 * condition the switch was written for: a pose the map cannot explain. The
 * evidence that every candidate failed is not taken on trust. The case runs the
 * sift itself, with the same rotation stage, the same candidate list including
 * the carried-over best angle, and the same two trial iterations `Match::fmt`
 * uses, and asserts that every one of them comes back with the `-2.0` sentinel.
 *
 * What differs is the work: with the switch off the matcher adopts the failed
 * angle and runs one more translation stage, which is refused by the room once
 * more before returning the same sentinel. So the run with the switch off puts
 * exactly one more position to the room and has it refused. The returned pose
 * does not differ, and that is a property of the algorithm rather than of this
 * fixture: the recovery replaces the pose with a fresh draw around the original
 * estimate, which discards whatever the failed candidate did to it, and both
 * runs then continue from the same drawn pose.
 */
TEST(ScanToMapSwitches, ARecoveryReplacesTheAngleOfACandidateThatFailed)
{
  const RecordingRoom room(roomRing());

  const FSM::Pose truth{.x = 1.50, .y = 0.80, .t = 0.20};

  /* Outside the room, past the right hand wall, and inside `xy_bound` of it so
   * that the recovery can find free space to draw. A recovery that cannot is
   * the case this switch reads an empty criterion history in, and it is not
   * exercised here. */
  const FSM::Pose estimate{.x = kRoomRight + 0.30, .y = 0.80, .t = 0.20};

  const std::vector<double> real_scan = scanFrom(truth, room.ring());

  FSM::input_params ip = matcherParameters();
  ip.xy_bound = 0.5;
  ip.max_recoveries = 1;

  ASSERT_FALSE(FSM::Utils::isPositionInMap(estimate, room.ring()))
    << "the estimate is inside the room, so no candidate has any reason to fail "
       "and the switch cannot fire";

  /*
   * The sift, run here exactly as `Match::fmt` runs it: the rotation stage's
   * angles, plus the carried-over best angle, which starts at zero and is
   * appended when it is not already among them, each given a trial translation
   * of `kSiftTrialIterations` iterations without the bound.
   */
  const FSM::RotationOutput rotation = FSM::Rotation::fmt(real_scan, estimate,
    FSM::Reference(room, kMaxRange), ip.min_magnification_size, "batch",
    FSM::DFTUtils::forwardPlan(kScanSize), FSM::DFTUtils::inversePlan(kScanSize),
    ip.ray_search);

  std::vector<double> candidate_angles = rotation.angles;
  if (!std::any_of(candidate_angles.begin(), candidate_angles.end(),
        [](const double angle) { return angle == 0.0; }))
    candidate_angles.push_back(0.0);

  ASSERT_GT(candidate_angles.size(), 1U)
    << "the sift only runs when there is more than one candidate angle, and "
       "this fixture offers " << candidate_angles.size();

  for (const double angle : candidate_angles)
  {
    FSM::Pose candidate = estimate;
    candidate.t += angle;

    const FSM::TranslationOutput trial = FSM::Translation::tff(real_scan,
      candidate, FSM::Reference(room, kMaxRange), kSiftTrialIterations,
      ip.xy_bound, false, FSM::DFTUtils::forwardPlan(kScanSize), ip.ray_search,
      false);

    ASSERT_DOUBLE_EQ(trial.criterion, -2.0)
      << "the candidate at " << angle
      << " rad survives its trial translation with a criterion of "
      << trial.criterion << ", so not every candidate fails and the switch's "
         "own condition is not met by this fixture";
  }

  const FSM::MatchOutput off = runMatch(room, real_scan, estimate, ip);
  const unsigned int refusals_off = room.refusalCount();

  ip.recover_on_failed_sift = true;

  const FSM::MatchOutput on = runMatch(room, real_scan, estimate, ip);
  const unsigned int refusals_on = room.refusalCount();

  /*
   * Each candidate's trial translation is refused once, in both runs. With the
   * switch off the matcher then adopts the failed angle and runs a translation
   * stage from it, which is refused once more before it returns the sentinel;
   * with the switch on that stage never runs. Every other refusal in either run
   * belongs to the recovery's own draws, and those are the same draws in both,
   * so the difference between the two counts is exactly that one stage.
   */
  EXPECT_EQ(refusals_off, refusals_on + 1U)
    << "with the switch off the match had " << refusals_off
    << " positions refused and with it on " << refusals_on
    << ". One more with it off is the translation stage the matcher runs from "
       "the angle of a candidate it already knows failed, and a difference of "
       "anything else means that stage either still ran or never did";

  EXPECT_GT(refusals_on, 0U)
    << "no position was refused at all, so nothing failed and neither run can "
       "have taken the path this case is about";

  /* Both runs recover, because a sift every candidate failed is followed by a
   * translation stage that fails for the same reason. The switch changes when
   * the recovery happens, not whether it does. */
  EXPECT_EQ(on.op.num_recoveries, off.op.num_recoveries)
    << "the switch changed how many recoveries the match needed, which it does "
       "not do: the stage it skips was going to fail anyway";

  EXPECT_GT(on.op.num_recoveries, 0U)
    << "neither run recovered, so the failed sift went unnoticed by both";
}

/*
 * The one thing every ported behaviour has to be true of: with all three off,
 * the matcher is the one that was there before they existed. Parameters left
 * alone and parameters set to false must give the same answer, and the same
 * answer means bit for bit, not to a tolerance, since nothing between the two
 * runs is allowed to differ at all.
 *
 * This is the cheap half of the guarantee. The other half is the golden
 * comparison, which holds the whole matcher against numbers recorded before any
 * of this existed and is not something a unit test can reproduce.
 */
TEST(ScanToMapSwitches, WithAllThreeOffTheMatchIsTheOneThatWasThereBefore)
{
  const RecordingRoom room(roomRing());

  const FSM::Pose truth{.x = 1.50, .y = 0.80, .t = 0.20};
  const FSM::Pose estimate{.x = 1.53, .y = 0.78, .t = 0.16};

  const std::vector<double> real_scan = scanFrom(truth, room.ring());

  /* Never mentions the three. */
  const FSM::input_params untouched = matcherParameters();

  FSM::input_params explicitly_off = matcherParameters();
  explicitly_off.enforce_early_gearup = false;
  explicitly_off.enforce_translation_bound = false;
  explicitly_off.recover_on_failed_sift = false;

  ASSERT_FALSE(untouched.enforce_early_gearup)
    << "the ported behaviours must default off, and this one does not";
  ASSERT_FALSE(untouched.enforce_translation_bound)
    << "the ported behaviours must default off, and this one does not";
  ASSERT_FALSE(untouched.recover_on_failed_sift)
    << "the ported behaviours must default off, and this one does not";

  const FSM::MatchOutput left_alone = runMatch(room, real_scan, estimate, untouched);
  const unsigned int casts_left_alone = room.castCount();

  const FSM::MatchOutput set_off =
    runMatch(room, real_scan, estimate, explicitly_off);
  const unsigned int casts_set_off = room.castCount();

  EXPECT_EQ(left_alone.pose.x, set_off.pose.x)
    << "turning the ported behaviours off moved the answer, so one of them is "
       "not off when it is off";
  EXPECT_EQ(left_alone.pose.y, set_off.pose.y);
  EXPECT_EQ(left_alone.pose.t, set_off.pose.t);

  EXPECT_EQ(left_alone.op.rc, set_off.op.rc);
  EXPECT_EQ(left_alone.op.tc, set_off.op.tc);
  EXPECT_EQ(left_alone.op.num_recoveries, set_off.op.num_recoveries);

  EXPECT_EQ(casts_left_alone, casts_set_off)
    << "the two matches did not even do the same amount of work, which they "
       "must, whatever they arrived at";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
