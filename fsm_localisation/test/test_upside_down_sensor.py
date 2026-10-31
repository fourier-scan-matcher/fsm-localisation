# Copyright 2026 Alexandros Filotheou
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
A laser mounted upside down, which only the transform tree says it is.

A laser hung beneath a plate is rolled half a turn about its forward axis.
Forward stays forward and left becomes right, so the ray such a laser calls a
points where an upright one would call -a, and the scan it reports is the
mirror image of the scan an upright laser in its place would report. Nothing
in the scan says so and no setting of this node does. The rotation of the
laser's frame in the transform tree is the only place it is written, so a
node that keeps the heading of that rotation and drops the rest takes the
laser for an upright one and corrects against the mirror image of what was
seen.

**One scan, two mountings.** The room is scanned once from where the laser
really is. That scan is handed in as an upright laser reports it, in a frame
mounted upright, and then as the same laser reports it upside down, in a
frame mounted with the same translation and heading and half a turn of roll.
The robot stands in the same place and the estimate is pushed off it by the
same amount both times, so both answers are held to the same pose and to each
other. The upright case is the control. Without it a failure of the other
could be the fixture's and not the mounting's.

**The placement is the one every correcting mode file uses.** It is shifted
forward and turned, so a node that lost the heading of the rolled frame alone
would answer a tenth of a radian off, and one that lost its translation alone
would answer a fifth of a metre off.

**Why a mirrored scan cannot pass here by luck.** The room is the rectangle
of the other correcting mode files with one pillar added. A rectangle is its
own mirror image about either of its axes, but the pillar stands on neither,
so the mirror image of the room is a different room. The pillar stands to the
laser's right and nothing stands where its mirror image would, to the left.

**Two layouts of the same turn.** The first scan begins at minus pi and ends
a ray short of pi. It is not symmetric about nought, so reversing its ranges
is not its mirror image but the mirror turned by one ray. The second begins
half a ray later and is symmetric about nought, so there reversing its ranges
is the mirror, and pairing ray i with ray size - i, the first scan's mirror,
is one ray out. A repair exact for one layout alone fails the other. The node
takes both, since each begins within a ray of half a turn behind the heading.
The case comparing the first pair's answers says what that means for the
bound.

**The map scan of the laser mounted upside down is the upright one
mirrored.** With each answer the node publishes a cast from the map in the
scan's own frame, one ray along each ray of the scan. For the laser mounted
upside down its rays have to run the way that laser's do, or it is drawn as
the mirror image of what the laser saw.

**A laser that sees less than a turn.** A third pair reports 271 of the
turn's rays, from 135 degrees right of the heading to 135 degrees left of it,
the way a real laser reports a narrower scan. The map scan of each has to lie
over the scan sent, ray for ray.
"""

import math
import unittest

from fsm_amelioration_test_support import (
    AmeliorationHarness,
    blocked_cells,
    displaced,
    heading_tolerance,
    RESOLUTION,
    SENSOR_FRAME,
    SENSOR_ON_ROBOT,
    TRUE_ROBOT_POSE,
    true_sensor_pose,
    upside_down,
    wrap,
    yaw_of,
)
from fsm_localisation_test_support import room_scan, SCAN_SIZE
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy

POSITION_TOLERANCE = 2.0 * RESOLUTION
MIRROR_TOLERANCE = 0.01 * RESOLUTION
OVERLAY_TOLERANCE = 0.7 * RESOLUTION

PILLAR = (4.80, 1.10, 5.20, 1.50)
PUSH = (0.10, -0.08, 0.12)
SYMMETRIC_HEADING_BOUND = math.radians(0.25)

SYMMETRIC_FIRST_ANGLE = -math.pi + math.pi / SCAN_SIZE

NARROW_RAYS = 271
NARROW_FIRST_ANGLE = -math.pi * (NARROW_RAYS - 1) / SCAN_SIZE

UPSIDE_DOWN = math.pi
UPSIDE_DOWN_SENSOR_FRAME = 'upside_down_laser_link'

FIRST = (1700000000, 0)
SECOND = (1700000000, 200000000)


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='fsm_localisation',
        executable='fsm_localisation_interface_node',
        name='fsm_localisation',
        output='screen',
        parameters=[{'mode': 'amelioration'}],
    )
    return (
        launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]),
        {'fsm_localisation': node},
    )


def mean_gap(ranges, reference):
    """Return how far apart two scans are, ray for ray, on average."""
    return (sum(abs(cast - expected)
                for cast, expected in zip(ranges, reference))
            / len(reference))


class TestUpsideDownSensor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.ranges = room_scan(*true_sensor_pose(), pillar=PILLAR)
        cls.symmetric_ranges = room_scan(*true_sensor_pose(), pillar=PILLAR,
                                         first_angle=SYMMETRIC_FIRST_ANGLE)
        narrow_turn = room_scan(*true_sensor_pose(), pillar=PILLAR,
                                first_angle=NARROW_FIRST_ANGLE)
        cls.narrow_ranges = narrow_turn[:NARROW_RAYS]
        cls.narrow_upside_down_ranges = upside_down(
            narrow_turn, NARROW_FIRST_ANGLE)[:NARROW_RAYS]

        cls.harness = AmeliorationHarness()
        cls.harness.wait_for_connection()
        cls.harness.publish_sensor_placement(
            mountings=((SENSOR_FRAME, 0.0),
                       (UPSIDE_DOWN_SENSOR_FRAME, UPSIDE_DOWN)))
        cls.harness.publish_map(FIRST, blocked_cells(PILLAR))
        cls.harness.spin(1.0)

        cls.outcomes = {
            'upright': cls.correct(cls.ranges, SENSOR_FRAME),
            'upside down': cls.correct(upside_down(cls.ranges),
                                       UPSIDE_DOWN_SENSOR_FRAME),
            'upright, symmetric': cls.correct(
                cls.symmetric_ranges, SENSOR_FRAME, SYMMETRIC_FIRST_ANGLE),
            'upside down, symmetric': cls.correct(
                upside_down(cls.symmetric_ranges, SYMMETRIC_FIRST_ANGLE),
                UPSIDE_DOWN_SENSOR_FRAME, SYMMETRIC_FIRST_ANGLE),
            'upright, narrow': cls.correct(
                cls.narrow_ranges, SENSOR_FRAME, NARROW_FIRST_ANGLE),
            'upside down, narrow': cls.correct(
                cls.narrow_upside_down_ranges, UPSIDE_DOWN_SENSOR_FRAME,
                NARROW_FIRST_ANGLE),
        }

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    @classmethod
    def correct(cls, ranges, frame_id, first_angle=-math.pi):
        """
        Correct the pushed estimate once and return what the node answered.

        That is the response, the poses published and the map scans published.

        The scan and the estimate reach the node while it is stopped, so it
        holds them and corrects nothing until asked. Started, it is asked once
        and stopped again, so each mounting is corrected exactly once.
        """
        cls.harness.clear()
        cls.harness.publish_scan(FIRST, ranges=ranges, frame_id=frame_id,
                                 first_angle=first_angle)
        cls.harness.spin(0.5)
        cls.harness.publish_estimate(SECOND, displaced(PUSH))
        cls.harness.spin(0.5)

        cls.harness.call('start')
        response = cls.harness.call('correct_once')
        cls.harness.spin(1.0)
        cls.harness.call('stop')

        return (response, list(cls.harness.corrected),
                list(cls.harness.map_scans))

    def answer(self, mounting):
        """Return the one robot pose corrected for a mounting, as x, y, yaw."""
        response, corrected, _ = self.outcomes[mounting]

        self.assertTrue(response.success, response.message)
        self.assertEqual(len(corrected), 1,
                         'the call published %d poses where one was asked for'
                         % len(corrected))

        pose = corrected[0].pose.pose
        return (pose.position.x, pose.position.y, yaw_of(pose.orientation))

    def assert_laid_over(self, mounting, measured):
        """Require the map scan for a mounting to lie over the scan sent."""
        response, _, casts = self.outcomes[mounting]

        self.assertTrue(response.success, response.message)
        self.assertEqual(len(casts), 1,
                         'the call for "%s" published %d map scans where one '
                         'was asked for' % (mounting, len(casts)))

        published = casts[0]
        overlay = mean_gap(published.ranges, measured)

        message = (
            'the map scan for "%s" carries %d rays and the scan sent carries '
            '%d. Ray for ray the two disagree by %.4f m on average, against a '
            'bound of %.4f m'
            % (mounting, len(published.ranges), len(measured), overlay,
               OVERLAY_TOLERANCE))

        self.assertEqual(len(published.ranges), len(measured), message)
        self.assertLessEqual(overlay, OVERLAY_TOLERANCE, message)

    def assert_put_where_the_upright_one_is(self, upright_mounting,
                                            inverted_mounting, layout):
        """
        Hold one answer to another within half a ray.

        The second case says why the bound is what it is.
        """
        upright = self.answer(upright_mounting)
        inverted = self.answer(inverted_mounting)

        heading_bound = math.pi / len(self.ranges)
        position_bound = (math.hypot(SENSOR_ON_ROBOT[0], SENSOR_ON_ROBOT[1])
                          * heading_bound)

        position_difference = math.hypot(inverted[0] - upright[0],
                                         inverted[1] - upright[1])
        heading_difference = abs(wrap(inverted[2] - upright[2]))

        message = (
            'on the scan %s the laser mounted upside down put the robot at '
            '(%.4f, %.4f, %.4f) and the one mounted upright at (%.4f, %.4f, '
            '%.4f), %.4f m and %.3f degrees apart. The robot stands at (%.4f, '
            '%.4f, %.4f). The bound is %.4f m and %.3f degrees, half of what '
            'turning the robot about the laser by one ray moves it'
            % ((layout,) + inverted + upright
               + (position_difference, math.degrees(heading_difference))
               + TRUE_ROBOT_POSE
               + (position_bound, math.degrees(heading_bound))))

        self.assertLessEqual(position_difference, position_bound, message)
        self.assertLessEqual(heading_difference, heading_bound, message)

    def test_1_a_sensor_mounted_upright_is_put_back(self):
        """
        Put the robot back where it stands, to a cell or two.

        The bound is the one the other correcting mode files hold an answer
        to: two cells of position, and that distance seen at the ranges this
        scan carries in heading.
        """
        found = self.answer('upright')
        heading_bound = heading_tolerance(self.ranges)

        message = (
            'a laser mounted upright put the robot at (%.4f, %.4f, %.4f) where '
            'it stands at (%.4f, %.4f, %.4f). The bound is %g m on each axis '
            'and %.2f degrees'
            % (found + TRUE_ROBOT_POSE
               + (POSITION_TOLERANCE, math.degrees(heading_bound))))

        self.assertAlmostEqual(found[0], TRUE_ROBOT_POSE[0],
                               delta=POSITION_TOLERANCE, msg=message)
        self.assertAlmostEqual(found[1], TRUE_ROBOT_POSE[1],
                               delta=POSITION_TOLERANCE, msg=message)
        self.assertAlmostEqual(wrap(found[2] - TRUE_ROBOT_POSE[2]), 0.0,
                               delta=heading_bound, msg=message)

    def test_2_a_sensor_mounted_upside_down_is_put_where_the_upright_one_is(
            self):
        """
        Hold the upside down answer to the upright one, within half a ray.

        A node that keeps the heading of the laser's frame alone reads this
        scan as an upright laser's, which is the mirror image of the room the
        scan was taken in, and answers somewhere else or not at all.

        The bound is half a ray of heading, and in position half of what
        turning the robot about the laser by one ray moves it. A repair that
        reverses the ranges and stops there hands the node the scan turned by
        one ray, which answers a ray of heading away and is off in position by
        the placement's reach swung through one ray. That answer is inside
        the control's own bound, so it is this case that catches it.

        Measured at this placement before the repair, three runs alike: the
        node answered 0.3004 m and 44.25 degrees from the control. The ranges
        a plain reversal hands the node answer 0.0036 m and 1.00 degree from
        the control, and 0.006 m and 1.1 degrees from where the robot stands,
        inside the control's two cells and 1.7 degrees. A placement nudged by
        a billionth of a radian answers 1e-8 m from the control. The bound
        comes to 0.0018 m and 0.5 degrees.
        """
        self.assert_put_where_the_upright_one_is(
            'upright', 'upside down', 'beginning at minus pi')

    def test_3_the_map_scan_upside_down_is_the_upright_one_mirrored(self):
        """
        Lay the upside down cast over its scan and over the upright one.

        The cast is published in the frame the scan names, so its ray j has to
        lie along the rolled laser's ray j, which points where the upright
        laser's ray size - j does, ray 0 being its own. Laid over the scan
        sent, it has to be the room seen from the answer. Laid over the upright
        cast mirrored, its rays have to run the way the rolled laser's do: the
        two answers stand 3e-7 m apart at the same heading, so mirrored, the
        two casts are all but the same rays. A fault common to both mountings,
        such as a cast turned half a turn or by a single ray, leaves the two
        casts mirror images of each other, so it is the scan sent that catches
        it.

        Against the scan sent the bound is seven tenths of a cell a ray on
        average, 0.035 m, between the right cast and one turned by a ray in
        both mountings. Measured on this fixture, three runs alike: the cast
        disagrees by 0.0249 m a ray, and turned by a ray by 0.0544 m, which a
        bound of two cells let through. Published in the order an upright
        laser's rays run it disagreed by 0.8775 m, and before the answer
        itself was repaired by 0.3179 m.

        Against the upright cast mirrored the bound is a hundredth of a cell a
        ray on average, 0.0005 m, room for three rays of 360 to stop a cell
        apart. Measured on this fixture: the two disagree by 8.9e-8 m a ray,
        none by more than 1e-6 m. A cast reversed from the rolled laser's
        heading rather than from the upright laser its scan was handed over as
        is one ray out of turn and disagrees by 0.0432 m. Against the scan
        sent that cast disagrees by 0.0490 m, over the bound there but by a far
        narrower margin than here. Published in the order an upright laser's
        rays run, the cast disagrees by 0.8786 m, and before the answer itself
        was repaired by 0.3209 m.
        """
        response, _, casts = self.outcomes['upside down']
        _, _, upright_casts = self.outcomes['upright']

        self.assertTrue(response.success, response.message)
        for mounting, published in (('upright', upright_casts),
                                    ('upside down', casts)):
            self.assertEqual(len(published), 1,
                             'the call for the laser mounted %s published %d '
                             'map scans where one was asked for'
                             % (mounting, len(published)))

        published = casts[0]
        mirrored = upside_down(upright_casts[0].ranges)
        overlay = mean_gap(published.ranges, upside_down(self.ranges))
        disagreement = mean_gap(published.ranges, mirrored)

        message = (
            'the map scan of the laser mounted upside down carries %d rays in '
            '"%s", and the upright one mirrored carries %d. Ray for ray it '
            'disagrees with the scan sent by %.4f m on average, against a '
            'bound of %.4f m, and with the upright one mirrored by %.6f m, '
            'against a bound of %.6f m'
            % (len(published.ranges), published.header.frame_id,
               len(mirrored), overlay, OVERLAY_TOLERANCE, disagreement,
               MIRROR_TOLERANCE))

        self.assertEqual(published.header.frame_id, UPSIDE_DOWN_SENSOR_FRAME,
                         message)
        self.assertEqual(len(published.ranges), len(mirrored), message)
        self.assertLessEqual(overlay, OVERLAY_TOLERANCE, message)
        self.assertLessEqual(disagreement, MIRROR_TOLERANCE, message)

    def test_4_a_scan_symmetric_about_nought_is_put_where_the_upright_one_is(
            self):
        """
        Hold the two answers to each other on a scan symmetric about nought.

        On the first layout the sum of the scan's first and last angles is
        minus one ray. A repair that turned the reversed scan by one ray
        whatever the scan, or that paired ray i with ray size - i as that
        layout's mirror does, passes the second case and answers a ray of
        heading away here, where the sum is nought.

        The bound is the second case's. Measured on this fixture: the two
        answers stand 6.8e-8 m apart at the same heading. A node pairing ray i
        with ray size - i, and one turning the reversed scan by one ray
        whatever the scan, both answer 0.0036 m and 1.000 degree from the
        upright one here, and pass the second case. Those two were measured
        before the matcher laid the map out from the scan's first ray. The
        pair is held to each other here and to the truth in the case below.
        """
        self.assert_put_where_the_upright_one_is(
            'upright, symmetric', 'upside down, symmetric',
            'symmetric about nought')

    def test_5_the_map_scan_of_a_narrower_sensor_lies_over_its_scan(self):
        """
        Lay the cast over the scan of a laser that sees less than a turn.

        A cast spread over a whole turn from half a turn behind the heading,
        published at the scan's own angles, is drawn turned and squeezed over
        a scan of less than a turn. The bound is the third case's against the
        scan sent, 0.035 m a ray on average. Measured on this fixture, three
        runs alike: the cast disagrees with the scan by 0.0257 m a ray, and
        turned by a ray by 0.0615 m. Spread over a whole turn it disagreed by
        0.5602 m, in one run.
        """
        self.assert_laid_over('upright, narrow', self.narrow_ranges)

    def test_6_the_map_scan_of_a_narrower_sensor_upside_down_lies_over_it(
            self):
        """
        Lay the cast over the scan of a narrower laser mounted upside down.

        On this layout, symmetric about nought, the mirror of a scan is its
        reversal, so the cast reversed is held to the upright laser's cast as
        the third case holds the first layout's, and the bounds are that
        case's. Measured on this fixture, three runs alike: the cast disagrees
        with the scan by 0.0257 m a ray, and with the upright cast reversed by
        4.4e-8 m, none by more than 1e-6 m. Spread over a whole turn it
        disagreed with the scan by 0.5602 m, in one run. Cast ray by ray but
        published in the order an upright laser's rays run, it disagrees by
        1.0216 m with the scan and 1.0240 m with the upright cast reversed,
        three runs alike. A cast taken from the rolled laser's heading rather
        than from the upright laser its scan was handed over as is the same
        cast here, 0.0257 m and nought, three runs alike, since on this layout
        the two headings coincide. Only the third case catches it. A cast
        turned by a ray in both mountings disagrees with the scan by 0.0615 m,
        three runs alike.
        """
        self.assert_laid_over('upside down, narrow',
                              self.narrow_upside_down_ranges)

        _, _, upright_casts = self.outcomes['upright, narrow']
        _, _, casts = self.outcomes['upside down, narrow']

        self.assertEqual(len(upright_casts), 1,
                         'the call for "upright, narrow" published %d map '
                         'scans where one was asked for' % len(upright_casts))

        disagreement = mean_gap(casts[0].ranges,
                                list(reversed(upright_casts[0].ranges)))

        self.assertLessEqual(
            disagreement, MIRROR_TOLERANCE,
            'the map scan for "upside down, narrow" disagrees with the one for '
            '"upright, narrow" reversed by %.6f m a ray on average, against a '
            'bound of %.6f m' % (disagreement, MIRROR_TOLERANCE))

    def test_7_a_scan_symmetric_about_nought_is_put_where_the_robot_stands(
            self):
        """
        Hold the upright answer on a scan symmetric about nought to the truth.

        The answer may not depend on where the scan's first ray sits. This
        scan begins half a ray after the one of the first case, and the
        matcher has to read it as that.

        Measured on this fixture, from where the robot stands: 0.500 degrees
        of heading on this scan before the repair and 0.0005 degrees after,
        against 0.125 degrees on the scan beginning at minus pi before it. The
        bound is a quarter of a degree, between the two.
        """
        found = self.answer('upright, symmetric')

        heading_difference = abs(wrap(found[2] - TRUE_ROBOT_POSE[2]))

        message = (
            'on the scan symmetric about nought a laser mounted upright put '
            'the robot at (%.4f, %.4f, %.4f) where it stands at (%.4f, %.4f, '
            '%.4f), %.3f degrees of heading away. The bound is %.3f degrees'
            % (found + TRUE_ROBOT_POSE
               + (math.degrees(heading_difference),
                  math.degrees(SYMMETRIC_HEADING_BOUND))))

        self.assertLessEqual(heading_difference, SYMMETRIC_HEADING_BOUND,
                             message)
