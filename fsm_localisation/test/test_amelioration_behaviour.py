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
A running node in the correcting mode: what it recovers, and what it refuses.

The room is handed to the node as an occupancy grid whose free interior is
exactly the rectangle the scans are computed from, so no cell straddles a wall
and the grid is the room rather than an approximation of it. A pose estimate is
then pushed off the truth by a known amount and the node is required to put it
back.

**What the answer is held to.** Two cells of position. A cast ray stops at a
cell, so one cell is what the map itself can support; the second is because the
scan handed in is computed in closed form while the scan the node compares it
against is cast into the raster, and those two can differ by a cell of their
own. The heading is held to that same distance seen at the ranges the scan
actually carries, computed from the scan rather than chosen.

**The laser is not at the robot's centre and is turned with respect to it**, so
a node that confused the two poses, or dropped the placement altogether, would
fail every case here rather than pass by coincidence.
"""

import math
import unittest

from fsm_amelioration_test_support import (
    AmeliorationHarness,
    displaced,
    GLOBAL_FRAME,
    heading_tolerance,
    RESOLUTION,
    TRUE_ROBOT_POSE,
    true_sensor_pose,
    wrap,
    yaw_of,
)
from fsm_localisation_test_support import room_scan
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy

POSITION_TOLERANCE = 2.0 * RESOLUTION

FIRST = (1700000000, 0)
SECOND = (1700000000, 200000000)
THIRD = (1700000000, 400000000)
FOURTH = (1700000000, 600000000)
FIFTH = (1700000000, 800000000)
BEFORE_ZERO = (-1, 0)


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


class TestAmeliorationBehaviour(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = AmeliorationHarness()
        self.harness.wait_for_connection()
        self.harness.publish_sensor_placement()
        self.harness.publish_map(FIRST)
        self.harness.spin(1.0)

    def tearDown(self):
        self.harness.call('stop')
        self.harness.destroy_node()

    def correct(self, displacement, stamp=SECOND, scan_stamp=FIRST):
        """Push the estimate off by displacement and return what came back."""
        self.harness.call('start')
        self.harness.clear()
        self.harness.publish_scan(scan_stamp)
        self.harness.spin(0.5)
        self.harness.publish_estimate(stamp, displaced(displacement))
        self.harness.spin(3.0)
        return self.harness.corrected

    def assert_recovered(self, corrected, ranges):
        """Assert the corrected robot pose is the true one, to a cell or two."""
        self.assertEqual(len(corrected), 1,
                         'expected exactly one corrected pose, got %d'
                         % len(corrected))

        pose = corrected[0].pose.pose

        self.assertAlmostEqual(pose.position.x, TRUE_ROBOT_POSE[0],
                               delta=POSITION_TOLERANCE)
        self.assertAlmostEqual(pose.position.y, TRUE_ROBOT_POSE[1],
                               delta=POSITION_TOLERANCE)

        self.assertAlmostEqual(
            wrap(yaw_of(pose.orientation) - TRUE_ROBOT_POSE[2]), 0.0,
            delta=heading_tolerance(ranges))

    def test_an_estimate_displaced_in_position_is_put_back(self):
        ranges = room_scan(*true_sensor_pose())
        self.assert_recovered(self.correct((0.12, -0.09, 0.0)), ranges)

    def test_an_estimate_displaced_in_heading_is_put_back(self):
        ranges = room_scan(*true_sensor_pose())
        self.assert_recovered(self.correct((0.0, 0.0, 0.15)), ranges)

    def test_an_estimate_displaced_in_both_is_put_back(self):
        ranges = room_scan(*true_sensor_pose())
        self.assert_recovered(self.correct((0.10, -0.08, 0.12)), ranges)

    def test_the_correct_once_service_corrects_the_estimate_it_holds(self):
        self.harness.call('start')
        self.harness.publish_scan(FIRST)
        self.harness.publish_estimate(SECOND, displaced((0.11, 0.09, -0.10)))
        self.harness.spin(3.0)
        self.harness.clear()

        response = self.harness.call('correct_once')
        self.harness.spin(3.0)

        self.assertTrue(response.success, response.message)
        self.assert_recovered(self.harness.corrected,
                              room_scan(*true_sensor_pose()))

    def test_nothing_is_published_before_start(self):
        self.harness.clear()
        self.harness.publish_scan(FIRST)
        self.harness.spin(0.5)
        self.harness.publish_estimate(SECOND, displaced((0.10, 0.0, 0.0)))
        self.harness.spin(2.0)

        self.assertEqual(self.harness.corrected, [])
        self.assertEqual(self.harness.map_scans, [])

    def test_the_output_carries_the_stamp_of_the_estimate_that_caused_it(self):
        corrected = self.correct((0.10, -0.08, 0.0), stamp=THIRD)

        self.assertEqual(len(corrected), 1)
        self.assertEqual(
            (corrected[0].header.stamp.sec, corrected[0].header.stamp.nanosec),
            THIRD)
        self.assertEqual(corrected[0].header.frame_id, GLOBAL_FRAME)

    def test_the_input_covariance_is_copied_through_untouched(self):
        corrected = self.correct((0.10, -0.08, 0.0))

        self.assertEqual(len(corrected), 1)
        self.assertAlmostEqual(corrected[0].pose.covariance[0], 0.25)
        self.assertAlmostEqual(corrected[0].pose.covariance[7], 0.36)
        self.assertAlmostEqual(corrected[0].pose.covariance[35], 0.09)

    def test_both_transforms_are_broadcast_to_the_frames_named(self):
        self.correct((0.10, -0.08, 0.05))

        pairs = {(t.header.frame_id, t.child_frame_id)
                 for message in self.harness.transforms
                 for t in message.transforms}

        self.assertIn((GLOBAL_FRAME, 'fsm_localisation/estimate'), pairs)
        self.assertIn(('fsm_localisation/estimate', 'fsm_localisation/corrected'),
                      pairs)

    def test_composing_the_two_transforms_gives_the_corrected_pose(self):
        """
        Compose the two broadcast transforms and get the corrected pose back.

        They are only useful if they chain, which is the whole reason there are
        two of them rather than one.
        """
        corrected = self.correct((0.10, -0.08, 0.05))
        self.assertEqual(len(corrected), 1)

        chain = {}
        for message in self.harness.transforms:
            for t in message.transforms:
                chain[(t.header.frame_id, t.child_frame_id)] = t.transform

        first = chain[(GLOBAL_FRAME, 'fsm_localisation/estimate')]
        second = chain[('fsm_localisation/estimate', 'fsm_localisation/corrected')]

        angle = yaw_of(first.rotation)
        composed = (
            first.translation.x + math.cos(angle) * second.translation.x
            - math.sin(angle) * second.translation.y,
            first.translation.y + math.sin(angle) * second.translation.x
            + math.cos(angle) * second.translation.y,
            wrap(angle + yaw_of(second.rotation)))

        pose = corrected[0].pose.pose
        self.assertAlmostEqual(composed[0], pose.position.x, delta=1e-6)
        self.assertAlmostEqual(composed[1], pose.position.y, delta=1e-6)
        self.assertAlmostEqual(wrap(composed[2] - yaw_of(pose.orientation)), 0.0,
                               delta=1e-6)

    def test_the_inspection_scan_is_published_alongside_the_pose(self):
        self.correct((0.10, -0.08, 0.0))

        self.assertEqual(len(self.harness.map_scans), 1)

        published = self.harness.map_scans[0]
        self.assertEqual(len(published.ranges), 360)
        self.assertAlmostEqual(published.angle_min, -math.pi, places=6)
        self.assertAlmostEqual(published.range_max, 100.0, places=6)

    def test_an_estimate_outside_the_map_is_refused(self):
        self.harness.call('start')
        self.harness.publish_scan(FIRST)
        self.harness.spin(0.5)
        self.harness.clear()
        self.harness.publish_estimate(SECOND, (-20.0, -20.0, 0.0))
        self.harness.spin(2.0)

        self.assertEqual(self.harness.corrected, [])

        response = self.harness.call('correct_once')
        self.assertFalse(response.success)
        self.assertIn('outside the map', response.message)

    def test_an_estimate_in_another_frame_is_refused(self):
        self.harness.call('start')
        self.harness.publish_scan(FIRST)
        self.harness.spin(0.5)
        self.harness.clear()
        self.harness.publish_estimate(SECOND, displaced((0.1, 0.0, 0.0)),
                                      frame_id='odom')
        self.harness.spin(2.0)

        self.assertEqual(self.harness.corrected, [])

        response = self.harness.call('correct_once')
        self.assertFalse(response.success)
        self.assertIn('odom', response.message)

    def test_an_estimate_stamped_before_zero_is_refused_and_the_next_served(
            self):
        """
        Refuse an estimate stamped before zero and correct the one after it.

        A ROS time holds no time before zero, and building one throws. A throw
        that escaped the correction would stop the node, so the case asks it
        for the estimate after that one as well.
        """
        self.harness.call('start')
        self.harness.publish_scan(FIRST)
        self.harness.spin(0.5)
        self.harness.clear()
        self.harness.publish_estimate(BEFORE_ZERO,
                                      displaced((0.10, -0.08, 0.0)))
        self.harness.spin(2.0)

        self.assertEqual(self.harness.corrected, [])

        response = self.harness.call('correct_once')
        self.assertFalse(response.success)
        self.assertIn('before zero', response.message)

        self.harness.publish_estimate(SECOND, displaced((0.10, -0.08, 0.05)))
        self.harness.spin(3.0)

        self.assert_recovered(self.harness.corrected,
                              room_scan(*true_sensor_pose()))

    def test_a_scan_with_no_usable_range_is_refused(self):
        self.harness.call('start')
        self.harness.publish_scan(FIRST, ranges=[0.0] * 360)
        self.harness.spin(0.5)
        self.harness.clear()
        self.harness.publish_estimate(SECOND, displaced((0.10, 0.0, 0.0)))
        self.harness.spin(2.0)

        self.assertEqual(self.harness.corrected, [])

        response = self.harness.call('correct_once')
        self.assertFalse(response.success)
        self.assertIn('usable', response.message)

    def test_the_node_serves_the_next_estimate_after_every_refusal(self):
        """
        Refuse three estimates in a row and still answer the fourth.

        This is the point of every refusal above gathered into one case: a node
        that refused and then stopped answering would pass each of them.
        """
        self.harness.call('start')
        self.harness.publish_scan(FIRST)
        self.harness.spin(0.5)

        for stamp, pose in ((SECOND, (-20.0, -20.0, 0.0)),
                            (THIRD, (float('nan'), 0.0, 0.0))):
            self.harness.publish_estimate(stamp, pose)
            self.harness.spin(1.0)

        self.harness.clear()
        self.harness.publish_estimate(FOURTH, displaced((0.10, -0.08, 0.05)))
        self.harness.spin(3.0)

        self.assert_recovered(self.harness.corrected,
                              room_scan(*true_sensor_pose()))

    def test_a_correction_slower_than_the_estimates_is_reported(
            self, proc_output, fsm_localisation):
        """
        Report a correction that outlasts the gap between the estimates.

        The interval the node compares against is the gap between the stamps
        the estimates carry, not the gap between their arrivals, so two
        estimates stamped a millisecond apart put the node behind whatever the
        machine running this happens to be doing. That makes the case a
        statement about the node rather than a race against the test runner.
        """
        self.harness.call('start')
        self.harness.publish_scan(FIRST)
        self.harness.spin(0.5)

        for nanosec in range(0, 5000000, 1000000):
            self.harness.publish_estimate((1700000001, nanosec),
                                          displaced((0.10, -0.08, 0.0)))
            self.harness.spin(0.6)

        proc_output.assertWaitFor('size_scan', process=fsm_localisation,
                                  timeout=20, stream='stderr')
