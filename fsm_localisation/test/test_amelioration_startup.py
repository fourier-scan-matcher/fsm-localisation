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
What a node in the correcting mode does before it has been given anything.

These cases need a node that has never received a map and never received a
scan, and neither state can be recovered once one has arrived: the node holds
the newest of each for the rest of its life. So they launch their own node
rather than sharing the one the behaviour cases drive, which is the same reason
the initial pose cases launch their own.

The order the cases run in is therefore load bearing and is written into their
names, since the map case leaves a node that has a map and the scan case leaves
one that has a scan.
"""

import unittest

from fsm_amelioration_test_support import AmeliorationHarness, displaced
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy

FIRST = (1700000000, 0)
SECOND = (1700000000, 200000000)
THIRD = (1700000000, 400000000)


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


class TestAmeliorationStartup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness = AmeliorationHarness()
        cls.harness.wait_for_connection()
        cls.harness.publish_sensor_placement()
        cls.harness.spin(1.0)
        cls.harness.call('start')

    @classmethod
    def tearDownClass(cls):
        cls.harness.call('stop')
        cls.harness.destroy_node()
        rclpy.shutdown()

    def test_1_an_estimate_before_any_map_is_refused(self):
        self.harness.clear()
        self.harness.publish_estimate(FIRST, displaced((0.10, 0.0, 0.0)))
        self.harness.spin(2.0)

        self.assertEqual(self.harness.corrected, [])

        response = self.harness.call('correct_once')
        self.assertFalse(response.success)
        self.assertIn('no map', response.message)
        self.assertIn('/map', response.message)

    def test_2_an_estimate_before_any_scan_is_refused(self):
        self.harness.publish_map(FIRST)
        self.harness.spin(1.0)
        self.harness.clear()

        self.harness.publish_estimate(SECOND, displaced((0.10, 0.0, 0.0)))
        self.harness.spin(2.0)

        self.assertEqual(self.harness.corrected, [])

        response = self.harness.call('correct_once')
        self.assertFalse(response.success)
        self.assertIn('no scan', response.message)
        self.assertIn('/base_scan', response.message)

    def test_3_with_both_in_hand_the_node_corrects(self):
        """
        Correct as soon as both a map and a scan are in hand.

        This is the other half of the two cases above: what they assert is
        missing is genuinely all that was missing, so the same node answers as
        soon as it has both.
        """
        self.harness.publish_scan(FIRST)
        self.harness.spin(0.5)
        self.harness.clear()

        self.harness.publish_estimate(THIRD, displaced((0.10, -0.08, 0.0)))
        self.harness.spin(3.0)

        self.assertEqual(len(self.harness.corrected), 1)
