# Copyright 2022 Alexandros Filotheou
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
The contract a freshly started node honours, once.

These three properties are about a node that has never been started and has
never seen a scan, so they need an untouched node and cannot share one with the
other behaviour tests. They live in their own launch for that reason: the whole
file is a single test, so nothing can have disturbed the node before it runs.
"""

import unittest

from fsm_lidar_odometry_test_support import build_scan, FIRST_STAMP, Harness, SECOND_STAMP
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='fsm_lidar_odometry',
        executable='fsm_lidar_odometry_interface_node',
        name='fsm_lidar_odometry',
        output='screen',
    )
    return (
        launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]),
        {'fsm_lidar_odometry': node},
    )


class TestStartupContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = Harness()
        self.harness.wait_for_connection()
        self.harness.spin(0.5)

    def tearDown(self):
        self.harness.destroy_node()

    def test_the_startup_contract(self):
        self.harness.publish(build_scan(FIRST_STAMP, (3.0, 2.5, 0.0)))
        self.harness.spin()
        self.harness.publish(build_scan(SECOND_STAMP, (3.02, 2.5, 0.0)))
        self.harness.spin()

        self.assertEqual(self.harness.odometry, [],
                         'the node produced output before it was started')

        self.harness.call('start')
        self.harness.clear()

        self.harness.publish(build_scan(FIRST_STAMP, (3.0, 2.5, 0.0)))
        self.harness.spin()
        self.assertEqual(len(self.harness.odometry), 0,
                         'the first scan has nothing to be matched against, '
                         'so it must produce nothing')

        self.harness.publish(build_scan(SECOND_STAMP, (3.02, 2.5, 0.0)))
        self.harness.spin()
        self.assertEqual(len(self.harness.odometry), 1)
        self.assertEqual(len(self.harness.poses), 1)
        self.assertEqual(len(self.harness.paths), 1)
        self.assertEqual(len(self.harness.transforms), 1)


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_node_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
