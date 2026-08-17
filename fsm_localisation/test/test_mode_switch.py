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
What each mode offers, and what it does not.

Both nodes are launched at once, which is worth more than launching them in
turn: it is the arrangement an operator running odometry and correction side by
side would use, and it is the one that would show a collision between the two.
Nothing here drives either node. What is asserted is the surface each presents,
which is settled entirely at construction.
"""

import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node

ODOMETRY_NODE = 'fsm_localisation_odometry'
AMELIORATION_NODE = 'fsm_localisation_amelioration'


@pytest.mark.launch_test
def generate_test_description():
    odometry = launch_ros.actions.Node(
        package='fsm_localisation',
        executable='fsm_localisation_interface_node',
        name=ODOMETRY_NODE,
        output='screen',
        parameters=[{'mode': 'odometry'}],
    )
    amelioration = launch_ros.actions.Node(
        package='fsm_localisation',
        executable='fsm_localisation_interface_node',
        name=AMELIORATION_NODE,
        output='screen',
        parameters=[{'mode': 'amelioration'}],
    )
    return (
        launch.LaunchDescription(
            [odometry, amelioration, launch_testing.actions.ReadyToTest()]),
        {'odometry': odometry, 'amelioration': amelioration},
    )


class TestModeSwitch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.observer = Node('mode_switch_observer')

        import time
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            names = cls.observer.get_node_names()
            if ODOMETRY_NODE in names and AMELIORATION_NODE in names:
                break
            rclpy.spin_once(cls.observer, timeout_sec=0.1)
        else:
            raise AssertionError('both nodes never appeared in the graph')

        rclpy.spin_once(cls.observer, timeout_sec=1.0)
        time.sleep(2.0)

    @classmethod
    def tearDownClass(cls):
        cls.observer.destroy_node()
        rclpy.shutdown()

    def services_of(self, node_name):
        """Return the service names a node offers, less the parameter machinery."""
        return {name for name, _ in
                self.observer.get_service_names_and_types_by_node(node_name, '/')
                if 'parameter' not in name and 'describe' not in name
                and 'type_description' not in name}

    def publications_of(self, node_name):
        return {name for name, _ in
                self.observer.get_publisher_names_and_types_by_node(node_name, '/')
                if name not in ('/rosout', '/parameter_events')}

    def subscriptions_of(self, node_name):
        return {name for name, _ in
                self.observer.get_subscriber_names_and_types_by_node(node_name, '/')
                if name not in ('/parameter_events',)}

    def test_each_mode_offers_its_own_services_and_only_those(self):
        odometry = self.services_of(ODOMETRY_NODE)
        amelioration = self.services_of(AMELIORATION_NODE)

        self.assertEqual(
            odometry,
            {'/%s/%s' % (ODOMETRY_NODE, name) for name in
             ('start', 'stop', 'clear_estimated_trajectory', 'set_initial_pose')})

        self.assertEqual(
            amelioration,
            {'/%s/%s' % (AMELIORATION_NODE, name) for name in
             ('start', 'stop', 'correct_once')})

    def test_start_and_stop_are_the_only_services_both_modes_offer(self):
        shared = {name.rsplit('/', 1)[-1] for name in self.services_of(ODOMETRY_NODE)} \
            & {name.rsplit('/', 1)[-1] for name in self.services_of(AMELIORATION_NODE)}

        self.assertEqual(shared, {'start', 'stop'})

    def test_only_the_correcting_mode_reads_a_map_and_an_estimate(self):
        odometry = self.subscriptions_of(ODOMETRY_NODE)
        amelioration = self.subscriptions_of(AMELIORATION_NODE)

        self.assertIn('/base_scan', odometry)
        self.assertIn('/base_scan', amelioration)

        for topic in ('/map', '/amcl_pose'):
            self.assertNotIn(topic, odometry)
            self.assertIn(topic, amelioration)

    def test_only_the_odometry_mode_publishes_a_path_and_an_odometry(self):
        odometry = self.publications_of(ODOMETRY_NODE)
        amelioration = self.publications_of(AMELIORATION_NODE)

        for topic in ('/%s/lo' % ODOMETRY_NODE,
                      '/%s/path_estimate' % ODOMETRY_NODE):
            self.assertIn(topic, odometry)

        self.assertFalse(
            [t for t in amelioration if t.endswith('/lo')
             or t.endswith('/path_estimate')],
            'the correcting mode published something only odometry has')

    def test_only_the_correcting_mode_publishes_an_inspection_scan(self):
        self.assertIn('/%s/map_scan' % AMELIORATION_NODE,
                      self.publications_of(AMELIORATION_NODE))

        self.assertFalse(
            [t for t in self.publications_of(ODOMETRY_NODE)
             if t.endswith('/map_scan')],
            'the odometry mode published an inspection scan')

    def test_the_two_nodes_share_no_topic_they_both_write(self):
        """
        Find no topic that both nodes write to.

        This is the reason the two parameter files carry different topic names.
        Both modes publish a pose on the same setting and the two are different
        message types, so an overlap here would be two nodes putting
        incompatible messages on one topic.
        """
        self.assertEqual(
            self.publications_of(ODOMETRY_NODE)
            & self.publications_of(AMELIORATION_NODE) - {'/tf'},
            set())
