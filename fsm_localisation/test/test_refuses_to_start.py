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
A misconfigured node dies, and says which setting killed it.

test_node_params.cpp already asserts which configurations the class refuses.
What it cannot show is what an operator sees, because it constructs the class
rather than running the programme. This file runs the programme. What it asserts
is that a refusal reaches the terminal as a sentence naming the setting and its
value, and that the process then exits on purpose rather than aborting under an
exception nobody caught.

Every case is launched at once, since each one dies on its own and none of them
needs anything from the others.
"""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

CASES = {
    'no_mode': ({}, 'mode must be'),
    'unknown_mode': ({'mode': 'localise'}, 'localise'),
    'bad_threshold': ({'mode': 'amelioration', 'occupancy_threshold': 900},
                      'occupancy_threshold'),
    'bad_timeout': ({'mode': 'amelioration', 'tf_lookup_timeout': -1.0},
                    'tf_lookup_timeout'),
    'bad_caster': ({'mode': 'amelioration', 'map_scan_method': 'raymarch'},
                   'map_scan_method'),
    'empty_topic': ({'mode': 'odometry', 'lo_topic': ''}, 'lo_topic'),
}


@pytest.mark.launch_test
def generate_test_description():
    processes = {
        name: launch_ros.actions.Node(
            package='fsm_localisation',
            executable='fsm_localisation_interface_node',
            name='fsm_localisation_' + name,
            output='screen',
            parameters=[parameters],
        )
        for name, (parameters, _) in CASES.items()
    }

    return (
        launch.LaunchDescription(
            list(processes.values()) + [launch_testing.actions.ReadyToTest()]),
        processes,
    )


class TestRefusalIsSaidOutLoud(unittest.TestCase):

    def test_every_refusal_names_its_setting(
            self, proc_output, no_mode, unknown_mode, bad_threshold,
            bad_timeout, bad_caster, empty_topic):
        processes = {
            'no_mode': no_mode,
            'unknown_mode': unknown_mode,
            'bad_threshold': bad_threshold,
            'bad_timeout': bad_timeout,
            'bad_caster': bad_caster,
            'empty_topic': empty_topic,
        }

        for name, (_, expected) in CASES.items():
            proc_output.assertWaitFor(expected, process=processes[name],
                                      timeout=30, stream='stderr')


@launch_testing.post_shutdown_test()
class TestRefusalEndsTheProcess(unittest.TestCase):

    def test_every_refusal_exits_rather_than_aborts(
            self, proc_info, no_mode, unknown_mode, bad_threshold,
            bad_timeout, bad_caster, empty_topic):
        """
        End the process on purpose rather than by aborting.

        What is being ruled out is the abort an uncaught exception leaves,
        which would bury the sentence above under a message about an exception
        nobody caught, and that is not what an operator needs to read.

        Two codes are allowed rather than one. Run on its own the node exits
        with one, and that is the number an operator sees. Here six of them
        refuse at once, and the launch system tears the survivors down as soon
        as the first is gone, so a process can be signalled while it is still
        on its way out. An abort is neither of those two numbers, which is what
        makes this worth asserting at all.
        """
        for process in (no_mode, unknown_mode, bad_threshold, bad_timeout,
                        bad_caster, empty_topic):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=[1, -2], process=process)
