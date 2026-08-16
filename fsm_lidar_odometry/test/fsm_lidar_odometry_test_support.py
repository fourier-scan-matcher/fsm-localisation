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
Shared machinery for the node level tests.

Builds synthetic scans of a rectangular room and wraps the node's topics and
services in something a test can drive a few lines at a time.
"""

import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

SCAN_SIZE = 360
SETTLE = 1.5
SERVICE_TIMEOUT = 20.0

FIRST_STAMP = (1700000000, 0)
SECOND_STAMP = (1700000000, 100000000)
THIRD_STAMP = (1700000000, 200000000)
FOURTH_STAMP = (1700000000, 300000000)


def room_scan(x, y, theta, width=8.0, height=5.0, size=SCAN_SIZE):
    """Return the ranges seen from a pose inside an axis aligned rectangle."""
    ranges = []
    for i in range(size):
        angle = i * 2.0 * math.pi / size + theta - math.pi
        best = float('inf')
        for distance, normal in ((x, (-1.0, 0.0)), (width - x, (1.0, 0.0)),
                                 (y, (0.0, -1.0)), (height - y, (0.0, 1.0))):
            projection = math.cos(angle) * normal[0] + math.sin(angle) * normal[1]
            if projection > 1e-9:
                best = min(best, distance / projection)
        ranges.append(best if best != float('inf') else 0.0)
    return ranges


def build_scan(stamp, pose, size=SCAN_SIZE):
    """Return a LaserScan of a rectangular room, stamped as asked."""
    scan = LaserScan()
    scan.header.stamp.sec, scan.header.stamp.nanosec = stamp
    scan.header.frame_id = 'base_laser_link'
    scan.angle_min = -math.pi
    scan.angle_max = math.pi - 2.0 * math.pi / size
    scan.angle_increment = 2.0 * math.pi / size
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = 0.0
    scan.range_max = 100.0
    scan.ranges = room_scan(*pose, size=size)
    scan.intensities = []
    return scan


class Harness(Node):
    """Drives the node under test and collects everything it publishes."""

    def __init__(self):
        super().__init__('node_behaviour_harness')

        self.odometry = []
        self.poses = []
        self.paths = []
        self.transforms = []

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(Odometry, '/fsm_lidar_odometry/lo',
                                 self.odometry.append, qos)
        self.create_subscription(PoseStamped, '/fsm_lidar_odometry/pose_estimate',
                                 self.poses.append, qos)
        self.create_subscription(Path, '/fsm_lidar_odometry/path_estimate',
                                 self.paths.append, qos)
        self.create_subscription(TFMessage, '/tf', self.transforms.append, qos)

        self.scan_publisher = self.create_publisher(LaserScan, '/base_scan', 1)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/fsm_lidar_odometry/initial_pose', 1)

        self.service_clients = {
            name: self.create_client(Trigger, '/fsm_lidar_odometry/' + name)
            for name in ('start', 'stop', 'clear_estimated_trajectory',
                         'set_initial_pose')
        }

    def call(self, name, timeout=SERVICE_TIMEOUT):
        """Call one of the node's services and return its response."""
        client = self.service_clients[name]
        assert client.wait_for_service(timeout_sec=timeout), \
            'service %s never appeared' % name
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        assert future.result() is not None, 'service %s did not respond' % name
        return future.result()

    def clear(self):
        """Forget everything collected so far."""
        self.odometry.clear()
        self.poses.clear()
        self.paths.clear()
        self.transforms.clear()

    def publish(self, scan):
        """Send one scan to the node."""
        self.scan_publisher.publish(scan)

    def spin(self, seconds=SETTLE):
        """Process callbacks for a while."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_connection(self, timeout=20.0):
        """Block until the node has subscribed to the scan topic."""
        deadline = time.monotonic() + timeout
        while self.scan_publisher.get_subscription_count() == 0:
            assert time.monotonic() < deadline, 'node never subscribed'
            rclpy.spin_once(self, timeout_sec=0.05)
