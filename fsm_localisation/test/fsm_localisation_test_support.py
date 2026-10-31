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
BEFORE_ZERO_STAMP = (-1, 0)


def distance_to_box(x, y, angle, box):
    """
    Return how far a ray from outside an axis aligned box travels to reach it.

    The box is given as its lowest x, lowest y, highest x and highest y. Along
    each axis the ray is within the box's extent between two distances. It is
    inside the box where those two stretches overlap, and it reaches the box
    where the overlap begins. A ray that misses the box travels forever.
    """
    near = 0.0
    far = float('inf')
    for origin, direction, low, high in ((x, math.cos(angle), box[0], box[2]),
                                         (y, math.sin(angle), box[1], box[3])):
        if direction == 0.0:
            if not low <= origin <= high:
                return float('inf')
            continue
        enter = (low - origin) / direction
        leave = (high - origin) / direction
        near = max(near, min(enter, leave))
        far = min(far, max(enter, leave))
    return near if near <= far else float('inf')


def room_scan(x, y, theta, width=8.0, height=5.0, size=SCAN_SIZE,
              pillar=None, first_angle=-math.pi):
    """
    Return the ranges seen from a pose inside an axis aligned rectangle.

    A pillar, a box given the way distance_to_box takes one, stands in the
    room and hides whatever is behind it. The scan is a whole turn whose first
    ray lies first_angle from theta.
    """
    ranges = []
    for i in range(size):
        angle = i * 2.0 * math.pi / size + theta + first_angle
        best = float('inf')
        for distance, normal in ((x, (-1.0, 0.0)), (width - x, (1.0, 0.0)),
                                 (y, (0.0, -1.0)), (height - y, (0.0, 1.0))):
            projection = math.cos(angle) * normal[0] + math.sin(angle) * normal[1]
            if projection > 1e-9:
                best = min(best, distance / projection)
        if pillar is not None:
            best = min(best, distance_to_box(x, y, angle, pillar))
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

        self.create_subscription(Odometry, '/fsm_localisation/lo',
                                 self.odometry.append, qos)
        self.create_subscription(PoseStamped, '/fsm_localisation/pose_estimate',
                                 self.poses.append, qos)
        self.create_subscription(Path, '/fsm_localisation/path_estimate',
                                 self.paths.append, qos)
        self.create_subscription(TFMessage, '/tf', self.transforms.append, qos)

        self.scan_publisher = self.create_publisher(LaserScan, '/base_scan', 1)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/fsm_localisation/initial_pose', 1)

        self.service_clients = {
            name: self.create_client(Trigger, '/fsm_localisation/' + name)
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
