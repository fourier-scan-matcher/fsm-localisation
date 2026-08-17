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
Shared machinery for the node level tests of the correcting mode.

The room is the one fsm_localisation_test_support already draws scans of, an
eight by five metre rectangle, so a scan can be computed in closed form here
and the same room handed to the node as an occupancy grid. The grid's cells are
laid so that its free interior is exactly that rectangle: no cell straddles a
wall, so the grid is the room rather than an approximation of it, and what is
left to disagree is only what the cell size forces.
"""

import math
import time

from fsm_localisation_test_support import room_scan, SCAN_SIZE, SERVICE_TIMEOUT, SETTLE
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

ROOM_WIDTH = 8.0
ROOM_HEIGHT = 5.0
RESOLUTION = 0.05

GRID_WIDTH = int(round(ROOM_WIDTH / RESOLUTION)) + 2
GRID_HEIGHT = int(round(ROOM_HEIGHT / RESOLUTION)) + 2
GRID_ORIGIN = (-RESOLUTION, -RESOLUTION)

SENSOR_ON_ROBOT = (0.20, 0.05, 0.10)

TRUE_ROBOT_POSE = (3.10, 2.30, 0.35)

ROBOT_FRAME = 'base_footprint'
SENSOR_FRAME = 'base_laser_link'
GLOBAL_FRAME = 'map'

CORRECTED_POSE_TOPIC = '/fsm_localisation/pose_estimate'
MAP_SCAN_TOPIC = '/fsm_localisation/map_scan'


def compose(a, b):
    """Return b expressed in the frame a is expressed in."""
    return (a[0] + math.cos(a[2]) * b[0] - math.sin(a[2]) * b[1],
            a[1] + math.sin(a[2]) * b[0] + math.cos(a[2]) * b[1],
            wrap(a[2] + b[2]))


def wrap(angle):
    """Return angle folded into [-pi, pi)."""
    return math.remainder(angle, 2.0 * math.pi)


def true_sensor_pose():
    """Where the laser really is, given where the robot really is."""
    return compose(TRUE_ROBOT_POSE, SENSOR_ON_ROBOT)


def displaced(displacement):
    """Return the true robot pose pushed off by a known amount."""
    return (TRUE_ROBOT_POSE[0] + displacement[0],
            TRUE_ROBOT_POSE[1] + displacement[1],
            wrap(TRUE_ROBOT_POSE[2] + displacement[2]))


def build_map(stamp, blocked_interior=()):
    """
    Return the room as an occupancy grid.

    Every cell of the outer ring is blocked and everything inside it is free,
    so the free region is exactly the rectangle room_scan draws. Cells named in
    blocked_interior are blocked as well, which is how a test puts an obstacle
    where it wants one.
    """
    message = OccupancyGrid()
    message.header.stamp.sec, message.header.stamp.nanosec = stamp
    message.header.frame_id = GLOBAL_FRAME
    message.info.resolution = RESOLUTION
    message.info.width = GRID_WIDTH
    message.info.height = GRID_HEIGHT
    message.info.origin.position.x = GRID_ORIGIN[0]
    message.info.origin.position.y = GRID_ORIGIN[1]
    message.info.origin.orientation.w = 1.0

    cells = [0] * (GRID_WIDTH * GRID_HEIGHT)

    for column in range(GRID_WIDTH):
        cells[column] = 100
        cells[(GRID_HEIGHT - 1) * GRID_WIDTH + column] = 100

    for row in range(GRID_HEIGHT):
        cells[row * GRID_WIDTH] = 100
        cells[row * GRID_WIDTH + GRID_WIDTH - 1] = 100

    for column, row in blocked_interior:
        cells[row * GRID_WIDTH + column] = 100

    message.data = cells
    return message


def build_estimate(stamp, pose, frame_id=GLOBAL_FRAME):
    """Return a pose estimate of the robot, stamped as asked."""
    message = PoseWithCovarianceStamped()
    message.header.stamp.sec, message.header.stamp.nanosec = stamp
    message.header.frame_id = frame_id
    message.pose.pose.position.x = float(pose[0])
    message.pose.pose.position.y = float(pose[1])
    message.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
    message.pose.pose.orientation.w = math.cos(pose[2] / 2.0)

    message.pose.covariance = [0.0] * 36
    message.pose.covariance[0] = 0.25
    message.pose.covariance[7] = 0.36
    message.pose.covariance[35] = 0.09
    return message


def yaw_of(orientation):
    """Return the heading a quaternion carries, about z."""
    return wrap(2.0 * math.atan2(orientation.z, orientation.w))


def heading_tolerance(ranges, cells=2.0):
    """
    Return the heading a scan of these ranges can be resolved to.

    A heading wrong by dt moves what the sensor would see at range r sideways
    by r * dt, so a heading is resolved no better than the position bound seen
    at the ranges the scan actually carries.
    """
    usable = [r for r in ranges if 0.0 < r < 1e3]
    mean = sum(usable) / len(usable)
    return math.atan(cells * RESOLUTION / mean)


class AmeliorationHarness(Node):
    """Drives a node in the correcting mode and collects what it publishes."""

    def __init__(self):
        super().__init__('amelioration_harness')

        self.corrected = []
        self.map_scans = []
        self.transforms = []

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(PoseWithCovarianceStamped,
                                 CORRECTED_POSE_TOPIC, self.corrected.append, qos)
        self.create_subscription(LaserScan, MAP_SCAN_TOPIC,
                                 self.map_scans.append, qos)
        self.create_subscription(TFMessage, '/tf', self.transforms.append, qos)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', latched)
        self.scan_publisher = self.create_publisher(LaserScan, '/base_scan', 1)
        self.estimate_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 1)

        static = QoSProfile(depth=1)
        static.reliability = ReliabilityPolicy.RELIABLE
        static.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.static_transform_publisher = self.create_publisher(
            TFMessage, '/tf_static', static)

        self.service_clients = {
            name: self.create_client(Trigger, '/fsm_localisation/' + name)
            for name in ('start', 'stop', 'correct_once')
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
        self.corrected.clear()
        self.map_scans.clear()
        self.transforms.clear()

    def publish_estimate(self, stamp, pose, frame_id=GLOBAL_FRAME):
        """Send one pose estimate to the node."""
        self.estimate_publisher.publish(build_estimate(stamp, pose, frame_id))

    def publish_map(self, stamp, blocked_interior=()):
        """Send one map to the node."""
        self.map_publisher.publish(build_map(stamp, blocked_interior))

    def publish_scan(self, stamp, sensor_pose=None, size=SCAN_SIZE, ranges=None):
        """Send one scan to the node, seen from where the laser really is."""
        if sensor_pose is None:
            sensor_pose = true_sensor_pose()

        scan = LaserScan()
        scan.header.stamp.sec, scan.header.stamp.nanosec = stamp
        scan.header.frame_id = SENSOR_FRAME
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - 2.0 * math.pi / size
        scan.angle_increment = 2.0 * math.pi / size
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.0
        scan.range_max = 100.0
        scan.ranges = ranges if ranges is not None else room_scan(
            *sensor_pose, width=ROOM_WIDTH, height=ROOM_HEIGHT, size=size)
        scan.intensities = []
        self.scan_publisher.publish(scan)
        return scan

    def publish_sensor_placement(self, placement=SENSOR_ON_ROBOT):
        """Put where the laser sits on the robot into the transform tree."""
        message = TransformStamped()
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        message.header.frame_id = ROBOT_FRAME
        message.child_frame_id = SENSOR_FRAME
        message.transform.translation.x = placement[0]
        message.transform.translation.y = placement[1]
        message.transform.rotation.z = math.sin(placement[2] / 2.0)
        message.transform.rotation.w = math.cos(placement[2] / 2.0)

        self.static_transform_publisher.publish(TFMessage(transforms=[message]))

    def spin(self, seconds=SETTLE):
        """Process callbacks for a while."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_connection(self, timeout=20.0):
        """Block until the node has subscribed to everything it drives."""
        deadline = time.monotonic() + timeout
        while (self.scan_publisher.get_subscription_count() == 0
               or self.map_publisher.get_subscription_count() == 0
               or self.estimate_publisher.get_subscription_count() == 0):
            assert time.monotonic() < deadline, 'node never subscribed'
            rclpy.spin_once(self, timeout_sec=0.05)
