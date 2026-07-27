#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, Header, UInt64


class RuntimeContractTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._ready_history = []
        self._goal = None
        self._generation = None
        self._frontier = None
        self._odom_pub = rospy.Publisher(
            "/daib_slam/odom", Odometry, queue_size=1
        )
        self._cloud_pub = rospy.Publisher(
            "/daib_slam/planning_cloud", PointCloud2, queue_size=1
        )
        self._degenerate_pub = rospy.Publisher(
            "/daib_slam/degenerate", Bool, queue_size=1
        )
        self._score_pub = rospy.Publisher(
            "/daib_slam/degeneracy_score", Float64, queue_size=1
        )
        self._runtime_pub = rospy.Publisher(
            "/daib_slam/lio_runtime_ms", Float64, queue_size=1
        )
        self._ready_sub = rospy.Subscriber(
            "/daib_explorer/ready", Bool, self._ready_callback, queue_size=1
        )
        self._goal_sub = rospy.Subscriber(
            "/daib_explorer/goal",
            PoseStamped,
            self._goal_callback,
            queue_size=1,
        )
        self._frontier_sub = rospy.Subscriber(
            "/daib_explorer/frontiers",
            PointCloud2,
            self._frontier_callback,
            queue_size=1,
        )
        self._generation_sub = rospy.Subscriber(
            "/daib_explorer/generation",
            UInt64,
            self._generation_callback,
            queue_size=1,
        )

    def _ready_callback(self, message):
        with self._lock:
            self._ready_history.append(message.data)

    def _goal_callback(self, message):
        with self._lock:
            self._goal = message

    def _frontier_callback(self, message):
        with self._lock:
            self._frontier = message

    def _generation_callback(self, message):
        with self._lock:
            self._generation = message.data

    def _publish_frame(self):
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "camera_init"
        odom.child_frame_id = "aft_mapped"
        odom.pose.pose.orientation.w = 1.0
        header = Header(stamp=stamp, frame_id="camera_init")
        points = []
        for degree in range(0, 360, 10):
            angle = math.radians(degree)
            points.append(
                (8.0 * math.cos(angle), 8.0 * math.sin(angle), 0.0)
            )
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self._degenerate_pub.publish(Bool(data=False))
        self._score_pub.publish(Float64(data=0.2))
        self._runtime_pub.publish(Float64(data=50.0))
        self._odom_pub.publish(odom)
        self._cloud_pub.publish(cloud)

    def test_synchronized_inputs_goal_and_watchdog(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_frame()
            with self._lock:
                if (
                    True in self._ready_history
                    and self._goal is not None
                    and self._generation is not None
                    and self._frontier is not None
                    and self._frontier.width > 0
                ):
                    break
            rate.sleep()

        with self._lock:
            self.assertIn(True, self._ready_history)
            self.assertIsNotNone(self._goal)
            self.assertIsNotNone(self._frontier)
            self.assertGreater(self._frontier.width, 0)
            self.assertEqual(self._goal.header.frame_id, "camera_init")
            self.assertEqual(self._goal.header.seq, self._generation)
            self.assertEqual(self._frontier.header.frame_id, "camera_init")

        stale_deadline = rospy.Time.now() + rospy.Duration(2.0)
        while not rospy.is_shutdown() and rospy.Time.now() < stale_deadline:
            with self._lock:
                if self._ready_history and self._ready_history[-1] is False:
                    break
            rospy.sleep(0.05)
        with self._lock:
            self.assertFalse(self._ready_history[-1])


if __name__ == "__main__":
    rospy.init_node("runtime_contract_test")
    rostest.rosrun(
        "daib_explorer", "runtime_contract_test", RuntimeContractTest
    )
