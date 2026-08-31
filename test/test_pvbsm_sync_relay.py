#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import PointCloud2


class PvbsmSyncRelayRecoveryTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._received = []
        self._publisher = rospy.Publisher(
            "/test/uav0/pvbsm_delta", PointCloud2, queue_size=32
        )
        self._subscriber = rospy.Subscriber(
            "/test/uav1/peer_delta", PointCloud2, self._callback, queue_size=64
        )

    def _callback(self, message):
        with self._lock:
            self._received.append(message.header.seq)

    def test_recovers_deterministic_twenty_percent_loss(self):
        deadline = time.time() + 5.0
        while self._publisher.get_num_connections() == 0 and time.time() < deadline:
            rospy.sleep(0.05)
        self.assertGreater(self._publisher.get_num_connections(), 0)
        rospy.sleep(0.5)

        expected = list(range(1, 31))
        for sequence in expected:
            message = PointCloud2()
            message.header.seq = sequence
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "uav0/camera_init"
            message.height = 1
            message.width = 1
            message.point_step = 4
            message.row_step = 4
            message.data = bytes(
                [sequence & 0xFF, (sequence >> 8) & 0xFF, 0xDA, 0x1B]
            )
            self._publisher.publish(message)
            rospy.sleep(0.02)

        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self._lock:
                received = list(self._received)
            if received == expected:
                break
            rospy.sleep(0.05)

        with self._lock:
            self.assertEqual(self._received, expected)


if __name__ == "__main__":
    rospy.init_node("test_pvbsm_sync_relay")
    rostest.rosrun(
        "daib_explorer",
        "pvbsm_sync_relay_recovery",
        PvbsmSyncRelayRecoveryTest,
    )
