#!/usr/bin/env python3
"""Diagnostic node: confirm /scan and /odom are received from the ROS-gz bridge."""
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Diag(Node):
    def __init__(self):
        super().__init__("diag_sub")
        self.scan_count = 0
        self.odom_count = 0
        self.last_len = 0
        self.create_subscription(LaserScan, "/scan", self._scan, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)

    def _scan(self, msg):
        self.scan_count += 1
        self.last_len = len(msg.ranges)

    def _odom(self, msg):
        self.odom_count += 1


def main():
    rclpy.init()
    node = Diag()
    t0 = time.time()
    while time.time() - t0 < 12.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"RESULT scan_msgs={node.scan_count} odom_msgs={node.odom_count} "
          f"scan_len={node.last_len}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
