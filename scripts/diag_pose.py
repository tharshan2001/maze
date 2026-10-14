#!/usr/bin/env python3
"""Print /odom, /amcl_pose and map->base_footprint tf for comparison."""
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf2_ros


class DiagPose(Node):
    def __init__(self):
        super().__init__("diag_pose")
        self.odom = None
        self.amcl = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._amcl, 10)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)

    def _amcl(self, m):
        p = m.pose.pose.position
        self.amcl = (p.x, p.y)

    def tf(self, target, source):
        try:
            t = self.buf.lookup_transform(target, source, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:  # noqa: BLE001
            return f"ERR {e}"


def main():
    rclpy.init()
    n = DiagPose()
    t0 = time.time()
    while time.time() - t0 < 15.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    print(f"odom(x,y)        = {n.odom}")
    print(f"amcl_pose(x,y)   = {n.amcl}")
    print(f"map->base_fp tf  = {n.tf('map', 'base_footprint')}")
    print(f"odom->base_fp tf = {n.tf('odom', 'base_footprint')}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
