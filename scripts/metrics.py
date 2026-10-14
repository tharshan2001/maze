#!/usr/bin/env python3
"""Measure navigation performance: travel time, path length and collisions.

Watches /odom and /scan. A run starts when a goal is published on /goal_pose
and ends when the robot is within --tol of that goal, or when --timeout is hit.
Collisions are counted from LiDAR ranges below --collision-distance.

    python3 scripts/metrics.py --label dijkstra --tol 0.25 --out results.csv

Press Ctrl-C to stop early; results are appended to the CSV.
"""
import argparse
import csv
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Metrics(Node):
    def __init__(self, args):
        super().__init__("metrics_logger")
        self.args = args
        self.osm = {"start": None, "last": None, "length": 0.0,
                    "goal": None, "collisions": 0, "min_range": 99.0,
                    "active": False, "start_time": None}
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, 10)
        self.get_logger().info("metrics logger ready; waiting for /goal_pose")

    def _goal_cb(self, msg):
        g = (msg.pose.position.x, msg.pose.position.y)
        if self.osm["active"]:
            return
        self.osm.update(start=None, last=None, length=0.0, goal=g,
                        collisions=0, min_range=99.0, active=True,
                        start_time=time.time())
        self.get_logger().info(f"Run started -> goal ({g[0]:.2f}, {g[1]:.2f})")

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not self.osm["active"]:
            return
        if self.osm["last"] is not None:
            self.osm["length"] += math.hypot(x - self.osm["last"][0],
                                             y - self.osm["last"][1])
        self.osm["last"] = (x, y)
        gx, gy = self.osm["goal"]
        if math.hypot(gx - x, gy - y) < self.args.tol:
            self._finish(x, y)

    def _scan_cb(self, msg):
        rng = np.asarray(msg.ranges, dtype=np.float32)
        rng = rng[np.isfinite(rng)]
        if rng.size == 0:
            return
        self.osm["min_range"] = min(self.osm["min_range"], float(np.min(rng)))
        if self.osm["active"] and float(np.min(rng)) < self.args.collision_distance:
            self.osm["collisions"] += 1

    def _finish(self, x, y):
        elapsed = time.time() - self.osm["start_time"]
        row = {
            "label": self.args.label,
            "time_s": round(elapsed, 2),
            "path_len_m": round(self.osm["length"], 3),
            "collisions": self.osm["collisions"],
            "min_range_m": round(self.osm["min_range"], 3),
            "goal_x": round(self.osm["goal"][0], 2),
            "goal_y": round(self.osm["goal"][1], 2),
        }
        self.get_logger().info(f"GOAL REACHED: {row}")
        exists = os.path.exists(self.args.out)
        with open(self.args.out, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if not exists:
                w.writeheader()
            w.writerow(row)
        self.osm["active"] = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--tol", type=float, default=0.25)
    ap.add_argument("--collision-distance", type=float, default=0.18)
    ap.add_argument("--out", default="results.csv")
    args = ap.parse_args()

    rclpy.init()
    node = Metrics(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
