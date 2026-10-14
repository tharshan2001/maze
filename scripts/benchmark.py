#!/usr/bin/env python3
"""Benchmark Nav2 navigation over a fixed goal suite.

For each goal it sends a NavigateToPose action, measures the elapsed time and
the *map-frame* travelled distance (via tf map->base_footprint, which corrects
the large wheel-odometry drift in this simulation), counts near-collision
events from the LiDAR, and appends a row to a CSV.

    python3 scripts/benchmark.py --label dijkstra \
        --goals "1.25,-1.25;1.25,1.25;-1.25,-1.25" --out results.csv
"""
import argparse
import csv
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


class Benchmark(Node):
    def __init__(self, collision_distance, map_frame, base_frame):
        super().__init__("benchmark")
        self.collision_distance = collision_distance
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.min_range = 99.0
        self.collisions = 0
        self._in_collision = False
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _scan_cb(self, msg):
        rng = np.asarray(msg.ranges, dtype=np.float32)
        rng = rng[np.isfinite(rng)]
        if rng.size == 0:
            return
        r = float(np.min(rng))
        self.min_range = min(self.min_range, r)
        if r < self.collision_distance:
            if not self._in_collision:
                self.collisions += 1
                self._in_collision = True
        elif r > self.collision_distance * 1.3:
            self._in_collision = False

    def pose(self):
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame,
                                          Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:  # noqa: BLE001
            return None

    def wait_for_tf(self, timeout=30.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose() is not None:
                return True
        return False

    def run_goal(self, gx, gy):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.w = 1.0

        self.min_range = 99.0
        self.collisions = 0
        self._in_collision = False

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return {"status": "REJECTED", "time_s": 0.0, "path_len_m": 0.0,
                    "collisions": 0, "min_range_m": 99.0}

        result_future = handle.get_result_async()
        t0 = time.time()
        last = self.pose()
        length = 0.0
        while not result_future.done() and time.time() - t0 < 180.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            p = self.pose()
            if p is not None and last is not None:
                length += math.hypot(p[0] - last[0], p[1] - last[1])
            if p is not None:
                last = p
        elapsed = time.time() - t0
        status = result_future.result().status if result_future.done() else "TIMEOUT"
        return {"status": status, "time_s": round(elapsed, 2),
                "path_len_m": round(length, 3),
                "collisions": self.collisions,
                "min_range_m": round(self.min_range, 3),
                "goal_x": round(gx, 2), "goal_y": round(gy, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--out", default="results.csv")
    ap.add_argument("--collision-distance", type=float, default=0.15)
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--goals", default="1.25,-1.25;1.25,1.25;-1.25,-1.25",
                    help="semicolon-separated list of x,y goals")
    args = ap.parse_args()
    goals = [tuple(float(v) for v in g.split(","))
             for g in args.goals.split(";") if g.strip()]

    rclpy.init()
    node = Benchmark(args.collision_distance, args.map_frame, args.base_frame)

    if not node.client.wait_for_server(timeout_sec=60.0):
        print("navigate_to_pose action server not available")
        return 1
    if not node.wait_for_tf(60.0):
        print("map->base_footprint tf not available")
        return 1

    print(f"benchmark '{args.label}': {len(goals)} goals")
    rows = []
    for i, (gx, gy) in enumerate(goals, 1):
        p = node.pose()
        print(f"[{i}/{len(goals)}] goal ({gx},{gy}) from "
              f"({p[0]:.2f},{p[1]:.2f}) ...")
        row = node.run_goal(gx, gy)
        row["label"] = args.label
        s = row["status"]
        name = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(s, s)
        print(f"    -> {name}  time={row['time_s']}s "
              f"path={row['path_len_m']}m collisions={row['collisions']} "
              f"min_range={row['min_range_m']}m")
        rows.append(row)

    fields = ["label", "status", "time_s", "path_len_m", "collisions",
              "min_range_m", "goal_x", "goal_y"]
    exists = os.path.exists(args.out)
    with open(args.out, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if not exists:
            w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"appended {len(rows)} rows to {args.out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
