#!/usr/bin/env python3
"""BFS reachability test over the live global costmap.

Checks whether the goal cell is reachable from the start cell using only cells
whose cost is below the given threshold (default 253 = inscribed/lethal).
"""
import sys
import time
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

START = (-1.25, 1.25)
GOAL = (1.25, -1.25)
if len(sys.argv) > 3:
    START = (float(sys.argv[2]), float(sys.argv[3]))


class Diag(Node):
    def __init__(self):
        super().__init__("diag_reach")
        self.cost = None
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._cm, qos)

    def _cm(self, msg):
        self.cost = msg


def w2c(x, y, info):
    r = info.resolution
    cx = int((x - info.origin.position.x) / r)
    cy = int((y - info.origin.position.y) / r)
    return cx, cy


def main():
    thr = int(sys.argv[1]) if len(sys.argv) > 1 else 253
    rclpy.init()
    n = Diag()
    t0 = time.time()
    while time.time() - t0 < 20.0 and n.cost is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.cost is None:
        print("no costmap")
        return
    m = n.cost
    w, h = m.info.width, m.info.height
    data = m.data
    sx, sy = w2c(*START, m.info)
    gx, gy = w2c(*GOAL, m.info)
    print(f"start cell=({sx},{sy}) cost={data[sy*w+sx]}")
    print(f"goal  cell=({gx},{gy}) cost={data[gy*w+gx]}")

    def free(cx, cy):
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return False
        return data[cy * w + cx] < thr

    seen = [[False] * w for _ in range(h)]
    q = deque([(sx, sy)])
    seen[sy][sx] = True
    reachable = 0
    while q:
        cx, cy = q.popleft()
        reachable += 1
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if free(nx, ny) and not seen[ny][nx]:
                seen[ny][nx] = True
                q.append((nx, ny))
    print(f"threshold={thr}  reachable cells from start={reachable}")
    print(f"GOAL reachable from START: {seen[gy][gx]}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
