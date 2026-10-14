#!/usr/bin/env python3
"""Print the global costmap as ASCII and the cost at the robot pose."""
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class DiagMap(Node):
    def __init__(self):
        super().__init__("diag_costmap")
        self.cost = None
        self.pose = None
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._cm, qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._pose, 10)

    def _cm(self, msg):
        self.cost = msg

    def _pose(self, msg):
        self.pose = msg.pose.pose

    def spin_for(self, t):
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.2)


def main():
    rclpy.init()
    node = DiagMap()
    node.spin_for(20.0)
    if node.cost is None:
        print("no /global_costmap/costmap received")
        return
    m = node.cost
    w, h, r = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    print(f"costmap {w}x{h} res={r} origin=({ox},{oy})")

    if node.pose:
        px, py = node.pose.position.x, node.pose.position.y
        cx = int((px - ox) / r)
        cy = int((py - oy) / r)
        idx = cy * w + cx
        val = m.data[idx] if 0 <= idx < len(m.data) else None
        print(f"robot pose=({px:.2f},{py:.2f}) -> cost cell=({cx},{cy}) "
              f"value={val} (0=free,100=lethal,-1=unknown)")

    # ASCII render: sample every 0.25 m
    step = max(1, int(0.25 / r))
    print("ASCII (row 0 = max y; dot=free #=lethal ?=unknown blank=inflated)")
    for py in range(h - 1, -1, -step):
        line = []
        for px in range(0, w, step):
            v = m.data[py * w + px]
            if v < 0:
                line.append("?")
            elif v >= 99:
                line.append("#")
            elif v > 0:
                line.append(" ")  # some inflation cost
            else:
                line.append(".")
        print("".join(line))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
