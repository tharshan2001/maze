#!/usr/bin/env python3
"""Diagnostic: wait for /map, then send one NavigateToPose goal and report."""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class DiagNav(Node):
    def __init__(self):
        super().__init__("diag_nav")
        self.map_info = None
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, qos)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _map_cb(self, msg):
        self.map_info = (msg.info.width, msg.info.height, msg.info.resolution)

    def wait_for_map(self, timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout and self.map_info is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.map_info


def main():
    rclpy.init()
    node = DiagNav()

    print("waiting for /map ...")
    info = node.wait_for_map(60.0)
    print(f"MAP {'OK' if info else 'NOT RECEIVED'}: {info}")

    if not node.client.wait_for_server(timeout_sec=60.0):
        print("ACTION SERVER navigate_to_pose NOT AVAILABLE")
        node.destroy_node()
        rclpy.shutdown()
        return
    print("action server available")

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = 1.25
    goal.pose.pose.position.y = -1.25
    goal.pose.pose.orientation.w = 1.0

    send_future = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=30.0)
    handle = send_future.result()
    print(f"goal accepted: {handle.accepted if handle else None}")
    if not handle or not handle.accepted:
        node.destroy_node()
        rclpy.shutdown()
        return

    result_future = handle.get_result_async()
    t0 = time.time()
    while not result_future.done() and time.time() - t0 < 180.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if result_future.done():
        status = result_future.result().status
        print(f"NAV RESULT status={status} (4=SUCCEEDED) elapsed={time.time()-t0:.1f}s")
    else:
        print("NAV TIMEOUT after 180s")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
