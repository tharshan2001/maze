#!/usr/bin/env python3
"""Send a sequence of goal points for the automated demo / benchmarks.

Nav2:
    ros2 run ... (Nav2 stack running)
    python3 scripts/send_goals.py --method nav2 --goals "1.25,-1.25" "1.25,1.25"

DQN (publishes /goal_pose; ROS 2 must be sourced):
    python3 scripts/send_goals.py --method dqn --goals "1.25,-1.25"

Goals are given in the map/odom frame as "x,y". Coordinates below are all
free cells of the 8x8 maze.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped


def make_pose(x, y, frame):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = rclpy.clock.Clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


def run_publisher(goals, frame, pause):
    node = rclpy.create_node("goal_sender")
    pub = node.create_publisher(PoseStamped, "/goal_pose", 10)
    for (x, y) in goals:
        node.get_logger().info(f"Publishing goal ({x}, {y}) in {frame}")
        for _ in range(10):
            pub.publish(make_pose(x, y, frame))
            time.sleep(0.05)
        time.sleep(pause)
    node.destroy_node()


def run_action(goals, pause):
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    node = rclpy.create_node("goal_sender")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error("navigate_to_pose action server not available")
        node.destroy_node()
        return

    for (x, y) in goals:
        node.get_logger().info(f"Sending NavigateToPose goal ({x}, {y})")
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, "map")
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            node.get_logger().warn("Goal rejected")
            continue
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        node.get_logger().info("Goal finished")
        time.sleep(pause)
    node.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", choices=["nav2", "dqn"], default="dqn")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--pause", type=float, default=2.0)
    ap.add_argument(
        "--goals",
        nargs="*",
        default=["1.25,-1.25", "1.25,1.25", "-1.25,-1.25"],
    )
    args = ap.parse_args()
    goals = [tuple(float(v) for v in g.split(",")) for g in args.goals]

    rclpy.init()
    try:
        if args.method == "nav2":
            run_action(goals, args.pause)
        else:
            run_publisher(goals, args.frame, args.pause)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
