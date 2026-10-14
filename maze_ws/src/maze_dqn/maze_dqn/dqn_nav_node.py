"""Autonomous navigation node driven by a trained DQN policy.

Click a destination in RViz ("2D Goal Pose" tool) -> the node drives the
TurtleBot3 to the goal using the learned policy.

  ros2 launch maze_gazebo maze.launch.py headless:=true
  ros2 run maze_dqn dqn_nav_node --ros-args -p model_path:=models/dqn_maze.zip

If no model is given (or loading fails) the node falls back to a simple
reactive "turn-then-go" controller so the system is still demonstrable.
"""
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from maze_dqn.dqn_env import RosBridge

try:
    import tf2_ros

    _HAS_TF = True
except ImportError:  # pragma: no cover
    _HAS_TF = False


class DqnNavNode(Node):
    def __init__(self):
        super().__init__("dqn_nav_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("n_rays", 36)
        self.declare_parameter("max_range", 3.5)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("collision_distance", 0.18)
        self.declare_parameter("control_period", 0.2)
        self.declare_parameter("max_linear_vel", 0.18)
        self.declare_parameter("max_angular_vel", 1.2)
        self.declare_parameter("world_name", "maze")
        self.declare_parameter("robot_name", "turtlebot3_burger")
        self.declare_parameter("spawn_x", -1.25)
        self.declare_parameter("spawn_y", 1.25)
        self.declare_parameter("spawn_yaw", 0.0)

        self.n_rays = self.get_parameter("n_rays").value
        self.max_range = self.get_parameter("max_range").value
        self.goal_tol = self.get_parameter("goal_tolerance").value
        self.coll_dist = self.get_parameter("collision_distance").value
        self.dt = self.get_parameter("control_period").value
        self.max_lin = self.get_parameter("max_linear_vel").value
        self.max_ang = self.get_parameter("max_angular_vel").value
        self.spawn = (
            self.get_parameter("spawn_x").value,
            self.get_parameter("spawn_y").value,
            self.get_parameter("spawn_yaw").value,
        )

        # --- bridge (own node spun in a background thread) ---
        self.bridge = RosBridge(
            self.get_parameter("world_name").value,
            self.get_parameter("robot_name").value,
        )
        self._bridge_thread = threading.Thread(
            target=rclpy.spin, args=(self.bridge,), daemon=True
        )
        self._bridge_thread.start()
        self.get_logger().info("Waiting for /scan and /odom ...")
        if not self.bridge.wait_for_data(timeout=20.0):
            self.get_logger().warn("No sensor data yet; is the simulator running?")

        # --- policy ---
        self.model = None
        self.actions = [
            (self.max_lin, 0.0),
            (0.0, self.max_ang),
            (0.0, -self.max_ang),
            (0.6 * self.max_lin, 0.6 * self.max_ang),
            (0.6 * self.max_lin, -0.6 * self.max_ang),
        ]
        model_path = self.get_parameter("model_path").value
        if model_path:
            self._load_model(model_path)

        # --- goal handling ---
        self.goal = None
        self._goal_lock = threading.Lock()
        self.create_subscription(PoseStamped, "goal_pose", self._goal_cb, 10)

        if _HAS_TF:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True
        )
        self._control_thread.start()
        self.get_logger().info("DQN navigation node ready. Click a goal in RViz.")

    # ----------------------------------------------------------------- util
    def _load_model(self, path):
        try:
            from stable_baselines3 import DQN

            if not os.path.exists(path):
                for cand in (path + ".zip", path):
                    if os.path.exists(cand):
                        path = cand
                        break
            self.model = DQN.load(path)
            self.get_logger().info(f"Loaded DQN model from {path}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load model: {exc}. Using fallback.")

    def _goal_cb(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        if _HAS_TF and msg.header.frame_id and msg.header.frame_id != "odom":
            try:
                tf = self._tf_buffer.lookup_transform(
                    "odom", msg.header.frame_id, rclpy.time.Time()
                )
                # apply the 2D transform manually (yaw-only maps)
                q = tf.transform.rotation
                yaw = math.atan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z),
                )
                tx, ty = tf.transform.translation.x, tf.transform.translation.y
                x, y = (tx + x * math.cos(yaw) - y * math.sin(yaw),
                        ty + x * math.sin(yaw) + y * math.cos(yaw))
            except Exception:  # noqa: BLE001
                pass
        with self._goal_lock:
            self.goal = (x, y)
        self.get_logger().info(f"New goal received: ({x:.2f}, {y:.2f})")

    def _obs(self):
        scan = self.bridge.scan()
        if scan is None:
            scan = np.full(360, self.max_range, dtype=np.float32)
        idx = np.linspace(0, len(scan) - 1, self.n_rays).astype(int)
        rays = np.clip(scan[idx], 0.0, self.max_range) / self.max_range

        x, y, yaw = self.bridge.pose() or self.spawn
        gx, gy = self.goal
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        extra = np.array(
            [min(dist / 5.0, 1.0), (math.sin(bearing) + 1) / 2, (math.cos(bearing) + 1) / 2],
            dtype=np.float32,
        )
        return np.concatenate([rays.astype(np.float32), extra]), dist, bearing

    def _fallback_action(self, scan, bearing, min_range):
        if min_range < self.coll_dist:
            return 1 if scan[len(scan) // 2] < scan[-1] else 2
        if abs(bearing) > 0.3:
            return 1 if bearing > 0 else 2
        return 0

    def _control_loop(self):
        while rclpy.ok():
            with self._goal_lock:
                goal = self.goal
            if goal is None:
                time.sleep(0.1)
                continue

            obs, dist, bearing = self._obs()
            if dist < self.goal_tol:
                self.bridge.stop()
                self.get_logger().info("Goal reached.")
                with self._goal_lock:
                    self.goal = None
                continue

            scan = self.bridge.scan()
            min_range = float(np.min(scan)) if scan is not None else self.max_range

            if self.model is not None:
                action, _ = self.model.predict(obs, deterministic=True)
                lin, ang = self.actions[int(action)]
            else:
                lin, ang = self.actions[self._fallback_action(scan, bearing, min_range)]

            if min_range < self.coll_dist:
                lin = 0.0

            self.bridge.publish_cmd(lin, ang)
            time.sleep(self.dt)

    def destroy_node(self):
        try:
            self.bridge.stop()
            self.bridge.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DqnNavNode()
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
