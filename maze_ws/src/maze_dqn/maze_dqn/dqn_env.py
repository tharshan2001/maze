"""Gymnasium environment that drives the TurtleBot3 in Gazebo over ROS 2.

The environment does not own the simulator; start it first, e.g.:

    ros2 launch maze_gazebo maze.launch.py headless:=true

Then create the env (in train.py or a notebook). A background thread spins a
rclpy node that bridges /scan, /odom and /cmd_vel, so it can be used inside a
normal Stable-Baselines3 training loop.
"""
from __future__ import annotations

import math
import random
import threading
import time
from typing import Optional

import numpy as np

# Free cells (row, col) in the 8x8 maze, world coords helper. Kept in sync with
# scripts/gen_maze.py so random goals always land in free space.
_LAYOUT = [
    "11111111",
    "10000001",
    "10111101",
    "10010001",
    "10100101",
    "10111101",
    "10000001",
    "11111111",
]
_CELL = 0.5
_ROWS, _COLS = len(_LAYOUT), len(_LAYOUT[0])
_X_OFF, _Y_OFF = (_COLS * _CELL) / 2.0, (_ROWS * _CELL) / 2.0


def _cell_to_xy(row, col):
    return (col * _CELL + _CELL / 2.0 - _X_OFF,
            (_ROWS - 1 - row) * _CELL + _CELL / 2.0 - _Y_OFF)


FREE_CELLS = [
    (r, c) for r in range(_ROWS) for c in range(_COLS) if _LAYOUT[r][c] == "0"
]

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover
    import gym
    from gym import spaces

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    _HAS_SET_POSE = True
except ImportError:  # pragma: no cover
    _HAS_SET_POSE = False


def _quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RosBridge(Node):
    """Thin ROS 2 <-> Python bridge used by the DQN environment."""

    def __init__(self, world_name="maze", robot_name="turtlebot3_burger"):
        super().__init__("dqn_env_bridge")
        self._lock = threading.Lock()
        self._scan: Optional[np.ndarray] = None
        self._scan_meta = None
        self._pose = None
        self._have_scan = False
        self._have_odom = False

        self.create_subscription(LaserScan, "scan", self._scan_cb, 10)
        self.create_subscription(Odometry, "odom", self._odom_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self._set_pose_cli = None
        if _HAS_SET_POSE:
            self._set_pose_cli = self.create_client(
                SetEntityPose, f"/world/{world_name}/set_pose"
            )
        self._world = world_name
        self._robot = robot_name

    def _scan_cb(self, msg: LaserScan):
        rng = np.asarray(msg.ranges, dtype=np.float32)
        rng = np.where(np.isfinite(rng), rng, msg.range_max)
        with self._lock:
            self._scan = rng
            self._scan_meta = (msg.angle_min, msg.angle_max, msg.range_max)
            self._have_scan = True

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = _quat_to_yaw(msg.pose.pose.orientation)
        with self._lock:
            self._pose = (p.x, p.y, yaw)
            self._have_odom = True

    def wait_for_data(self, timeout=10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self._have_scan and self._have_odom:
                    return True
            time.sleep(0.05)
        return False

    def scan(self):
        with self._lock:
            return None if self._scan is None else self._scan.copy()

    def scan_meta(self):
        with self._lock:
            return self._scan_meta

    def pose(self):
        with self._lock:
            return None if self._pose is None else tuple(self._pose)

    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def reset_pose(self, x, y, yaw, timeout=5.0) -> bool:
        if self._set_pose_cli is None:
            return False
        if not self._set_pose_cli.wait_for_service(timeout_sec=timeout):
            return False
        req = SetEntityPose.Request()
        req.entity.name = self._robot
        req.entity.type = Entity.MODEL
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = 0.05
        req.pose.orientation.z = math.sin(yaw / 2.0)
        req.pose.orientation.w = math.cos(yaw / 2.0)
        future = self._set_pose_cli.call_async(req)
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        self.stop()
        return True


class MazeNavEnv(gym.Env):
    """Discrete-action goal-reaching environment for the TurtleBot3 maze.

    Observation (Box):
        N down-sampled LiDAR ranges (normalised 0..1) followed by
        [goal_distance_norm, goal_bearing_sin, goal_bearing_cos].

    Actions (Discrete 5):
        0: forward, 1: turn left, 2: turn right, 3: forward-left, 4: forward-right
    """

    metadata = {"render_modes": []}

    ACTIONS = [
        (0.18, 0.0),
        (0.0, 1.2),
        (0.0, -1.2),
        (0.12, 0.7),
        (0.12, -0.7),
    ]

    def __init__(
        self,
        n_rays=36,
        max_range=3.5,
        control_period=0.2,
        goal_tolerance=0.20,
        collision_distance=0.18,
        world_name="maze",
        robot_name="turtlebot3_burger",
        spawn=(-1.25, 1.25, 0.0),
        goal=(1.25, -1.25),
        node: Optional[RosBridge] = None,
        max_steps=400,
        random_goal=False,
    ):
        super().__init__()
        self.n_rays = n_rays
        self.max_range = max_range
        self.dt = control_period
        self.goal_tolerance = goal_tolerance
        self.collision_distance = collision_distance
        self.world_name = world_name
        self.robot_name = robot_name
        self.spawn = spawn
        self.goal = goal
        self.max_steps = max_steps
        self.random_goal = random_goal
        self._steps = 0
        self._prev_dist = None

        self._owns_node = node is None
        if node is not None:
            self.node = node
        else:
            if not rclpy.ok():
                rclpy.init()
            self.node = RosBridge(world_name, robot_name)
            self._spin_thread = threading.Thread(
                target=rclpy.spin, args=(self.node,), daemon=True
            )
            self._spin_thread.start()
            self.node.wait_for_data(timeout=15.0)

        self.action_space = spaces.Discrete(len(self.ACTIONS))
        obs_dim = self.n_rays + 3
        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )

    # ------------------------------------------------------------------ obs
    def _downsample(self, scan: np.ndarray) -> np.ndarray:
        n = len(scan)
        idx = np.linspace(0, n - 1, self.n_rays).astype(int)
        return scan[idx]

    def _get_obs(self) -> np.ndarray:
        scan = self.node.scan()
        if scan is None:
            scan = np.full(360, self.max_range, dtype=np.float32)
        scan = np.clip(self._downsample(scan), 0.0, self.max_range) / self.max_range

        x, y, yaw = self.node.pose() or (0.0, 0.0, 0.0)
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))

        obs = np.concatenate([
            scan.astype(np.float32),
            np.array([
                min(dist / 5.0, 1.0),
                (math.sin(bearing) + 1.0) / 2.0,
                (math.cos(bearing) + 1.0) / 2.0,
            ], dtype=np.float32),
        ])
        return obs

    def _dist_to_goal(self) -> float:
        x, y, _ = self.node.pose() or (0.0, 0.0, 0.0)
        return math.hypot(self.goal[0] - x, self.goal[1] - y)

    # ---------------------------------------------------------------- gym API
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if options and "goal" in options:
            self.goal = options["goal"]
        elif self.random_goal:
            r, c = random.choice(FREE_CELLS)
            self.goal = _cell_to_xy(r, c)
        self.node.reset_pose(*self.spawn)
        time.sleep(0.4)
        self.node.wait_for_data(timeout=5.0)
        self._steps = 0
        self._prev_dist = self._dist_to_goal()
        return self._get_obs(), {}

    def step(self, action: int):
        lin, ang = self.ACTIONS[int(action)]
        self.node.publish_cmd(lin, ang)
        time.sleep(self.dt)
        self.node.publish_cmd(lin, ang)  # keep command fresh for gz bridge

        self._steps += 1
        scan = self.node.scan()
        min_range = float(np.min(scan)) if scan is not None else self.max_range
        dist = self._dist_to_goal()

        reward = 10.0 * (self._prev_dist - dist) - 0.02
        terminated = False
        truncated = self._steps >= self.max_steps

        if dist < self.goal_tolerance:
            reward += 100.0
            terminated = True
        elif min_range < self.collision_distance:
            reward -= 10.0
            terminated = True
            self.node.stop()

        self._prev_dist = dist
        info = {"distance": dist, "min_range": min_range, "steps": self._steps}
        return self._get_obs(), reward, terminated, truncated, info

    def close(self):
        self.node.stop()
        if self._owns_node:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
