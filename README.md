# Autonomous Navigation of TurtleBot3 Burger in Gazebo (ROS 2 Jazzy)

Assignment 02 — SLIIT, Department of Computer Systems Engineering.

A TurtleBot3 Burger autonomously navigates an 8x8 (0.5 m cell) maze in Gazebo:
SLAM mapping, AMCL localization, Nav2 path planning (Dijkstra / A*) and a
Deep Q-Learning policy. Goals are clicked by the user in RViz.

## Repository layout

```
maze_ws/src/
  maze_gazebo/       # maze.world (SDF), TurtleBot3 Burger URDF, sim launch, RViz
  maze_bringup/      # SLAM, localization, navigation, demo launch files + configs
  maze_navigation/   # saved map (pgm/yaml) and Nav2 params (Dijkstra + A*)
  maze_dqn/          # Gym env + Deep Q-Learning training and inference nodes
scripts/             # maze/map generators, goal sender, metrics, docker helpers
```

## Requirements

* Docker Desktop (macOS/Windows/Linux) — the project runs in an Ubuntu 24.04 /
  ROS 2 Jazzy container because ROS 2 Jazzy + Gazebo have no native macOS build.
* macOS GUI only: use the built-in Screen Sharing app to view the VNC desktop
  (see *Viewing the GUI* below). XQuartz is optional and unreliable for Gazebo.

## 1. Build

```bash
bash scripts/setup_xquartz.sh   # macOS only, one-time GUI setup
bash scripts/build.sh           # docker build + colcon build
```

Open a shell inside the container:

```bash
docker compose run --rm ros2 bash
source /opt/ros/jazzy/setup.bash
source /workspace/maze_ws/install/setup.bash
```

## 2. Mapping with SLAM

```bash
ros2 launch maze_bringup slam.launch.py
# in a second shell, drive the robot around the maze:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# when the map looks complete, save it:
ros2 run nav2_map_server map_saver_cli -f /workspace/maze_ws/src/maze_navigation/maps/maze
```

A ground-truth map is also generated from the maze layout for reference:

```bash
python3 scripts/gen_map.py
```

## 3. Autonomous navigation (Nav2)

```bash
# Dijkstra (default)
ros2 launch maze_bringup demo.launch.py

# A*
ros2 launch maze_bringup demo.launch.py planner:=astar

# Headless (no GUI)
ros2 launch maze_bringup demo.launch.py headless:=true
```

In RViz use the **2D Goal Pose** tool (or `scripts/send_goals.py`) to pick a
destination. The robot plans and follows a path while avoiding obstacles.

## 4. Deep Q-Learning navigation

Terminal A — start the simulator headless:

```bash
ros2 launch maze_gazebo maze.launch.py headless:=true
```

Terminal B — train (Gazebo runs in real time, so keep timesteps moderate):

```bash
ros2 run maze_dqn train --timesteps 60000 --out models/dqn_maze
```

Terminal C — run the trained policy and click a goal in RViz:

```bash
ros2 launch maze_bringup demo.launch.py method:=dqn model_path:=models/dqn_maze.zip
```

Without a model the node falls back to a reactive controller (still demonstrable).

## 5. Performance measurement

```bash
ros2 run maze_dqn ... # or Nav2 stack running
python3 scripts/metrics.py --label dijkstra --out results.csv
python3 scripts/send_goals.py --method nav2 --goals "1.25,-1.25" "1.25,1.25" "-1.25,-1.25"
```

`results.csv` records travel time, path length, collisions and minimum LiDAR
range per goal — used for the comparison table in the report.

## Viewing the GUI (VNC — recommended on macOS)

XQuartz cannot give Gazebo/RViz a working OpenGL context on Apple Silicon, and
Gazebo's Qt GUI hangs without a DBus session. The reliable path is a headless
VNC desktop (`Xvfb` + `x11vnc`) with software Mesa. One command sets it all up:

```bash
bash scripts/viva_demo.sh          # Dijkstra (default)
bash scripts/viva_demo.sh astar    # A*
bash scripts/viva_demo.sh dqn      # Deep Q-Learning policy
bash scripts/viva_demo.sh stop     # stop everything
```

Then, on the host, open the desktop:

```bash
open vnc://localhost:5901          # macOS Screen Sharing (no password)
```

Two windows appear: **Gazebo Sim** (3D maze) and **RViz**. In RViz choose the
**2D Goal Pose** tool and click a free cell; the robot plans and drives there.

## VIVA checklist

1. `bash scripts/build.sh`
2. `bash scripts/viva_demo.sh` and (in RViz) reach 3 different clicked goals.
3. `bash scripts/viva_demo.sh astar` to compare path length / time.
4. `bash scripts/viva_demo.sh dqn` to run the DQN policy node.
5. `bash scripts/viva_demo.sh stop` when finished.

## Environment notes

* **DDS middleware:** the default Fast DDS intermittently drops ROS 2 service
  responses under Docker Desktop / x86-on-ARM emulation, which makes the Nav2
  lifecycle managers fail to activate nodes ("failed to send response to
  /amcl/change_state"). The image therefore defaults to **CycloneDDS**
  (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI` pinned to loopback
  via `scripts/cyclonedds.xml`). Export the same two variables in any extra
  shells you open so all nodes use the same middleware.
* **Headless rendering:** the 2D LiDAR uses the `gpu_lidar` sensor, so the
  simulator must run with `--headless-rendering` and software Mesa
  (`LIBGL_ALWAYS_SOFTWARE=1`). This is enabled automatically with
  `headless:=true`.

## Notes / limitations

* The world spans 4 x 4 m with 0.5 m corridors, so a corridor half-width is
  0.25 m. `inflation_radius` is set to **0.15 m** (below the half-width) to keep
  a cost-free centreline through every corridor; a larger radius (e.g. 0.30 m)
  makes a single spurious obstacle seal a corridor and the planner fail.
* The global costmap plans on the **static map only** (no obstacle layer); the
  local costmap keeps the LiDAR obstacle layer for reactive avoidance. This
  keeps global planning deterministic on the known maze.
* `maze_navigation/maps/maze.*` is generated from the known layout to guarantee
  a reliable map; replace it with your SLAM result for the report screenshots.
