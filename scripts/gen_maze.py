#!/usr/bin/env python3
"""Generate a Gazebo (gz sim / SDF) maze world for the TurtleBot3 assignment.

Grid cells are 0.5 m. A '1' cell becomes a solid wall block (0.5 x 0.5 x 1.0 m),
a '0' cell is free space. The maze is centred on the world origin.

Run:  python3 scripts/gen_maze.py
Out:  maze_ws/src/maze_gazebo/worlds/maze.world
"""
import os

# 8 x 8 cell grid. 0 = free, 1 = wall. Border is fully closed.
LAYOUT = [
    "11111111",
    "10000001",
    "10111101",
    "10010001",
    "10100101",
    "10111101",
    "10000001",
    "11111111",
]

# Free cell where the robot is spawned (row, col).
SPAWN_CELL = (1, 1)

CELL = 0.5          # cell size in metres
WALL_HEIGHT = 1.0

ROWS = len(LAYOUT)
COLS = len(LAYOUT[0])
X_OFF = (COLS * CELL) / 2.0
Y_OFF = (ROWS * CELL) / 2.0


def cell_to_xy(row, col):
    x = col * CELL + CELL / 2.0 - X_OFF
    y = (ROWS - 1 - row) * CELL + CELL / 2.0 - Y_OFF
    return x, y


def wall_models():
    chunks = []
    for r in range(ROWS):
        for c in range(COLS):
            if LAYOUT[r][c] != "1":
                continue
            x, y = cell_to_xy(r, c)
            chunks.append(
                f"""    <model name="wall_{r}_{c}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {WALL_HEIGHT / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{CELL} {CELL} {WALL_HEIGHT}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{CELL} {CELL} {WALL_HEIGHT}</size></box></geometry>
          <material><ambient>0.6 0.3 0.2 1</ambient><diffuse>0.8 0.4 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>"""
            )
    return "\n".join(chunks)


WORLD = f"""<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="maze">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>

{wall_models()}

    <spawn_pose>
      <!-- Spawn cell (row, col) = {SPAWN_CELL} -->
      <x>{cell_to_xy(*SPAWN_CELL)[0]:.3f}</x>
      <y>{cell_to_xy(*SPAWN_CELL)[1]:.3f}</y>
      <z>0.05</z>
      <yaw>0.0</yaw>
    </spawn_pose>
  </world>
</sdf>
"""


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(
        here, "..", "maze_ws", "src", "maze_gazebo", "worlds", "maze.world"
    )
    out = os.path.normpath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(WORLD)
    print(f"Wrote {out}")
    print(f"Maze: {COLS}x{ROWS} cells @ {CELL} m -> {COLS * CELL:.1f} x {ROWS * CELL:.1f} m")
    print(f"Spawn (x, y): {cell_to_xy(*SPAWN_CELL)}")


if __name__ == "__main__":
    main()
