#!/usr/bin/env bash
# Build the Docker image and compile the ROS 2 workspace inside it.
set -euo pipefail

cd "$(dirname "$0")/.."

echo ">> Building Docker image (tb3-maze-nav:jazzy)"
docker compose build

echo ">> Building the ROS 2 workspace"
docker compose run --rm ros2 bash -lc \
  "cd /workspace/maze_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

echo ">> Done. Start a shell with: docker compose run --rm ros2 bash"
