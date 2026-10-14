#!/usr/bin/env bash
# One-time XQuartz setup so Gazebo / RViz can render from the Docker container.
# Re-run after every XQuartz restart if needed.
set -euo pipefail

echo ">> Allowing connections from network clients in XQuartz"
defaults write org.xquartz.X11 nolisten_tcp -bool false
defaults write org.xquartz.X11 enable_iglx -bool true

echo ">> Starting XQuartz"
open -a XQuartz || true
sleep 3

echo ">> Granting access to the Docker host"
xhost + 127.0.0.1 || xhost + || true

cat <<'EOF'

Done. Notes:
  * XQuartz may ask you to log out/in the first time you enable network clients.
  * If a window does not appear, restart XQuartz and run: xhost +
  * Headless mode (no GUI) needs none of this:
        ros2 launch maze_gazebo maze.launch.py headless:=true
EOF
