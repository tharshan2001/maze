#!/usr/bin/env bash
# VIVA demo launcher.
#
# Starts the maze navigation demo inside a headless VNC desktop so it can be
# viewed on macOS (or any VNC client) without XQuartz/OpenGL problems:
#
#   bash scripts/viva_demo.sh            # Nav2 + Dijkstra (default)
#   bash scripts/viva_demo.sh astar      # Nav2 + A*
#   bash scripts/viva_demo.sh dqn        # Deep Q-Learning policy
#   bash scripts/viva_demo.sh stop       # stop the demo
#
# Then view it:
#   open vnc://localhost:5901            # macOS Screen Sharing (no password)
#
# Two windows appear: "Gazebo Sim" (3D maze) and "RViz" (map + click goals).
# In RViz choose the "2D Goal Pose" tool and click a free cell in the maze.
#
# Why this works: Gazebo's GUI (Qt) hangs when it has no DBus session, and
# XQuartz cannot give it a GL context. Running gz under `dbus-run-session` on a
# virtual X server (Xvfb) with software Mesa fixes both.
set -euo pipefail

CONTAINER="tb3gui"
IMAGE="tb3-maze-nav:jazzy"
HOST_VNC_PORT="5901"
VNC_PORT="5900"
DISP=":1"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MODE="${1:-dijkstra}"

exec_in()    { docker exec -i "$CONTAINER" bash -lc "$1"; }
exec_daemon(){ docker exec -d "$CONTAINER" bash -lc "$1"; }

stop_demo() {
  # Bracket tricks ([x]) stop each pkill -f pattern from matching this very
  # command line, which would otherwise SIGKILL this shell before it finishes.
  local patterns=(
    "[r]os2 launch"
    "maze_brin[g]up"
    "nav2_[a-z]"
    "parameter_bridg[e]"
    "robot_state_pub[l]isher"
    "gz si[m]"
    "gz-si[m]"
    "[r]uby /opt/ros/jazzy/opt/gz"
    "rviz[2]"
    "dbus-run-sessio[n]"
    "dbus-daemo[n]"
    "[r]uby"
  )
  local p
  for p in "${patterns[@]}"; do
    exec_in "pkill -9 -f '$p' 2>/dev/null; true" >/dev/null 2>&1 || true
  done
  sleep 2
  exec_in "pkill -9 -f 'gz si[m]' 2>/dev/null; pkill -9 -f 'parameter_bridg[e]' 2>/dev/null; pkill -9 -f '[r]uby' 2>/dev/null; true" >/dev/null 2>&1 || true
}

ensure_container() {
  if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo ">> creating GUI container '$CONTAINER' (VNC ${HOST_VNC_PORT} -> ${VNC_PORT})"
    docker run -d --name "$CONTAINER" --init --shm-size=2g \
      -p "${HOST_VNC_PORT}:${VNC_PORT}" \
      -v "$PROJECT_DIR":/workspace \
      -e ROS_DOMAIN_ID=7 -e LIBGL_ALWAYS_SOFTWARE=1 \
      -e GALLIUM_DRIVER=llvmpipe -e QT_X11_NO_MITSHM=1 \
      "$IMAGE" sleep infinity >/dev/null
  elif [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]; then
    echo ">> starting existing container '$CONTAINER'"
    docker start "$CONTAINER" >/dev/null
  fi
}

ensure_deps() {
  if ! exec_in "command -v Xvfb >/dev/null && command -v x11vnc >/dev/null && command -v fluxbox >/dev/null && command -v dbus-run-session >/dev/null && [ -f /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ]"; then
    echo ">> installing GUI dependencies inside the container (one time, ~1-2 min)"
    exec_in "apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
             xvfb x11vnc fluxbox dbus-x11 ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cyclonedds \
             >/tmp/apt_viva.log 2>&1 && echo '   deps OK' || { tail -5 /tmp/apt_viva.log; exit 1; }"
  fi
}

ensure_desktop() {
  exec_in "mkdir -p /tmp/xdg && chmod 700 /tmp/xdg"
  if ! exec_in "test -S /tmp/.X11-unix/X1"; then
    echo ">> starting Xvfb $DISP"
    exec_daemon "Xvfb $DISP -screen 0 1600x900x24 -ac +extension GLX +render -noreset >/tmp/xvfb.log 2>&1"
    sleep 3
  fi
  if ! exec_in "pgrep -x fluxbox >/dev/null"; then
    echo ">> starting window manager (fluxbox)"
    exec_daemon "DISPLAY=$DISP fluxbox >/tmp/fluxbox.log 2>&1"
    sleep 1
  fi
  if ! exec_in "pgrep -x x11vnc >/dev/null"; then
    echo ">> starting x11vnc on port $VNC_PORT"
    exec_daemon "x11vnc -display $DISP -forever -shared -nopw -rfbport $VNC_PORT -bg -o /tmp/x11vnc.log"
    sleep 3
  fi
}

launch_demo() {
  case "$MODE" in
    astar) ARGS="planner:=astar" ;;
    dqn)   ARGS="method:=dqn model_path:=models/dqn_maze.zip" ;;
    *)     ARGS="planner:=dijkstra" ;;
  esac
  echo ">> launching demo ($MODE)"
  exec_daemon "source /opt/ros/jazzy/setup.bash; source /workspace/maze_ws/install/setup.bash; \
    export DISPLAY=$DISP ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///workspace/scripts/cyclonedds.xml \
    LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_X11_NO_MITSHM=1 XDG_RUNTIME_DIR=/tmp/xdg; \
    dbus-run-session -- ros2 launch maze_bringup demo.launch.py $ARGS >/tmp/demo_viva.log 2>&1"
}

if [ "$MODE" = "stop" ]; then
  stop_demo
  echo ">> demo stopped"
  exit 0
fi

ensure_container
ensure_deps
ensure_desktop
stop_demo
sleep 2
launch_demo

echo
echo "===================================================================="
echo " Simulation starting - allow ~60-90 s for Nav2 to activate."
echo
echo " Open the GUI:   open vnc://localhost:${HOST_VNC_PORT}"
echo " (Finder > Go > Connect to Server > vnc://localhost:${HOST_VNC_PORT})"
echo
echo " In RViz pick the '2D Goal Pose' tool and click a free maze cell."
echo " Watch the robot move in the 'Gazebo Sim' window."
echo
echo " Stop with:  bash scripts/viva_demo.sh stop"
echo "===================================================================="
