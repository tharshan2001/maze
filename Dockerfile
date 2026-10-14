FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Optional packages: install individually so a missing one does not break the build.
RUN for p in \
      turtlebot3 \
      turtlebot3-msgs \
      turtlebot3-gazebo \
      turtlebot3-simulations \
      nav2-minimal-tb3-sim \
      gazebo-ros-pkgs \
    ; do \
      apt-get update >/dev/null 2>&1; \
      apt-get install -y --no-install-recommends "ros-jazzy-$p" \
        || echo "WARN: ros-jazzy-$p not available, skipping"; \
    done; rm -rf /var/lib/apt/lists/*

# Machine-learning (DQN) dependencies are intentionally NOT installed here.
# PyTorch is a multi-hundred-MB download and would make image builds fragile on
# slow networks. Install them on demand inside the running container with:
#     bash /workspace/scripts/install_ml_deps.sh
# See scripts/install_ml_deps.sh. The DQN node also works via its reactive
# fallback controller if the ML stack is absent.

# Fast DDS (the default RMW) intermittently drops ROS 2 service responses under
# Docker Desktop / x86 emulation on Apple Silicon, which makes the Nav2
# lifecycle managers fail to activate nodes. CycloneDDS is far more reliable in
# this environment, so make it the default. The config pins traffic to the
# loopback interface (all nodes live in the same container).
COPY scripts/cyclonedds.xml /opt/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///opt/cyclonedds.xml

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "[ -f /workspace/maze_ws/install/setup.bash ] && source /workspace/maze_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/workspace/maze_ws/src:/opt/ros/jazzy/share" >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]
