"""Saved map + AMCL localization component (no simulator, no Nav2).

Normally included by navigation.launch.py / demo.launch.py. Start the simulator
separately (maze_gazebo/maze.launch.py) if running this on its own.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("maze_navigation")
    map_yaml = os.path.join(nav_share, "maps", "maze.yaml")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml, "use_sim_time": True}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            os.path.join(nav_share, "config", "amcl_params.yaml"),
            {"use_sim_time": True},
        ],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    return LaunchDescription([
        TimerAction(period=3.0, actions=[map_server, amcl, lifecycle]),
    ])
