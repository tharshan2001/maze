"""Simulation + SLAM Toolbox mapping.

Drive the robot around the maze (teleop) to build the map, then save it with:

  ros2 run nav2_map_server map_saver_cli -f maze_ws/src/maze_navigation/maps/maze
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("maze_bringup")

    maze = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("maze_gazebo"),
                         "launch", "maze.launch.py")
        )
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(share, "config", "slam_toolbox.yaml"),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        maze,
        TimerAction(period=3.0, actions=[slam]),
    ])
