"""One-command demo for the VIVA.

  # Nav2 (Dijkstra) + RViz, click goals with the 2D Goal Pose tool
  ros2 launch maze_bringup demo.launch.py

  # Nav2 with A* planner
  ros2 launch maze_bringup demo.launch.py planner:=astar

  # DQN policy instead of Nav2
  ros2 launch maze_bringup demo.launch.py method:=dqn model_path:=models/dqn_maze.zip
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    method = LaunchConfiguration("method")
    planner = LaunchConfiguration("planner")
    model_path = LaunchConfiguration("model_path")
    headless = LaunchConfiguration("headless")

    nav_share = get_package_share_directory("maze_navigation")
    bringup_share = get_package_share_directory("maze_bringup")

    dijkstra_params = os.path.join(nav_share, "config", "nav2_params.yaml")
    astar_params = os.path.join(nav_share, "config", "nav2_params_astar.yaml")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "planner": planner,
            "params_file": PythonExpression([
                "'", astar_params, "' if '", planner, "' == 'astar' else '",
                dijkstra_params, "'",
            ]),
        }.items(),
    )

    dqn_node = Node(
        package="maze_dqn",
        executable="dqn_nav_node",
        name="dqn_nav_node",
        output="screen",
        parameters=[{"model_path": model_path, "use_sim_time": True}],
        condition=IfCondition(PythonExpression(["'", method, "' == 'dqn'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument("method", default_value="nav2",
                              description="nav2 or dqn"),
        DeclareLaunchArgument("planner", default_value="dijkstra",
                              description="dijkstra or astar (nav2 only)"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="trained DQN model (.zip)"),
        DeclareLaunchArgument("headless", default_value="false"),
        navigation,
        TimerAction(period=15.0, actions=[dqn_node]),
    ])
