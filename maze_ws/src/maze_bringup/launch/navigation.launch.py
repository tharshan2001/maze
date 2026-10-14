"""Full autonomous navigation: sim + saved map + AMCL + Nav2.

  ros2 launch maze_bringup navigation.launch.py                       # Dijkstra
  ros2 launch maze_bringup navigation.launch.py params_file:=<astar yaml>   # A*
  ros2 launch maze_bringup navigation.launch.py headless:=true

Nav2 nodes are declared explicitly (instead of nav2_bringup's default) so we do
not pull in route_server / collision_monitor / opennav_docking, which are not
needed for this maze and require extra parameters. cmd_vel is routed through the
velocity smoother: controller -> cmd_vel_nav -> smoother -> cmd_vel (to gz).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (package, executable, node_name, remappings)
NAV2_NODES = [
    ("nav2_controller", "controller_server", "controller_server",
     [("cmd_vel", "cmd_vel_nav")]),
    ("nav2_smoother", "smoother_server", "smoother_server", []),
    ("nav2_planner", "planner_server", "planner_server", []),
    ("nav2_behaviors", "behavior_server", "behavior_server",
     [("cmd_vel", "cmd_vel_nav")]),
    ("nav2_bt_navigator", "bt_navigator", "bt_navigator", []),
    ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower", []),
    ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother",
     [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
]


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    gz_share = get_package_share_directory("maze_gazebo")
    nav_share = get_package_share_directory("maze_navigation")
    bringup_share = get_package_share_directory("maze_bringup")

    dijkstra_params = os.path.join(nav_share, "config", "nav2_params.yaml")

    maze = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, "launch", "maze.launch.py")
        ),
        launch_arguments={"headless": headless,
                          "use_sim_time": use_sim_time}.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "localization.launch.py")
        )
    )

    nav2_nodes = [
        Node(package=pkg, executable=exe, name=name, output="screen",
             parameters=[params_file, {"use_sim_time": use_sim_time}],
             remappings=remaps)
        for (pkg, exe, name, remaps) in NAV2_NODES
    ]

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": [n for (_, _, n, _) in NAV2_NODES]},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=dijkstra_params,
                              description="Nav2 params; use nav2_params_astar.yaml for A*"),
        maze,
        TimerAction(period=6.0, actions=[localization]),
        TimerAction(period=9.0, actions=nav2_nodes + [lifecycle]),
    ])
