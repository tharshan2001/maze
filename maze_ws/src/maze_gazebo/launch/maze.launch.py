"""Start the Gazebo maze world and spawn the TurtleBot3 Burger.

  ros2 launch maze_gazebo maze.launch.py            # GUI
  ros2 launch maze_gazebo maze.launch.py headless:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

PKG = "maze_gazebo"
xacro_file = os.path.join(
    get_package_share_directory(PKG), "urdf", "turtlebot3_burger.urdf.xacro"
)


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            # headless:=true -> server only (-s) with offscreen rendering so the
            # GPU LIDAR still produces scans without a display.
            "gz_args": PythonExpression([
                "'-r -v 3 ' + '",
                PathJoinSubstitution([FindPackageShare(PKG), "worlds", "maze.world"]),
                "' if '", headless, "' != 'true' else '-s -r -v 3 --headless-rendering ' + '",
                PathJoinSubstitution([FindPackageShare(PKG), "worlds", "maze.world"]),
                "'",
            ]),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "turtlebot3_burger",
            "-topic", "robot_description",
            "-x", "-1.25", "-y", "1.25", "-z", "0.05", "-Y", "0.0",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory(PKG), "rviz", "maze.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(headless),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               os.path.join(get_package_share_directory(PKG), "..")),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
    ])
