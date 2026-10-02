# SPDX-License-Identifier: Apache-2.0
"""Launch AMCL and Nav2; simulation owns robot odometry and robot TF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory("factory_bringup"))
    nav2 = Path(get_package_share_directory("nav2_bringup"))
    common = {
        "use_sim_time": "true",
        "autostart": "true",
        "use_composition": "False",
        "params_file": LaunchConfiguration("params_file"),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", default_value=str(package / "maps/factory_map.yaml")
            ),
            DeclareLaunchArgument(
                "params_file", default_value=str(package / "config/nav2_params.yaml")
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2 / "launch/localization_launch.py")
                ),
                launch_arguments={**common, "map": LaunchConfiguration("map")}.items(),
            ),
            # Both motion producers feed the sole smoother. Only its output reaches
            # the Gazebo drive plugin, including during a recovery behavior.
            *[
                Node(
                    package=node_package,
                    executable=name,
                    name=name,
                    output="screen",
                    parameters=[
                        LaunchConfiguration("params_file"),
                        {"use_sim_time": True},
                    ],
                    remappings=remappings,
                )
                for node_package, name, remappings in [
                    (
                        "nav2_controller",
                        "controller_server",
                        [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ("nav2_smoother", "smoother_server", []),
                    ("nav2_planner", "planner_server", []),
                    ("nav2_behaviors", "behavior_server", [("cmd_vel", "cmd_vel_nav")]),
                    ("nav2_bt_navigator", "bt_navigator", []),
                    ("nav2_waypoint_follower", "waypoint_follower", []),
                    (
                        "nav2_velocity_smoother",
                        "velocity_smoother",
                        [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
                    ),
                ]
            ],
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": [
                            "controller_server",
                            "smoother_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                            "velocity_smoother",
                        ],
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", str(package / "config/rviz_factory.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
