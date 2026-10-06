# SPDX-License-Identifier: Apache-2.0
"""Launch one reusable namespaced AMCL/Nav2 stack, including V1 empty namespace."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from factory_bringup.fleet_launch import navigation_launch_actions, robot_launch_spec
from fleet_manager.config import Pose2D, RobotConfig
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_navigation(context):
    values = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "namespace",
            "frame_prefix",
            "x",
            "y",
            "yaw",
            "params_file",
            "map",
            "rviz",
        )
    }
    pose = Pose2D(*(float(values[name]) for name in ("x", "y", "yaw")))
    robot = RobotConfig(
        "factory_amr", values["namespace"], values["frame_prefix"], pose, 100.0
    )
    return navigation_launch_actions(
        robot_launch_spec(robot),
        params_file=values["params_file"],
        map_file=values["map"],
        rviz=values["rviz"],
    )


def generate_launch_description():
    package = Path(get_package_share_directory("factory_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("frame_prefix", default_value=""),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="-3.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map", default_value=str(package / "maps/factory_map.yaml")
            ),
            DeclareLaunchArgument(
                "params_file", default_value=str(package / "config/nav2_params.yaml")
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=launch_navigation),
        ]
    )
