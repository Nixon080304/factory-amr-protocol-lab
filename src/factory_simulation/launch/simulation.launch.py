# SPDX-License-Identifier: Apache-2.0
"""Compatible single-robot simulation; fleet.launch.py repeats the same actions."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from factory_simulation.description import robot_actions, simulator_actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = Path(get_package_share_directory("factory_simulation"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument(
                "world", default_value=str(package / "worlds/factory_floor.world")
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="-3.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            *simulator_actions(
                LaunchConfiguration("world"), LaunchConfiguration("gui")
            ),
            *robot_actions(
                "",
                "",
                "factory_amr",
                LaunchConfiguration("x"),
                LaunchConfiguration("y"),
                LaunchConfiguration("yaw"),
            ),
        ]
    )
