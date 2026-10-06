# SPDX-License-Identifier: Apache-2.0
"""Authoritative configuration-driven fleet simulation with one Gazebo world."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from factory_bringup.fleet_launch import fleet_launch_actions
from fleet_manager.config import load_fleet_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_fleet(context):
    values = {
        name: LaunchConfiguration(name).perform(context)
        for name in ("fleet_file", "gui", "rviz", "world", "map", "params_file")
    }
    config = load_fleet_config(Path(values.pop("fleet_file")))
    values["map_file"] = values.pop("map")
    return fleet_launch_actions(config, **values)


def generate_launch_description():
    bringup = Path(get_package_share_directory("factory_bringup"))
    simulation = Path(get_package_share_directory("factory_simulation"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fleet_file", default_value=str(bringup / "config/fleet.yaml")
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "world", default_value=str(simulation / "worlds/factory_floor.world")
            ),
            DeclareLaunchArgument(
                "map", default_value=str(bringup / "maps/factory_map.yaml")
            ),
            DeclareLaunchArgument(
                "params_file", default_value=str(bringup / "config/nav2_params.yaml")
            ),
            OpaqueFunction(function=launch_fleet),
        ]
    )
