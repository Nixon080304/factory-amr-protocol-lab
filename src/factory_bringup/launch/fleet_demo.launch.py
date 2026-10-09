# SPDX-License-Identifier: Apache-2.0
"""Two complete robot stacks and one shared set of fleet/protocol owners."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from fleet_manager.config import load_fleet_config
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def shared_nodes(context):
    bringup = Path(get_package_share_directory("factory_bringup"))
    values = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "fleet_file",
            "broker_port",
            "plc_port",
            "control_port",
            "output_dir",
            "dashboard_port",
            "journal_path",
        )
    }
    config = load_fleet_config(Path(values["fleet_file"]))
    robots = [r.robot_id for r in config.robots]
    shared = [
        (
            "fleet_manager",
            "fleet_manager",
            {
                "fleet_file": values["fleet_file"],
                "journal_path": values["journal_path"],
                "dashboard_enabled": True,
                "dashboard_port": int(values["dashboard_port"]),
            },
        ),
        (
            "mqtt_gateway",
            "mqtt_gateway",
            {
                "fleet_file": values["fleet_file"],
                "broker_port": int(values["broker_port"]),
            },
        ),
        (
            "modbus_gateway",
            "modbus_gateway",
            {
                "robot_ids": robots,
                "plc_port": int(values["plc_port"]),
                "fault_control_port": int(values["control_port"]),
            },
        ),
        (
            "payload_simulator",
            "payload_simulator",
            {
                "robot_ids": robots,
                "robot_entities": robots,
                "part_entities": [r + "_part" for r in robots],
            },
        ),
        (
            "protocol_observer",
            "protocol_observer",
            {"output_dir": values["output_dir"]},
        ),
        (
            "fault_injector",
            "fault_injector",
            {
                "robot_ids": robots,
                "fault_owners": [
                    "mqtt_gateway",
                    "modbus_gateway",
                    "mission_coordinator",
                    "simulation",
                    "qos_experiment",
                ],
            },
        ),
    ]
    return [
        Node(
            package=p,
            executable=e,
            output="screen",
            parameters=[{"use_sim_time": True, **parameters}],
        )
        for p, e, parameters in shared
    ] + [
        Node(
            package="station_perception",
            executable="station_detector",
            namespace=r.namespace,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "stations_file": str(bringup / "config/stations.yaml"),
                }
            ],
        )
        for r in config.robots
    ]


def after_readiness(event, context, nodes):
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(reason="fleet Nav2 readiness failed"))]
    return nodes


def compose(context):
    config = load_fleet_config(Path(LaunchConfiguration("fleet_file").perform(context)))
    nodes = shared_nodes(context)
    probe = Node(
        package="factory_bringup",
        executable="factory_wait_ready",
        output="screen",
        parameters=[{"robot_namespaces": [r.namespace for r in config.robots]}],
    )
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=probe,
                on_exit=lambda event, ctx: after_readiness(event, ctx, nodes),
            )
        ),
        probe,
    ]


def generate_launch_description():
    bringup = Path(get_package_share_directory("factory_bringup"))
    simulation = Path(get_package_share_directory("factory_simulation"))
    defaults = {
        "fleet_file": str(bringup / "config/fleet.yaml"),
        "gui": "true",
        "rviz": "false",
        "world": str(simulation / "worlds/factory_floor.world"),
        "broker_port": "1883",
        "plc_port": "1502",
        "control_port": "1503",
        "dashboard_port": "8080",
        "output_dir": "artifacts/fleet",
        "journal_path": "artifacts/fleet/missions.sqlite3",
    }
    return LaunchDescription(
        [
            *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup / "launch/fleet.launch.py")),
                launch_arguments={
                    k: LaunchConfiguration(k)
                    for k in ("fleet_file", "gui", "rviz", "world")
                }.items(),
            ),
            OpaqueFunction(function=compose),
        ]
    )
