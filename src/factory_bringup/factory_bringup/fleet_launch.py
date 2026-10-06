# SPDX-License-Identifier: Apache-2.0
"""Pure robot contracts and their production launch consumers."""

from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

from ament_index_python.packages import get_package_share_directory
from factory_simulation.description import robot_actions, simulator_actions
from fleet_manager.config import FleetConfig, Pose2D, RobotConfig
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import yaml


LOCALIZATION_NODES = ("map_server", "amcl")
NAVIGATION_NODES = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
)
ROBOT_FRAMES = (
    "map",
    "odom",
    "base_footprint",
    "base_link",
    "wheel_left_link",
    "wheel_right_link",
    "caster_link",
    "base_scan",
    "imu_link",
    "camera_link",
    "camera_optical_frame",
)


@dataclass(frozen=True)
class RobotLaunchSpec:
    robot_id: str
    namespace: str
    frame_prefix: str
    entity_name: str
    spawn: Pose2D
    description_topic: str
    xacro_args: Mapping[str, str]
    frames: Mapping[str, str]
    nav2_root: str
    localization_nodes: tuple[str, ...]
    navigation_nodes: tuple[str, ...]


def robot_launch_specs(config: FleetConfig) -> tuple[RobotLaunchSpec, ...]:
    """Derive identities without filesystem, ROS graph, or launch-context access."""
    return tuple(robot_launch_spec(robot) for robot in config.robots)


def coordinator_parameters(config: FleetConfig, robot: RobotConfig) -> dict:
    """Carry validated fleet routes across the Python/C++ parameter boundary."""
    result = {
        "robot_id": robot.robot_id,
        "frame_prefix": robot.frame_prefix,
        "resource_leases_enabled": True,
        "route_names": list(config.routes),
    }
    traffic = {r.resource_id for r in config.resources if r.kind == "traffic_zone"}
    for name, resources in config.routes.items():
        segments = config.route_segments.get(name, ())
        if tuple(s.resource_id for s in segments) != tuple(
            r for r in resources if r in traffic
        ):
            raise ValueError(
                f"route_segments.{name}: missing traffic staging/exit geometry"
            )
        # Humble launch cannot evaluate an empty array. The declared C++ default
        # is an empty string array for routes without a traffic segment.
        if segments:
            result[f"routes.{name}.resources"] = [s.resource_id for s in segments]
        for segment in segments:
            result[f"routes.{name}.{segment.resource_id}.bounds"] = list(
                config.traffic_bounds[segment.resource_id]
            )
            for key in ("staging_pose", "exit_pose"):
                pose = getattr(segment, key)
                result[f"routes.{name}.{segment.resource_id}.{key}"] = [
                    pose.x,
                    pose.y,
                    pose.yaw,
                ]
    return result


def agent_parameters(config, robot, stations):
    """Pass the same configured policy and map-origin station poses to each agent."""
    result = {
        "robot_id": robot.robot_id,
        "frame_prefix": robot.frame_prefix,
        "use_sim_time": True,
        "battery_start_percent": robot.battery_start_percent,
        "station_names": list(stations),
        "dock_id": config.energy.dock_id,
    }
    for name, values in stations.items():
        result[f"stations.{name}.pose"] = [float(values[k]) for k in ("x", "y", "yaw")]
    dock = config.docks[config.energy.dock_id]
    for name in ("staging_pose", "charging_pose"):
        pose = getattr(dock, name)
        result[f"dock.{name}"] = [pose.x, pose.y, pose.yaw]
    for name in (
        "reserve_percent",
        "idle_percent_per_sec",
        "move_percent_per_m",
        "operation_percent",
        "charge_percent_per_sec",
        "dock_allowance_m",
    ):
        result["energy." + name] = getattr(config.energy, name)
    return result


def robot_launch_spec(robot: RobotConfig) -> RobotLaunchSpec:
    return RobotLaunchSpec(
        robot.robot_id,
        robot.namespace,
        robot.frame_prefix,
        robot.robot_id,
        robot.spawn,
        robot.namespace + "/robot_description",
        MappingProxyType(
            {"namespace": robot.namespace, "frame_prefix": robot.frame_prefix}
        ),
        MappingProxyType({frame: robot.frame_prefix + frame for frame in ROBOT_FRAMES}),
        robot.namespace.lstrip("/"),
        tuple(robot.namespace + "/" + name for name in LOCALIZATION_NODES),
        tuple(robot.namespace + "/" + name for name in NAVIGATION_NODES),
    )


def nav2_parameters(spec: RobotLaunchSpec, source_file) -> RewrittenYaml:
    """Use Humble's standard rewrite, including full paths for map versus odom."""
    frames = spec.frames
    rewrites = {
        "use_sim_time": "true",
        "base_frame_id": frames["base_footprint"],
        "robot_base_frame": frames["base_footprint"],
        "odom_frame_id": frames["odom"],
        "global_frame_id": frames["map"],
        "amcl.ros__parameters.initial_pose.x": str(spec.spawn.x),
        "amcl.ros__parameters.initial_pose.y": str(spec.spawn.y),
        "amcl.ros__parameters.initial_pose.yaw": str(spec.spawn.yaw),
        "bt_navigator.ros__parameters.global_frame": frames["map"],
        "behavior_server.ros__parameters.global_frame": frames["odom"],
        "map_server.ros__parameters.frame_id": frames["map"],
    }
    for name, frame in (("local_costmap", "odom"), ("global_costmap", "map")):
        path = f"{name}.{name}.ros__parameters"
        rewrites[f"{path}.global_frame"] = frames[frame]
        # Costmap child nodes have an extra namespace. Resolve sensors/maps at the
        # robot root explicitly, never against /<robot>/<costmap>/scan or /scan.
        rewrites[f"{path}.obstacle_layer.scan.topic"] = spec.namespace + "/scan"
        rewrites[f"{path}.static_layer.map_topic"] = spec.namespace + "/map"
    return RewrittenYaml(
        source_file=str(source_file),
        root_key=spec.nav2_root,
        param_rewrites=rewrites,
        convert_types=True,
    )


def navigation_launch_actions(spec, *, params_file, map_file, rviz):
    bringup = Path(get_package_share_directory("factory_bringup"))
    nav2 = Path(get_package_share_directory("nav2_bringup"))
    rewritten = nav2_parameters(spec, params_file)
    configured = ParameterFile(rewritten, allow_substs=True)
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    actions = [
        GroupAction(
            actions=[
                PushRosNamespace(spec.namespace),
                # The upstream localization launch rewrites again. Its empty root keeps
                # the already rooted file intact; PushRosNamespace scopes the real nodes.
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(nav2 / "launch/localization_launch.py")
                    ),
                    launch_arguments={
                        "namespace": "",
                        "use_sim_time": "true",
                        "autostart": "true",
                        "use_composition": "False",
                        "params_file": rewritten,
                        "map": map_file,
                    }.items(),
                ),
            ]
        )
    ]
    for package, name, remaps in (
        ("nav2_controller", "controller_server", [("cmd_vel", "cmd_vel_nav")]),
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
    ):
        actions.append(
            Node(
                package=package,
                executable=name,
                name=name,
                namespace=spec.namespace,
                parameters=[configured],
                remappings=tf_remaps + remaps,
                output="screen",
            )
        )
    actions.extend(
        [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                namespace=spec.namespace,
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": list(spec.navigation_nodes),
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=spec.namespace,
                condition=IfCondition(rviz),
                arguments=[
                    "-d",
                    str(bringup / "config/rviz_factory.rviz"),
                    "-f",
                    spec.frames["map"],
                ],
                parameters=[{"use_sim_time": True}],
                remappings=tf_remaps,
                output="screen",
            ),
        ]
    )
    return actions


def fleet_launch_actions(
    config: FleetConfig, *, gui, rviz, world=None, map_file=None, params_file=None
):
    simulation = Path(get_package_share_directory("factory_simulation"))
    bringup = Path(get_package_share_directory("factory_bringup"))
    actions = simulator_actions(
        world or str(simulation / "worlds/factory_floor.world"), gui
    )
    stations = yaml.safe_load((bringup / "config/stations.yaml").read_text())[
        "stations"
    ]
    for spec in robot_launch_specs(config):
        actions.extend(
            robot_actions(
                spec.namespace,
                spec.frame_prefix,
                spec.entity_name,
                str(spec.spawn.x),
                str(spec.spawn.y),
                str(spec.spawn.yaw),
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup / "launch/navigation.launch.py")
                ),
                launch_arguments={
                    "namespace": spec.namespace,
                    "frame_prefix": spec.frame_prefix,
                    "x": str(spec.spawn.x),
                    "y": str(spec.spawn.y),
                    "yaw": str(spec.spawn.yaw),
                    "map": map_file or str(bringup / "maps/factory_map.yaml"),
                    "params_file": params_file
                    or str(bringup / "config/nav2_params.yaml"),
                    "rviz": rviz,
                }.items(),
            )
        )
        robot = next(
            robot for robot in config.robots if robot.robot_id == spec.robot_id
        )
        actions.append(
            Node(
                package="mission_coordinator",
                executable="mission_coordinator",
                namespace=spec.namespace,
                output="screen",
                parameters=[
                    {
                        **coordinator_parameters(config, robot),
                        **{
                            f"stations.{name}.pose": [
                                float(values[k]) for k in ("x", "y", "yaw")
                            ]
                            for name, values in stations.items()
                        },
                    }
                ],
            )
        )
        actions.append(
            Node(
                package="robot_agent",
                executable="robot_agent",
                namespace=spec.namespace,
                output="screen",
                parameters=[agent_parameters(config, robot, stations)],
            )
        )
    return actions
