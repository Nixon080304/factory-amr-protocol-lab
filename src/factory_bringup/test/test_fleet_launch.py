# SPDX-License-Identifier: Apache-2.0
"""Inspect real launch actions and generated Nav2 parameters without DDS."""

from dataclasses import replace
import importlib
from pathlib import Path
import math
from types import MappingProxyType
import xml.etree.ElementTree as ET

from fleet_manager.config import Pose2D, RobotConfig, load_fleet_config
from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.utilities import evaluate_parameters
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def production():
    return importlib.import_module("factory_bringup.fleet_launch")


def test_launch_selects_installed_general_and_dock_trees():
    module = production()
    config = fleet(2)
    share = Path(module.get_package_share_directory("factory_bringup"))
    for robot in config.robots:
        parameters = module.agent_parameters(config, robot, {})
        assert parameters.get("dock.behavior_tree") == str(
            share / "behavior_trees/dock_to_pose.xml"
        )
        assert Path(parameters["dock.behavior_tree"]).is_file()
        path = module.nav2_parameters(
            module.robot_launch_spec(robot), PACKAGE / "config/nav2_params.yaml"
        ).perform(LaunchContext())
        navigator = yaml.safe_load(Path(path).read_text())[robot.namespace.strip("/")][
            "bt_navigator"
        ]["ros__parameters"]
        assert navigator.get("default_nav_to_pose_bt_xml") == str(
            share / "behavior_trees/navigate_to_pose.xml"
        )
        assert navigator.get("default_nav_through_poses_bt_xml") == str(
            share / "behavior_trees/navigate_through_poses.xml"
        )
        assert Path(navigator["default_nav_to_pose_bt_xml"]).is_file()
        assert Path(navigator["default_nav_through_poses_bt_xml"]).is_file()
        for path, controller, checker in (
            (parameters["dock.behavior_tree"], "DockFollowPath", "dock_goal_checker"),
            (
                navigator["default_nav_to_pose_bt_xml"],
                "GeneralFollowPath",
                "general_goal_checker",
            ),
            (
                navigator["default_nav_through_poses_bt_xml"],
                "GeneralFollowPath",
                "general_goal_checker",
            ),
        ):
            follows = ET.parse(path).findall(".//FollowPath")
            assert len(follows) == 1
            assert follows[0].get("controller_id") == controller
            assert follows[0].get("goal_checker_id") == checker


def test_custom_navigation_trees_remain_explicit_overrides(tmp_path):
    module = production()
    source = tmp_path / "custom.yaml"
    explicit = {
        "default_nav_to_pose_bt_xml": "/operator/custom_pose.xml",
        "default_nav_through_poses_bt_xml": "/operator/custom_poses.xml",
    }
    source.write_text(yaml.safe_dump({"bt_navigator": {"ros__parameters": explicit}}))
    robot = fleet(2).robots[0]
    path = module.nav2_parameters(module.robot_launch_spec(robot), source).perform(
        LaunchContext()
    )
    values = yaml.safe_load(Path(path).read_text())["amr_01"]["bt_navigator"][
        "ros__parameters"
    ]
    assert values["default_nav_to_pose_bt_xml"] == "/operator/custom_pose.xml"
    assert values["default_nav_through_poses_bt_xml"] == "/operator/custom_poses.xml"


def test_two_real_coordinators_receive_distinct_resolved_terminal_bays():
    config = fleet(2)
    values = [
        production().coordinator_parameters(config, robot) for robot in config.robots
    ]
    assert [params["stations.inspection.exit_pose"] for params in values] == [
        [4.2, 0.0, 0.0],
        [5.0, 2.4, 0.0],
    ]
    assert [params["frame_prefix"] + "map" for params in values] == [
        "amr_01/map",
        "amr_02/map",
    ]


def test_launch_rejects_station_file_divergence_from_authoritative_approach():
    config = fleet(2)
    approaches = dict(config.station_approach)
    approaches["inspection"] = Pose2D(3.1, 0.8, math.pi / 2)
    with pytest.raises(
        ValueError, match="diverges from authoritative station_approach"
    ):
        production().fleet_launch_actions(
            replace(config, station_approach=MappingProxyType(approaches)),
            gui="false",
            rviz="false",
        )


def test_launch_rejects_foreign_station_pose_frame(tmp_path, monkeypatch):
    module = production()
    document = yaml.safe_load((PACKAGE / "config/stations.yaml").read_text())
    document["frame_id"] = "amr_02/map"
    (tmp_path / "config").mkdir()
    (tmp_path / "config/stations.yaml").write_text(yaml.safe_dump(document))
    original = module.get_package_share_directory
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: (
            str(tmp_path) if package == "factory_bringup" else original(package)
        ),
    )
    with pytest.raises(ValueError, match="fleet approaches require the map frame"):
        module.fleet_launch_actions(fleet(2), gui="false", rviz="false")


def test_coordinator_parameters_carry_validated_routes_for_every_robot():
    # A missing route or copied identity would leave the coordinator ungated.
    module = production()
    config = fleet(10)
    for robot in config.robots:
        params = module.coordinator_parameters(config, robot)
        assert params["stations.clearance_distance"] == 0.60
        assert params["robot_id"] == robot.robot_id
        assert params["frame_prefix"] == robot.frame_prefix
        assert params["stations.assembly.bounds"] == [-3.5, -0.8, -1.7, 2.5]
        assert params["stations.assembly.staging_pose"] == [
            -2.3,
            -2.2,
            1.5707963267948966,
        ]
        assert params["stations.inspection.bounds"] == [1.7, -0.8, 3.5, 2.5]
        for station in ("assembly", "inspection"):
            pose = config.station_exit_poses[station][robot.robot_id]
            assert params[f"stations.{station}.exit_pose"] == [pose.x, pose.y, pose.yaw]
            approach = config.station_approach[station]
            assert params[f"stations.{station}.pose"] == [
                approach.x,
                approach.y,
                approach.yaw,
            ]
        assert params["resource_leases_enabled"] is True
        assert tuple(params["routes.to_assembly.resources"]) == ("central_aisle",)
        assert params["routes.to_assembly.central_aisle.staging_pose"] == [
            0.0,
            -3.5,
            1.5707963267948966,
        ]
        assert tuple(params["routes.assembly_to_inspection.resources"]) == (
            "central_aisle",
        )
        assert params["routes.assembly_to_inspection.central_aisle.staging_pose"] == [
            -1.5,
            -2.2,
            0.0,
        ]
        assert params["routes.assembly_to_inspection.central_aisle.exit_pose"] == [
            1.5,
            -2.2,
            0.0,
        ]


def test_fleet_launch_passes_route_geometry_to_each_real_coordinator():
    actions = production().fleet_launch_actions(fleet(2), gui="false", rviz="false")
    coordinators = [
        action
        for action in actions
        if isinstance(action, Node) and action.node_package == "mission_coordinator"
    ]
    assert len(coordinators) == 2
    context = LaunchContext()
    for robot, node in zip(fleet(2).robots, coordinators):
        node._perform_substitutions(context)
        assert node.expanded_node_namespace == robot.namespace
        params = evaluate_parameters(context, node._Node__parameters)[0]
        assert params["robot_id"] == robot.robot_id
        assert params["resource_leases_enabled"] is True
        assert tuple(params["routes.assembly_to_inspection.resources"]) == (
            "central_aisle",
        )


def fleet(size):
    config = load_fleet_config(PACKAGE / "config/fleet.yaml")
    if size == 2:
        return config
    robots = tuple(
        RobotConfig(
            f"cart_{i:02}",
            f"/warehouse/cart_{i:02}",
            f"floor/cart_{i:02}/",
            # Bounded north fan-out clears idle bays and committed map footprints.
            # Namespace inspection still does not claim ten-robot physical motion.
            Pose2D(
                7.04 * math.cos(math.pi / 2 + (i - 5.5) * 0.09),
                -3.5 + 7.04 * math.sin(math.pi / 2 + (i - 5.5) * 0.09),
                i / 10,
            ),
            100.0,
        )
        for i in range(1, size + 1)
    )
    # Namespace-only launch inspection uses generic configuration-derived bays.
    # These fan-out poses are not a ten-robot physical world or motion claim.
    parking = MappingProxyType(
        {
            station: MappingProxyType(
                {
                    robot.robot_id: Pose2D(
                        sign * (3 + 10 * math.cos(i * 0.12)),
                        0.8 + 10 * math.sin(i * 0.12),
                        0.0 if sign > 0 else math.pi,
                    )
                    for i, robot in enumerate(robots)
                }
            )
            for station, sign in (("assembly", -1), ("inspection", 1))
        }
    )
    return replace(
        config,
        robots=robots,
        station_exit_poses=parking,
        docks=MappingProxyType(
            {
                name: replace(
                    dock, exit_poses=MappingProxyType({}), departure_stations=()
                )
                for name, dock in config.docks.items()
            }
        ),
    )


@pytest.mark.parametrize("size", [2, 10])
def test_each_configured_robot_gets_energy_agent_station_frames_and_dock(size):
    # Missing agent or copied parameters leaves fleet candidates silent or coupled.
    config = fleet(size)
    actions = production().fleet_launch_actions(config, gui="false", rviz="false")
    agents = [
        a for a in actions if isinstance(a, Node) and a.node_package == "robot_agent"
    ]
    assert len(agents) == size
    context = LaunchContext()
    for robot, node in zip(config.robots, agents):
        node._perform_substitutions(context)
        assert node.expanded_node_namespace == robot.namespace
        params = evaluate_parameters(context, node._Node__parameters)[0]
        assert params["robot_id"] == robot.robot_id
        assert params["frame_prefix"] == robot.frame_prefix
        assert params["use_sim_time"] is True
        assert params["battery_start_percent"] == robot.battery_start_percent
        assert params["station_names"] == ("assembly", "inspection")
        assert params["stations.assembly.pose"] == (-3.0, 0.8, 1.5707963267948966)
        assert params["dock_id"] == "dock_01"
        assert params["dock.charging_pose"] == (4.0, -2.0, math.pi)
        assert params["energy.reserve_percent"] == 20.0
        assert params["energy.dock_allowance_m"] == 10.0


@pytest.mark.parametrize("size", [2, 10])
def test_specs_isolate_every_robot_stack(size):
    # Reusing entity, namespace, frame, description topic, or lifecycle paths couples robots.
    config = fleet(size)
    specs = production().robot_launch_specs(config)
    assert isinstance(specs, tuple) and len(specs) == size
    for attr in ("entity_name", "namespace", "description_topic", "nav2_root", "spawn"):
        assert len({getattr(spec, attr) for spec in specs}) == size
    all_frames = [frame for spec in specs for frame in spec.frames.values()]
    assert len(all_frames) == len(set(all_frames))
    for robot, spec in zip(config.robots, specs):
        assert spec.robot_id == robot.robot_id
        assert spec.namespace == robot.namespace
        assert spec.entity_name == robot.robot_id
        assert spec.description_topic == robot.namespace + "/robot_description"
        assert spec.xacro_args == {
            "namespace": robot.namespace,
            "frame_prefix": robot.frame_prefix,
        }
        assert spec.frames["map"] == robot.frame_prefix + "map"
        assert spec.frames["odom"] == robot.frame_prefix + "odom"
        assert spec.frames["base_footprint"] == robot.frame_prefix + "base_footprint"
        assert (
            spec.frames["camera_optical_frame"]
            == robot.frame_prefix + "camera_optical_frame"
        )
        assert spec.spawn == robot.spawn
        assert spec.nav2_root == robot.namespace.lstrip("/")
        assert spec.localization_nodes == (
            robot.namespace + "/map_server",
            robot.namespace + "/amcl",
        )
        assert spec.navigation_nodes == tuple(
            robot.namespace + "/" + node
            for node in (
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            )
        )


def test_rewritten_nav2_parameters_match_frames_initial_pose_and_namespace():
    # A stale shared frame, wrong pose, or doubly rooted parameter file breaks localization.
    module = production()
    context = LaunchContext()
    for spec in module.robot_launch_specs(fleet(2)):
        path = module.nav2_parameters(
            spec, PACKAGE / "config/nav2_params.yaml"
        ).perform(context)
        document = yaml.safe_load(Path(path).read_text())
        assert list(document) == [spec.nav2_root]
        params = document[spec.nav2_root]
        amcl = params["amcl"]["ros__parameters"]
        assert (
            amcl["global_frame_id"],
            amcl["odom_frame_id"],
            amcl["base_frame_id"],
        ) == (
            spec.frame_prefix + "map",
            spec.frame_prefix + "odom",
            spec.frame_prefix + "base_footprint",
        )
        assert amcl["initial_pose"] == {
            "x": spec.spawn.x,
            "y": spec.spawn.y,
            "z": 0.0,
            "yaw": spec.spawn.yaw,
        }
        assert amcl["scan_topic"] == "scan"
        assert (
            params["map_server"]["ros__parameters"]["frame_id"]
            == spec.frame_prefix + "map"
        )
        for name, frame in (("local_costmap", "odom"), ("global_costmap", "map")):
            costmap = params[name][name]["ros__parameters"]
            assert costmap["global_frame"] == spec.frame_prefix + frame
            assert costmap["robot_base_frame"] == spec.frame_prefix + "base_footprint"
            assert (
                costmap["obstacle_layer"]["scan"]["topic"] == spec.namespace + "/scan"
            )
        assert (
            params["global_costmap"]["global_costmap"]["ros__parameters"][
                "static_layer"
            ]["map_topic"]
            == spec.namespace + "/map"
        )
        assert (
            params["behavior_server"]["ros__parameters"]["global_frame"]
            == spec.frame_prefix + "odom"
        )
        assert params["bt_navigator"]["ros__parameters"]["odom_topic"] == "odom"


def test_custom_nav2_parameters_still_use_one_simulation_clock(tmp_path):
    # A custom file can otherwise put navigation on wall time while AMCL uses /clock.
    source = tmp_path / "nav2.yaml"
    source.write_text(
        "controller_server:\n  ros__parameters:\n    use_sim_time: false\n"
    )
    module = production()
    spec = module.robot_launch_specs(fleet(2))[0]
    path = module.nav2_parameters(spec, source).perform(LaunchContext())
    params = yaml.safe_load(Path(path).read_text())
    assert (
        params["amr_01"]["controller_server"]["ros__parameters"]["use_sim_time"] is True
    )


def test_fleet_launch_contains_two_descriptions_spawns_and_one_world():
    # Missing robot iteration, duplicate servers, or shared spawn description sources fails this.
    module = production()
    actions = module.fleet_launch_actions(fleet(2), gui="false", rviz="false")
    servers = [
        action
        for action in actions
        if isinstance(action, ExecuteProcess)
        and not isinstance(action, Node)
        and action.cmd[0][0].text == "gzserver"
    ]
    assert len(servers) == 1
    assert "factory_floor.world" in servers[0].cmd[1][0].text
    nodes = [action for action in actions if isinstance(action, Node)]
    descriptions = [
        node for node in nodes if node.node_package == "robot_state_publisher"
    ]
    spawns = [node for node in nodes if node.node_package == "gazebo_ros"]
    nav2 = [
        action for action in actions if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(descriptions) == len(spawns) == len(nav2) == 2
    context = LaunchContext()
    for index, robot in enumerate(fleet(2).robots):
        publisher = descriptions[index]
        publisher._perform_substitutions(context)
        assert publisher.expanded_node_namespace == robot.namespace
        params = evaluate_parameters(context, publisher._Node__parameters)[0]
        root = ET.fromstring(params["robot_description"])
        assert (
            root.find(f"link[@name='{robot.frame_prefix}base_footprint']") is not None
        )
        spawn = spawns[index]
        spawn._perform_substitutions(context)
        arguments = [
            perform_substitutions(context, normalize_to_list_of_substitutions(item))
            for item in spawn._Node__arguments
        ]
        for flag, value in (
            ("-entity", robot.robot_id),
            ("-topic", robot.namespace + "/robot_description"),
            ("-x", str(robot.spawn.x)),
            ("-y", str(robot.spawn.y)),
            ("-Y", str(robot.spawn.yaw)),
        ):
            assert arguments[arguments.index(flag) + 1] == value
        args = {
            key: perform_substitutions(
                context, normalize_to_list_of_substitutions(value)
            )
            for key, value in nav2[index].launch_arguments
        }
        assert args["namespace"] == robot.namespace
        assert args["frame_prefix"] == robot.frame_prefix


def test_fleet_rviz_observes_robot_frames_and_topics():
    # A V1 fixed frame or absolute sensor/map topic displays the wrong robot.
    module = production()
    spec = module.robot_launch_specs(fleet(2))[1]
    actions = module.navigation_launch_actions(
        spec,
        params_file=str(PACKAGE / "config/nav2_params.yaml"),
        map_file=str(PACKAGE / "maps/factory_map.yaml"),
        rviz="true",
    )
    rviz = next(
        action
        for action in actions
        if isinstance(action, Node) and action.node_package == "rviz2"
    )
    assert rviz._Node__arguments[-2:] == ["-f", "amr_02/map"]
    config = yaml.safe_load((PACKAGE / "config/rviz_factory.rviz").read_text())
    topics = []
    for display in (
        config["Visualization Manager"]["Displays"]
        + config["Visualization Manager"]["Tools"]
    ):
        for key in ("Topic", "Description Topic"):
            if key in display:
                topic = (
                    display[key]["Value"]
                    if isinstance(display[key], dict)
                    else display[key]
                )
                if not topic.startswith("/factory/"):
                    topics.append(topic)
    assert "scan" in topics and "robot_description" in topics and "map" in topics
    assert all(not topic.startswith("/") for topic in topics)


@pytest.mark.parametrize("size", [2, 10])
def test_navigation_actions_resolve_lifecycle_nodes_and_localization_parameters(size):
    # Unscoped Nav2 nodes, wrong lifecycle paths, or doubled YAML roots break startup.
    module = production()
    for spec in module.robot_launch_specs(fleet(size)):
        actions = module.navigation_launch_actions(
            spec,
            params_file=str(PACKAGE / "config/nav2_params.yaml"),
            map_file=str(PACKAGE / "maps/factory_map.yaml"),
            rviz="false",
        )
        context = LaunchContext()
        nodes = [action for action in actions if isinstance(action, Node)]
        resolved = []
        for node in nodes:
            node._perform_substitutions(context)
            assert node.expanded_node_namespace == spec.namespace
            resolved.append(node.node_name)
            if node.node_package == "nav2_lifecycle_manager":
                params = evaluate_parameters(context, node._Node__parameters)[0]
                # Nav2 bonds use server basenames inside the robot namespace.
                assert tuple(params["node_names"]) == tuple(
                    name.rsplit("/", 1)[-1] for name in spec.navigation_nodes
                )
                assert (
                    tuple(f"{spec.namespace}/{name}" for name in params["node_names"])
                    == spec.navigation_nodes
                )
            else:
                assert ("/tf", "tf") in node.expanded_remapping_rules
        assert set(spec.navigation_nodes) <= set(resolved)
        group = actions[0]
        included = next(
            action
            for action in group.get_sub_entities()
            if isinstance(action, IncludeLaunchDescription)
        )
        args = {
            key: perform_substitutions(
                context, normalize_to_list_of_substitutions(value)
            )
            for key, value in included.launch_arguments
        }
        assert args["namespace"] == ""
        params = yaml.safe_load(Path(args["params_file"]).read_text())
        assert list(params) == [spec.nav2_root]
        next(
            action
            for action in group.get_sub_entities()
            if isinstance(action, PushRosNamespace)
        ).execute(context)
        context.launch_configurations.update(args)
        upstream = included.launch_description_source.get_launch_description(context)
        for action in upstream.entities:
            if (
                isinstance(action, DeclareLaunchArgument)
                and action.name not in context.launch_configurations
            ):
                action.execute(context)
        upstream_group = next(
            action for action in upstream.entities if isinstance(action, GroupAction)
        )
        localization = [
            node for node in upstream_group.get_sub_entities() if isinstance(node, Node)
        ]
        assert len(localization) == 3
        for node in localization:
            node._perform_substitutions(context)
            assert node.expanded_node_namespace == spec.namespace
            params = evaluate_parameters(context, node._Node__parameters)
            if node.node_package == "nav2_lifecycle_manager":
                assert params[2]["node_names"] == ("map_server", "amcl")
            else:
                document = yaml.safe_load(Path(params[0]).read_text())
                assert list(document) == [spec.nav2_root]
                parsed = document[spec.nav2_root][
                    "map_server" if node.node_package == "nav2_map_server" else "amcl"
                ]["ros__parameters"]
                if node.node_package == "nav2_map_server":
                    assert parsed["frame_id"] == spec.frame_prefix + "map"
                    assert parsed["yaml_filename"].endswith("maps/factory_map.yaml")
                else:
                    assert parsed["global_frame_id"] == spec.frame_prefix + "map"


@pytest.mark.parametrize("size", [2, 10])
def test_installed_fleet_entrypoint_consumes_config_and_world_overrides(size, tmp_path):
    # Ignoring the launch fleet_file override silently launches the default two robots.
    from ament_index_python.packages import get_package_share_directory
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    data = yaml.safe_load((PACKAGE / "config/fleet.yaml").read_text())
    model = fleet(size)
    data["robots"] = [
        {
            "robot_id": robot.robot_id,
            "namespace": robot.namespace,
            "frame_prefix": robot.frame_prefix,
            "spawn": [robot.spawn.x, robot.spawn.y, robot.spawn.yaw],
            "battery_start_percent": robot.battery_start_percent,
        }
        for robot in model.robots
    ]
    data["station_exit_poses"] = {
        station: {
            robot_id: [pose.x, pose.y, pose.yaw] for robot_id, pose in poses.items()
        }
        for station, poses in model.station_exit_poses.items()
    }
    if size != 2:
        for dock in data["docks"].values():
            dock.pop("exit_poses", None)
            dock.pop("departure_stations", None)
    config = tmp_path / "fleet.yaml"
    config.write_text(yaml.safe_dump(data))
    # The scale-only fixture must satisfy the same full initial-route validator;
    # namespace inspection is not permission to use overlapping idle bays.
    validated = load_fleet_config(config)
    assert len(validated.robots) == size
    if size != 2:
        from PIL import Image

        metadata = yaml.safe_load((PACKAGE / "maps/factory_map.yaml").read_text())
        raster = Image.open(PACKAGE / "maps" / metadata["image"])
        resolution = metadata["resolution"]
        origin_x, origin_y, _ = metadata["origin"]
        for index, robot in enumerate(validated.robots):
            assert all(
                math.hypot(robot.spawn.x - other.spawn.x, robot.spawn.y - other.spawn.y)
                >= 0.60
                for other in validated.robots[:index]
            )
            for dx in range(-6, 7):
                for dy in range(-6, 7):
                    if math.hypot(dx * resolution, dy * resolution) > 0.30:
                        continue
                    column = math.floor((robot.spawn.x - origin_x) / resolution) + dx
                    row = (
                        raster.height
                        - 1
                        - math.floor((robot.spawn.y - origin_y) / resolution)
                        - dy
                    )
                    assert 0 <= column < raster.width and 0 <= row < raster.height
                    assert raster.getpixel((column, row)) >= 254
    share = Path(get_package_share_directory("factory_bringup"))
    context = LaunchContext()
    context.launch_configurations.update(
        fleet_file=str(config),
        world="/tmp/custom_factory.world",
        gui="false",
        rviz="false",
        map=str(PACKAGE / "maps/factory_map.yaml"),
        params_file=str(PACKAGE / "config/nav2_params.yaml"),
    )
    description = PythonLaunchDescriptionSource(
        str(share / "launch/fleet.launch.py")
    ).get_launch_description(context)
    callback = next(
        action for action in description.entities if isinstance(action, OpaqueFunction)
    )
    actions = callback.execute(context)
    publishers = [
        action
        for action in actions
        if isinstance(action, Node) and action.node_package == "robot_state_publisher"
    ]
    assert len(publishers) == size
    assert (
        len(
            [
                action
                for action in actions
                if isinstance(action, Node) and action.node_package == "gazebo_ros"
            ]
        )
        == size
    )
    assert (
        len(
            [
                action
                for action in actions
                if isinstance(action, IncludeLaunchDescription)
            ]
        )
        == size
    )
    servers = [
        action
        for action in actions
        if isinstance(action, ExecuteProcess)
        and not isinstance(action, Node)
        and perform_substitutions(context, action.cmd[0]) == "gzserver"
    ]
    assert len(servers) == 1
    assert (
        perform_substitutions(context, servers[0].cmd[1]) == "/tmp/custom_factory.world"
    )
