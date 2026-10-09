# SPDX-License-Identifier: Apache-2.0
"""Inspect generated launch entities without starting processes or DDS."""

import importlib.util
import math
from pathlib import Path
import shutil

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


@pytest.mark.parametrize("namespace", ["/amr_01", "/warehouse/cart"])
def test_demo_routes_gateway_through_fleet_and_configured_robot(
    namespace, tmp_path, monkeypatch
):
    package = Path(__file__).resolve().parents[1]
    share = tmp_path / "factory_bringup"
    shutil.copytree(package / "config", share / "config")
    fleet_file = share / "config/fleet.yaml"
    config = yaml.safe_load(fleet_file.read_text())
    config["robots"][0]["namespace"] = namespace
    fleet_file.write_text(yaml.safe_dump(config))
    spec = importlib.util.spec_from_file_location(
        "demo_launch", package / "launch/demo.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda name: str(share if name == "factory_bringup" else package.parent / name),
    )
    description = module.generate_launch_description()
    nodes = {
        node.node_package: node
        for node in description.entities
        if isinstance(node, Node)
    }
    assert "fleet_manager" in nodes, "MQTT fleet action needs its durable server"
    context = LaunchContext()
    context.launch_configurations.update(
        broker_port="1883",
        plc_port="1502",
        output_dir="artifacts/traces",
        fleet_file=str(fleet_file),
        journal_path=str(tmp_path / "missions.sqlite3"),
    )
    for name in ("fleet_manager", "mqtt_gateway"):
        parameters = evaluate_parameters(context, nodes[name]._Node__parameters)
        merged = {
            key: value for parameter in parameters for key, value in parameter.items()
        }
        assert merged["fleet_file"] == str(fleet_file)
    coordinator = nodes["mission_coordinator"]
    coordinator._perform_substitutions(context)
    assert coordinator.expanded_node_namespace == "/"
    assert coordinator.expanded_remapping_rules is None, (
        "V1 root action compatibility must not rely on ineffective action-base remaps"
    )
    from launch.actions import IncludeLaunchDescription

    includes = [
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(includes) == 2
    for include in includes:
        arguments = dict(include.launch_arguments)
        assert [float(arguments[key]) for key in ("x", "y", "yaw")] == [-1.1, -3.4, 0.0]
    assert "robot_agent" in nodes, "V1 demo must publish configured robot health"
    agent = nodes["robot_agent"]
    agent._perform_substitutions(context)
    assert agent.expanded_node_namespace == namespace
    values = evaluate_parameters(context, agent._Node__parameters)[0]
    assert values["robot_id"] == "amr_01" and values["frame_prefix"] == ""
    assert ("odom", "/odom") in agent.expanded_remapping_rules
    assert ("amcl_pose", "/amcl_pose") in agent.expanded_remapping_rules
    assert (
        "factory/mission_state",
        "/factory/mission_state",
    ) in agent.expanded_remapping_rules
    assert not any(
        source in ("compute_path_to_pose", "navigate_to_pose")
        for source, _ in agent.expanded_remapping_rules
    ), "legacy Nav2 actions must use explicit endpoints, not ineffective base remaps"
    sensor = nodes.get("factory_simulation")
    assert sensor is not None, "V1 demo requires independent simulation dock contact"
    sensor._perform_substitutions(context)
    assert sensor.expanded_node_namespace == namespace
    sensor_values = evaluate_parameters(context, sensor._Node__parameters)[0]
    assert sensor_values["entity_name"] == "factory_amr"
    assert sensor_values["dock_id"] == "dock_01"
    assert sensor_values["charging_pose"] == (4.0, -2.0, math.pi)


def test_fleet_contact_publishers_follow_configured_namespaces_and_dock(monkeypatch):
    from factory_bringup import fleet_launch
    from fleet_manager.config import load_fleet_config
    from dataclasses import replace

    package = Path(__file__).resolve().parents[1]
    monkeypatch.setattr(
        fleet_launch,
        "get_package_share_directory",
        lambda name: str(package.parent / name),
    )
    config = load_fleet_config(package / "config/fleet.yaml")
    config = replace(
        config,
        robots=tuple(
            replace(robot, namespace="/warehouse/" + robot.robot_id)
            for robot in config.robots
        ),
    )
    nodes = [
        node
        for node in fleet_launch.fleet_launch_actions(config, gui="false", rviz="false")
        if isinstance(node, Node) and node.node_package == "factory_simulation"
    ]
    assert len(nodes) == len(config.robots)
    context = LaunchContext()
    for node, robot in zip(nodes, config.robots):
        node._perform_substitutions(context)
        assert node.expanded_node_namespace == robot.namespace
        values = evaluate_parameters(context, node._Node__parameters)[0]
        assert (
            values["entity_name"] == robot.robot_id
            and values["dock_id"] == config.energy.dock_id
        )
        assert values["charging_pose"] == (4.0, -2.0, math.pi)


@pytest.mark.parametrize("enabled,port", [("true", "8080"), ("false", "9090")])
def test_demo_has_one_configurable_node_owned_dashboard(monkeypatch, enabled, port):
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "dashboard_demo_launch", package / "launch/demo.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, "get_package_share_directory", lambda name: str(package.parent / name)
    )
    description = module.generate_launch_description()
    managers = [
        node
        for node in description.entities
        if isinstance(node, Node) and node.node_package == "fleet_manager"
    ]
    assert len(managers) == 1
    context = LaunchContext()
    context.launch_configurations.update(journal_path="artifacts/fleet/test.sqlite3")
    if enabled == "false":
        context.launch_configurations.update(
            dashboard_enabled=enabled, dashboard_port=port
        )
    values = evaluate_parameters(context, managers[0]._Node__parameters)
    merged = {key: value for parameter in values for key, value in parameter.items()}
    assert merged["dashboard_enabled"] is (enabled == "true")
    assert merged["dashboard_port"] == int(port)


@pytest.mark.parametrize("port", [None, "32001"])
def test_demo_forwards_optional_owned_plc_listener(monkeypatch, port):
    """Removing listener forwarding must break the ownership-enabled case."""
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "ownership_demo_launch", package / "launch/demo.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, "get_package_share_directory", lambda name: str(package.parent / name)
    )
    description = module.generate_launch_description()
    gateways = [
        entity
        for entity in description.entities
        if isinstance(entity, Node) and entity.node_package == "modbus_gateway"
    ]
    assert len(gateways) == 1
    context = LaunchContext()
    context.launch_configurations.update(plc_port="1502")
    if port is not None:
        context.launch_configurations["fault_control_port"] = port
    values = evaluate_parameters(context, gateways[0]._Node__parameters)
    merged = {key: value for parameter in values for key, value in parameter.items()}
    assert merged.get("fault_control_port") == (0 if port is None else int(port))
