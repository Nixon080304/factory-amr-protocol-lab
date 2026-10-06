# SPDX-License-Identifier: Apache-2.0
"""Inspect generated launch entities without starting processes or DDS."""

import importlib.util
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
    assert (
        "/factory/execute_mission",
        namespace + "/factory/execute_mission",
    ) in coordinator.expanded_remapping_rules
