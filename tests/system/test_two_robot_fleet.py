# SPDX-License-Identifier: Apache-2.0
"""Fleet release contracts: run the production launch and public drivers."""

import importlib.util
from pathlib import Path
import subprocess

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]


def test_fleet_demo_wires_both_robots_and_owned_station_payload_boundaries(monkeypatch):
    path = ROOT / "src/factory_bringup/launch/fleet_demo.launch.py"
    assert path.is_file(), "production two-robot protocol composition is missing"
    spec = importlib.util.spec_from_file_location("fleet_demo_contract", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, "get_package_share_directory", lambda p: str(ROOT / "src" / p)
    )
    context = LaunchContext()
    context.launch_configurations.update(
        fleet_file=str(ROOT / "src/factory_bringup/config/fleet.yaml"),
        broker_port="1883",
        plc_port="1502",
        control_port="1503",
        output_dir="artifacts/contract",
        dashboard_port="8080",
        journal_path="artifacts/contract/fleet.sqlite3",
    )
    nodes = [n for n in module.shared_nodes(context) if isinstance(n, Node)]
    shared = {
        n.node_package: n
        for n in nodes
        if n.node_package in ("fleet_manager", "modbus_gateway", "payload_simulator")
    }
    assert set(shared) == {"fleet_manager", "modbus_gateway", "payload_simulator"}
    values = {
        p: {
            k: v
            for group in evaluate_parameters(context, n._Node__parameters)
            for k, v in group.items()
        }
        for p, n in shared.items()
    }
    assert values["modbus_gateway"]["robot_ids"] == ("amr_01", "amr_02")
    assert values["modbus_gateway"]["fault_control_port"] == 1503
    assert values["payload_simulator"]["robot_entities"] == ("amr_01", "amr_02")
    assert values["payload_simulator"]["part_entities"] == (
        "amr_01_part",
        "amr_02_part",
    )
    perception = [n for n in nodes if n.node_package == "station_perception"]
    assert len(perception) == 2
    for robot, node in zip(("amr_01", "amr_02"), perception):
        node._perform_substitutions(context)
        assert node.expanded_node_namespace == "/" + robot
    starting = [n for n in module.compose(context) if isinstance(n, Node)]
    assert len(starting) == 1 and starting[0].node_executable == "factory_wait_ready"
    params = evaluate_parameters(context, starting[0]._Node__parameters)[0]
    assert params["robot_namespaces"] == ("/amr_01", "/amr_02")
    from types import SimpleNamespace
    from launch.actions import EmitEvent

    assert (
        module.after_readiness(SimpleNamespace(returncode=0), context, nodes) == nodes
    )
    assert isinstance(
        module.after_readiness(SimpleNamespace(returncode=1), context, nodes)[0],
        EmitEvent,
    )


@pytest.mark.parametrize("script", ["run_fleet_demo.sh", "run_fleet_acceptance.sh"])
@pytest.mark.parametrize(
    "arguments",
    [
        ["--unknown"],
        ["--headless", "--headless"],
        ["--timeout", "0"],
        ["--scenario", "missing"],
        ["--timeout"],
        ["--headless", "one", "two"],
    ],
)
def test_public_driver_rejects_bad_arguments_before_starting_services(
    script, arguments, tmp_path
):
    path = ROOT / "scripts" / script
    assert path.is_file(), "bounded fleet driver is missing"
    result = subprocess.run(
        ["bash", str(path), *arguments],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 2, result.stdout + result.stderr
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("name", ["nominal", "contention", "robot_failure", "charging"])
def test_scenario_declares_real_proof_and_bounded_requirements(name):
    path = ROOT / f"tests/scenarios/fleet_{name}.yaml"
    assert path.is_file(), "fleet acceptance scenario is missing"
    scenario = yaml.safe_load(path.read_text())
    assert scenario["name"] == name
    assert scenario["timeout_sec"] > 0
    assert scenario["proof"] in ("gazebo_nav2", "production_dds")
    assert scenario["assertions"]


def test_charging_scenario_requires_real_post_mission_dock_handoff():
    scenario = yaml.safe_load(
        (ROOT / "tests/scenarios/fleet_charging.yaml").read_text()
    )
    assert scenario["initial_battery"] == {"amr_01": 31.0, "amr_02": 25.0}
    assert {
        "both_contact_to_80",
        "post_mission_below_30",
        "exact_dock_handoff",
    }.issubset(scenario["assertions"])


def test_real_dds_robot_failure_blocks_until_stop_clear_and_preserves_carrier(tmp_path):
    probe = ROOT / "tests/system/fleet_failure_probe.py"
    assert probe.is_file(), "production DDS robot failure proof is missing"
    result = subprocess.run(
        [str(ROOT / ".venv/bin/python3"), str(probe), str(tmp_path)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    import json

    receipt = json.loads((tmp_path / "failure-receipt.json").read_text())
    assert receipt["before_pickup"] == "COMPLETED"
    assert receipt["blocked_before_stop_clear"] is True
    assert receipt["after_pickup"] == "RECOVERY_REQUIRED"
    assert receipt["carrier"] == "amr_01"
    assert receipt["reassigned_robot"] == "amr_02"
