"""Read-only HTTP readiness must come from the manager's own DDS clients."""

import json
from pathlib import Path
import time
from urllib.request import urlopen

from factory_interfaces.action import ExecuteFactoryMission
from factory_interfaces.srv import EstimateMissionCost
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from fleet_manager.node import FleetManagerNode


@pytest.mark.parametrize("offered", ["cost", "action", "both"])
def test_http_readiness_observes_manager_clients_without_changing_journal(
    tmp_path, offered
):
    context = Context()
    rclpy.init(context=context, domain_id=79)
    manager = FleetManagerNode(
        context=context,
        parameter_overrides=[
            Parameter(
                "fleet_file",
                value=str(
                    Path(__file__).resolve().parents[2]
                    / "factory_bringup/config/fleet.yaml"
                ),
            ),
            Parameter("journal_path", value=str(tmp_path / "fleet.sqlite3")),
            Parameter("dashboard_enabled", value=True),
            Parameter("dashboard_port", value=0),
        ],
    )
    peer = rclpy.create_node("readiness_peer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    service, action = None, None
    if offered in ("cost", "both"):
        service = peer.create_service(
            EstimateMissionCost,
            "/amr_01/factory/estimate_mission_cost",
            lambda request, response: response,
        )
    if offered in ("action", "both"):
        action = ActionServer(
            peer,
            ExecuteFactoryMission,
            "/amr_01/factory/execute_mission",
            lambda goal: None,
        )
    expected = {
        "amr_01": {
            "cost_service_ready": offered in ("cost", "both"),
            "mission_action_ready": offered in ("action", "both"),
        },
        "amr_02": {"cost_service_ready": False, "mission_action_ready": False},
    }
    try:
        deadline = time.monotonic() + 3
        while True:
            actual = manager._dispatch_readiness()
            if actual == expected:
                break
            assert time.monotonic() < deadline, actual
            executor.spin_once(timeout_sec=0.01)
        changes = manager.journal._connection.total_changes
        assert manager.dashboard.capture(manager.adapter, actual)
        while True:
            with urlopen(
                f"http://127.0.0.1:{manager.dashboard.address[1]}/api/snapshot",
                timeout=1,
            ) as response:
                snapshot = json.load(response)
            rows = {
                robot["robot_id"]: {
                    field: robot.get(field)
                    for field in ("cost_service_ready", "mission_action_ready")
                }
                for robot in snapshot["robots"]
            }
            if rows == expected:
                break
            assert time.monotonic() < deadline, rows
            executor.spin_once(timeout_sec=0.01)
        assert manager.journal._connection.total_changes == changes
    finally:
        if action is not None:
            action.destroy()
        if service is not None:
            peer.destroy_service(service)
        executor.remove_node(peer)
        executor.shutdown()
        peer.destroy_node()
        manager.destroy_node()
        context.shutdown()
