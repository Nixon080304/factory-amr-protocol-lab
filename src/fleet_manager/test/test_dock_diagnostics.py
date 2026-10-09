"""Docking transport evidence survives result mapping without logging authority tokens."""

import json
from types import SimpleNamespace
import pytest
import test_node

from fleet_manager.adapter import RobotReply
from fleet_manager.models import RobotSnapshot
from fleet_manager.config import Pose2D
from fleet_manager.node import FleetManagerNode
from fleet_manager.resources import LeaseRequest


@pytest.fixture
def rig(tmp_path):
    yield from test_node.rig.__wrapped__(tmp_path)


def charge_for(adapter):
    adapter.observe(RobotSnapshot("amr_01", "AVAILABLE", Pose2D(0, 0, 0), 20, "EMPTY"))
    return adapter.core.queue_charging(adapter.config.energy, 100)[0]


def test_dispatch_missing_server_diagnostic_survives_adapter_mapping_without_spam(rig):
    _, adapter, _, _, _ = rig
    logs = []
    adapter.diagnostic = logs.append
    node = object.__new__(FleetManagerNode)
    node.adapter = adapter
    node.docks = {"amr_01": SimpleNamespace(server_is_ready=lambda: False)}
    adapter.transport = node
    charge = charge_for(adapter)
    adapter._start_charging(charge)
    adapter.tick()
    for _ in range(100):
        adapter.tick()
    rows = logs
    assert len([row for row in rows if row["event"] == "fleet_dock_dispatch"]) == 1
    assert (
        next(row for row in rows if row["event"] == "fleet_dock_dispatch")[
            "server_ready"
        ]
        is False
    )
    row = next(row for row in rows if row["event"] == "fleet_dock_acceptance")
    assert row["message"] == "dock action unavailable" and row["accepted"] is False
    assert row["robot_id"] == "amr_01" and row["generation"] == charge.generation


def test_failed_result_preserves_code_message_and_redacts_raw_lease(rig):
    _, adapter, _, _, _ = rig
    logs = []
    adapter.diagnostic = logs.append
    charge = charge_for(adapter)
    adapter._start_charging(charge)
    flight = adapter._dock_flights["amr_01"]
    lease = adapter.resources.acquire(
        LeaseRequest("amr_01", "private-action-token", charge.dock_id), 100
    ).lease
    reply = RobotReply(
        False,
        "NAVIGATION_FAILED",
        "navigation unavailable " + lease.lease_id + " " + "x" * 1000,
    )
    adapter._dock_result(charge, flight, reply)
    adapter._dock_result(charge, flight, reply)
    row = next(row for row in logs if row["event"] == "fleet_dock_result")
    assert row["error_code"] == "NAVIGATION_FAILED"
    assert row["mapped_state"] == "RECOVERY_REQUIRED"
    assert row["message"].startswith("navigation unavailable ")
    assert len(row["message"]) <= 256
    assert lease.lease_id not in json.dumps(
        logs
    ) and "private-action-token" not in json.dumps(logs)
    assert len([row for row in logs if row["event"] == "fleet_dock_result"]) == 1
