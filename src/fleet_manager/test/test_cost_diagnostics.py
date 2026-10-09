"""Observe cost readiness and round outcomes without changing scheduling."""

import json
from types import SimpleNamespace

from factory_interfaces.srv import EstimateMissionCost
from rclpy.task import Future
import pytest
import test_node

from fleet_manager.models import CostEstimate
from fleet_manager.node import FleetManagerNode
from fleet_manager.resources import LeaseRequest


@pytest.fixture
def rig(tmp_path):
    yield from test_node.rig.__wrapped__(tmp_path)


def cost_node(adapter, *, service=True, action=True):
    node = object.__new__(FleetManagerNode)
    node.adapter = adapter
    future = Future()
    node.costs = {
        "amr_01": SimpleNamespace(
            service_is_ready=lambda: service,
            call_async=lambda request: future,
            remove_pending_request=lambda request: None,
        )
    }
    node.actions = {"amr_01": SimpleNamespace(server_is_ready=lambda: action)}
    return node, future


@pytest.mark.parametrize(
    "service,action", [(False, True), (True, False), (False, False)]
)
def test_cost_dispatch_identifies_exact_missing_readiness_without_retries(
    rig, service, action
):
    _, adapter, _, _, _ = rig
    logs, replies = [], []
    adapter.diagnostic = logs.append
    node, _ = cost_node(adapter, service=service, action=action)
    node.estimate("amr_01", test_node.request(), replies.append)
    assert replies == [None]
    row = next(row for row in logs if row["event"] == "fleet_cost_dispatch")
    assert row["cost_service_ready"] is service
    assert row["mission_action_ready"] is action
    assert row["robot_id"] == "amr_01" and row["mission_id"] == "m1"


def test_cost_result_preserves_typed_rejection_and_redacts_authority(rig):
    _, adapter, _, _, _ = rig
    lease = adapter.resources.acquire(
        LeaseRequest("amr_01", "private-action-token", "assembly"), 100
    ).lease
    logs, replies = [], []
    adapter.diagnostic = logs.append
    node, future = cost_node(adapter)
    node.estimate("amr_01", test_node.request(), replies.append)
    reason = "path pending " + lease.lease_id + " private-action-token"
    future.set_result(
        EstimateMissionCost.Response(
            feasible=False,
            path_cost=float("inf"),
            predicted_final_battery=50.0,
            reason=reason,
        )
    )
    assert replies == [CostEstimate(False, float("inf"), 50, reason)]
    row = next(row for row in logs if row["event"] == "fleet_cost_result")
    assert row["feasible"] is False and row["path_cost"] is None
    assert row["reason"].startswith("path pending ")
    assert row["elapsed_sec"] >= 0
    assert lease.lease_id not in json.dumps(logs)
    assert "private-action-token" not in json.dumps(logs)


def test_cost_timeout_is_single_transition_at_original_deadline_and_late_reply_fenced(
    rig,
):
    _, adapter, journal, robots, now = rig
    logs = []
    adapter.diagnostic = logs.append
    robots.costs["amr_01"] = "timeout"
    adapter.submit(test_node.request(pin="amr_01"))
    adapter.tick()
    for _ in range(50):
        adapter.tick()
    assert not any(row["event"] == "fleet_cost_timeout" for row in logs)
    now[0] += 1.0
    adapter.tick()
    robots.waiting_costs[0](CostEstimate(True, 1, 50))
    for _ in range(50):
        adapter.tick()
    assert journal.get("m1").state == "FAILED" and robots.goals == []
    rows = [row for row in logs if row["event"] == "fleet_cost_timeout"]
    assert len(rows) == 1 and rows[0]["robot_id"] == "amr_01"
    assert rows[0]["deadline"] == 101.0


def test_cost_candidate_rejection_keeps_exact_estimator_reason_without_tick_spam(rig):
    _, adapter, _, robots, _ = rig
    logs = []
    adapter.diagnostic = logs.append
    robots.costs["amr_01"] = CostEstimate(False, 0, 50, "path pending")
    adapter.submit(test_node.request(pin="amr_01"))
    for _ in range(50):
        adapter.tick()
    rows = [row for row in logs if row["event"] == "fleet_cost_candidate_rejected"]
    assert len(rows) == 1 and rows[0]["reason"] == "path pending"
    assert robots.goals == []


def test_cost_diagnostic_failure_cannot_change_real_assignment(rig):
    _, adapter, journal, robots, _ = rig

    def broken(row):
        raise RuntimeError("logger unavailable")

    adapter.diagnostic = broken
    test_node.dispatch(rig)
    assert journal.get("m1").assigned_robot_id == "amr_02"
    assert len(robots.goals) == 1


def test_diagnostic_inspection_cannot_fail_a_cost_discarded_for_reserved_robot(rig):
    _, adapter, journal, robots, _ = rig
    adapter.diagnostic = [].append
    callbacks = test_node.delayed_costs(robots)
    adapter.submit(test_node.request("auto"))
    adapter.tick()
    adapter.submit(test_node.request("pin", "amr_01"))
    adapter.tick()
    test_node.answer_costs(callbacks, "pin")
    adapter.tick()
    # The original dispatcher never inspects this now-reserved candidate.
    callbacks["auto"]["amr_01"](CostEstimate(True, 1, "unreadable"))
    callbacks["auto"]["amr_02"](CostEstimate(True, 2, 50))
    adapter.tick()
    assert journal.get("auto").assigned_robot_id == "amr_02"
    assert journal.get("pin").assigned_robot_id == "amr_01"
