"""Diagnostics retain docking evidence without changing authority or flooding logs."""

import json
import pytest
from types import SimpleNamespace

from factory_interfaces.action import DockRobot
from builtin_interfaces.msg import Time
from robot_agent.docking import DockKey
from test_node import api, runtime, sensors
from test_docking import Wire, rig, start


def boundary():
    host, agent, _, wall = runtime()
    sensors(host)
    logs = []
    host.get_logger = lambda: SimpleNamespace(info=logs.append)
    wire = Wire()
    docking = api().DockRuntime(
        host,
        agent.adapter,
        wire,
        "charger",
        (2, 0, 0),
        (3, 0, 0),
        server_factory=lambda *args, **kwargs: SimpleNamespace(destroy=lambda: None),
    )
    goal = DockRobot.Goal(
        dock_id="charger",
        target_percent=80.0,
        staging_pose=api().pose_message((2, 0, 0), "floor/cart_1/map", Time()),
        charging_pose=api().pose_message((3, 0, 0), "floor/cart_1/map", Time()),
    )
    return docking, agent.adapter, wire, logs, goal, wall


def test_goal_diagnostics_capture_readiness_and_suppress_identical_rejections():
    docking, agent, _, logs, goal, wall = boundary()
    agent._pose_receipt = None
    for _ in range(100):
        assert docking.goal(goal) == api().GoalResponse.REJECT
    assert len(logs) == 1
    row = json.loads(logs[0])
    assert row["event"] == "dock_goal" and row["decision"] == "REJECT"
    assert row["reason"] == "robot_unavailable"
    assert row["pose_receipt_age_sec"] is None
    assert row["health_detail"] == "unknown localization"
    agent._pose_receipt = wall[0]
    assert docking.goal(goal) == api().GoalResponse.ACCEPT
    assert len(logs) == 2 and json.loads(logs[-1])["decision"] == "ACCEPT"


def test_immediate_navigation_failure_logs_result_and_never_logs_lease_tokens():
    docking, agent, wire, logs, goal, _ = boundary()
    wire.navigate = lambda *args, **kwargs: (
        args[2](False, "navigation unavailable"),
        None,
    )[1]
    assert docking.goal(goal) == api().GoalResponse.ACCEPT
    handle = SimpleNamespace(
        request=goal, publish_feedback=lambda _: None, execute=lambda: None
    )
    docking.accepted(handle)
    rows = [json.loads(line) for line in logs]
    result = next(row for row in rows if row["event"] == "dock_result")
    assert result["error_code"] == "NAVIGATION_FAILED"
    assert result["message"] == "navigation unavailable"
    assert result["state"] == "FAILED" and agent.mode == "AVAILABLE"
    assert result["lease_present"] is False
    assert all("lease_id" not in row for row in rows)


def test_result_message_redacts_known_lease_and_bounds_untrusted_text():
    docking, _, _, logs, goal, _ = boundary()
    docking.goal(goal)
    docking.accepted(
        SimpleNamespace(
            request=goal, publish_feedback=lambda _: None, execute=lambda: None
        )
    )
    docking.controller.key = DockKey(
        "cart_1", "private-action-token", "charger", "private-lease"
    )
    docking.controller._finish(
        False, "LEASE_LOST", "private-lease " + "x" * 1000, recovery=True
    )
    row = next(
        json.loads(line) for line in logs if json.loads(line)["event"] == "dock_result"
    )
    assert "private-lease" not in "".join(logs)
    assert "private-action-token" not in "".join(logs)
    assert len(row["message"]) <= 256 and row["lease_present"] is True


def test_refresh_fence_diagnostics_are_once_per_reason_not_once_per_pose():
    logs = []
    r = rig(diagnostic=lambda row: logs.append(row))
    agent, _, wire, _, _, controller = r
    start(r)
    wire.moves[-1][2](True, "")
    for _ in range(100):
        controller.tick()
    blocked = [row for row in logs if row["event"] == "dock_refresh_fence"]
    assert len(blocked) == 1 and blocked[0]["reason"] == "source_not_newer"
    agent.localization(2, 2, -3, 0, agent.frame_prefix + "map")
    controller.tick()
    assert controller.state == "WAITING_FOR_LEASE"
    assert [row["reason"] for row in logs if row["event"] == "dock_refresh_fence"] == [
        "source_not_newer",
        "verified",
    ]


def test_diagnostic_sink_failure_does_not_change_controller_execution():
    def broken(_):
        raise RuntimeError("logger unavailable")

    r = rig(diagnostic=broken)
    start(r)
    assert r[-1].state == "STAGING" and len(r[2].moves) == 1


def test_transport_records_missing_navigation_and_localization_readiness():
    from test_node import Host

    logs, results = [], []
    host = Host("/cart_1")
    host.create_client = lambda *args: SimpleNamespace(service_is_ready=lambda: False)
    transport = api().DockTransport(
        host,
        SimpleNamespace(server_is_ready=lambda: False),
        {},
        diagnostic=lambda event, **fields: logs.append(dict(event=event, **fields)),
    )
    transport.navigate(
        (2, 0, 0), "floor/cart_1/map", lambda *args: results.append(args)
    )
    transport.request_localization(results.append)
    assert logs == [
        dict(
            event="dock_navigation_ready",
            server_ready=False,
            precise=False,
            behavior_tree_configured=False,
        ),
        dict(event="dock_localization_ready", service_ready=False),
    ]
    assert results == [(False, "navigation unavailable"), False]


@pytest.mark.parametrize(
    "pose,exit_pose,reason",
    [
        ((3, -3, 0), None, "already_at_charging"),
        ((1.5, -3, 0), None, "initial_clearance"),
        ((0, -3, 0), (2, -3, 0), "exit_clearance"),
    ],
)
def test_start_fence_logs_exact_pre_motion_failure(pose, exit_pose, reason):
    logs = []
    agent, _, wire, results, feedback, controller = rig(
        exit_pose=exit_pose, diagnostic=logs.append
    )
    agent.localization(2, *pose, agent.frame_prefix + "map")
    assert not controller.start(80, feedback.append, results.append)
    assert not wire.moves and not wire.requests
    assert logs[0]["event"] == "dock_start_fence" and logs[0]["reason"] == reason
