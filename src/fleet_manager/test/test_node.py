"""Exercise fleet transport decisions against real policies and SQLite."""

from dataclasses import replace
import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_manager.config import Pose2D, load_fleet_config
from fleet_manager.journal import MissionConflictError, MissionJournal
from fleet_manager.models import CostEstimate, MissionRequest, RobotSnapshot
from fake_robot_agent import FakeRobots


def adapter_api():
    try:
        return importlib.import_module("fleet_manager.adapter")
    except ModuleNotFoundError:
        pytest.fail("fleet transport orchestration is not implemented")


@pytest.fixture
def rig(tmp_path):
    api = adapter_api()
    config = load_fleet_config(
        Path(__file__).resolve().parents[2] / "factory_bringup/config/fleet.yaml"
    )
    journal = MissionJournal(tmp_path / "missions.sqlite3")
    robots = FakeRobots(journal)
    now = [100.0]
    adapter = api.FleetAdapter(config, journal, robots, clock=lambda: now[0])
    for robot_id in ("amr_01", "amr_02"):
        adapter.observe(
            RobotSnapshot(robot_id, "AVAILABLE", Pose2D(0, 0, 0), 80, "EMPTY")
        )
    yield api, adapter, journal, robots, now
    journal.close()


def request(mission_id="m1", pin=None):
    return MissionRequest(mission_id, "assembly", "inspection", "motor", pin)


def test_cancel_resource_wait_removes_only_exact_waiter_without_releasing_owner(rig):
    from types import SimpleNamespace

    _, adapter, _, _, _ = rig
    owner = SimpleNamespace(
        robot_id="amr_01", mission_id="m1", resource_id="central_aisle"
    )
    waiter = SimpleNamespace(
        robot_id="amr_02", mission_id="m2", resource_id="central_aisle"
    )
    grant = adapter.resource("acquire", owner)
    assert grant["granted"]
    assert not adapter.resource("acquire", waiter)["granted"]
    reply = adapter.resource("cancel_wait", waiter)
    assert reply["cancelled"]
    assert not adapter.resource("cancel_wait", waiter)["cancelled"]
    assert adapter.resource("acquire", owner)["lease_id"] == grant["lease_id"]


def test_cancel_wait_ros_handler_preserves_typed_result_and_offline_cleanup(rig):
    from types import SimpleNamespace
    from factory_interfaces.srv import AcquireResource, CancelResourceWait
    from fleet_manager.node import FleetManagerNode

    _, adapter, _, _, now = rig
    adapter.resource(
        "acquire",
        AcquireResource.Request(
            robot_id="amr_01", mission_id="m1", resource_id="assembly"
        ),
    )
    adapter.resource(
        "acquire",
        AcquireResource.Request(
            robot_id="amr_02", mission_id="m2", resource_id="assembly"
        ),
    )
    now[0] += 5.1
    response = FleetManagerNode._resource(
        SimpleNamespace(adapter=adapter),
        "cancel_wait",
        CancelResourceWait.Request(
            robot_id="amr_02", mission_id="m2", resource_id="assembly"
        ),
        CancelResourceWait.Response(),
    )
    assert response.cancelled and response.reason == "waiter cancelled"
    assert adapter.resources.snapshot(now[0])[0].lease.robot_id == "amr_01"


def dispatch(rig, mission_id="m1", pin=None):
    _, adapter, _, robots, _ = rig
    adapter.submit(request(mission_id, pin))
    adapter.tick()
    adapter.tick()
    adapter.tick()
    return robots.goals[-1]


@pytest.mark.parametrize("pin,want", [(None, "amr_02"), ("amr_01", "amr_01")])
def test_assignment_selects_cost_or_pin_and_commits_before_robot_goal(rig, pin, want):
    goal = dispatch(rig, pin=pin)
    _, _, journal, robots, _ = rig
    assert goal.robot_id == want
    assert journal.get("m1").assigned_robot_id == want
    assert robots.statuses[0][0].assigned_robot_id is None
    assert robots.statuses[-1][0].assigned_robot_id == want


def test_duplicate_reuses_durable_state_without_second_robot_goal(rig):
    dispatch(rig)
    _, adapter, journal, robots, _ = rig
    before = journal.events("m1")
    adapter.submit(request())
    adapter.tick()
    assert len(robots.goals) == 1
    assert journal.events("m1") == before
    with pytest.raises(MissionConflictError):
        adapter.submit(request(pin="amr_01"))


def test_unknown_pinned_robot_is_rejected_before_journal_registration(rig):
    _, adapter, journal, _, _ = rig
    with pytest.raises(ValueError, match="configured"):
        adapter.submit(request(pin="unknown"))
    assert journal.load_active() == ()


def test_feedback_and_result_commit_before_status_propagation(rig):
    goal = dispatch(rig)
    api, adapter, journal, robots, _ = rig
    goal.feedback(api.RobotFeedback("NAVIGATING_TO_DROPOFF", "loaded", 0.6))
    assert journal.get("m1").state == "ASSIGNED"  # callback only queues
    adapter.tick()
    assert journal.get("m1").payload_ownership == "PICKED_UP"
    assert robots.statuses[-1][1:] == ("NAVIGATING_TO_DROPOFF", "loaded", 0.6)
    goal.result(api.RobotReply(True, "", "delivered"))
    adapter.tick()
    assert journal.get("m1").state == "COMPLETED"
    assert journal.get("m1").result["message"] == "delivered"
    assert robots.statuses[-1][0].state == "COMPLETED"


@pytest.mark.parametrize("missing", [None, "timeout"])
def test_missing_cost_excludes_only_that_robot_for_current_round(rig, missing):
    _, adapter, _, robots, now = rig
    robots.costs["amr_02"] = missing
    adapter.submit(request())
    adapter.tick()
    now[0] += 1.1
    for _ in range(3):
        adapter.tick()
    assert robots.goals[0].robot_id == "amr_01"


def test_no_cost_keeps_mission_queued_and_retries_without_busy_loop(rig):
    _, adapter, journal, robots, now = rig
    robots.costs.clear()
    adapter.submit(request())
    for _ in range(50):
        adapter.tick()
    assert journal.get("m1").state == "QUEUED"
    assert len(robots.cost_calls) == 2
    robots.costs["amr_02"] = CostEstimate(True, 2, 50)
    now[0] += 1.1
    adapter.tick()
    adapter.tick()
    assert robots.goals[0].robot_id == "amr_02"


def test_cancel_reserves_robot_until_terminal_result(rig):
    goal = dispatch(rig, pin="amr_01")
    api, adapter, journal, robots, now = rig
    adapter.cancel("m1")
    assert journal.get("m1").state == "ASSIGNED"
    assert goal.cancel_ack is not None
    robots.costs["amr_02"] = None
    adapter.submit(request("m2"))
    adapter.tick()
    adapter.tick()
    assert len(robots.goals) == 1
    goal.cancel_ack(True)
    adapter.tick()
    assert journal.get("m1").result is None
    assert len(robots.goals) == 1
    goal.result(api.RobotReply(False, "", "stopped", cancelled=True))
    now[0] += 1.1
    adapter.tick()
    adapter.tick()
    assert len(robots.goals) == 2
    assert robots.goals[-1].request.mission_id == "m2"
    assert journal.get("m1").state == "CANCELLED"


def test_cancel_before_robot_acceptance_cancels_late_goal_and_waits_for_ack(rig):
    _, adapter, _, robots, now = rig
    robots.auto_accept = False
    goal = dispatch(rig, pin="amr_01")
    adapter.cancel("m1")
    adapter.submit(request("m2", "amr_01"))
    goal.accepted(goal, "")
    adapter.tick()
    assert goal.cancel_ack is not None
    assert len(robots.goals) == 1
    goal.cancel_ack(False)
    now[0] += 1.1
    adapter.tick()
    assert len(robots.goals) == 1


def test_cancel_consumes_queued_payload_evidence_before_persisting_intent(rig):
    goal = dispatch(rig)
    api, adapter, journal, _, _ = rig
    goal.feedback(api.RobotFeedback("LOADING"))
    adapter.cancel("m1")
    assert journal.get("m1").state == "EXECUTING"
    assert journal.get("m1").payload_ownership == "UNKNOWN"
    assert journal.get("m1").result is None
    goal.result(api.RobotReply(False, "CANCELLED", cancelled=True))
    adapter.tick()
    assert journal.get("m1").state == "RECOVERY_REQUIRED"


@pytest.mark.parametrize("acknowledged", [False, True])
def test_cancellation_preserves_delayed_loading_evidence_until_robot_result(
    rig, acknowledged
):
    goal = dispatch(rig)
    api, adapter, journal, robots, _ = rig
    adapter.cancel("m1")
    assert journal.get("m1").state == "ASSIGNED"
    assert journal.get("m1").result is None
    goal.feedback(api.RobotFeedback("LOADING"))
    goal.cancel_ack(acknowledged)
    adapter.tick()
    assert journal.get("m1").state == "EXECUTING"
    assert journal.get("m1").payload_ownership == "UNKNOWN"
    goal.feedback(api.RobotFeedback("NAVIGATING_TO_DROPOFF"))
    adapter.tick()
    assert journal.get("m1").payload_ownership == "PICKED_UP"
    goal.result(api.RobotReply(False, "MISSION_CANCELED", "stopped", cancelled=True))
    adapter.tick()
    assert journal.get("m1").state == "RECOVERY_REQUIRED"
    assert journal.get("m1").result["error_code"] == "MISSION_CANCELED"
    assert robots.statuses[-1][0].state == "RECOVERY_REQUIRED"


def test_rejected_cancellation_reports_actual_success_and_delivered_payload(rig):
    goal = dispatch(rig)
    api, adapter, journal, robots, _ = rig
    adapter.cancel("m1")
    goal.feedback(api.RobotFeedback("LOADING"))
    goal.cancel_ack(False)
    goal.feedback(api.RobotFeedback("NAVIGATING_TO_DROPOFF"))
    adapter.tick()
    assert journal.get("m1").payload_ownership == "PICKED_UP"
    goal.result(api.RobotReply(True, "", "delivered despite cancellation request"))
    adapter.tick()
    assert journal.get("m1").state == "COMPLETED"
    assert journal.get("m1").payload_ownership == "DELIVERED"
    assert journal.get("m1").result["success"]
    assert robots.statuses[-1][0].state == "COMPLETED"


def test_cancel_acknowledgement_does_not_release_still_executing_robot(rig):
    goal = dispatch(rig, pin="amr_01")
    _, adapter, journal, robots, now = rig
    robots.costs["amr_02"] = None
    adapter.cancel("m1")
    goal.cancel_ack(True)
    adapter.submit(request("m2"))
    adapter.tick()
    adapter.tick()
    now[0] += 1.1
    adapter.tick()
    adapter.tick()
    assert len(robots.goals) == 1
    assert journal.get("m2").state == "QUEUED"
    assert journal.get("m1").result is None


@pytest.mark.parametrize("acknowledged", [False, True])
def test_offline_during_pending_cancel_preserves_execution_uncertainty(
    rig, acknowledged
):
    goal = dispatch(rig)
    _, adapter, journal, robots, now = rig
    adapter.cancel("m1")
    goal.cancel_ack(acknowledged)
    adapter.tick()
    now[0] += 5.1
    adapter.observe(RobotSnapshot("amr_01", "AVAILABLE", Pose2D(0, 0, 0), 80, "EMPTY"))
    adapter.tick()
    assert journal.get("m1").state == "RECOVERY_REQUIRED"
    assert journal.get("m1").payload_ownership == "UNKNOWN"
    assert journal.get("m1").result["error_code"] == "ROBOT_OFFLINE"
    assert robots.statuses[-1][0].state == "RECOVERY_REQUIRED"
    assert len(robots.goals) == 1


@pytest.mark.parametrize("cost", [None, "timeout"])
def test_pinned_missing_cost_returns_bounded_explicit_rejection(rig, cost):
    _, adapter, journal, robots, now = rig
    robots.costs["amr_01"] = cost
    adapter.submit(request(pin="amr_01"))
    adapter.tick()
    now[0] += 1.1
    adapter.tick()
    assert journal.get("m1").state == "FAILED"
    assert journal.get("m1").result["error_code"] == "REQUESTED_ROBOT_UNAVAILABLE"
    assert "amr_01" in journal.get("m1").result["message"]
    assert robots.goals == []
    for _ in range(50):
        adapter.tick()
    assert len(robots.cost_calls) == 1


def test_pinned_offline_robot_returns_bounded_rejection_without_fallback(rig):
    _, adapter, journal, robots, now = rig
    now[0] += 5.1
    adapter.observe(RobotSnapshot("amr_02", "AVAILABLE", Pose2D(0, 0, 0), 80, "EMPTY"))
    adapter.submit(request(pin="amr_01"))
    adapter.tick()
    adapter.tick()
    assert journal.get("m1").state == "FAILED"
    assert journal.get("m1").result["error_code"] == "REQUESTED_ROBOT_UNAVAILABLE"
    assert robots.goals == []


@pytest.mark.parametrize("mode", ["UNHEALTHY", "OFFLINE", "CHARGING", "EXECUTING"])
def test_pinned_ineligible_robot_rejects_without_cost_or_fallback(rig, mode):
    _, adapter, journal, robots, _ = rig
    adapter.observe(RobotSnapshot("amr_01", mode, Pose2D(0, 0, 0), 80, "EMPTY"))
    adapter.submit(request(pin="amr_01"))
    adapter.tick()
    adapter.tick()
    assert journal.get("m1").state == "FAILED"
    assert journal.get("m1").result["error_code"] == "REQUESTED_ROBOT_UNAVAILABLE"
    assert robots.cost_calls == [] and robots.goals == []


@pytest.mark.parametrize(
    "cost", [CostEstimate(False, 1, 50), CostEstimate(True, 1, 19.9)]
)
def test_pinned_infeasible_or_low_energy_cost_rejects_without_fallback(rig, cost):
    _, adapter, journal, robots, _ = rig
    robots.costs["amr_01"] = cost
    adapter.submit(request(pin="amr_01"))
    adapter.tick()
    adapter.tick()
    assert journal.get("m1").state == "FAILED"
    assert journal.get("m1").result["error_code"] == "REQUESTED_ROBOT_UNAVAILABLE"
    assert robots.cost_calls == [("amr_01", "m1")] and robots.goals == []


def test_pinned_missing_action_server_returns_bounded_explicit_rejection(rig):
    from fleet_manager.node import FleetManagerNode
    from types import SimpleNamespace

    _, adapter, journal, robots, _ = rig
    node = object.__new__(FleetManagerNode)
    node.actions = {"amr_01": SimpleNamespace(server_is_ready=lambda: False)}
    node.costs = {"amr_01": SimpleNamespace(service_is_ready=lambda: True)}
    robots.estimate = node.estimate
    adapter.submit(request(pin="amr_01"))
    adapter.tick()
    adapter.tick()
    assert journal.get("m1").state == "FAILED"
    assert journal.get("m1").result["error_code"] == "REQUESTED_ROBOT_UNAVAILABLE"
    assert robots.goals == []


@pytest.mark.parametrize(
    "stage,want",
    [
        ("NAVIGATING_TO_PICKUP", "FAILED"),
        ("LOADING", "RECOVERY_REQUIRED"),
        ("NAVIGATING_TO_DROPOFF", "RECOVERY_REQUIRED"),
    ],
)
def test_rejected_cancellation_reports_eventual_failure_with_payload_truth(
    rig, stage, want
):
    goal = dispatch(rig)
    api, adapter, journal, robots, _ = rig
    adapter.cancel("m1")
    goal.cancel_ack(False)
    goal.feedback(api.RobotFeedback(stage))
    adapter.tick()
    assert journal.get("m1").result is None
    goal.result(api.RobotReply(False, "DRIVE_FAULT", "drive stopped"))
    adapter.tick()
    assert journal.get("m1").state == want
    assert journal.get("m1").result["error_code"] == "DRIVE_FAULT"
    assert robots.statuses[-1][0].state == want


@pytest.mark.parametrize("status,want", [(5, "CANCELLED"), (6, "FAILED")])
def test_ros_terminal_status_preserves_typed_cancellation_outcome(rig, status, want):
    from factory_interfaces.action import ExecuteFactoryMission
    from fleet_manager.node import FleetManagerNode
    from rclpy.task import Future
    from types import SimpleNamespace

    _, adapter, journal, robots, _ = rig
    response, completed = Future(), Future()
    sent_goals = []
    node = object.__new__(FleetManagerNode)

    def send_goal(goal, feedback_callback):
        sent_goals.append(goal)
        return response

    node.actions = {
        "amr_01": SimpleNamespace(
            server_is_ready=lambda: True, send_goal_async=send_goal
        )
    }
    robots.send_goal = node.send_goal
    adapter.submit(request(pin="amr_01"))
    adapter.tick()
    adapter.tick()
    assert sent_goals[0].robot_id == "amr_01"
    response.set_result(
        SimpleNamespace(accepted=True, get_result_async=lambda: completed)
    )
    adapter.tick()
    adapter.cancel("m1")
    completed.set_result(
        SimpleNamespace(
            status=status, result=ExecuteFactoryMission.Result(success=False)
        )
    )
    adapter.tick()
    assert journal.get("m1").state == want
    assert journal.get("m1").result["error_code"] == (
        "CANCELLED" if status == 5 else ""
    )


def test_cancellation_persistence_failure_sends_no_robot_cancel(rig, tmp_path):
    import sqlite3

    goal = dispatch(rig)
    _, adapter, journal, _, _ = rig
    with sqlite3.connect(tmp_path / "missions.sqlite3") as connection:
        connection.execute(
            "CREATE TRIGGER reject_cancel BEFORE INSERT ON mission_events "
            "WHEN json_extract(NEW.detail_json, '$.cancellation_requested') = 1 "
            "BEGIN SELECT RAISE(ABORT, 'cancel write failed'); END"
        )
    with pytest.raises(sqlite3.IntegrityError, match="cancel write failed"):
        adapter.cancel("m1")
    assert goal.cancel_ack is None
    assert journal.get("m1").state == "ASSIGNED"


def test_cost_response_received_after_deadline_cannot_win_assignment(rig):
    _, adapter, _, robots, now = rig
    robots.costs["amr_02"] = "timeout"
    adapter.submit(request())
    adapter.tick()
    now[0] += 1.1
    robots.waiting_costs[0](CostEstimate(True, 0, 50))
    adapter.tick()
    assert robots.goals[0].robot_id == "amr_01"


def test_assignment_persistence_failure_emits_no_robot_goal_or_assigned_status(
    rig, tmp_path
):
    import sqlite3

    _, adapter, _, robots, _ = rig
    adapter.submit(request())
    adapter.tick()
    with sqlite3.connect(tmp_path / "missions.sqlite3") as connection:
        connection.execute(
            "CREATE TRIGGER reject_assignment BEFORE INSERT ON mission_events "
            "WHEN NEW.state = 'ASSIGNED' BEGIN SELECT RAISE(ABORT, 'write failed'); END"
        )
    with pytest.raises(sqlite3.IntegrityError, match="write failed"):
        adapter.tick()
    assert robots.goals == []
    assert all(record.assigned_robot_id is None for record, *_ in robots.statuses)


def test_callback_from_transport_thread_only_mutates_journal_on_tick(rig):
    import threading

    goal = dispatch(rig)
    api, adapter, journal, _, _ = rig
    thread = threading.Thread(target=lambda: goal.result(api.RobotReply(True)))
    thread.start()
    thread.join()
    assert journal.get("m1").state == "ASSIGNED"
    adapter.tick()
    assert journal.get("m1").state == "COMPLETED"


def test_offline_before_pickup_reassigns_and_old_callbacks_cannot_finish_new_goal(rig):
    old = dispatch(rig)
    api, adapter, journal, robots, now = rig
    now[0] += 5.1
    adapter.observe(RobotSnapshot("amr_01", "AVAILABLE", Pose2D(0, 0, 0), 80, "EMPTY"))
    adapter.tick()
    adapter.tick()
    adapter.tick()
    assert len(robots.goals) == 2
    assert robots.goals[-1].robot_id == "amr_01"
    old.feedback(api.RobotFeedback("UNLOADING", "stale", 0.9))
    old.result(api.RobotReply(True))
    adapter.tick()
    assert journal.get("m1").assigned_robot_id == "amr_01"
    assert journal.get("m1").state == "ASSIGNED"
    assert journal.get("m1").payload_ownership == "NOT_PICKED_UP"


@pytest.mark.parametrize("stage", ["LOADING", "NAVIGATING_TO_DROPOFF"])
def test_offline_with_uncertain_or_loaded_payload_requires_recovery(rig, stage):
    goal = dispatch(rig)
    api, adapter, journal, robots, now = rig
    goal.feedback(api.RobotFeedback(stage))
    adapter.tick()
    now[0] += 5.1
    adapter.observe(RobotSnapshot("amr_01", "AVAILABLE", Pose2D(0, 0, 0), 80, "EMPTY"))
    adapter.tick()
    assert journal.get("m1").state == "RECOVERY_REQUIRED"
    assert len(robots.goals) == 1


def test_resource_service_translation_fences_wrong_lease_and_reports_ttl(rig):
    _, adapter, _, _, now = rig
    from types import SimpleNamespace

    request = SimpleNamespace(
        robot_id="amr_01", mission_id="m1", resource_id="assembly"
    )
    acquired = adapter.resource("acquire", request)
    assert acquired["granted"] and acquired["lease_ttl_sec"] == 10.0
    request.lease_id = "wrong"
    assert not adapter.resource("renew", request)["renewed"]
    assert not adapter.resource("release", request)["released"]
    request.lease_id = acquired["lease_id"]
    now[0] += 2
    assert adapter.resource("renew", request)["lease_ttl_sec"] == 10.0
    assert adapter.resource("release", request)["released"]
    with pytest.raises(ValueError):
        adapter.resource("clear_reconciliation", request)


def test_unhealthy_robot_cannot_renew_resource_lease(rig):
    from types import SimpleNamespace

    _, adapter, _, _, now = rig
    request = SimpleNamespace(
        robot_id="amr_01", mission_id="m1", resource_id="assembly"
    )
    request.lease_id = adapter.resource("acquire", request)["lease_id"]
    now[0] += 3.0
    assert not adapter.resource("renew", request)["renewed"]
    assert adapter.resource("release", request)["released"]


def test_cancellation_removes_resource_waiter_before_next_handoff(rig):
    from fleet_manager.resources import LeaseKey, LeaseRequest

    _, adapter, _, _, now = rig
    held = adapter.resources.acquire(
        LeaseRequest("amr_01", "holder", "assembly"), now[0]
    ).lease
    adapter.resources.acquire(LeaseRequest("amr_02", "m1", "assembly"), now[0])
    dispatch(rig, pin="amr_02")
    adapter.cancel("m1")
    adapter.resources.release(
        LeaseKey(held.robot_id, held.mission_id, held.resource_id, held.lease_id),
        now[0],
    )
    resource = next(
        item
        for item in adapter.resources.snapshot(now[0])
        if item.resource_id == "assembly"
    )
    assert resource.lease is None and resource.waiters == ()


def test_queued_retry_publishes_bounded_reason(rig):
    _, adapter, _, robots, _ = rig
    robots.costs.clear()
    adapter.submit(request())
    adapter.tick()
    adapter.tick()
    assert robots.statuses[-1][1:3] == ("QUEUED", "no eligible robot")


def test_ros_cost_timeout_removes_actual_client_pending_future(rig):
    from factory_interfaces.srv import EstimateMissionCost
    from fleet_manager.node import FleetManagerNode
    from rclpy.client import Client
    from threading import Lock

    _, adapter, _, robots, now = rig

    class WireService:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def service_server_is_available(self):
            return True

        def send_request(self, request):
            return 1

    client = object.__new__(Client)
    client._lock = Lock()
    client._pending_requests = {}
    client._Client__client = WireService()
    client.srv_type = EstimateMissionCost
    node = object.__new__(FleetManagerNode)
    node.costs = {"amr_02": client}
    from types import SimpleNamespace

    node.actions = {"amr_02": SimpleNamespace(server_is_ready=lambda: True)}
    original = robots.estimate
    robots.estimate = lambda robot_id, request, callback: (
        node.estimate(robot_id, request, callback)
        if robot_id == "amr_02"
        else original(robot_id, request, callback)
    )
    adapter.submit(request())
    adapter.tick()
    assert len(client._pending_requests) == 1
    now[0] += 1.1
    adapter.tick()
    assert client._pending_requests == {}
    assert robots.goals[-1].robot_id == "amr_01"


def test_ros_robot_without_action_server_is_excluded_from_assignment_round():
    from fleet_manager.node import FleetManagerNode
    from rclpy.task import Future
    from types import SimpleNamespace

    node = object.__new__(FleetManagerNode)
    node.actions = {"amr_02": SimpleNamespace(server_is_ready=lambda: False)}
    node.costs = {
        "amr_02": SimpleNamespace(
            service_is_ready=lambda: True, call_async=lambda request: Future()
        )
    }
    estimates = []
    node.estimate("amr_02", request(), estimates.append)
    assert estimates == [None]


def test_ros_queued_handle_schedules_completion_only_after_terminal_commit(rig):
    from factory_interfaces.action import ExecuteFleetMission
    from fleet_manager.node import FleetManagerNode
    from types import SimpleNamespace

    api, adapter, journal, robots, _ = rig
    node = object.__new__(FleetManagerNode)
    node.adapter, node._waiters, node._results = adapter, {}, {}
    feedback, executions = [], []
    handle = SimpleNamespace(
        request=ExecuteFleetMission.Goal(
            mission_id="m1",
            pickup_station="assembly",
            dropoff_station="inspection",
            part="motor",
        ),
        execute=lambda: executions.append(journal.get("m1").state),
        publish_feedback=feedback.append,
        is_cancel_requested=False,
        succeed=lambda: None,
        abort=lambda: None,
        canceled=lambda: None,
    )
    original_publish = robots.publish
    adapter.submit(request())  # The ROS goal callback registers before acceptance.

    def publish(record, state, detail, progress):
        original_publish(record, state, detail, progress)
        node.publish(record, state, detail, progress)

    robots.publish = publish
    node._accepted(handle)
    assert executions == [], "queued missions must not create executor polling tasks"
    assert feedback[0].state == "QUEUED" and feedback[0].assigned_robot_id == ""
    adapter.tick()
    adapter.tick()
    robots.goals[-1].result(api.RobotReply(True))
    adapter.tick()
    assert executions == ["COMPLETED"]
    result = node._execute(handle)
    assert isinstance(result, ExecuteFleetMission.Result)
    assert result.success and result.assigned_robot_id == "amr_02"


def test_names_derive_from_namespace_even_when_robot_id_differs(rig):
    api, _, _, _, _ = rig
    config = load_fleet_config(
        Path(__file__).resolve().parents[2] / "factory_bringup/config/fleet.yaml"
    )
    robot = replace(config.robots[0], robot_id="other", namespace="/warehouse/cart")
    assert api.robot_endpoints(robot) == {
        "action": "/warehouse/cart/factory/execute_mission",
        "cost": "/warehouse/cart/factory/estimate_mission_cost",
        "state": "/warehouse/cart/factory/robot_state",
    }


def test_wire_robot_state_uses_receipt_health_and_validates_identity_and_pose(rig):
    from factory_interfaces.msg import RobotState

    api, _, _, _, _ = rig
    message = RobotState(
        robot_id="amr_02",
        mode="AVAILABLE",
        frame_id="map",
        battery_percent=80.0,
        payload_state="EMPTY",
    )
    message.pose.orientation.w = 1.0
    snapshot = api.robot_snapshot("amr_02", message)
    assert snapshot.pose == Pose2D(0, 0, 0)
    assert snapshot.mode == "AVAILABLE"
    message.pose.orientation.w = 0.0
    assert api.robot_snapshot("amr_02", message).health == "UNHEALTHY"
    with pytest.raises(ValueError):
        api.robot_snapshot("amr_01", message)


@pytest.mark.parametrize(
    "mode,health", [("OFFLINE", "OFFLINE"), ("UNHEALTHY", "UNHEALTHY")]
)
def test_wire_robot_state_reports_operational_health_without_waiting_for_timeout(
    rig, mode, health
):
    from factory_interfaces.msg import RobotState

    api, _, _, _, _ = rig
    message = RobotState(
        robot_id="amr_02",
        mode=mode,
        frame_id="map",
        battery_percent=80.0,
        payload_state="EMPTY",
    )
    message.pose.orientation.w = 1.0
    assert api.robot_snapshot("amr_02", message).health == health


@pytest.mark.parametrize(
    "scenario",
    [
        "automatic",
        "pinned",
        "duplicate",
        "missing_cost",
        "cancel",
        "offline",
        "mqtt",
        "resources",
    ],
)
def test_real_ros_two_robot_fleet_action(scenario, tmp_path):
    """Authored ROS contract test: do not hide blocked DDS networking with skips."""
    try:
        from fleet_manager.node import FleetManagerNode
    except ModuleNotFoundError:
        pytest.fail("fleet ROS node is not implemented")
    import rclpy
    from factory_interfaces.action import ExecuteFleetMission
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    import time
    from fake_robot_agent import create_ros_robot

    context = Context()
    rclpy.init(context=context, domain_id=78)
    config_path = (
        Path(__file__).resolve().parents[2] / "factory_bringup/config/fleet.yaml"
    )
    fleet = FleetManagerNode(
        context=context,
        parameter_overrides=[
            Parameter("fleet_file", value=str(config_path)),
            Parameter("journal_path", value=str(tmp_path / "ros.sqlite3")),
        ],
    )
    first = create_ros_robot(context, "amr_01", "/amr_01", 9.0)
    second = create_ros_robot(
        context, "amr_02", "/amr_02", 2.0, cost_available=scenario != "missing_cost"
    )
    peer = rclpy.create_node("fleet_test_client", context=context)
    client = ActionClient(peer, ExecuteFleetMission, "/factory/execute_fleet_mission")
    executor = SingleThreadedExecutor(context=context)
    for node in (fleet, first, second, peer):
        executor.add_node(node)
    gateway = None

    def wait(predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        wait(
            lambda: (
                client.server_is_ready()
                and len(fleet.adapter.registry.eligible(time.monotonic())) == 2
            )
        )
        if scenario == "resources":
            from factory_interfaces.srv import (
                AcquireResource,
                CancelResourceWait,
                ReleaseResource,
            )

            service_clients = [
                peer.create_client(service, "/factory/resources/" + name)
                for service, name in (
                    (AcquireResource, "acquire"),
                    (CancelResourceWait, "cancel_wait"),
                    (ReleaseResource, "release"),
                )
            ]
            for service_client in service_clients:
                wait(service_client.service_is_ready)
            owner = service_clients[0].call_async(
                AcquireResource.Request(
                    robot_id="amr_01", mission_id="owner", resource_id="central_aisle"
                )
            )
            wait(owner.done)
            assert owner.result().granted
            waiter = service_clients[0].call_async(
                AcquireResource.Request(
                    robot_id="amr_02", mission_id="waiter", resource_id="central_aisle"
                )
            )
            wait(waiter.done)
            assert not waiter.result().granted
            cancel = service_clients[1].call_async(
                CancelResourceWait.Request(
                    robot_id="amr_02", mission_id="waiter", resource_id="central_aisle"
                )
            )
            wait(cancel.done)
            assert cancel.result().cancelled
            release = service_clients[2].call_async(
                ReleaseResource.Request(
                    robot_id="amr_01",
                    mission_id="owner",
                    resource_id="central_aisle",
                    lease_id=owner.result().lease_id,
                )
            )
            wait(release.done)
            assert release.result().released
            for service_client in service_clients:
                peer.destroy_client(service_client)
            return
        if scenario == "mqtt":
            from mqtt_gateway.node import MqttGatewayNode
            import json

            class Broker:
                def __init__(self):
                    self.statuses = []

                def set_handlers(self, message, connection):
                    self.message, self.connection = message, connection

                def set_fault_handlers(self, message, connection):
                    pass

                def start(self):
                    self.connection(True)

                def publish(self, topic, payload, qos=0, retain=False):
                    if topic.endswith("/status"):
                        self.statuses.append(json.loads(payload))
                    return True

                def close(self):
                    pass

            broker = Broker()
            gateway = MqttGatewayNode(
                context=context,
                mqtt_client=broker,
                parameter_overrides=[Parameter("fleet_file", value=str(config_path))],
            )
            executor.add_node(gateway)
            broker.message(
                b'{"mission_id":"vertical_ros","pickup":"assembly","dropoff":"inspection","part":"motor"}'
            )
            wait(
                lambda: any(
                    status["state"] == "COMPLETED" for status in broker.statuses
                )
            )
            assert broker.statuses[-1]["robot_id"] == "amr_02"
            assert any(
                status["state"] == "ASSIGNED" and status["robot_id"] == "amr_02"
                for status in broker.statuses
            )
            return
        feedback = []
        if scenario in ("cancel", "offline"):
            second.finish = False
            second.stage = "NAVIGATING_TO_PICKUP"
        goal = ExecuteFleetMission.Goal(
            mission_id="ros",
            pickup_station="assembly",
            dropoff_station="inspection",
            part="motor",
            requested_robot_id="amr_01" if scenario == "pinned" else "",
        )
        sent = client.send_goal_async(
            goal, feedback_callback=lambda message: feedback.append(message.feedback)
        )
        wait(sent.done)
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        if scenario == "cancel":
            wait(lambda: bool(second.goals))
            cancelled = handle.cancel_goal_async()
            wait(cancelled.done)
            wait(result.done)
            assert result.result().result.final_state == "CANCELLED"
        else:
            if scenario == "offline":
                wait(lambda: bool(second.goals))
                second.heartbeat_timer.cancel()
            wait(result.done)
            assert result.result().result.success
            want = (
                "amr_01"
                if scenario in ("pinned", "missing_cost", "offline")
                else "amr_02"
            )
            assert result.result().result.assigned_robot_id == want
            wait(lambda: any(item.assigned_robot_id == want for item in feedback))
            if scenario == "duplicate":
                repeated = client.send_goal_async(goal)
                wait(repeated.done)
                replay = repeated.result().get_result_async()
                wait(replay.done)
                assert replay.result().result.success
                assert len(second.goals) == 1
    finally:
        client.destroy()
        executor.shutdown()
        if gateway is not None:
            gateway.destroy_node()
        for node in (fleet, first, second, peer):
            node.destroy_node()
        context.shutdown()
