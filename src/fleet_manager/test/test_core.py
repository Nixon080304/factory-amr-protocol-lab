# SPDX-License-Identifier: Apache-2.0
"""Mission safety and durable decisions exercise real fleet policies and SQLite."""

from dataclasses import replace
import importlib
from pathlib import Path
import sqlite3
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_manager.config import EnergyPolicyConfig, Pose2D, ResourceConfig
from fleet_manager.dispatcher import Dispatcher
from fleet_manager.energy import EnergyPolicy
from fleet_manager.journal import MissionConflictError, MissionJournal, MissionState
from fleet_manager.models import CostEstimate, MissionRequest, RobotSnapshot
from fleet_manager.registry import RobotRegistry
from fleet_manager.resources import LeaseRequest, ResourceManager


def core_api():
    try:
        return importlib.import_module("fleet_manager.core")
    except ModuleNotFoundError:
        pytest.fail("durable fleet mission core is not implemented")


def request(mission_id="m1", **changes):
    return replace(
        MissionRequest(mission_id, "assembly", "inspection", "gear"), **changes
    )


def robot(robot_id="r1", **changes):
    return replace(
        RobotSnapshot(robot_id, "AVAILABLE", Pose2D(0, 0, 0), 80, "EMPTY"), **changes
    )


ESTIMATES = {"r1": CostEstimate(True, 1, 40), "r2": CostEstimate(True, 2, 40)}


@pytest.mark.parametrize(
    "unsafe",
    [
        {"health": "OFFLINE"},
        {"mode": "EXECUTING"},
        {"payload_state": "UNKNOWN"},
        {"mission_id": "m1"},
        {"pose": None},
    ],
)
def test_reassignment_requires_fresh_stopped_empty_owner_even_without_lease(
    fleet, unsafe
):
    _, core, journal, registry, *_ = fleet
    assigned(fleet)
    registry.observe(robot("r1", health="OFFLINE"), 102)
    core.handle_robot_offline("r1", 102)
    registry.observe(robot("r2"), 102.1)
    registry.observe(robot("r1", **unsafe), 102.1)
    assert core.assign("m1", {"r2": ESTIMATES["r2"]}, 102.1).robot_id is None
    assert journal.get("m1").state == "REASSIGNING"
    registry.observe(robot("r1"), 102.2)
    assert core.assign("m1", {"r2": ESTIMATES["r2"]}, 102.2).robot_id == "r2"


def test_reassignment_refuses_exact_quarantine_until_verified_clearance(fleet):
    _, core, journal, registry, resources, _ = fleet
    assigned(fleet)
    held = resources.acquire(LeaseRequest("r1", "m1", "assembly"), 101).lease
    registry.observe(robot("r1", health="OFFLINE"), 102)
    core.handle_robot_offline("r1", 102)
    registry.observe(robot("r1"), 102.1)
    registry.observe(robot("r2"), 102.1)
    assert core.assign("m1", {"r2": ESTIMATES["r2"]}, 102.1).robot_id is None
    assert journal.get("m1").state == "REASSIGNING"
    from fleet_manager.resources import LeaseKey

    assert resources.clear_reconciliation(
        LeaseKey(held.robot_id, held.mission_id, held.resource_id, held.lease_id), 102.2
    )
    assert core.assign("m1", {"r2": ESTIMATES["r2"]}, 102.2).robot_id == "r2"


def test_charging_queue_reserves_idle_low_battery_robots_before_missions(fleet):
    _, core, _, registry, *_ = fleet
    policy = EnergyPolicyConfig(20, 30, 80, "dock")
    registry.observe(robot("r2", battery_percent=20), 100)
    registry.observe(robot("r1", battery_percent=20), 100)
    assert hasattr(core, "queue_charging"), (
        "FleetCore charging reservations are missing"
    )
    queued = core.queue_charging(policy, 100)
    assert [item.robot_id for item in queued] == ["r1", "r2"]
    assert [item.state for item in queued] == ["CHARGE_QUEUED", "CHARGE_QUEUED"]
    core.submit(request(), 100)
    assert core.assign("m1", ESTIMATES, 100).robot_id is None
    first = queued[0]
    core.charging_feedback(first.robot_id, first.generation, "CHARGING")
    core.charging_result(first.robot_id, first.generation, True)
    registry.observe(robot("r1", battery_percent=80), 100)
    assert core.assign("m1", ESTIMATES, 100).robot_id == "r1"
    core.charging_feedback(first.robot_id, first.generation, "CHARGING")
    assert [item.robot_id for item in core.charging_snapshot()] == ["r2"]


@pytest.mark.parametrize(
    "changes",
    [
        {"mode": "EXECUTING"},
        {"mode": "RECOVERY_REQUIRED"},
        {"mode": "DOCKING"},
        {"mode": "CHARGING"},
        {"payload_state": "LOADED"},
        {"payload_state": "UNKNOWN"},
        {"mission_id": "other"},
        {"health": "UNHEALTHY"},
        {"fault": "drive fault"},
        {"battery_percent": 30},
    ],
)
def test_auto_charging_excludes_busy_payload_and_unhealthy_robots(fleet, changes):
    _, core, _, registry, *_ = fleet
    registry.observe(
        robot("r1", battery_percent=20, **changes)
        if "battery_percent" not in changes
        else robot("r1", **changes),
        100,
    )
    assert hasattr(core, "queue_charging"), "FleetCore charging policy is missing"
    assert core.queue_charging(EnergyPolicyConfig(20, 30, 80, "dock"), 100) == ()


def test_active_mission_reservation_prevents_auto_charge_on_late_idle_heartbeat(fleet):
    _, core, _, registry, *_ = fleet
    assigned(fleet)
    registry.observe(robot("r1", battery_percent=20), 101)
    assert hasattr(core, "queue_charging"), "FleetCore charging policy is missing"
    assert core.queue_charging(EnergyPolicyConfig(20, 30, 80, "dock"), 101) == ()


@pytest.fixture
def fleet(tmp_path):
    api = core_api()
    journal = MissionJournal(tmp_path / "missions.sqlite3")
    registry = RobotRegistry(("r1", "r2"))
    for robot_id in ("r1", "r2"):
        registry.observe(robot(robot_id), 100)
    resources = ResourceManager((ResourceConfig("assembly", "station", 1),))
    dispatcher = Dispatcher(EnergyPolicy(EnergyPolicyConfig(20, 30, 80, "dock")))
    core = api.FleetCore(dispatcher, registry, resources, journal)
    yield api, core, journal, registry, resources, dispatcher
    journal.close()


def assigned(fleet, **changes):
    _, core, journal, *_ = fleet
    core.submit(request(**changes), 100)
    decision = core.assign("m1", ESTIMATES, 101)
    return decision, journal.get("m1")


def feedback(api, decision, ownership="NOT_PICKED_UP"):
    return api.RobotMissionFeedback(
        decision.robot_id, decision.assignment_id, ownership
    )


def result(api, decision, success=True, **changes):
    return api.RobotMissionResult(
        decision.robot_id, decision.assignment_id, success, **changes
    )


def test_explicit_robot_recovery_overrides_pre_pickup_offline_reassignment(fleet):
    api, core, journal, registry, *_ = fleet
    decision, _ = assigned(fleet)
    outcome = core.record_robot_result(
        "m1",
        result(
            api, decision, False, error_code="ROBOT_OFFLINE", recovery_required=True
        ),
        110,
    )
    assert registry.get(decision.robot_id, 110).health == "OFFLINE"
    assert outcome.state == "RECOVERY_REQUIRED"
    assert outcome in journal.load_active()
    assert core.assign("m1", ESTIMATES, 110).robot_id is None


@pytest.mark.parametrize("pin,want", [(None, "r1"), ("r2", "r2")])
def test_assignment_decision_is_durable_before_external_goal_can_start(
    fleet, pin, want
):
    _, core, journal, *_ = fleet
    decision, record = assigned(fleet, requested_robot_id=pin)
    assert decision.robot_id == want
    assert record.state == "ASSIGNED"
    assert record.assigned_robot_id == want
    assert decision.assignment_id == journal.events("m1")[-1].sequence
    assert [event.state for event in journal.events("m1")] == [
        "QUEUED",
        "ASSIGNING",
        "ASSIGNED",
    ]
    assert core.assign("m1", ESTIMATES, 102).robot_id is None


def test_no_estimates_keeps_mission_schedulable_without_assignment(fleet):
    _, core, journal, *_ = fleet
    core.submit(request(), 100)
    decision = core.assign("m1", {}, 101)
    assert decision.robot_id is None
    assert decision.reason == "no eligible robot"
    assert journal.get("m1").state == "QUEUED"
    assert journal.get("m1").assigned_robot_id is None
    assert core.assign("m1", ESTIMATES, 102).robot_id == "r1"


def test_pinned_unavailable_robot_never_dispatches_alternative(fleet):
    _, core, journal, registry, *_ = fleet
    core.submit(request(requested_robot_id="r2"), 100)
    registry.observe(robot("r2", health="OFFLINE"), 101)
    decision = core.assign("m1", ESTIMATES, 101)
    assert decision.robot_id is None
    assert decision.reason == "requested robot unavailable"
    assert journal.get("m1").state == "QUEUED"


def test_active_reservation_prevents_two_goals_on_same_available_heartbeat(fleet):
    _, core, *_ = fleet
    assigned(fleet)
    core.submit(request("m2"), 100)
    assert core.assign("m2", ESTIMATES, 101).robot_id == "r2"


@pytest.mark.parametrize(
    "changes",
    [
        {"pickup_station": "other"},
        {"dropoff_station": "other"},
        {"part": "other"},
        {"requested_robot_id": "r2"},
    ],
)
def test_duplicate_hash_conflicts_for_every_immutable_request_field(fleet, changes):
    _, core, journal, *_ = fleet
    original = core.submit(request(), 100)
    assert core.submit(request(), 102) == original
    with pytest.raises(MissionConflictError):
        core.submit(request(**changes), 103)
    assert journal.get("m1") == original
    assert len(journal.events("m1")) == 1


def test_hash_is_stable_across_reopen_and_includes_mission_identity(fleet, tmp_path):
    api, core, journal, registry, resources, dispatcher = fleet
    first = core.submit(request(), 100)
    other = core.submit(request("m2"), 100)
    assert first.payload_hash != other.payload_hash
    reopened = MissionJournal(tmp_path / "missions.sqlite3")
    try:
        restarted = api.FleetCore(dispatcher, registry, resources, reopened)
        assert restarted.submit(request(), 103) == first
    finally:
        reopened.close()


@pytest.mark.parametrize("ownership", ["NOT_PICKED_UP", "PICKED_UP", "UNKNOWN"])
def test_cancellation_respects_payload_ownership_and_is_idempotent(fleet, ownership):
    api, core, journal, *_ = fleet
    decision, _ = assigned(fleet)
    core.record_robot_feedback("m1", feedback(api, decision, ownership), 101.1)
    cancelled = core.cancel("m1", 101.2)
    want = "CANCELLED" if ownership == "NOT_PICKED_UP" else "RECOVERY_REQUIRED"
    assert cancelled.state == want
    assert cancelled.result["final_state"] == want
    events = journal.events("m1")
    assert core.cancel("m1", 102) == cancelled
    assert core.record_robot_result("m1", result(api, decision), 102) == cancelled
    assert core.assign("m1", ESTIMATES, 102).robot_id is None
    assert journal.events("m1") == events


def test_queued_cancellation_needs_no_robot_and_is_durable(fleet):
    _, core, journal, *_ = fleet
    core.submit(request(), 100)
    assert core.cancel("m1", 101).state == "CANCELLED"
    assert journal.load_active() == ()


def test_running_cancellation_request_is_durable_idempotent_and_keeps_callback_fence(
    fleet,
):
    api, core, journal, *_ = fleet
    decision, _ = assigned(fleet)
    record = core.request_cancellation("m1", 101.1)
    assert record.state == "ASSIGNED" and record.result is None
    assert journal.events("m1")[-1].detail["cancellation_requested"] is True
    events = journal.events("m1")
    assert core.request_cancellation("m1", 101.2) == record
    assert journal.events("m1") == events
    assert (
        core.record_robot_feedback(
            "m1", feedback(api, decision, "PICKED_UP"), 101.3
        ).payload_ownership
        == "PICKED_UP"
    )


@pytest.mark.parametrize(
    "ownership,want",
    [
        ("NOT_PICKED_UP", "CANCELLED"),
        ("UNKNOWN", "RECOVERY_REQUIRED"),
        ("PICKED_UP", "RECOVERY_REQUIRED"),
    ],
)
def test_pending_cancellation_finishes_from_fenced_execution_result(
    fleet, ownership, want
):
    api, core, journal, *_ = fleet
    decision, _ = assigned(fleet)
    core.request_cancellation("m1", 101.1)
    core.record_robot_feedback("m1", feedback(api, decision, ownership), 101.2)
    outcome = core.record_robot_result(
        "m1",
        result(api, decision, False, error_code="MISSION_CANCELED", cancelled=True),
        101.3,
    )
    assert outcome.state == want
    assert outcome.result["error_code"] == "MISSION_CANCELED"
    assert journal.get("m1") == outcome


def test_cancellation_acknowledgement_is_durable_but_not_execution_completion(fleet):
    api, core, journal, *_ = fleet
    decision, _ = assigned(fleet)
    core.request_cancellation("m1", 101.1)
    ack = api.RobotCancellationAck(decision.robot_id, decision.assignment_id, True)
    outcome = core.record_cancellation_acknowledgement("m1", ack, 101.2)
    assert outcome.state == "ASSIGNED" and outcome.result is None
    assert journal.events("m1")[-1].detail["cancellation_acknowledged"] is True
    assert (
        core.record_robot_feedback(
            "m1", feedback(api, decision, "UNKNOWN"), 101.3
        ).payload_ownership
        == "UNKNOWN"
    )


def test_rejected_cancellation_does_not_relabel_unrelated_empty_robot_failure(fleet):
    api, core, *_ = fleet
    decision, _ = assigned(fleet)
    core.request_cancellation("m1", 101.1)
    core.record_cancellation_acknowledgement(
        "m1",
        api.RobotCancellationAck(decision.robot_id, decision.assignment_id, False),
        101.2,
    )
    outcome = core.record_robot_result(
        "m1", result(api, decision, False, error_code="DRIVE_FAULT"), 101.3
    )
    assert outcome.state == "FAILED" and outcome.result["error_code"] == "DRIVE_FAULT"


def test_execution_cancelled_outcome_finalizes_empty_robot_without_error_string(fleet):
    api, core, *_ = fleet
    decision, _ = assigned(fleet)
    core.request_cancellation("m1", 101.1)
    outcome = core.record_robot_result(
        "m1", result(api, decision, False, cancelled=True), 101.2
    )
    assert outcome.state == "CANCELLED" and outcome.result["error_code"] == "CANCELLED"


@pytest.mark.parametrize("ownership", ["UNKNOWN", "PICKED_UP"])
def test_rejected_cancellation_preserves_payload_on_eventual_fault(fleet, ownership):
    api, core, *_ = fleet
    decision, _ = assigned(fleet)
    core.request_cancellation("m1", 101.1)
    core.record_cancellation_acknowledgement(
        "m1",
        api.RobotCancellationAck(decision.robot_id, decision.assignment_id, False),
        101.2,
    )
    core.record_robot_feedback("m1", feedback(api, decision, ownership), 101.3)
    outcome = core.record_robot_result(
        "m1", result(api, decision, False, error_code="DRIVE_FAULT"), 101.4
    )
    assert outcome.state == "RECOVERY_REQUIRED"
    assert outcome.payload_ownership == ownership
    assert outcome.result["error_code"] == "DRIVE_FAULT"


def test_cancellation_acknowledgement_rejects_stale_assignment_and_duplicate(fleet):
    api, core, journal, *_ = fleet
    decision, _ = assigned(fleet)
    core.request_cancellation("m1", 101.1)
    events = journal.events("m1")
    core.record_cancellation_acknowledgement(
        "m1",
        api.RobotCancellationAck(decision.robot_id, decision.assignment_id + 1, True),
        101.2,
    )
    assert journal.events("m1") == events
    ack = api.RobotCancellationAck(decision.robot_id, decision.assignment_id, True)
    record = core.record_cancellation_acknowledgement("m1", ack, 101.3)
    events = journal.events("m1")
    assert core.record_cancellation_acknowledgement("m1", ack, 101.4) == record
    assert journal.events("m1") == events


def test_offline_during_pending_cancellation_requires_recovery_and_never_reassigns(
    fleet,
):
    api, core, journal, registry, *_ = fleet
    assigned(fleet)
    core.request_cancellation("m1", 101.1)
    registry.observe(robot("r1", health="OFFLINE"), 102)
    assert core.handle_robot_offline("r1", 102)[0].state == "RECOVERY_REQUIRED"
    assert journal.get("m1").payload_ownership == "UNKNOWN"
    assert journal.get("m1").result["error_code"] == "ROBOT_OFFLINE"
    assert core.assign("m1", ESTIMATES, 102).robot_id is None


def test_reject_scheduling_request_commits_explicit_replayable_result(fleet):
    _, core, journal, *_ = fleet
    core.submit(request(requested_robot_id="r1"), 100)
    rejected = core.reject(
        "m1", "REQUESTED_ROBOT_UNAVAILABLE", "Requested robot r1 unavailable", 101
    )
    assert rejected.state == "FAILED"
    assert rejected.result["error_code"] == "REQUESTED_ROBOT_UNAVAILABLE"
    assert rejected.result["assigned_robot_id"] is None
    assert journal.get("m1") == rejected
    assert core.submit(request(requested_robot_id="r1"), 102) == rejected


@pytest.mark.parametrize("ownership", ["NOT_PICKED_UP", "PICKED_UP", "UNKNOWN"])
def test_offline_robot_reassigns_only_before_pickup_and_quarantines_resources(
    fleet, ownership
):
    api, core, journal, registry, resources, _ = fleet
    decision, _ = assigned(fleet)
    core.record_robot_feedback("m1", feedback(api, decision, ownership), 101.1)
    held = resources.acquire(LeaseRequest("r1", "m1", "assembly"), 101).lease
    registry.observe(robot("r1", health="OFFLINE"), 102)
    records = core.handle_robot_offline("r1", 102)
    assert len(records) == 1
    state = records[0]
    if ownership == "NOT_PICKED_UP":
        assert state.state == "REASSIGNING"
        assert state.assigned_robot_id is None
        assert core.assign("m1", ESTIMATES, 102).robot_id is None
    else:
        assert state.state == "RECOVERY_REQUIRED"
        assert core.assign("m1", ESTIMATES, 102).robot_id is None
    resource = resources.snapshot(102)[0]
    assert resource.lease is None
    assert resource.former_lease == held
    assert resource.reconciliation_required
    if ownership == "NOT_PICKED_UP":
        registry.observe(robot("r1"), 102.1)
        assert core.assign("m1", {"r2": ESTIMATES["r2"]}, 102.1).robot_id is None
        from fleet_manager.resources import LeaseKey

        assert resources.clear_reconciliation(
            LeaseKey(held.robot_id, held.mission_id, held.resource_id, held.lease_id),
            102.2,
        )
        assert core.assign("m1", {"r2": ESTIMATES["r2"]}, 102.2).robot_id == "r2"
    events = journal.events("m1")
    assert core.handle_robot_offline("r1", 102) == ()
    assert journal.events("m1") == events


@pytest.mark.parametrize("reuse_same_robot", [False, True])
def test_old_assignment_feedback_and_results_cannot_mutate_new_goal(
    fleet, reuse_same_robot
):
    api, core, journal, registry, *_ = fleet
    old, _ = assigned(fleet)
    registry.observe(robot("r1", health="OFFLINE"), 102)
    core.handle_robot_offline("r1", 102)
    registry.observe(robot("r1"), 102.1)
    new = core.assign(
        "m1", ESTIMATES if reuse_same_robot else {"r2": ESTIMATES["r2"]}, 102.1
    )
    assert new.robot_id == ("r1" if reuse_same_robot else "r2")
    assert new.assignment_id is not None
    assert new.assignment_id != old.assignment_id
    record = journal.get("m1")
    events = journal.events("m1")
    assert (
        core.record_robot_feedback("m1", feedback(api, old, "PICKED_UP"), 102.2)
        == record
    )
    assert core.record_robot_result("m1", result(api, old), 102.2) == record
    assert journal.events("m1") == events


def test_feedback_cannot_downgrade_confirmed_or_uncertain_payload(fleet):
    api, core, _, *_ = fleet
    decision, _ = assigned(fleet)
    core.record_robot_feedback("m1", feedback(api, decision, "UNKNOWN"), 101.1)
    assert (
        core.record_robot_feedback(
            "m1", feedback(api, decision), 101.2
        ).payload_ownership
        == "UNKNOWN"
    )
    core.record_robot_feedback("m1", feedback(api, decision, "PICKED_UP"), 101.3)
    assert (
        core.record_robot_feedback(
            "m1", feedback(api, decision), 101.4
        ).payload_ownership
        == "PICKED_UP"
    )


@pytest.mark.parametrize(
    "success,want", [(True, "COMPLETED"), (False, "RECOVERY_REQUIRED")]
)
def test_robot_result_is_terminal_and_duplicate_operations_replay(fleet, success, want):
    api, core, journal, *_ = fleet
    decision, _ = assigned(fleet)
    core.record_robot_feedback("m1", feedback(api, decision, "PICKED_UP"), 101.1)
    outcome = core.record_robot_result(
        "m1",
        result(api, decision, success, error_code="" if success else "DRIVE_FAULT"),
        101.2,
    )
    assert outcome.state == want
    assert outcome.result["final_state"] == want
    assert outcome.result["success"] is success
    assert outcome.payload_ownership == ("DELIVERED" if success else "PICKED_UP")
    events = journal.events("m1")
    assert (
        core.record_robot_result("m1", result(api, decision, not success), 102)
        == outcome
    )
    assert core.record_robot_feedback("m1", feedback(api, decision), 102) == outcome
    assert core.cancel("m1", 102) == outcome
    assert core.submit(request(), 102) == outcome
    assert core.handle_robot_offline("r1", 102) == ()
    assert journal.load_active() == (() if success else (outcome,))
    assert journal.events("m1") == events


@pytest.mark.parametrize("ownership", ["PICKED_UP", "UNKNOWN"])
def test_ordinary_inspection_timeout_keeps_unresolved_carrier_reserved(
    fleet, ownership
):
    api, core, journal, registry, *_ = fleet
    decision, _ = assigned(fleet)
    core.record_robot_feedback("m1", feedback(api, decision, ownership), 101.1)
    outcome = core.record_robot_result(
        "m1",
        result(
            api,
            decision,
            False,
            error_code="RESOURCE_WAIT_TIMEOUT",
            recovery_required=False,
        ),
        101.2,
    )
    assert outcome.state == "RECOVERY_REQUIRED"
    assert outcome.payload_ownership == ownership
    assert outcome.assigned_robot_id == "r1"
    assert journal.load_recovery() == (outcome,)
    registry.observe(robot("r1"), 101.3)
    core.submit(request("next", requested_robot_id="r1"), 101.3)
    assert core.assign("next", ESTIMATES, 101.3).robot_id is None


def test_core_reconcile_preserves_failed_result_and_observed_custody(fleet):
    api, core, journal, *_ = fleet
    assigned(fleet)
    journal.record_observation(
        robot("r1", payload_state="LOADED", mission_id="m1"), 101.1
    )
    journal.transition(
        "m1",
        MissionState.ASSIGNED,
        MissionState.FAILED,
        {
            "result": {
                "success": False,
                "final_state": "FAILED",
                "error_code": "RESOURCE_WAIT_TIMEOUT",
            }
        },
        101.2,
    )
    core.reconcile(api.ReconciliationSnapshot((robot("r1"), robot("r2"))), 101.3)
    record = journal.get("m1")
    assert record.state == "RECOVERY_REQUIRED"
    assert record.payload_ownership == "PICKED_UP"
    assert record.result["final_state"] == "RECOVERY_REQUIRED"
    assert record.result["error_code"] == "RESOURCE_WAIT_TIMEOUT"


def test_offline_pre_pickup_failure_uses_specific_reassignment_rule(fleet):
    api, core, _, registry, *_ = fleet
    decision, _ = assigned(fleet)
    registry.observe(robot("r1", health="OFFLINE"), 102)
    assert (
        core.record_robot_result(
            "m1", result(api, decision, False, error_code="ROBOT_OFFLINE"), 102
        ).state
        == "REASSIGNING"
    )


def test_normal_failure_before_pickup_does_not_automatically_retry(fleet):
    api, core, *_ = fleet
    decision, _ = assigned(fleet)
    assert (
        core.record_robot_result(
            "m1", result(api, decision, False, error_code="NAVIGATION_FAILED"), 102
        ).state
        == "FAILED"
    )


def test_sqlite_assignment_failure_returns_no_goal_decision(fleet, tmp_path):
    _, core, journal, *_ = fleet
    core.submit(request(), 100)
    with sqlite3.connect(tmp_path / "missions.sqlite3") as connection:
        connection.execute(
            "CREATE TRIGGER fail_assignment BEFORE INSERT ON mission_events "
            "WHEN NEW.state = 'ASSIGNED' "
            "BEGIN SELECT RAISE(ABORT, 'assignment persistence failed'); END"
        )
    with pytest.raises(sqlite3.IntegrityError, match="assignment persistence failed"):
        core.assign("m1", ESTIMATES, 101)
    assert journal.get("m1").state == "ASSIGNING"
    assert journal.get("m1").assigned_robot_id is None


def test_offline_persistence_failure_does_not_release_held_resources(fleet, tmp_path):
    _, core, journal, _, resources, _ = fleet
    assigned(fleet)
    held = resources.acquire(LeaseRequest("r1", "m1", "assembly"), 101).lease
    with sqlite3.connect(tmp_path / "missions.sqlite3") as connection:
        connection.execute(
            "CREATE TRIGGER fail_offline BEFORE INSERT ON mission_events "
            "WHEN NEW.state = 'REASSIGNING' "
            "BEGIN SELECT RAISE(ABORT, 'offline persistence failed'); END"
        )
    with pytest.raises(sqlite3.IntegrityError, match="offline persistence failed"):
        core.handle_robot_offline("r1", 102)
    assert journal.get("m1").state == "ASSIGNED"
    assert resources.snapshot(102)[0].lease == held


@pytest.mark.parametrize("state", list(MissionState))
def test_restart_reconciliation_never_resumes_uncertain_assignment(
    fleet, state, tmp_path
):
    api, core, journal, registry, resources, dispatcher = fleet
    core.submit(request(), 100)
    if state != MissionState.QUEUED:
        journal.transition("m1", MissionState.QUEUED, state, {}, 101)
    reopened = MissionJournal(tmp_path / "missions.sqlite3")
    try:
        restarted = api.FleetCore(dispatcher, registry, resources, reopened)
        decision = restarted.reconcile(api.ReconciliationSnapshot(()), 102)
        want = {
            "RECEIVED": "QUEUED",
            "QUEUED": "QUEUED",
            "ASSIGNING": "QUEUED",
            "REASSIGNING": "REASSIGNING",
            "ASSIGNED": "RECOVERY_REQUIRED",
            "EXECUTING": "RECOVERY_REQUIRED",
            "COMPLETED": "COMPLETED",
            "FAILED": "FAILED",
            "CANCELLED": "CANCELLED",
            "RECOVERY_REQUIRED": "RECOVERY_REQUIRED",
        }[state.value]
        assert reopened.get("m1").state == want
        assert decision.schedulable_mission_ids == (
            ("m1",) if want in ("QUEUED", "REASSIGNING") else ()
        )
        events = reopened.events("m1")
        assert restarted.reconcile(api.ReconciliationSnapshot(()), 102) == decision
        assert reopened.events("m1") == events
    finally:
        reopened.close()


def test_reconciliation_retains_execution_with_matching_robot_and_payload(fleet):
    api, core, journal, *_ = fleet
    decision, _ = assigned(fleet)
    core.record_robot_feedback("m1", feedback(api, decision, "PICKED_UP"), 101.1)
    observation = robot("r1", mission_id="m1", mode="EXECUTING", payload_state="LOADED")
    before = journal.get("m1")
    reconciled = core.reconcile(api.ReconciliationSnapshot((observation,)), 102)
    assert journal.get("m1") == before
    assert reconciled.recovery_required_mission_ids == ()
    assert reconciled.schedulable_mission_ids == ()


@pytest.mark.parametrize(
    "changes",
    [
        {"mission_id": "other"},
        {"payload_state": "UNKNOWN"},
        {"payload_state": "EMPTY"},
        {"health": "OFFLINE"},
        {"mode": "AVAILABLE"},
        {"fault": "DRIVE_FAULT"},
    ],
)
def test_reconciliation_fences_conflicting_robot_or_payload_evidence(fleet, changes):
    api, core, journal, *_ = fleet
    decision, _ = assigned(fleet)
    core.record_robot_feedback("m1", feedback(api, decision, "PICKED_UP"), 101.1)
    observation = robot("r1", mission_id="m1", mode="EXECUTING", payload_state="LOADED")
    reconciled = core.reconcile(
        api.ReconciliationSnapshot((replace(observation, **changes),)), 102
    )
    assert journal.get("m1").state == "RECOVERY_REQUIRED"
    assert reconciled.recovery_required_mission_ids == ("m1",)


def test_reconciliation_requires_durable_assignment_event(fleet):
    api, core, journal, *_ = fleet
    core.submit(request(), 100)
    journal.transition(
        "m1",
        MissionState.QUEUED,
        MissionState.ASSIGNED,
        {"assigned_robot_id": "r1"},
        101,
    )
    # This event is syntactically ASSIGNED but lacks the expected assigning predecessor.
    observed = robot("r1", mission_id="m1", mode="RESERVED")
    core.reconcile(api.ReconciliationSnapshot((observed,)), 102)
    assert journal.get("m1").state == "RECOVERY_REQUIRED"


@pytest.mark.parametrize("state", ["QUEUED", "ASSIGNING", "REASSIGNING"])
def test_reconciliation_fences_scheduling_state_with_uncertain_payload(fleet, state):
    api, core, journal, *_ = fleet
    core.submit(request(), 100)
    journal.transition(
        "m1",
        MissionState.QUEUED,
        MissionState(state),
        {"payload_ownership": "UNKNOWN"},
        101,
    )
    reconciled = core.reconcile(api.ReconciliationSnapshot(()), 102)
    assert journal.get("m1").state == "RECOVERY_REQUIRED"
    assert reconciled.schedulable_mission_ids == ()


def test_reconciliation_fences_robot_claim_on_queued_mission(fleet):
    api, core, journal, *_ = fleet
    core.submit(request(), 100)
    observed = robot("r1", mission_id="m1", mode="EXECUTING")
    core.reconcile(api.ReconciliationSnapshot((observed,)), 102)
    assert journal.get("m1").state == "RECOVERY_REQUIRED"


def test_reconciliation_fences_duplicate_robot_observations(fleet):
    api, core, journal, *_ = fleet
    assigned(fleet)
    observed = robot("r1", mission_id="m1", mode="RESERVED")
    core.reconcile(api.ReconciliationSnapshot((observed, observed)), 102)
    assert journal.get("m1").state == "RECOVERY_REQUIRED"


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), True, "100"])
def test_invalid_core_clock_cannot_create_a_mission(fleet, invalid):
    _, core, journal, *_ = fleet
    with pytest.raises(ValueError, match="finite"):
        core.submit(request(), invalid)
    assert journal.load_active() == ()
