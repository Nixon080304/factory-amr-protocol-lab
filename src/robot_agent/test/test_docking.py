"""Exercise physical gates and exact lease cleanup through the production controller."""

import importlib
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_agent_adapter import rig as agent_rig, ready


def api():
    try:
        return importlib.import_module("robot_agent.docking")
    except ModuleNotFoundError:
        pytest.fail("production DockingController is missing")


class Wire:
    def __init__(self):
        self.moves, self.requests, self.cancelled = [], [], []

    def navigate(self, pose, frame, callback):
        index = len(self.moves)
        self.moves.append((pose, frame, callback))
        return lambda: self.cancelled.append(index)

    def resource(self, operation, key, callback):
        self.requests.append((operation, key, callback))

    def reply(self, **values):
        self.requests[-1][2](SimpleNamespace(**values))


def rig():
    agent, _, wall = agent_rig()
    agent.energy.battery_percent = 20
    ready(agent)
    wire, results, feedback = Wire(), [], []
    controller = api().DockingController(
        agent, wire, "charger", (2, -3, 0), (3, -3, 0), clock=lambda: wall[0]
    )
    return agent, wall, wire, results, feedback, controller


def start(rig_):
    *_, results, feedback, controller = rig_
    assert controller.start(80, feedback.append, results.append)


def arrive(rig_, index, pose):
    agent, wall, wire, *_, controller = rig_
    stamp = agent._pose_stamp + 1
    agent.odometry(stamp, 0, 0, agent.frame_prefix + "odom")
    agent.localization(stamp, *pose, agent.frame_prefix + "map")
    wire.moves[index][2](True, "")
    controller.tick()


def stage(rig_):
    start(rig_)
    arrive(rig_, 0, (2, -3, 0))


def enter(rig_):
    stage(rig_)
    rig_[2].reply(granted=True, lease_id="lease-1", lease_ttl_sec=10.0)
    arrive(rig_, 1, (3, -3, 0))


def test_stage_without_lease_then_wait_before_entering():
    r = rig()
    start(r)
    agent, _, wire, _, feedback, controller = r
    assert agent.mode == "DOCKING" and wire.requests == []
    assert wire.moves[0][:2] == ((2, -3, 0), "floor/cart_1/map")
    wire.moves[0][2](True, "")
    controller.tick()
    assert wire.requests == []  # Action success is not localization evidence.
    arrive(r, 0, (2, -3, 0))
    assert wire.requests[-1][0] == "acquire"
    wire.reply(granted=False, lease_id="", lease_ttl_sec=0.0)
    assert len(wire.moves) == 1 and feedback[-1].state == "WAITING_FOR_LEASE"
    r[1][0] += 0.3
    controller.tick()
    wire.reply(granted=True, lease_id="lease-1", lease_ttl_sec=10.0)
    assert wire.moves[-1][0] == (3, -3, 0)
    assert wire.requests[-1][1].resource_id == "charger"


def test_charge_requires_contact_pose_and_current_lease_then_verified_exit():
    r = rig()
    enter(r)
    agent, wall, wire, results, _, controller = r
    assert agent.mode == "DOCKING" and controller.state == "WAITING_FOR_CONTACT"
    agent.energy.battery_percent = 79
    controller.contact(True)
    assert agent.mode == "CHARGING"
    wall[0] += 0.5
    controller.contact(True)
    assert agent.energy.battery_percent == pytest.approx(79.4995)
    wall[0] += 0.6
    controller.contact(True)
    assert controller.state == "EXITING" and not results
    assert wire.moves[-1][0] == (2, -3, 0)
    assert all(operation != "release" for operation, _, _ in wire.requests)
    arrive(r, 2, (2, -3, 0))
    assert wire.requests[-1][0] == "release"
    assert wire.requests[-1][1].lease_id == "lease-1"
    wire.reply(released=True)
    assert len(results) == 1 and results[0].success and agent.mode == "AVAILABLE"
    wire.moves[1][2](True, "late duplicate")
    assert len(results) == 1 and agent.mode == "AVAILABLE"


@pytest.mark.parametrize("when", ["staging", "waiting", "acquiring", "docked"])
def test_cancel_resolves_waiter_and_releases_only_after_safe_exit(when):
    r = rig()
    agent, _, wire, results, _, controller = r
    if when == "docked":
        enter(r)
        controller.contact(True)
    elif when == "staging":
        start(r)
    else:
        stage(r)
        if when == "waiting":
            wire.reply(granted=False, lease_id="", lease_ttl_sec=0.0)
    controller.cancel()
    if when == "docked":
        assert agent.mode == "DOCKING" and not results
        assert wire.moves[-1][0] == (2, -3, 0)
        arrive(r, 2, (2, -3, 0))
        wire.reply(released=True)
    elif when == "staging":
        # A cancellation acknowledgement must prove that navigation stopped.
        assert not results
        wire.moves[0][2](False, "cancelled")
    else:
        assert wire.requests[-1][0] == "cancel_wait"
        if when == "acquiring":
            acquire = wire.requests[0][2]
            wire.reply(lease_id="", lease_ttl_sec=0.0, reconciliation_required=False)
            assert not results  # An outstanding grant can still arrive.
            acquire(
                SimpleNamespace(granted=True, lease_id="late-grant", lease_ttl_sec=10.0)
            )
            assert wire.requests[-1][0] == "release"
            assert wire.requests[-1][1].lease_id == "late-grant"
            wire.reply(released=True)
        else:
            wire.reply(lease_id="", lease_ttl_sec=0.0, reconciliation_required=False)
    assert len(results) == 1 and results[0].error_code == "CANCELLED"
    assert agent.mode == "AVAILABLE"


@pytest.mark.parametrize("phase", ["stage", "enter", "exit"])
def test_navigation_failure_retains_uncertain_dock_occupancy(phase):
    r = rig()
    agent, _, wire, results, _, controller = r
    if phase == "stage":
        start(r)
        wire.moves[0][2](False, "blocked")
    elif phase == "enter":
        stage(r)
        wire.reply(granted=True, lease_id="lease-1", lease_ttl_sec=10.0)
        wire.moves[1][2](False, "blocked")
    else:
        enter(r)
        controller.cancel()
        wire.moves[2][2](False, "blocked")
    assert len(results) == 1 and results[0].error_code == "NAVIGATION_FAILED"
    assert agent.mode == ("AVAILABLE" if phase == "stage" else "RECOVERY_REQUIRED")
    assert not any(operation == "release" for operation, _, _ in wire.requests)


@pytest.mark.parametrize("loss", ["lease", "contact", "pose", "health"])
def test_loss_stops_charge_and_preserves_recovery_reservation(loss):
    r = rig()
    enter(r)
    agent, wall, wire, results, _, controller = r
    controller.contact(True)
    before = agent.energy.battery_percent
    if loss == "lease":
        wall[0] += 0.5
        controller.tick()
        # Explicit renewal denial fences the still unexpired local lease.
        controller.renew()
        wire.reply(renewed=False, lease_ttl_sec=0.0)
    elif loss == "contact":
        controller.contact(False)
    elif loss == "pose":
        agent.localization(agent._pose_stamp + 1, 0, -3, 0, agent.frame_prefix + "map")
        controller.tick()
    else:
        wall[0] += 3.1
        controller.tick()
    stopped = agent.energy.battery_percent
    wall[0] += 1
    controller.tick()
    assert agent.energy.battery_percent <= stopped
    assert len(results) == 1 and not results[0].success
    assert agent.mode == "RECOVERY_REQUIRED"
    assert not any(operation == "release" for operation, _, _ in wire.requests)
    assert stopped <= before + 1.0  # Never charge beyond fresh contact evidence.


def test_restart_at_charging_pose_never_resumes_or_acquires():
    r = rig()
    agent, _, wire, results, _, controller = r
    agent.localization(2, 3, -3, 0, agent.frame_prefix + "map")
    controller.contact(True)
    start_allowed = controller.start(80, lambda _: None, results.append)
    assert not start_allowed and agent.mode == "RECOVERY_REQUIRED"
    assert wire.moves == wire.requests == []
    assert agent._lease_until == 0


def test_restart_inside_dock_marks_recovery_without_automatic_goal():
    r = rig()
    agent, _, wire, _, _, controller = r
    agent.localization(2, 3, -3, 0, agent.frame_prefix + "map")
    controller.tick()
    assert agent.heartbeat().mode == "RECOVERY_REQUIRED"
    assert not wire.moves and not wire.requests


@pytest.mark.parametrize("change", ["mode", "payload", "mission", "health"])
def test_busy_loaded_or_unhealthy_agent_rejects_docking(change):
    r = rig()
    agent, wall, wire, results, _, controller = r
    if change == "mode":
        agent.mode = "EXECUTING"
    elif change == "payload":
        agent.payload_state = "LOADED"
    elif change == "mission":
        agent.mission_id = "transport"
    else:
        wall[0] += 3.0
    assert not controller.start(80, lambda _: None, results.append)
    assert not wire.moves and not wire.requests


def test_invalid_expired_grant_and_duplicate_reply_never_authorize_entry():
    r = rig()
    stage(r)
    _, wall, wire, _, _, controller = r
    callback = wire.requests[-1][2]
    wall[0] += 11
    callback(SimpleNamespace(granted=True, lease_id="old", lease_ttl_sec=10.0))
    callback(SimpleNamespace(granted=True, lease_id="old", lease_ttl_sec=10.0))
    assert len(wire.moves) == 1 and controller.agent.mode == "RECOVERY_REQUIRED"


def test_wrong_lease_identity_cannot_charge_even_with_contact_and_pose():
    from dataclasses import replace

    r = rig()
    enter(r)
    agent, wall, _, _, _, controller = r
    controller.contact(True)
    agent.set_charge_authorization(
        lease_until=110,
        contact=True,
        lease_key=replace(controller.key, lease_id="different-lease"),
        contact_until=110,
    )
    before = agent.energy.battery_percent
    wall[0] += 0.5
    agent.tick()
    assert agent.energy.battery_percent < before


def test_unconfigured_adapter_cannot_charge_from_bare_deadline_and_contact():
    agent, _, wall = agent_rig()
    ready(agent)
    agent.mode = "CHARGING"
    agent.set_charge_authorization(lease_until=110, contact=True)
    before = agent.energy.battery_percent
    wall[0] += 0.5
    agent.tick()
    assert agent.energy.battery_percent < before


def test_contact_receipt_expiry_limits_energy_even_before_next_controller_tick():
    r = rig()
    enter(r)
    agent, wall, _, results, _, controller = r
    controller.contact(True)
    wall[0] += 2
    assert agent.heartbeat().battery_percent == pytest.approx(20.998)
    controller.tick()
    assert results[0].error_code == "CONTACT_LOST"


def test_cancel_during_entry_waits_for_navigation_stop_before_exit():
    r = rig()
    stage(r)
    agent, _, wire, results, _, controller = r
    wire.reply(granted=True, lease_id="held", lease_ttl_sec=10.0)
    controller.cancel()
    assert wire.cancelled == [1] and len(wire.moves) == 2 and not results
    wire.moves[1][2](False, "cancelled")
    assert len(wire.moves) == 3 and wire.moves[-1][0] == (2, -3, 0)
    arrive(r, 2, (2, -3, 0))
    wire.reply(released=True)
    assert results[0].error_code == "CANCELLED" and agent.mode == "AVAILABLE"


def test_missing_acquire_reply_during_cancel_keeps_recovery_barrier():
    r = rig()
    stage(r)
    agent, wall, wire, results, _, controller = r
    controller.cancel()
    wire.reply(lease_id="", lease_ttl_sec=0.0, reconciliation_required=False)
    assert not results
    wall[0] += 5.1
    agent.odometry(3, 0, 0, agent.frame_prefix + "odom")
    controller.tick()
    assert results[0].error_code == "CLEANUP_FAILED"
    assert agent.mode == "RECOVERY_REQUIRED"


def test_release_failure_never_makes_robot_available():
    r = rig()
    enter(r)
    agent, _, wire, results, _, controller = r
    controller.cancel()
    arrive(r, 2, (2, -3, 0))
    wire.reply(released=False)
    assert results[0].error_code == "RELEASE_FAILED"
    assert agent.mode == "RECOVERY_REQUIRED"


def test_multiple_late_acquires_of_same_identity_release_only_once():
    r = rig()
    stage(r)
    agent, wall, wire, results, _, controller = r
    first = wire.requests[-1][2]
    wall[0] += 1.1
    controller.tick()
    second = wire.requests[-1][2]
    controller.cancel()
    wire.reply(lease_id="", lease_ttl_sec=0.0, reconciliation_required=False)
    first(SimpleNamespace(granted=True, lease_id="same", lease_ttl_sec=10.0))
    wire.reply(released=True)
    assert not results
    second(SimpleNamespace(granted=True, lease_id="same", lease_ttl_sec=10.0))
    assert len([call for call in wire.requests if call[0] == "release"]) == 1
    assert results[0].error_code == "CANCELLED" and agent.mode == "AVAILABLE"


def test_unanswered_acquires_are_bounded_and_fail_closed():
    r = rig()
    stage(r)
    agent, wall, wire, results, _, controller = r
    for stamp in range(3, 25):
        wall[0] += 1.1
        agent.odometry(stamp, 0, 0, agent.frame_prefix + "odom")
        controller.tick()
    assert len(wire.requests) <= 8
    assert results[0].error_code == "ACQUIRE_UNRESOLVED"
    assert agent.mode == "RECOVERY_REQUIRED"


def test_grant_received_after_health_loss_never_dispatches_entry():
    r = rig()
    stage(r)
    agent, wall, wire, results, _, _ = r
    wall[0] += 3.1
    wire.reply(granted=True, lease_id="fresh", lease_ttl_sec=10.0)
    assert len(wire.moves) == 1
    assert results[0].error_code == "ROBOT_UNAVAILABLE"
    assert agent.mode == "RECOVERY_REQUIRED"


def test_late_renewal_cannot_resurrect_expired_local_authority():
    r = rig()
    enter(r)
    agent, wall, wire, results, _, controller = r
    wall[0] += 5
    agent.odometry(4, 0, 0, agent.frame_prefix + "odom")
    controller.renew()
    wall[0] += 5.1
    agent.odometry(5, 0, 0, agent.frame_prefix + "odom")
    wire.reply(renewed=True, lease_ttl_sec=10.0)
    assert not controller.active and results[0].error_code == "LEASE_LOST"
    assert agent.mode == "RECOVERY_REQUIRED"
