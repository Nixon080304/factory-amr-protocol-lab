"""Restart decisions require current physical evidence, not stale authority."""

from dataclasses import replace
import importlib
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_manager.adapter import FleetAdapter
from fleet_manager.config import Pose2D, load_fleet_config
from fleet_manager.journal import MissionJournal, MissionState
from fleet_manager.models import MissionRequest, RobotSnapshot
from fleet_manager.resources import Lease, LeaseKey, LeaseRequest, ResourceManager
from fake_robot_agent import FakeRobots

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def config():
    return load_fleet_config(ROOT / "src/factory_bringup/config/fleet.yaml")


def planner(config):
    try:
        module = importlib.import_module("fleet_manager.recovery")
    except ModuleNotFoundError:
        pytest.fail("restart recovery policy is missing")
    return module.RecoveryPlanner(config)


def seed(journal, state="EXECUTING", ownership="NOT_PICKED_UP", mission_id="m1"):
    record = journal.register(
        MissionRequest(mission_id, "assembly", "inspection", "motor"), "hash", 1
    ).record
    if state != "QUEUED":
        record = journal.transition(
            mission_id,
            record.state,
            MissionState(state),
            {"assigned_robot_id": "amr_01", "payload_ownership": ownership},
            2,
        )
    return record


def safe(robot_id="amr_01", **changes):
    return replace(
        RobotSnapshot(robot_id, "AVAILABLE", Pose2D(-3, -3, 0), 80, "EMPTY"),
        **changes,
    )


@pytest.mark.parametrize(
    "state,ownership,mode,payload,mission,want",
    [
        ("QUEUED", "NOT_PICKED_UP", "AVAILABLE", "EMPTY", None, "QUEUED"),
        ("ASSIGNING", "NOT_PICKED_UP", "AVAILABLE", "EMPTY", None, "QUEUED"),
        ("ASSIGNED", "NOT_PICKED_UP", "AVAILABLE", "EMPTY", None, "QUEUED"),
        ("EXECUTING", "NOT_PICKED_UP", "AVAILABLE", "EMPTY", None, "QUEUED"),
        ("EXECUTING", "NOT_PICKED_UP", "EXECUTING", "EMPTY", "m1", "RECOVERY_REQUIRED"),
        ("EXECUTING", "PICKED_UP", "AVAILABLE", "EMPTY", None, "RECOVERY_REQUIRED"),
        ("EXECUTING", "UNKNOWN", "AVAILABLE", "EMPTY", None, "RECOVERY_REQUIRED"),
        (
            "EXECUTING",
            "NOT_PICKED_UP",
            "AVAILABLE",
            "LOADED",
            None,
            "RECOVERY_REQUIRED",
        ),
        (
            "EXECUTING",
            "NOT_PICKED_UP",
            "AVAILABLE",
            "UNKNOWN",
            None,
            "RECOVERY_REQUIRED",
        ),
        ("COMPLETED", "DELIVERED", "AVAILABLE", "EMPTY", None, "COMPLETED"),
        ("FAILED", "UNKNOWN", "AVAILABLE", "EMPTY", None, "RECOVERY_REQUIRED"),
        ("CANCELLED", "NOT_PICKED_UP", "AVAILABLE", "EMPTY", None, "CANCELLED"),
    ],
)
def test_persisted_state_failure_matrix(
    config, tmp_path, state, ownership, mode, payload, mission, want
):
    with_journal = MissionJournal(tmp_path / "matrix.sqlite3")
    try:
        record = seed(with_journal, state, ownership)
        robot = safe(mode=mode, payload_state=payload, mission_id=mission)
        decisions = planner(config).plan([record], {"amr_01": robot}, ())
        assert len(decisions) == 1
        assert decisions[0].state == want
        assert decisions[0].robot_id == (None if want == "QUEUED" else "amr_01")
    finally:
        with_journal.close()


def test_missing_robot_never_proves_empty_payload(config, tmp_path):
    journal = MissionJournal(tmp_path / "offline.sqlite3")
    try:
        decisions = planner(config).plan([seed(journal)], {}, ())
        assert decisions[0].state == "RECOVERY_REQUIRED"
        assert decisions[0].robot_id == "amr_01"
    finally:
        journal.close()


@pytest.mark.parametrize("x,want", [(0, False), (-1.1, False), (-1.251, True)])
def test_clearance_uses_configured_bounds_and_footprint(config, x, want):
    # Dock bounds supply the configured fleet footprint and arrival error.
    lease = Lease("amr_01", "m1", "central_aisle", "old-token", 10)
    policy = planner(config)
    assert policy.proves_outside(lease, safe(pose=Pose2D(x, -2.2, 0))) is want


@pytest.mark.parametrize(
    "robot",
    [safe(pose=None), safe(pose=Pose2D(float("nan"), 0, 0)), safe(health="OFFLINE")],
)
def test_malformed_or_offline_pose_never_clears(config, robot):
    lease = Lease("amr_01", "m1", "central_aisle", "old-token", 10)
    assert not planner(config).proves_outside(lease, robot)


def test_station_without_configured_geometry_stays_quarantined(config):
    from types import MappingProxyType

    config = replace(config, resource_bounds=MappingProxyType({}))
    lease = Lease("amr_01", "m1", "assembly", "old-token", 10)
    assert not planner(config).proves_outside(lease, safe())


def test_station_clearance_requires_valid_configured_region(tmp_path):
    data = yaml.safe_load((ROOT / "src/factory_bringup/config/fleet.yaml").read_text())
    data["resource_bounds"] = {"assembly": [-3.5, 0.5, -2.5, 1.5]}
    # This focused geometry fixture deliberately omits inspection bounds. Its
    # waiting/parking layout must not reference that absent physical region.
    data.pop("station_staging")
    data.pop("station_approach")
    data.pop("station_exit_poses")
    # This fixture tests an isolated station region, not a terminal-to-dock path.
    data["docks"]["dock_01"].pop("departure_stations")
    path = tmp_path / "fleet.yaml"
    path.write_text(yaml.safe_dump(data))
    config = load_fleet_config(path)
    lease = Lease("amr_01", "m1", "assembly", "old-token", 10)
    assert planner(config).proves_outside(lease, safe())
    assert not planner(config).proves_outside(lease, safe(pose=Pose2D(-3, 1, 0)))


@pytest.mark.parametrize("frame", ["map", "amr_02/map", "", "amr_01/odom"])
def test_wrong_frame_ros_observation_never_proves_restart_clearance(
    config, tmp_path, frame
):
    from factory_interfaces.msg import RobotState
    from fleet_manager.node import FleetManagerNode

    journal = MissionJournal(tmp_path / "frame.sqlite3")
    try:
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        node = object.__new__(FleetManagerNode)
        node.adapter = adapter
        node.get_logger = lambda: SimpleNamespace(warning=lambda _: None)
        message = RobotState(
            robot_id="amr_01",
            mode="AVAILABLE",
            frame_id=frame,
            battery_percent=80.0,
            payload_state="EMPTY",
        )
        message.pose.orientation.w = 1.0
        node._observe("amr_01", message)
        assert adapter.registry.get("amr_01", 100).pose is None
    finally:
        journal.close()


def test_resource_history_restores_only_exact_quarantine(config, tmp_path):
    journal = MissionJournal(tmp_path / "resources.sqlite3")
    resources = ResourceManager(config.resources)
    old = resources.acquire(LeaseRequest("amr_01", "m1", "central_aisle"), 5).lease
    try:
        assert hasattr(journal, "save_resources"), "durable lease evidence is missing"
        journal.save_resources(resources.snapshot(5), 5)
        journal.close()
        journal = MissionJournal(tmp_path / "resources.sqlite3")
        new = ResourceManager(config.resources)
        new.restore_evidence(journal.load_resources())
        snapshot = next(s for s in new.snapshot(6) if s.resource_id == "central_aisle")
        assert snapshot.lease is None and snapshot.former_lease == old
        assert not new.renew(
            LeaseKey("amr_01", "m1", "central_aisle", old.lease_id), 6
        ).granted
        assert not new.clear_reconciliation(
            LeaseKey("amr_01", "m1", "central_aisle", "wrong"), 6
        )
        assert new.clear_reconciliation(
            LeaseKey("amr_01", "m1", "central_aisle", old.lease_id), 6
        )
        grant = new.acquire(LeaseRequest("amr_02", "m2", "central_aisle"), 6)
        assert grant.granted and grant.lease.lease_id != old.lease_id
    finally:
        journal.close()


def test_startup_blocks_work_and_grants_until_every_robot_accounted(config, tmp_path):
    journal = MissionJournal(tmp_path / "startup.sqlite3")
    now = [100.0]
    wire = FakeRobots(journal)
    adapter = FleetAdapter(config, journal, wire, clock=lambda: now[0])
    try:
        adapter.submit(MissionRequest("new", "assembly", "inspection", "motor"))
        adapter.observe(safe())
        adapter.tick()
        assert wire.cost_calls == [] and wire.goals == []
        assert adapter.state == "RECONCILING"
        response = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_01", mission_id="new", resource_id="assembly"
            ),
        )
        assert not response["granted"] and response["reason"] == "fleet reconciling"
        now[0] = 105.0
        adapter.tick()
        assert adapter.state == "RUNNING"
        assert adapter.registry.get("amr_02", now[0]).health == "OFFLINE"
    finally:
        journal.close()


def test_prestart_dds_sample_cannot_satisfy_barrier(config, tmp_path):
    journal = MissionJournal(tmp_path / "old.sqlite3")
    try:
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        assert hasattr(adapter, "startup_wall_ns"), "DDS freshness boundary is missing"
        adapter.observe(safe(), source_time_ns=adapter.startup_wall_ns - 1)
        adapter.observe(safe("amr_02"), source_time_ns=adapter.startup_wall_ns + 1)
        adapter.tick()
        assert adapter.state == "RECONCILING"
        assert adapter.registry.get("amr_01", 100).health == "OFFLINE"
    finally:
        journal.close()


def test_recovery_clearance_and_mission_requeue_commit_before_dispatch(
    config, tmp_path
):
    path = tmp_path / "restart.sqlite3"
    journal = MissionJournal(path)
    seed(journal)
    resources = ResourceManager(config.resources)
    old = resources.acquire(LeaseRequest("amr_01", "m1", "central_aisle"), 5).lease
    try:
        assert hasattr(journal, "save_resources"), "durable lease evidence is missing"
        journal.save_resources(resources.snapshot(5), 5)
        journal.close()
        journal = MissionJournal(path)
        wire = FakeRobots(journal)
        adapter = FleetAdapter(config, journal, wire, clock=lambda: 100)
        assert journal.get("m1").state == "EXECUTING"
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        adapter.tick()
        assert journal.get("m1").state == "QUEUED"
        assert journal.get("m1").assigned_robot_id is None
        assert not any(s.reconciliation_required for s in journal.load_resources())
        assert wire.goals == []
        before = journal.events("m1")
        journal.close()
        journal = MissionJournal(path)
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 200)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        adapter.tick()
        assert journal.events("m1") == before
        assert all(s.lease is None for s in adapter.resources.snapshot(200))
        assert old.lease_id
    finally:
        journal.close()


def test_sqlite_failure_fences_following_grants(config, tmp_path):
    journal = MissionJournal(tmp_path / "failure.sqlite3")
    adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
    try:
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        adapter.tick()
        journal._connection.execute("PRAGMA query_only = ON")
        request = SimpleNamespace(
            robot_id="amr_01", mission_id="m1", resource_id="assembly"
        )
        response = adapter.resource("acquire", request)
        assert not response["granted"] and adapter.state == "STORAGE_FAILED"
        journal._connection.execute("PRAGMA query_only = OFF")
        assert not adapter.resource("acquire", request)["granted"]
    finally:
        journal.close()


@pytest.mark.parametrize("offline", [False, True])
def test_missing_history_does_not_make_observed_or_unknown_occupancy_safe(
    config, tmp_path, offline
):
    journal = MissionJournal(tmp_path / "occupancy.sqlite3")
    now = [100.0]
    adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: now[0])
    try:
        if not offline:
            adapter.observe(safe(pose=Pose2D(0, -2.2, 0)))
        else:
            now[0] = 105.0
        adapter.observe(safe("amr_02"))
        adapter.tick()
        request = SimpleNamespace(
            robot_id="amr_02", mission_id="m2", resource_id="central_aisle"
        )
        assert not adapter.resource("acquire", request)["granted"]
        resource = next(
            value
            for value in journal.load_resources()
            if value.resource_id == "central_aisle"
        )
        assert resource.reconciliation_required and resource.lease is None
    finally:
        journal.close()


def test_second_production_manager_cannot_write_same_journal(tmp_path):
    import rclpy
    from fleet_manager.node import FleetManagerNode
    from rclpy.context import Context
    from rclpy.parameter import Parameter

    context = Context()
    rclpy.init(context=context, domain_id=92)
    nodes = []
    overrides = [
        Parameter(
            "fleet_file", value=str(ROOT / "src/factory_bringup/config/fleet.yaml")
        ),
        Parameter("journal_path", value=str(tmp_path / "single-writer.sqlite3")),
    ]
    try:
        nodes.append(FleetManagerNode(context=context, parameter_overrides=overrides))
        with pytest.raises(
            RuntimeError, match="journal already has a fleet manager writer"
        ):
            nodes.append(
                FleetManagerNode(context=context, parameter_overrides=overrides)
            )
    finally:
        for node in nodes:
            node.destroy_node()
        context.shutdown()


def test_unique_live_payload_claim_preserves_robot_identity(config, tmp_path):
    journal = MissionJournal(tmp_path / "claim.sqlite3")
    try:
        record = seed(journal, "QUEUED")
        robot = safe(payload_state="LOADED", mission_id="m1", mode="EXECUTING")
        decision = planner(config).plan([record], {"amr_01": robot}, ())[0]
        assert decision.state == "RECOVERY_REQUIRED"
        assert (
            decision.robot_id == "amr_01" and decision.payload_ownership == "PICKED_UP"
        )
    finally:
        journal.close()


def test_charging_reconstruction_requires_contact_pose_and_new_authority(config):
    policy = planner(config)
    assert hasattr(policy, "dock_state"), "dock reconstruction policy is missing"
    resources = ResourceManager(config.resources)
    resources.acquire(LeaseRequest("amr_01", "dock-cycle", "dock_01"), 100)
    robot = safe(mode="CHARGING", pose=Pose2D(4, -2, 0))
    snapshot = next(s for s in resources.snapshot(100) if s.resource_id == "dock_01")
    assert policy.dock_state(robot, snapshot, contact=True) == "CHARGING"
    assert policy.dock_state(robot, snapshot, contact=False) == "RECOVERY_REQUIRED"
    assert (
        policy.dock_state(safe(mode="CHARGING"), snapshot, contact=True)
        == "RECOVERY_REQUIRED"
    )
    stale = ResourceManager(config.resources)
    stale.restore_evidence([snapshot])
    stale_snapshot = next(s for s in stale.snapshot(100) if s.resource_id == "dock_01")
    assert policy.dock_state(robot, stale_snapshot, contact=True) == "RECOVERY_REQUIRED"


def test_fresh_contact_is_evidence_and_never_restores_former_dock_token(
    config, tmp_path
):
    journal = MissionJournal(tmp_path / "contact.sqlite3")
    try:
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        assert hasattr(adapter, "observe_contact"), (
            "fresh dock contact evidence is missing"
        )
        adapter.observe_contact(
            "amr_01", True, source_time_ns=adapter.startup_wall_ns - 1
        )
        assert (
            journal._connection.execute(
                "SELECT COUNT(*) FROM contact_observations"
            ).fetchone()[0]
            == 0
        )
        adapter.observe_contact(
            "amr_01", True, source_time_ns=adapter.startup_wall_ns + 1
        )
        adapter.observe(safe(mode="CHARGING", pose=Pose2D(4, -2, 0)))
        adapter.observe(safe("amr_02"))
        assert adapter.registry.get("amr_01", 100).mode == "RECOVERY_REQUIRED"
        dock = next(
            s for s in adapter.resources.snapshot(100) if s.resource_id == "dock_01"
        )
        assert dock.lease is None and dock.reconciliation_required
        assert (
            journal._connection.execute(
                "SELECT COUNT(*) FROM contact_observations"
            ).fetchone()[0]
            == 1
        )
    finally:
        journal.close()


def test_recovery_write_failure_rolls_back_clearance_and_mission_together(
    config, tmp_path
):
    journal = MissionJournal(tmp_path / "atomic.sqlite3")
    now = [100.0]
    try:
        seed(journal)
        resources = ResourceManager(config.resources)
        old = resources.acquire(LeaseRequest("amr_01", "m1", "central_aisle"), 1).lease
        journal.save_resources(resources.snapshot(1), 1)
        journal._connection.execute(
            "CREATE TRIGGER reject_recovery BEFORE INSERT ON mission_events "
            "WHEN NEW.state='QUEUED' BEGIN SELECT RAISE(ABORT, 'recovery write failed'); END"
        )
        adapter = FleetAdapter(
            config, journal, FakeRobots(journal), clock=lambda: now[0]
        )
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        assert adapter.state == "STORAGE_FAILED"
        assert journal.get("m1").state == "EXECUTING"
        persisted = next(
            s for s in journal.load_resources() if s.resource_id == "central_aisle"
        )
        assert persisted.lease == old
        assert not adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="m2", resource_id="central_aisle"
            ),
        )["granted"]
    finally:
        journal.close()


@pytest.mark.parametrize(
    "bounds",
    [[0, 0, 0, 1], [1, 0, 0, 1], [0, 0, float("nan"), 1], [0, 0, 1], [0, False, 1, 1]],
)
def test_invalid_station_geometry_never_licenses_clearance(tmp_path, bounds):
    data = yaml.safe_load((ROOT / "src/factory_bringup/config/fleet.yaml").read_text())
    data["resource_bounds"] = {"assembly": bounds}
    path = tmp_path / "fleet.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="resource_bounds.assembly"):
        load_fleet_config(path)


def test_recovery_decisions_are_deterministic_for_reordered_records(config, tmp_path):
    journal = MissionJournal(tmp_path / "order.sqlite3")
    try:
        first = seed(journal, "QUEUED", mission_id="a")
        second = seed(journal, "EXECUTING", "PICKED_UP", mission_id="b")
        policy = planner(config)
        decisions = policy.plan([second, first], {"amr_01": safe()}, ())
        assert [decision.mission_id for decision in decisions] == ["a", "b"]
        assert [decision.state for decision in decisions] == [
            "QUEUED",
            "RECOVERY_REQUIRED",
        ]
        assert decisions == policy.plan([first, second], {"amr_01": safe()}, ())
    finally:
        journal.close()


def test_removed_former_owner_cannot_be_cleared_by_other_robots(config, tmp_path):
    journal = MissionJournal(tmp_path / "removed.sqlite3")
    try:
        resources = ResourceManager(config.resources)
        resources.acquire(LeaseRequest("amr_01", "old", "central_aisle"), 1)
        journal.save_resources(resources.snapshot(1), 1)
        changed = replace(config, robots=(config.robots[1],))
        adapter = FleetAdapter(changed, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe("amr_02"))
        response = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="new", resource_id="central_aisle"
            ),
        )
        assert not response["granted"]
        former = next(
            s.former_lease
            for s in adapter.resources.snapshot(100)
            if s.resource_id == "central_aisle"
        )
        assert former.robot_id == "amr_01"
    finally:
        journal.close()


@pytest.mark.parametrize(
    "contact,want",
    [(None, "RECOVERY_REQUIRED"), (False, "RECOVERY_REQUIRED"), (True, "CHARGING")],
)
def test_new_charging_cycle_reconstructs_only_from_current_contact_and_new_lease(
    config, tmp_path, contact, want
):
    journal = MissionJournal(tmp_path / "new-dock.sqlite3")
    try:
        wire = FakeRobots(journal)
        wire.send_dock_goal = lambda robot_id, charge, feedback, result, accepted: (
            accepted(object(), "")
        )
        adapter = FleetAdapter(config, journal, wire, clock=lambda: 100)
        adapter.observe(safe(battery_percent=20))
        adapter.observe(safe("amr_02"))
        adapter.tick()
        adapter.tick()
        grant = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_01", mission_id="dock-cycle", resource_id="dock_01"
            ),
        )
        assert grant["granted"]
        if contact is not None:
            adapter.observe_contact(
                "amr_01", contact, source_time_ns=adapter.startup_wall_ns + 1
            )
        adapter.observe(
            safe(mode="CHARGING", pose=Pose2D(4, -2, 0), battery_percent=20)
        )
        assert adapter.registry.get("amr_01", 100).mode == want
        assert adapter.core.charging_snapshot()[0].robot_id == "amr_01"
    finally:
        journal.close()


def test_failed_mission_write_fences_future_grants_even_if_storage_recovers(
    config, tmp_path
):
    import sqlite3

    journal = MissionJournal(tmp_path / "submit-failure.sqlite3")
    try:
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        journal._connection.execute("PRAGMA query_only = ON")
        with pytest.raises(sqlite3.OperationalError):
            adapter.submit(MissionRequest("new", "assembly", "inspection", "motor"))
        journal._connection.execute("PRAGMA query_only = OFF")
        response = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_01", mission_id="new", resource_id="assembly"
            ),
        )
        assert not response["granted"] and adapter.state == "STORAGE_FAILED"
    finally:
        journal.close()


@pytest.mark.parametrize("boundary", ["cancel", "pickup_observation"])
def test_failed_runtime_mission_write_latches_storage_and_blocks_new_work(
    config, tmp_path, boundary
):
    import sqlite3

    journal = MissionJournal(tmp_path / f"{boundary}-failure.sqlite3")
    try:
        wire = FakeRobots(journal)
        adapter = FleetAdapter(config, journal, wire, clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        adapter.submit(
            MissionRequest("running", "assembly", "inspection", "motor", "amr_01")
        )
        for _ in range(3):
            adapter.tick()
        goal = wire.goals[0]
        goal.feedback(
            SimpleNamespace(state="NAVIGATING_TO_PICKUP", detail="", progress=0.1)
        )
        adapter.tick()
        before = journal.get("running")
        assert (
            before.state == "EXECUTING" and before.payload_ownership == "NOT_PICKED_UP"
        )
        events = journal.events("running")
        adapter.submit(
            MissionRequest("waiting", "assembly", "inspection", "motor", "amr_02")
        )
        counts = len(wire.cost_calls), len(wire.goals)
        if boundary == "cancel":
            journal._connection.execute("PRAGMA query_only = ON")
            with pytest.raises(sqlite3.OperationalError):
                adapter.cancel("running")
            journal._connection.execute("PRAGMA query_only = OFF")
        else:
            # The observation INSERT succeeds. Only its payload mission write
            # fails, proving that the outer mutation boundary also stays fenced.
            journal._connection.execute(
                "CREATE TRIGGER fail_pickup BEFORE UPDATE ON missions "
                "WHEN NEW.payload_ownership = 'PICKED_UP' "
                "BEGIN SELECT RAISE(ABORT, 'pickup write failed'); END"
            )
            with pytest.raises(sqlite3.IntegrityError, match="pickup write failed"):
                adapter.observe(
                    safe(mode="EXECUTING", mission_id="running", payload_state="LOADED")
                )
            journal._connection.execute("DROP TRIGGER fail_pickup")
            evidence = journal._connection.execute(
                "SELECT evidence_json FROM robot_observations ORDER BY sequence DESC LIMIT 1"
            ).fetchone()[0]
            assert '"payload_state":"LOADED"' in evidence
        assert journal.get("running") == before
        assert journal.events("running") == events
        response = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="waiting", resource_id="assembly"
            ),
        )
        assert not response["granted"] and adapter.state == "STORAGE_FAILED"
        # Late accepted action feedback must not reopen scheduling after storage
        # becomes writable. Preserve the queued mission without dispatching it.
        goal.feedback(
            SimpleNamespace(state="NAVIGATING_TO_DROPOFF", detail="", progress=0.5)
        )
        for _ in range(3):
            adapter.tick()
        assert (len(wire.cost_calls), len(wire.goals)) == counts
        assert journal.get("waiting").state == "QUEUED"
        assert journal.get("running") == before
        assert goal.cancel_ack is None
    finally:
        journal.close()


@pytest.mark.parametrize(
    "payload,want", [("LOADED", "PICKED_UP"), ("UNKNOWN", "UNKNOWN")]
)
def test_restart_preserves_durable_correlated_payload_after_failed_mission_write(
    config, tmp_path, payload, want
):
    import sqlite3

    path = tmp_path / "durable-payload.sqlite3"
    journal = MissionJournal(path)
    try:
        wire = FakeRobots(journal)
        adapter = FleetAdapter(config, journal, wire, clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        adapter.submit(
            MissionRequest("carried", "assembly", "inspection", "motor", "amr_01")
        )
        for _ in range(3):
            adapter.tick()
        journal._connection.execute(
            "CREATE TRIGGER fail_payload BEFORE UPDATE ON missions "
            "BEGIN SELECT RAISE(ABORT, 'payload write failed'); END"
        )
        with pytest.raises(sqlite3.IntegrityError, match="payload write failed"):
            adapter.observe(
                safe(mode="EXECUTING", mission_id="carried", payload_state=payload)
            )
        journal._connection.execute("DROP TRIGGER fail_payload")
        assert journal.get("carried").payload_ownership == "NOT_PICKED_UP"
        journal.close()
        journal = MissionJournal(path)
        restarted_wire = FakeRobots(journal)
        adapter = FleetAdapter(config, journal, restarted_wire, clock=lambda: 200)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        for _ in range(3):
            adapter.tick()
        record = journal.get("carried")
        assert record.state == "RECOVERY_REQUIRED"
        assert record.assigned_robot_id == "amr_01" and record.payload_ownership == want
        assert restarted_wire.goals == []
        events = journal.events("carried")
        journal.close()
        journal = MissionJournal(path)
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 300)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        adapter.tick()
        assert journal.get("carried") == record and journal.events("carried") == events
    finally:
        journal.close()


@pytest.mark.parametrize("legacy_geometry", [True, False])
def test_unresolved_station_work_without_lease_history_never_grants_overlap(
    config, tmp_path, legacy_geometry
):
    from types import MappingProxyType

    if legacy_geometry:
        config = replace(config, resource_bounds=MappingProxyType({}))
    journal = MissionJournal(tmp_path / "old-station.sqlite3")
    try:
        seed(journal, ownership="UNKNOWN")
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(
            safe(
                mode="WAITING_FOR_RESOURCE",
                payload_state="UNKNOWN",
                mission_id="m1",
                pose=Pose2D(-3, 1, 0),
            )
        )
        adapter.observe(safe("amr_02"))
        response = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="other", resource_id="assembly"
            ),
        )
        assert journal.get("m1").state == "RECOVERY_REQUIRED"
        assert not response["granted"]
        snapshot = next(
            value
            for value in adapter.resources.snapshot(100)
            if value.resource_id == "assembly"
        )
        assert snapshot.lease is None and snapshot.reconciliation_required
        assert (
            snapshot.former_lease.robot_id == "amr_01"
            and snapshot.former_lease.mission_id == "m1"
        )
        # A pose from another robot cannot clear the exact former owner. Missing
        # station geometry remains insufficient even once the former owner leaves.
        adapter.observe(safe())
        adapter.tick()
        if legacy_geometry:
            assert next(
                value
                for value in adapter.resources.snapshot(100)
                if value.resource_id == "assembly"
            ).reconciliation_required
        else:
            assert not next(
                value
                for value in adapter.resources.snapshot(100)
                if value.resource_id == "assembly"
            ).reconciliation_required
    finally:
        journal.close()


def test_unresolved_route_evidence_does_not_quarantine_unrelated_resource(
    config, tmp_path
):
    from types import MappingProxyType
    from fleet_manager.config import ResourceConfig

    config = replace(
        config,
        resources=(*config.resources, ResourceConfig("other_station", "station", 1)),
        resource_bounds=MappingProxyType({}),
    )
    journal = MissionJournal(tmp_path / "scoped-station.sqlite3")
    try:
        seed(journal, ownership="UNKNOWN")
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(
            safe(
                mode="WAITING_FOR_RESOURCE",
                payload_state="UNKNOWN",
                mission_id="m1",
                pose=Pose2D(-3, 1, 0),
            )
        )
        adapter.observe(safe("amr_02"))
        related = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="other", resource_id="assembly"
            ),
        )
        assert not related["granted"]
        unrelated = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="other", resource_id="other_station"
            ),
        )
        assert unrelated["granted"]
    finally:
        journal.close()


@pytest.mark.parametrize("late_acceptance", [False, True])
def test_storage_failure_bounds_late_callbacks_and_cleans_action_handles(
    config, tmp_path, late_acceptance
):
    import sqlite3
    from fleet_manager.adapter import RobotReply

    journal = MissionJournal(tmp_path / "failed-callbacks.sqlite3")
    try:
        wire = FakeRobots(journal)
        cancelled_handles = []
        original_cancel = wire.cancel

        def cancel(handle, callback):
            cancelled_handles.append(handle)
            original_cancel(handle, callback)

        wire.cancel = cancel
        wire.auto_accept = not late_acceptance
        adapter = FleetAdapter(config, journal, wire, clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        adapter.submit(
            MissionRequest("active", "assembly", "inspection", "motor", "amr_01")
        )
        for _ in range(3):
            adapter.tick()
        goal = wire.goals[0]
        flight = adapter._flights["active"]
        before, events = journal.get("active"), journal.events("active")
        # Include an accepted response already queued when a journal write fails.
        if late_acceptance:
            goal.accepted(goal, "")
        journal._connection.execute("PRAGMA query_only = ON")
        with pytest.raises(sqlite3.OperationalError):
            adapter.submit(
                MissionRequest("rejected", "assembly", "inspection", "motor")
            )
        journal._connection.execute("PRAGMA query_only = OFF")
        for batch in range(2):
            for _ in range(1000):
                goal.feedback(
                    SimpleNamespace(state="COMPLETED", detail="", progress=1.0)
                )
                goal.result(RobotReply(True))
                goal.accepted(goal, "")
                # An obsolete acknowledgement is still an asynchronous callback;
                # it cannot authorize transport cancellation or a terminal write.
                adapter._enqueue(adapter._cancelled, "active", flight, True)
            adapter.tick()
            assert adapter._callbacks.qsize() <= 200, (
                f"failed callback queue grew in batch {batch}"
            )
        assert goal.cancel_ack is None
        assert cancelled_handles == []
        adapter.tick()
        assert adapter._callbacks.empty()
        assert len(adapter._flights) <= len(config.robots)
        assert journal.get("active") == before and journal.events("active") == events
        assert adapter.state == "STORAGE_FAILED" and len(wire.goals) == 1
        assert not adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="probe", resource_id="assembly"
            ),
        )["granted"]
    finally:
        journal.close()


def test_durable_payload_evidence_requires_correlation_and_preserves_delivery(
    config, tmp_path
):
    journal = MissionJournal(tmp_path / "correlated-payload.sqlite3")
    try:
        seed(journal, state="QUEUED", mission_id="queued")
        seed(journal, ownership="PICKED_UP", mission_id="done")
        journal.record_observation(
            safe(mode="EXECUTING", mission_id="other", payload_state="LOADED"), 5
        )
        journal.record_observation(
            safe(mode="EXECUTING", mission_id="done", payload_state="LOADED"), 6
        )
        journal.transition(
            "done",
            MissionState.EXECUTING,
            MissionState.COMPLETED,
            {"payload_ownership": "DELIVERED"},
            7,
        )
        # A genuinely ordered, correlated completed delivery is terminal. It must
        # not be undone by the carrying evidence from before that delivery.
        done = journal.get("done")
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        assert journal.get("queued").state == "QUEUED"
        assert journal.get("queued").payload_ownership == "NOT_PICKED_UP"
        assert journal.get("done") == done
    finally:
        journal.close()


def test_prior_correlated_pickup_cannot_be_lost_by_a_later_assignment(config, tmp_path):
    journal = MissionJournal(tmp_path / "prior-assignment.sqlite3")
    try:
        seed(journal)
        journal.record_observation(
            safe(mode="EXECUTING", mission_id="m1", payload_state="LOADED"), 5
        )
        # A journal from the previously unsafe recovery can already contain a
        # later assignment. Its new empty robot cannot erase the former payload.
        journal.transition(
            "m1",
            MissionState.EXECUTING,
            MissionState.ASSIGNED,
            {"assigned_robot_id": "amr_02"},
            6,
        )
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        assert journal.get("m1").state == "RECOVERY_REQUIRED"
        assert journal.get("m1").payload_ownership != "NOT_PICKED_UP"
        assert journal.get("m1").assigned_robot_id == "amr_01"
    finally:
        journal.close()


def test_unresolved_live_claim_fences_station_when_old_assignment_was_cleared(
    config, tmp_path
):
    from types import MappingProxyType

    config = replace(config, resource_bounds=MappingProxyType({}))
    journal = MissionJournal(tmp_path / "cleared-assignment.sqlite3")
    try:
        seed(journal, state="REASSIGNING")
        journal.transition(
            "m1",
            MissionState.REASSIGNING,
            MissionState.REASSIGNING,
            {"assigned_robot_id": None},
            3,
        )
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(
            safe(mode="WAITING_FOR_RESOURCE", mission_id="m1", pose=Pose2D(-3, 1, 0))
        )
        adapter.observe(safe("amr_02"))
        assert journal.get("m1").state == "RECOVERY_REQUIRED"
        response = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="other", resource_id="assembly"
            ),
        )
        assert not response["granted"]
        snapshot = next(
            value
            for value in adapter.resources.snapshot(100)
            if value.resource_id == "assembly"
        )
        assert snapshot.former_lease.robot_id == "amr_01" and snapshot.lease is None
    finally:
        journal.close()


def test_conflicting_durable_carriers_stay_unknown_without_delivery(config, tmp_path):
    journal = MissionJournal(tmp_path / "conflicting-carriers.sqlite3")
    try:
        seed(journal)
        for sequence, robot_id in enumerate(("amr_01", "amr_02", "amr_01"), 3):
            journal.record_observation(
                safe(
                    robot_id, mode="EXECUTING", mission_id="m1", payload_state="LOADED"
                ),
                sequence,
            )
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        record = journal.get("m1")
        assert (
            record.state == "RECOVERY_REQUIRED"
            and record.payload_ownership == "UNKNOWN"
        )
        assert record.assigned_robot_id == "amr_01"
    finally:
        journal.close()


def test_conflicting_observation_preserves_durable_former_payload_owner(
    config, tmp_path
):
    journal = MissionJournal(tmp_path / "durable-owner.sqlite3")
    try:
        seed(journal, ownership="PICKED_UP")
        journal.record_observation(
            safe("amr_02", mode="EXECUTING", mission_id="m1", payload_state="LOADED"), 3
        )
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        record = journal.get("m1")
        assert (
            record.state == "RECOVERY_REQUIRED" and record.assigned_robot_id == "amr_01"
        )
        assert record.payload_ownership == "UNKNOWN"
    finally:
        journal.close()


def test_every_legacy_owner_remains_fenced_after_known_owner_clears(config, tmp_path):
    path = tmp_path / "multiple-legacy-owners.sqlite3"
    journal = MissionJournal(path)
    try:
        seed(journal, ownership="UNKNOWN", mission_id="first")
        seed(journal, ownership="UNKNOWN", mission_id="removed")
        journal.transition(
            "removed",
            MissionState.EXECUTING,
            MissionState.EXECUTING,
            {"assigned_robot_id": "amr_03"},
            3,
        )
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        response = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="probe", resource_id="assembly"
            ),
        )
        assert not response["granted"], "removed second owner must remain quarantined"
        snapshot = next(
            s for s in adapter.resources.snapshot(100) if s.resource_id == "assembly"
        )
        assert snapshot.lease is None
        assert {lease.robot_id for lease in snapshot.former_leases} == {"amr_03"}
        journal.close()
        journal = MissionJournal(path)
        restored = ResourceManager(config.resources)
        restored.restore_evidence(journal.load_resources())
        assert not restored.acquire(
            LeaseRequest("amr_02", "probe", "assembly"), 200
        ).granted
    finally:
        journal.close()


def test_operator_clearance_removes_only_exact_claim_and_preserves_fairness(config):
    manager = ResourceManager(config.resources)
    first = Lease("amr_01", "first", "assembly", "claim-one", 0)
    second = Lease("amr_03", "second", "assembly", "claim-two", 0)
    manager.quarantine_evidence(first)
    manager.quarantine_evidence(second)
    assert not manager.acquire(LeaseRequest("amr_02", "queued", "assembly"), 1).granted
    assert not manager.clear_reconciliation(
        LeaseKey("amr_03", "second", "assembly", "claim-one"), 2
    )
    assert manager.clear_reconciliation(
        LeaseKey("amr_01", "first", "assembly", "claim-one"), 2
    )
    assert not manager.acquire(LeaseRequest("amr_04", "late", "assembly"), 3).granted
    snapshot = next(s for s in manager.snapshot(3) if s.resource_id == "assembly")
    assert snapshot.lease is None and snapshot.former_lease == second
    assert manager.clear_reconciliation(
        LeaseKey("amr_03", "second", "assembly", "claim-two"), 4
    )
    granted = manager.acquire(LeaseRequest("amr_02", "queued", "assembly"), 4)
    assert granted.granted and granted.lease.lease_id not in ("claim-one", "claim-two")


@pytest.mark.parametrize("active", [False, True])
def test_recovery_query_never_reads_telemetry_history(tmp_path, active):
    import sqlite3

    journal = MissionJournal(tmp_path / "bounded.sqlite3")
    try:
        if active:
            seed(journal)
            journal.record_observation(safe(mission_id="m1", payload_state="LOADED"), 3)
        # Thousands of unrelated positive observations must not affect recovery work.
        evidence = '{"mission_id":"old","payload_state":"LOADED"}'
        journal._connection.executemany(
            "INSERT INTO robot_observations(robot_id,evidence_json,received_at) VALUES ('old',?,0)",
            [(evidence,)] * 10000,
        )

        def authorize(action, table, column, database, trigger):
            if action == sqlite3.SQLITE_READ and table == "robot_observations":
                return sqlite3.SQLITE_DENY
            return sqlite3.SQLITE_OK

        journal._connection.set_authorizer(authorize)
        records = journal.load_recovery()
        assert len(records) == int(active)
        if active:
            assert records[0].payload_ownership == "PICKED_UP"
    finally:
        journal.close()


def test_payload_summary_backfills_legacy_once_and_uses_indexes(tmp_path):
    import json
    import sqlite3

    path = tmp_path / "legacy-payload.sqlite3"
    journal = MissionJournal(path)
    seed(journal)
    # Simulate the old journal writer: carrying INSERT committed independently
    # of a mission transition, and no compact summary schema existed yet.
    for robot_id, payload in (
        ("amr_01", "LOADED"),
        ("amr_02", "UNKNOWN"),
        ("amr_01", "LOADED"),
    ):
        journal._connection.execute(
            "INSERT INTO robot_observations(robot_id,evidence_json,received_at) VALUES (?,?,0)",
            (robot_id, json.dumps({"mission_id": "m1", "payload_state": payload})),
        )
    journal._connection.execute("DROP TABLE IF EXISTS payload_evidence")
    journal._connection.execute("DROP TABLE IF EXISTS journal_schema")
    journal.close()
    journal = MissionJournal(path)
    try:
        record = journal.load_recovery()[0]
        assert (
            record.assigned_robot_id == "amr_01"
            and record.payload_ownership == "UNKNOWN"
        )
        assert journal.recovery_carriers("m1") == ("amr_01", "amr_02")
        plan = journal._connection.execute(
            "EXPLAIN QUERY PLAN SELECT * FROM payload_evidence WHERE mission_id='m1'"
        ).fetchall()
        assert any("SEARCH" in row[3] and "INDEX" in row[3] for row in plan)
        carrier_plan = journal._connection.execute(
            "EXPLAIN QUERY PLAN SELECT robot_id FROM payload_carrier_claims WHERE mission_id='m1' ORDER BY robot_id"
        ).fetchall()
        assert any("SEARCH" in row[3] and "INDEX" in row[3] for row in carrier_plan)
        active_plan = journal._connection.execute(
            "EXPLAIN QUERY PLAN SELECT * FROM missions WHERE state NOT IN ('COMPLETED','FAILED','CANCELLED') ORDER BY created_at,mission_id"
        ).fetchall()
        assert any("missions_active" in row[3] for row in active_plan)
        history_plan = journal._connection.execute(
            "EXPLAIN QUERY PLAN SELECT robot_id,evidence_json FROM robot_observations WHERE json_extract(evidence_json,'$.payload_state') IN ('LOADED','UNKNOWN') ORDER BY sequence"
        ).fetchall()
        assert any("robot_payload_history" in row[3] for row in history_plan)
    finally:
        journal.close()
    # A later open must not scan even one raw telemetry row again.
    real_connect = sqlite3.connect

    class NoHistoryRead(sqlite3.Connection):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.set_authorizer(
                lambda action, table, column, db, trigger: (
                    sqlite3.SQLITE_DENY
                    if action == sqlite3.SQLITE_READ and table == "robot_observations"
                    else sqlite3.SQLITE_OK
                )
            )

    from unittest.mock import patch

    with patch(
        "fleet_manager.journal.sqlite3.connect",
        lambda *args, **kwargs: real_connect(*args, **kwargs, factory=NoHistoryRead),
    ):
        journal = MissionJournal(path)
        try:
            assert journal.load_recovery()[0] == record
        finally:
            journal.close()


def test_payload_summary_write_failure_rolls_back_raw_observation(tmp_path):
    import sqlite3

    journal = MissionJournal(tmp_path / "summary-atomic.sqlite3")
    try:
        seed(journal)
        journal._connection.execute(
            "CREATE TRIGGER reject_summary BEFORE INSERT ON payload_evidence "
            "BEGIN SELECT RAISE(ABORT,'summary failed'); END"
        )
        with pytest.raises(sqlite3.IntegrityError, match="summary failed"):
            journal.record_observation(safe(mission_id="m1", payload_state="LOADED"), 3)
        assert (
            journal._connection.execute(
                "SELECT COUNT(*) FROM robot_observations"
            ).fetchone()[0]
            == 0
        )
        assert journal.load_recovery()[0].payload_ownership == "NOT_PICKED_UP"
    finally:
        journal.close()


def test_unknown_payload_summary_is_monotonic_until_correlated_delivery(tmp_path):
    journal = MissionJournal(tmp_path / "unknown-monotonic.sqlite3")
    try:
        seed(journal)
        for timestamp, payload in enumerate(("UNKNOWN", "LOADED", "EMPTY"), 3):
            journal.record_observation(
                safe(mission_id="m1", payload_state=payload), timestamp
            )
        assert journal.load_recovery()[0].payload_ownership == "UNKNOWN"
        journal.transition(
            "m1",
            MissionState.EXECUTING,
            MissionState.COMPLETED,
            {"payload_ownership": "DELIVERED"},
            7,
        )
        assert journal.load_recovery() == ()
        assert journal.recovery_carriers("m1") == ()
        assert journal.get("m1").payload_ownership == "DELIVERED"
    finally:
        journal.close()


def test_later_durable_uncertainty_cannot_be_weakened_by_earlier_loaded_summary(
    tmp_path,
):
    journal = MissionJournal(tmp_path / "durable-unknown.sqlite3")
    try:
        seed(journal)
        journal.record_observation(safe(mission_id="m1", payload_state="LOADED"), 3)
        journal.transition(
            "m1",
            MissionState.EXECUTING,
            MissionState.EXECUTING,
            {"payload_ownership": "UNKNOWN"},
            4,
        )
        assert journal.load_recovery()[0].payload_ownership == "UNKNOWN"
    finally:
        journal.close()


def test_removed_conflicting_durable_carrier_keeps_related_resources_fenced(
    config, tmp_path
):
    path = tmp_path / "removed-carrier.sqlite3"
    journal = MissionJournal(path)
    try:
        seed(journal)
        journal.record_observation(safe(mission_id="m1", payload_state="LOADED"), 3)
        journal.record_observation(
            safe("amr_03", mission_id="m1", payload_state="UNKNOWN"), 4
        )
        journal.close()
        journal = MissionJournal(path)
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        record = journal.get("m1")
        assert (
            record.assigned_robot_id == "amr_01"
            and record.payload_ownership == "UNKNOWN"
        )
        response = adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="probe", resource_id="assembly"
            ),
        )
        assert not response["granted"], (
            "durable removed carrier cannot be cleared by another owner's pose"
        )
        claims = next(
            s.former_leases
            for s in adapter.resources.snapshot(100)
            if s.resource_id == "assembly"
        )
        assert {claim.robot_id for claim in claims} == {"amr_03"}
    finally:
        journal.close()


@pytest.mark.parametrize("legacy", [False, True])
def test_original_removed_executor_survives_carrier_projection_and_repeated_restart(
    config, tmp_path, legacy
):
    path = tmp_path / "weak-executor.sqlite3"
    journal = MissionJournal(path)
    try:
        seed(journal)
        journal.transition(
            "m1",
            MissionState.EXECUTING,
            MissionState.EXECUTING,
            {"assigned_robot_id": "amr_03"},
            3,
        )
        journal.record_observation(safe(mission_id="m1", payload_state="LOADED"), 4)
        if legacy:
            journal._connection.execute("DROP TABLE recovery_physical_claims")
            journal._connection.execute(
                "DELETE FROM journal_schema WHERE name='physical_claims'"
            )
        previous = None
        for now in (100, 200, 300):
            journal.close()
            journal = MissionJournal(path)
            adapter = FleetAdapter(
                config, journal, FakeRobots(journal), clock=lambda: now
            )
            adapter.observe(safe())
            adapter.observe(safe("amr_02"))
            record = journal.get("m1")
            assert record.state == "RECOVERY_REQUIRED"
            assert (
                record.assigned_robot_id == "amr_01"
                and record.payload_ownership == "PICKED_UP"
            )
            response = adapter.resource(
                "acquire",
                SimpleNamespace(
                    robot_id="amr_02", mission_id="probe", resource_id="assembly"
                ),
            )
            assert not response["granted"], (
                "original removed executor is not physical payload-carrier proof"
            )
            current = (journal.events("m1"), journal.load_resources())
            if previous is not None:
                assert current == previous
            previous = current
    finally:
        journal.close()


def test_exact_operator_clearance_survives_restart_and_new_claim_refences(
    config, tmp_path
):
    from types import MappingProxyType

    config = replace(config, resource_bounds=MappingProxyType({}))
    path = tmp_path / "resolved-generations.sqlite3"
    journal = MissionJournal(path)
    try:
        seed(journal)
        journal.transition(
            "m1",
            MissionState.EXECUTING,
            MissionState.EXECUTING,
            {"assigned_robot_id": "amr_03"},
            3,
        )
        journal.record_observation(safe(mission_id="m1", payload_state="LOADED"), 4)
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        claims = next(
            s.former_leases
            for s in adapter.resources.snapshot(100)
            if s.resource_id == "assembly"
        )
        assert {lease.robot_id for lease in claims} == {"amr_01", "amr_03"}
        adapter.resources.acquire(LeaseRequest("amr_02", "probe", "assembly"), 100)
        for index, claim in enumerate(claims):
            wrong = LeaseKey(
                claim.robot_id, claim.mission_id, claim.resource_id, "wrong-token"
            )
            assert not adapter.resources.clear_reconciliation(wrong, 101)
            assert adapter.resources.clear_reconciliation(
                LeaseKey(
                    claim.robot_id, claim.mission_id, claim.resource_id, claim.lease_id
                ),
                101,
            )
            snapshot = next(
                s
                for s in adapter.resources.snapshot(101)
                if s.resource_id == "assembly"
            )
            assert (snapshot.lease is not None) == (index == len(claims) - 1)
        # Drop the new test owner's authority before persisting operator clearance.
        lease = snapshot.lease
        assert adapter.resources.release(
            LeaseKey(
                lease.robot_id, lease.mission_id, lease.resource_id, lease.lease_id
            ),
            102,
        )
        journal.save_resources(adapter.resources.snapshot(102), 102)
        before = journal.events("m1")
        journal.close()
        journal = MissionJournal(path)
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 200)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        assert (
            journal.get("m1").state == "RECOVERY_REQUIRED"
            and journal.events("m1") == before
        )
        assert adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="probe", resource_id="assembly"
            ),
        )["granted"]
        # Different mission/evidence must not inherit an exemption for amr_03.
        seed(journal, mission_id="new")
        journal.transition(
            "new",
            MissionState.EXECUTING,
            MissionState.EXECUTING,
            {"assigned_robot_id": "amr_03"},
            201,
        )
        journal.close()
        journal = MissionJournal(path)
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 300)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        assert not adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="another", resource_id="assembly"
            ),
        )["granted"]
        assert any(
            s.former_leases
            for s in adapter.resources.snapshot(300)
            if s.resource_id == "assembly"
        )
    finally:
        journal.close()


def test_resolution_identity_includes_owner_even_when_evidence_tokens_collide(
    config, tmp_path
):
    journal = MissionJournal(tmp_path / "exact-resolution.sqlite3")
    try:
        manager = ResourceManager(config.resources)
        first = Lease("amr_01", "m1", "assembly", "same-evidence", 0)
        second = Lease("amr_03", "m2", "assembly", "same-evidence", 0)
        manager.quarantine_evidence(first)
        manager.quarantine_evidence(second)
        journal.save_resources(manager.snapshot(1), 1)
        key = LeaseKey(
            first.robot_id, first.mission_id, first.resource_id, first.lease_id
        )
        assert manager.clear_reconciliation(key, 2)
        journal.save_resources(manager.snapshot(2), 2)
        assert journal.claim_resolved(key)
        assert not journal.claim_resolved(
            LeaseKey(
                second.robot_id, second.mission_id, second.resource_id, second.lease_id
            )
        )
    finally:
        journal.close()


def test_new_same_mission_evidence_does_not_inherit_operator_clearance(
    config, tmp_path
):
    from types import MappingProxyType

    config = replace(config, resource_bounds=MappingProxyType({}))
    path = tmp_path / "new-evidence.sqlite3"
    journal = MissionJournal(path)
    try:
        seed(journal)
        journal.record_observation(safe(mission_id="m1", payload_state="LOADED"), 3)
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 100)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        claims = next(
            s.former_leases
            for s in adapter.resources.snapshot(100)
            if s.resource_id == "assembly"
        )
        old_ids = {claim.lease_id for claim in claims}
        for claim in claims:
            assert adapter.resources.clear_reconciliation(
                LeaseKey(
                    claim.robot_id, claim.mission_id, claim.resource_id, claim.lease_id
                ),
                101,
            )
        journal.save_resources(adapter.resources.snapshot(101), 101)
        journal.record_observation(
            safe(mode="EXECUTING", mission_id="m1", payload_state="LOADED"), 102
        )
        journal.close()
        journal = MissionJournal(path)
        adapter = FleetAdapter(config, journal, FakeRobots(journal), clock=lambda: 200)
        adapter.observe(safe())
        adapter.observe(safe("amr_02"))
        claims = next(
            s.former_leases
            for s in adapter.resources.snapshot(200)
            if s.resource_id == "assembly"
        )
        assert claims and all(claim.lease_id not in old_ids for claim in claims)
        assert not adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="probe", resource_id="assembly"
            ),
        )["granted"]
        assert journal.get("m1").state == "RECOVERY_REQUIRED"
    finally:
        journal.close()


def test_clearance_write_failure_rolls_back_mission_and_resource_proof(
    config, tmp_path
):
    journal = MissionJournal(tmp_path / "clearance-rollback.sqlite3")
    try:
        before = seed(journal)
        events = journal.events("m1")
        journal._connection.execute(
            "CREATE TRIGGER reject_clearance BEFORE INSERT ON resource_claim_resolutions "
            "BEGIN SELECT RAISE(ABORT,'clearance failed'); END"
        )
        wire = FakeRobots(journal)
        adapter = FleetAdapter(config, journal, wire, clock=lambda: 100)
        adapter.observe(safe())
        # Startup reconciliation reports failure through the permanent adapter
        # latch; its existing callback contract does not propagate this error.
        adapter.observe(safe("amr_02"))
        assert adapter.state == "STORAGE_FAILED"
        assert journal.get("m1") == before and journal.events("m1") == events
        assert (
            journal._connection.execute(
                "SELECT COUNT(*) FROM resource_claim_resolutions"
            ).fetchone()[0]
            == 0
        )
        assert journal.load_resources() == ()
        assert not adapter.resource(
            "acquire",
            SimpleNamespace(
                robot_id="amr_02", mission_id="probe", resource_id="assembly"
            ),
        )["granted"]
        assert not wire.goals
    finally:
        journal.close()


def test_legacy_snapshot_former_owner_is_not_mistaken_for_clearance(tmp_path):
    from fleet_manager.resources import ResourceSnapshot

    journal = MissionJournal(tmp_path / "legacy-former.sqlite3")
    try:
        former = Lease("amr_03", "m1", "assembly", "old-token", 0)
        snapshot = ResourceSnapshot("assembly", "station", 1, None, (), True, former)
        journal.save_resources((snapshot,), 1)
        journal.save_resources(
            (
                replace(
                    snapshot, waiters=(LeaseRequest("amr_01", "queued", "assembly"),)
                ),
            ),
            2,
        )
        assert not journal.claim_resolved(
            LeaseKey("amr_03", "m1", "assembly", "old-token")
        )
    finally:
        journal.close()
