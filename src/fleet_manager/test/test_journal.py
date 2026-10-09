# SPDX-License-Identifier: Apache-2.0
"""Durable mission identity, snapshots, and audit writes use real SQLite."""

from concurrent.futures import ThreadPoolExecutor
import importlib
from pathlib import Path
import sqlite3
import sys
from threading import Barrier

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def journal_api():
    try:
        return importlib.import_module("fleet_manager.journal")
    except ModuleNotFoundError:
        pytest.fail("SQLite mission journal is not implemented")


def request(mission_id="mission_01", requested_robot_id=None):
    models = importlib.import_module("fleet_manager.models")
    return models.MissionRequest(
        mission_id, "assembly", "inspection", "gear", requested_robot_id
    )


def test_get_reads_terminal_snapshot_and_rejects_unknown_identity(journal):
    handle = journal()
    api = journal_api()
    handle.register(request(), "hash_01", 10.0)
    completed = handle.transition(
        "mission_01", api.MissionState.QUEUED, api.MissionState.COMPLETED, {}, 11.0
    )
    assert handle.get("mission_01") == completed
    with pytest.raises(KeyError, match="missing"):
        handle.get("missing")


@pytest.fixture
def database(tmp_path):
    return tmp_path / "missions.sqlite3"


@pytest.fixture
def journal(database):
    handles = []

    def open_journal():
        handle = journal_api().MissionJournal(database)
        handles.append(handle)
        return handle

    yield open_journal
    for handle in handles:
        handle.close()


def test_first_registration_persists_canonical_request_and_initial_event(journal):
    journal = journal()
    result = journal.register(request(requested_robot_id="amr_02"), "hash_01", 10.0)
    assert result.created is True
    assert result.record.request.mission_id == "mission_01"
    assert result.record.request.pickup_station == "assembly"
    assert result.record.request.dropoff_station == "inspection"
    assert result.record.request.part == "gear"
    assert result.record.request.requested_robot_id == "amr_02"
    assert result.record.payload_hash == "hash_01"
    assert result.record.state == "QUEUED"
    assert result.record.assigned_robot_id is None
    assert result.record.payload_ownership == "NOT_PICKED_UP"
    assert result.record.created_at == 10.0
    assert result.record.updated_at == 10.0
    assert result.record.result is None
    assert journal.load_active() == (result.record,)
    events = journal.events("mission_01")
    assert len(events) == 1
    assert events[0].mission_id == "mission_01"
    assert events[0].previous_state is None
    assert events[0].state == "QUEUED"
    assert events[0].timestamp == 10.0
    assert events[0].detail == {}


def test_active_order_uses_first_durable_event_across_reopen_and_clock_changes(journal):
    handle = journal()
    api = journal_api()
    handle.register(request("z_first"), "hash_first", 10.0)
    handle.register(request("a_second"), "hash_second", 10.0)
    handle.register(request("clock_reset"), "hash_reset", 1.0)
    handle.transition(
        "z_first", api.MissionState.QUEUED, api.MissionState.ASSIGNING, {}, 20.0
    )
    reopened = journal()
    assert [record.request.mission_id for record in reopened.load_active()] == [
        "z_first",
        "a_second",
        "clock_reset",
    ]


def test_identical_duplicate_returns_current_state_without_new_event(journal):
    journal = journal()
    api = journal_api()
    journal.register(request(), "hash_01", 10.0)
    record = journal.transition(
        "mission_01", api.MissionState.QUEUED, api.MissionState.ASSIGNING, {}, 11.0
    )
    duplicate = journal.register(request(), "hash_01", 100.0)
    assert duplicate.created is False
    assert duplicate.record == record
    assert len(journal.events("mission_01")) == 2


def test_conflicting_duplicate_changes_neither_snapshot_nor_events(journal):
    journal = journal()
    api = journal_api()
    original = journal.register(request(), "hash_01", 10.0).record
    with pytest.raises(api.MissionConflictError) as caught:
        journal.register(request(), "hash_conflict", 100.0)
    assert caught.value.mission_id == "mission_01"
    assert caught.value.error_code == "MISSION_ID_CONFLICT"
    assert journal.load_active() == (original,)
    assert len(journal.events("mission_01")) == 1


def test_compare_and_set_rejects_stale_state_without_side_effects(journal):
    journal = journal()
    api = journal_api()
    journal.register(request(), "hash_01", 10.0)
    record = journal.transition(
        "mission_01",
        api.MissionState.QUEUED,
        api.MissionState.ASSIGNING,
        {"reason": "dispatch"},
        11.0,
    )
    with pytest.raises(api.MissionStateConflictError) as caught:
        journal.transition(
            "mission_01", api.MissionState.QUEUED, api.MissionState.FAILED, {}, 12.0
        )
    assert caught.value.expected == "QUEUED"
    assert caught.value.actual == "ASSIGNING"
    assert journal.load_active() == (record,)
    events = journal.events("mission_01")
    assert len(events) == 2
    assert events[1].previous_state == "QUEUED"
    assert events[1].state == "ASSIGNING"
    assert events[1].timestamp == 11.0
    assert events[1].detail == {"reason": "dispatch"}


@pytest.mark.parametrize("operation", ["register", "transition", "assign"])
def test_event_write_failure_rolls_back_snapshot_and_connection_remains_usable(
    journal, database, operation
):
    journal = journal()
    api = journal_api()
    if operation != "register":
        journal.register(request(), "hash_01", 10.0)
        journal.transition(
            "mission_01", api.MissionState.QUEUED, api.MissionState.ASSIGNING, {}, 11.0
        )
    before_records = journal.load_active()
    before_events = journal.events("mission_01")
    # A real SQLite trigger fails the audit insert after the snapshot write.
    with sqlite3.connect(database) as connection:
        connection.execute(
            "CREATE TRIGGER fail_audit BEFORE INSERT ON mission_events "
            "BEGIN SELECT RAISE(ABORT, 'injected audit failure'); END"
        )
    with pytest.raises(sqlite3.IntegrityError, match="injected audit failure"):
        if operation == "register":
            journal.register(request(), "hash_01", 12.0)
        elif operation == "transition":
            journal.transition(
                "mission_01",
                api.MissionState.ASSIGNING,
                api.MissionState.FAILED,
                {"result": {"success": False, "error_code": "ROBOT_LOST"}},
                12.0,
            )
        else:
            journal.assign("mission_01", "amr_01", 12.0)
    assert journal.load_active() == before_records
    assert journal.events("mission_01") == before_events
    with sqlite3.connect(database) as connection:
        connection.execute("DROP TRIGGER fail_audit")
    assert journal.register(request("mission_02"), "hash_02", 13.0).created


def test_close_reopen_preserves_assignment_payload_and_completed_replay(database):
    api = journal_api()
    journal = api.MissionJournal(database)
    journal.register(request(requested_robot_id="amr_02"), "hash_01", 10.0)
    journal.transition(
        "mission_01", api.MissionState.QUEUED, api.MissionState.ASSIGNING, {}, 11.0
    )
    assigned = journal.assign("mission_01", "amr_02", 12.0)
    assert assigned.state == "ASSIGNED"
    assert assigned.assigned_robot_id == "amr_02"
    journal.transition(
        "mission_01",
        api.MissionState.ASSIGNED,
        api.MissionState.EXECUTING,
        {"payload_ownership": "PICKED_UP", "station": "assembly"},
        13.0,
    )
    result = {
        "success": True,
        "final_state": "COMPLETED",
        "assigned_robot_id": "amr_02",
        "error_code": "",
        "message": "Part delivered",
    }
    completed = journal.transition(
        "mission_01",
        api.MissionState.EXECUTING,
        api.MissionState.COMPLETED,
        {"payload_ownership": "DELIVERED", "result": result},
        14.0,
    )
    journal.close()
    reopened = api.MissionJournal(database)
    try:
        duplicate = reopened.register(
            request(requested_robot_id="amr_02"), "hash_01", 20.0
        )
        assert duplicate.created is False
        assert duplicate.record == completed
        assert duplicate.record.state == "COMPLETED"
        assert duplicate.record.assigned_robot_id == "amr_02"
        assert duplicate.record.payload_ownership == "DELIVERED"
        assert duplicate.record.result == result
        assert duplicate.record.created_at == 10.0
        assert duplicate.record.updated_at == 14.0
        assert reopened.load_active() == ()
        assert [event.state for event in reopened.events("mission_01")] == [
            "QUEUED",
            "ASSIGNING",
            "ASSIGNED",
            "EXECUTING",
            "COMPLETED",
        ]
    finally:
        reopened.close()


def test_two_handles_contend_without_duplicate_registration_or_lost_cas(database):
    api = journal_api()
    setup = api.MissionJournal(database)
    setup.close()
    ready = Barrier(2)

    def submit_and_transition():
        handle = api.MissionJournal(database)
        try:
            ready.wait(timeout=5)
            registered = handle.register(request(), "hash_01", 10.0)
            ready.wait(timeout=5)
            try:
                handle.transition(
                    "mission_01",
                    api.MissionState.QUEUED,
                    api.MissionState.ASSIGNING,
                    {},
                    11.0,
                )
                return registered.created, True
            except api.MissionStateConflictError:
                return registered.created, False
        finally:
            handle.close()

    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [executor.submit(submit_and_transition) for _ in range(2)]
        outcomes = [future.result(timeout=10) for future in futures]
    assert sum(created for created, _ in outcomes) == 1
    assert sum(changed for _, changed in outcomes) == 1
    verifier = api.MissionJournal(database)
    try:
        assert verifier.load_active()[0].state == "ASSIGNING"
        assert len(verifier.events("mission_01")) == 2
    finally:
        verifier.close()


def test_writer_lock_timeout_rolls_back_and_allows_retry(database):
    api = journal_api()
    journal = api.MissionJournal(database, busy_timeout_ms=20)
    blocker = sqlite3.connect(database, isolation_level=None)
    try:
        blocker.execute("BEGIN IMMEDIATE")
        with pytest.raises(sqlite3.OperationalError, match="database is locked"):
            journal.register(request(), "hash_01", 10.0)
        assert journal.load_active() == ()
        assert journal.events("mission_01") == ()
        blocker.execute("ROLLBACK")
        assert journal.register(request(), "hash_01", 10.0).created
    finally:
        blocker.close()
        journal.close()


def test_wal_foreign_keys_and_busy_timeout_are_enabled(journal, database):
    journal = journal()
    with sqlite3.connect(database) as reader:
        assert reader.execute("PRAGMA journal_mode").fetchone()[0] == "wal"
    assert journal._connection.execute("PRAGMA foreign_keys").fetchone()[0] == 1
    assert journal._connection.execute("PRAGMA busy_timeout").fetchone()[0] == 5000


def test_events_follow_database_sequence_even_when_clock_moves_back(journal):
    journal = journal()
    api = journal_api()
    journal.register(request(), "hash_01", 10.0)
    journal.transition(
        "mission_01", api.MissionState.QUEUED, api.MissionState.ASSIGNING, {}, 9.0
    )
    journal.assign("mission_01", "amr_01", 9.0)
    assert [event.state for event in journal.events("mission_01")] == [
        "QUEUED",
        "ASSIGNING",
        "ASSIGNED",
    ]
    sequences = [event.sequence for event in journal.events("mission_01")]
    assert sequences == sorted(set(sequences))


@pytest.mark.parametrize("state", ["COMPLETED", "FAILED", "CANCELLED"])
def test_terminal_records_are_excluded_but_recovery_required_remains_active(
    journal, state
):
    journal = journal()
    api = journal_api()
    journal.register(request(), "hash_01", 10.0)
    journal.transition(
        "mission_01", api.MissionState.QUEUED, api.MissionState(state), {}, 11.0
    )
    journal.register(request("mission_02"), "hash_02", 12.0)
    recovery = journal.transition(
        "mission_02",
        api.MissionState.QUEUED,
        api.MissionState.RECOVERY_REQUIRED,
        {"payload_ownership": "UNKNOWN"},
        13.0,
    )
    assert journal.load_active() == (recovery,)


def test_parameterized_fields_preserve_sql_like_input(journal):
    journal = journal()
    mission_id = "mission'; DROP TABLE missions; --"
    record = journal.register(request(mission_id), "hash'01", 10.0).record
    assert record.request.mission_id == mission_id
    assert journal.events(mission_id)[0].state == "QUEUED"
    assert journal.register(request("mission_02"), "hash_02", 11.0).created


@pytest.mark.parametrize("operation", ["transition", "assign"])
def test_unknown_mission_mutation_raises_key_error(journal, operation):
    journal = journal()
    api = journal_api()
    with pytest.raises(KeyError, match="missing"):
        if operation == "transition":
            journal.transition(
                "missing", api.MissionState.QUEUED, api.MissionState.FAILED, {}, 10.0
            )
        else:
            journal.assign("missing", "amr_01", 10.0)
    assert journal.load_active() == ()


def test_assignment_requires_assignment_state(journal):
    journal = journal()
    api = journal_api()
    original = journal.register(request(), "hash_01", 10.0).record
    with pytest.raises(api.MissionStateConflictError):
        journal.assign("mission_01", "amr_01", 11.0)
    assert journal.load_active() == (original,)
    assert len(journal.events("mission_01")) == 1


def test_reassignment_replaces_robot_and_records_audit(journal):
    journal = journal()
    api = journal_api()
    journal.register(request(), "hash_01", 10.0)
    journal.transition(
        "mission_01", api.MissionState.QUEUED, api.MissionState.ASSIGNING, {}, 11.0
    )
    journal.assign("mission_01", "amr_01", 12.0)
    journal.transition(
        "mission_01", api.MissionState.ASSIGNED, api.MissionState.REASSIGNING, {}, 13.0
    )
    record = journal.assign("mission_01", "amr_02", 14.0)
    assert record.state == "ASSIGNED"
    assert record.assigned_robot_id == "amr_02"
    assert journal.events("mission_01")[-1].detail == {"assigned_robot_id": "amr_02"}


def test_requeue_can_clear_assigned_robot_without_losing_assignment_history(journal):
    journal = journal()
    api = journal_api()
    journal.register(request(), "hash_01", 10.0)
    journal.transition(
        "mission_01", api.MissionState.QUEUED, api.MissionState.ASSIGNING, {}, 11.0
    )
    journal.assign("mission_01", "amr_01", 12.0)
    requeued = journal.transition(
        "mission_01",
        api.MissionState.ASSIGNED,
        api.MissionState.QUEUED,
        {"assigned_robot_id": None, "reason": "robot unavailable before pickup"},
        13.0,
    )
    assert requeued.assigned_robot_id is None
    assert requeued.payload_ownership == "NOT_PICKED_UP"
    assert journal.events("mission_01")[2].detail == {"assigned_robot_id": "amr_01"}
    assert journal.events("mission_01")[-1].detail["assigned_robot_id"] is None


@pytest.mark.parametrize("robot_id", [None, "", 42])
def test_assignment_rejects_missing_or_nonstring_robot_without_writes(
    journal, robot_id
):
    journal = journal()
    api = journal_api()
    journal.register(request(), "hash_01", 10.0)
    before = journal.transition(
        "mission_01", api.MissionState.QUEUED, api.MissionState.ASSIGNING, {}, 11.0
    )
    with pytest.raises(ValueError, match="robot"):
        journal.assign("mission_01", robot_id, 12.0)
    assert journal.load_active() == (before,)
    assert len(journal.events("mission_01")) == 2


@pytest.mark.parametrize("now", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize("operation", ["register", "transition", "assign"])
def test_nonfinite_timestamps_are_rejected_without_writes(journal, now, operation):
    journal = journal()
    api = journal_api()
    original = journal.register(request(), "hash_01", 10.0).record
    with pytest.raises(ValueError, match="finite"):
        if operation == "register":
            journal.register(request("mission_02"), "hash_02", now)
        elif operation == "transition":
            journal.transition(
                "mission_01", api.MissionState.QUEUED, api.MissionState.FAILED, {}, now
            )
        else:
            journal.assign("mission_01", "amr_01", now)
    assert journal.load_active() == (original,)
    assert len(journal.events("mission_01")) == 1


@pytest.mark.parametrize(
    "detail",
    [
        {"invalid": object()},
        {"invalid": float("nan")},
        {"payload_ownership": "invalid"},
        {"result": "not a result mapping"},
        {"result": {"success": True, "final_state": "SUCCEEDED"}},
        {"assigned_robot_id": 42},
        {"assigned_robot_id": ""},
    ],
)
def test_invalid_detail_is_rejected_before_mutating_snapshot(journal, detail):
    journal = journal()
    api = journal_api()
    original = journal.register(request(), "hash_01", 10.0).record
    with pytest.raises((TypeError, ValueError)):
        journal.transition(
            "mission_01", api.MissionState.QUEUED, api.MissionState.FAILED, detail, 11.0
        )
    assert journal.load_active() == (original,)
    assert len(journal.events("mission_01")) == 1


def test_succeeded_state_cannot_be_persisted(journal):
    journal = journal()
    api = journal_api()
    journal.register(request(), "hash_01", 10.0)
    with pytest.raises(ValueError):
        journal.transition("mission_01", api.MissionState.QUEUED, "SUCCEEDED", {}, 11.0)
    assert journal.load_active()[0].state == "QUEUED"


def test_handle_rejects_calls_from_another_thread(journal):
    journal = journal()
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(journal.load_active)
        with pytest.raises(sqlite3.ProgrammingError, match="same thread"):
            future.result(timeout=5)
    assert journal.register(request(), "hash_01", 10.0).created
