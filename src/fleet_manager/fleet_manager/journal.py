# SPDX-License-Identifier: Apache-2.0
"""Thread-owned SQLite journal for the fleet manager's durable mission state."""

from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from enum import Enum
import json
import math
from pathlib import Path
import sqlite3
from types import MappingProxyType
from typing import Mapping

from fleet_manager.models import MissionRequest


class MissionState(str, Enum):
    RECEIVED = "RECEIVED"
    QUEUED = "QUEUED"
    ASSIGNING = "ASSIGNING"
    ASSIGNED = "ASSIGNED"
    EXECUTING = "EXECUTING"
    REASSIGNING = "REASSIGNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"


class PayloadOwnership(str, Enum):
    NOT_PICKED_UP = "NOT_PICKED_UP"
    PICKED_UP = "PICKED_UP"
    UNKNOWN = "UNKNOWN"
    DELIVERED = "DELIVERED"


@dataclass(frozen=True)
class MissionRecord:
    request: MissionRequest
    payload_hash: str
    state: MissionState
    assigned_robot_id: str | None
    payload_ownership: PayloadOwnership
    created_at: float
    updated_at: float
    result: Mapping[str, object] | None


@dataclass(frozen=True)
class RegisterResult:
    record: MissionRecord
    created: bool


@dataclass(frozen=True)
class MissionEvent:
    sequence: int
    mission_id: str
    previous_state: MissionState | None
    state: MissionState
    detail: Mapping[str, object]
    timestamp: float


class MissionConflictError(ValueError):
    error_code = "MISSION_ID_CONFLICT"

    def __init__(self, mission_id: str):
        self.mission_id = mission_id
        super().__init__(f"Mission {mission_id} already has a different payload hash")


class MissionStateConflictError(ValueError):
    def __init__(self, mission_id: str, expected, actual: MissionState):
        self.mission_id = mission_id
        self.expected = expected
        self.actual = actual
        super().__init__(
            f"Mission {mission_id}: expected state {expected}, actual state {actual}"
        )


def _timestamp(now: float) -> float:
    if not math.isfinite(now):
        raise ValueError("mission timestamp must be finite")
    return float(now)


def _json(value: Mapping[str, object]) -> str:
    return json.dumps(
        dict(value), sort_keys=True, separators=(",", ":"), allow_nan=False
    )


def _robot_id(robot_id: str) -> None:
    if not isinstance(robot_id, str) or not robot_id:
        raise ValueError("assigned robot must be a nonempty string")


def _record(row: sqlite3.Row) -> MissionRecord:
    return MissionRecord(
        request=MissionRequest(
            row["mission_id"],
            row["pickup_station"],
            row["dropoff_station"],
            row["part"],
            row["requested_robot_id"],
        ),
        payload_hash=row["payload_hash"],
        state=MissionState(row["state"]),
        assigned_robot_id=row["assigned_robot_id"],
        payload_ownership=PayloadOwnership(row["payload_ownership"]),
        created_at=row["created_at"],
        updated_at=row["updated_at"],
        result=(
            None
            if row["result_json"] is None
            else MappingProxyType(json.loads(row["result_json"]))
        ),
    )


class MissionJournal:
    """Own one connection on its creating thread; close it on that same thread.

    The fleet manager is the only production writer. Serialize its calls on
    the owning thread; do not share a handle across executor callback threads.
    Separate handles have SQLite's writer serialization and compare-and-set
    protection. Timestamps are supplied evidence, not an event ordering key.

    Transition legality belongs to FleetCore. In transition details, reserved
    keys ``payload_ownership``, ``result``, and ``assigned_robot_id`` update
    the durable snapshot as well as the audit event. ``assigned_robot_id=None``
    clears a former assignment when requeuing. Other keys are audit evidence.
    """

    def __init__(self, path: str | Path, busy_timeout_ms: int = 5000):
        self._connection = sqlite3.connect(
            path, timeout=busy_timeout_ms / 1000, isolation_level=None
        )
        self._connection.row_factory = sqlite3.Row
        try:
            self._connection.execute("PRAGMA journal_mode = WAL")
            self._connection.execute("PRAGMA foreign_keys = ON")
            with self._transaction():
                self._connection.execute(
                    """CREATE TABLE IF NOT EXISTS missions (
                        mission_id TEXT PRIMARY KEY,
                        pickup_station TEXT NOT NULL,
                        dropoff_station TEXT NOT NULL,
                        part TEXT NOT NULL,
                        requested_robot_id TEXT,
                        payload_hash TEXT NOT NULL,
                        state TEXT NOT NULL CHECK (state IN (
                            'RECEIVED', 'QUEUED', 'ASSIGNING', 'ASSIGNED',
                            'EXECUTING', 'REASSIGNING', 'COMPLETED', 'FAILED',
                            'CANCELLED', 'RECOVERY_REQUIRED')),
                        assigned_robot_id TEXT,
                        payload_ownership TEXT NOT NULL CHECK (payload_ownership IN (
                            'NOT_PICKED_UP', 'PICKED_UP', 'UNKNOWN', 'DELIVERED')),
                        created_at REAL NOT NULL,
                        updated_at REAL NOT NULL,
                        result_json TEXT
                    )"""
                )
                self._connection.execute(
                    """CREATE TABLE IF NOT EXISTS mission_events (
                        sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                        mission_id TEXT NOT NULL REFERENCES missions(mission_id),
                        previous_state TEXT,
                        state TEXT NOT NULL,
                        detail_json TEXT NOT NULL,
                        timestamp REAL NOT NULL
                    )"""
                )
                self._connection.execute(
                    "CREATE INDEX IF NOT EXISTS mission_events_by_mission "
                    "ON mission_events(mission_id, sequence)"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS resource_snapshots "
                    "(resource_id TEXT PRIMARY KEY, evidence_json TEXT NOT NULL)"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS resource_events "
                    "(sequence INTEGER PRIMARY KEY AUTOINCREMENT, resource_id TEXT NOT NULL, "
                    "evidence_json TEXT NOT NULL, timestamp REAL NOT NULL)"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS robot_observations "
                    "(sequence INTEGER PRIMARY KEY AUTOINCREMENT, robot_id TEXT NOT NULL, "
                    "evidence_json TEXT NOT NULL, source_time_ns INTEGER, "
                    "received_at REAL NOT NULL)"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS contact_observations "
                    "(sequence INTEGER PRIMARY KEY AUTOINCREMENT, robot_id TEXT NOT NULL, "
                    "contact INTEGER NOT NULL, source_time_ns INTEGER NOT NULL, received_at REAL NOT NULL)"
                )
        except BaseException:
            self._connection.close()
            raise

    @contextmanager
    def _transaction(self):
        self._connection.execute("BEGIN IMMEDIATE")
        try:
            yield
            self._connection.execute("COMMIT")
        except BaseException:
            if self._connection.in_transaction:
                self._connection.execute("ROLLBACK")
            raise

    def _find(self, mission_id: str) -> MissionRecord | None:
        row = self._connection.execute(
            "SELECT * FROM missions WHERE mission_id = ?", (mission_id,)
        ).fetchone()
        return None if row is None else _record(row)

    def _require(self, mission_id: str) -> MissionRecord:
        record = self._find(mission_id)
        if record is None:
            raise KeyError(mission_id)
        return record

    def _event(self, mission_id, previous_state, state, detail_json, now):
        self._connection.execute(
            "INSERT INTO mission_events "
            "(mission_id, previous_state, state, detail_json, timestamp) "
            "VALUES (?, ?, ?, ?, ?)",
            (mission_id, previous_state, state, detail_json, now),
        )

    def register(
        self, request: MissionRequest, payload_hash: str, now: float
    ) -> RegisterResult:
        now = _timestamp(now)
        with self._transaction():
            existing = self._find(request.mission_id)
            if existing is not None:
                if existing.payload_hash != payload_hash:
                    raise MissionConflictError(request.mission_id)
                return RegisterResult(existing, created=False)
            self._connection.execute(
                "INSERT INTO missions "
                "(mission_id, pickup_station, dropoff_station, part, "
                "requested_robot_id, payload_hash, state, payload_ownership, "
                "created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    request.mission_id,
                    request.pickup_station,
                    request.dropoff_station,
                    request.part,
                    request.requested_robot_id,
                    payload_hash,
                    MissionState.QUEUED,
                    PayloadOwnership.NOT_PICKED_UP,
                    now,
                    now,
                ),
            )
            self._event(request.mission_id, None, MissionState.QUEUED, "{}", now)
            return RegisterResult(self._require(request.mission_id), created=True)

    def _change(self, record, target, detail_json, now, assigned_robot_id):
        detail = json.loads(detail_json)
        ownership = PayloadOwnership(
            detail.get("payload_ownership", record.payload_ownership)
        )
        result = detail.get("result", record.result)
        self._connection.execute(
            "UPDATE missions SET state = ?, assigned_robot_id = ?, "
            "payload_ownership = ?, result_json = ?, updated_at = ? "
            "WHERE mission_id = ? AND state = ?",
            (
                target,
                assigned_robot_id,
                ownership,
                None if result is None else _json(result),
                now,
                record.request.mission_id,
                record.state,
            ),
        )
        self._event(record.request.mission_id, record.state, target, detail_json, now)
        return self._require(record.request.mission_id)

    def transition(
        self,
        mission_id: str,
        expected: MissionState,
        target: MissionState,
        detail: Mapping[str, object],
        now: float,
    ) -> MissionRecord:
        now = _timestamp(now)
        expected, target = MissionState(expected), MissionState(target)
        detail_json = _json(detail)
        if "payload_ownership" in detail:
            PayloadOwnership(detail["payload_ownership"])
        if "result" in detail:
            if not isinstance(detail["result"], Mapping):
                raise ValueError("result must be a mapping")
            if "final_state" in detail["result"]:
                MissionState(detail["result"]["final_state"])
        if detail.get("assigned_robot_id") is not None:
            _robot_id(detail["assigned_robot_id"])
        with self._transaction():
            record = self._require(mission_id)
            if record.state != expected:
                raise MissionStateConflictError(mission_id, expected, record.state)
            return self._change(
                record,
                target,
                detail_json,
                now,
                detail.get("assigned_robot_id", record.assigned_robot_id),
            )

    def assign(self, mission_id: str, robot_id: str, now: float) -> MissionRecord:
        now = _timestamp(now)
        _robot_id(robot_id)
        detail_json = _json({"assigned_robot_id": robot_id})
        with self._transaction():
            record = self._require(mission_id)
            assigning = (MissionState.ASSIGNING, MissionState.REASSIGNING)
            if record.state not in assigning:
                raise MissionStateConflictError(mission_id, assigning, record.state)
            return self._change(
                record, MissionState.ASSIGNED, detail_json, now, robot_id
            )

    def load_active(self) -> tuple[MissionRecord, ...]:
        rows = self._connection.execute(
            "SELECT * FROM missions WHERE state NOT IN (?, ?, ?) "
            "ORDER BY created_at, mission_id",
            (MissionState.COMPLETED, MissionState.FAILED, MissionState.CANCELLED),
        ).fetchall()
        return tuple(_record(row) for row in rows)

    def load_recovery(self) -> tuple[MissionRecord, ...]:
        """Merge durable correlated carrying evidence, never replay pose authority.

        An observation can commit before its corresponding mission transition
        fails. EMPTY is not an ordered delivery proof and cannot erase carrying
        evidence. Completed delivery rows are excluded by load_active().
        """
        active = {record.request.mission_id: record for record in self.load_active()}
        carriers = {
            mission_id: record.assigned_robot_id
            for mission_id, record in active.items()
            if record.assigned_robot_id is not None
            and record.payload_ownership
            in (PayloadOwnership.PICKED_UP, PayloadOwnership.UNKNOWN)
        }
        conflicts = set()
        for row in self._connection.execute(
            "SELECT robot_id, evidence_json FROM robot_observations "
            "WHERE json_extract(evidence_json, '$.payload_state') IN ('LOADED', 'UNKNOWN') "
            "ORDER BY sequence"
        ):
            evidence = json.loads(row["evidence_json"])
            record = active.get(evidence.get("mission_id"))
            if record is None:
                continue
            carrier = carriers.setdefault(record.request.mission_id, row["robot_id"])
            ownership = record.payload_ownership
            if ownership == PayloadOwnership.NOT_PICKED_UP:
                ownership = (
                    PayloadOwnership.PICKED_UP
                    if evidence["payload_state"] == "LOADED"
                    else PayloadOwnership.UNKNOWN
                )
            elif (
                ownership == PayloadOwnership.UNKNOWN
                and evidence["payload_state"] == "LOADED"
            ):
                ownership = PayloadOwnership.PICKED_UP
            if carrier != row["robot_id"]:
                conflicts.add(record.request.mission_id)
            if record.request.mission_id in conflicts:
                ownership = PayloadOwnership.UNKNOWN
            active[record.request.mission_id] = replace(
                record,
                assigned_robot_id=carrier,
                payload_ownership=ownership,
            )
        return tuple(active.values())

    def events(self, mission_id: str) -> tuple[MissionEvent, ...]:
        rows = self._connection.execute(
            "SELECT * FROM mission_events WHERE mission_id = ? ORDER BY sequence",
            (mission_id,),
        ).fetchall()
        return tuple(
            MissionEvent(
                row["sequence"],
                row["mission_id"],
                None
                if row["previous_state"] is None
                else MissionState(row["previous_state"]),
                MissionState(row["state"]),
                MappingProxyType(json.loads(row["detail_json"])),
                row["timestamp"],
            )
            for row in rows
        )

    def get(self, mission_id: str) -> MissionRecord:
        """Read the current snapshot, including terminal missions."""
        return self._require(mission_id)

    def _save_resources(self, snapshots, now):
        for snapshot in snapshots:
            evidence = _json(asdict(snapshot))
            row = self._connection.execute(
                "SELECT evidence_json FROM resource_snapshots WHERE resource_id = ?",
                (snapshot.resource_id,),
            ).fetchone()
            if row is not None and row[0] == evidence:
                continue
            self._connection.execute(
                "INSERT INTO resource_snapshots VALUES (?, ?) "
                "ON CONFLICT(resource_id) DO UPDATE SET evidence_json=excluded.evidence_json",
                (snapshot.resource_id, evidence),
            )
            self._connection.execute(
                "INSERT INTO resource_events(resource_id, evidence_json, timestamp) VALUES (?, ?, ?)",
                (snapshot.resource_id, evidence, now),
            )

    def save_resources(self, snapshots, now):
        """Persist authority evidence before a service can report a grant."""
        now = _timestamp(now)
        with self._transaction():
            self._save_resources(snapshots, now)

    def load_resources(self):
        """Read persisted snapshots without treating tokens as authority."""
        from fleet_manager.resources import Lease, LeaseRequest, ResourceSnapshot

        values = []
        for row in self._connection.execute(
            "SELECT evidence_json FROM resource_snapshots ORDER BY resource_id"
        ):
            value = json.loads(row[0])
            for name in ("lease", "former_lease"):
                if value[name] is not None:
                    value[name] = Lease(**value[name])
            value["waiters"] = tuple(LeaseRequest(**item) for item in value["waiters"])
            values.append(ResourceSnapshot(**value))
        return tuple(values)

    def record_observation(self, robot, received_at, source_time_ns=None):
        evidence = _json(asdict(robot))
        with self._transaction():
            self._connection.execute(
                "INSERT INTO robot_observations(robot_id, evidence_json, source_time_ns, received_at) "
                "VALUES (?, ?, ?, ?)",
                (robot.robot_id, evidence, source_time_ns, _timestamp(received_at)),
            )

    def record_contact(self, robot_id, contact, received_at, source_time_ns):
        with self._transaction():
            self._connection.execute(
                "INSERT INTO contact_observations(robot_id, contact, source_time_ns, received_at) VALUES (?, ?, ?, ?)",
                (robot_id, int(contact), source_time_ns, _timestamp(received_at)),
            )

    def reconcile(self, decisions, snapshots, now):
        """Commit mission decisions and physical resource clearance atomically.

        Repeated identical decisions do not append duplicate mission events.
        The adapter exposes RUNNING only after this transaction commits.
        """
        now = _timestamp(now)
        with self._transaction():
            for decision in decisions:
                record = self._require(decision.mission_id)
                if (
                    record.state == decision.state
                    and record.assigned_robot_id == decision.robot_id
                    and record.payload_ownership == decision.payload_ownership
                ):
                    continue
                self._change(
                    record,
                    decision.state,
                    _json(
                        {
                            "reason": decision.reason,
                            "assigned_robot_id": decision.robot_id,
                            "payload_ownership": decision.payload_ownership,
                        }
                    ),
                    now,
                    decision.robot_id,
                )
            self._save_resources(snapshots, now)

    def close(self) -> None:
        self._connection.close()
