# SPDX-License-Identifier: Apache-2.0
"""Thread-owned SQLite journal for the fleet manager's durable mission state."""

from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from enum import Enum
import json
import math
from pathlib import Path
import secrets
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
                self._connection.execute(
                    "CREATE INDEX IF NOT EXISTS missions_active ON missions(created_at, mission_id) "
                    "WHERE state NOT IN ('COMPLETED', 'FAILED', 'CANCELLED')"
                )
                self._connection.execute(
                    "CREATE INDEX IF NOT EXISTS robot_payload_history ON robot_observations(sequence) "
                    "WHERE json_extract(evidence_json, '$.payload_state') IN ('LOADED', 'UNKNOWN')"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS payload_evidence ("
                    "mission_id TEXT PRIMARY KEY REFERENCES missions(mission_id), "
                    "carrier_robot_id TEXT NOT NULL, ownership TEXT NOT NULL, "
                    "conflicted INTEGER NOT NULL CHECK (conflicted IN (0,1)))"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS journal_schema (name TEXT PRIMARY KEY, version INTEGER NOT NULL)"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS payload_carrier_claims ("
                    "mission_id TEXT NOT NULL REFERENCES missions(mission_id), "
                    "robot_id TEXT NOT NULL, PRIMARY KEY(mission_id, robot_id))"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS recovery_physical_claims ("
                    "mission_id TEXT NOT NULL REFERENCES missions(mission_id), "
                    "robot_id TEXT NOT NULL, role TEXT NOT NULL, evidence_id TEXT NOT NULL, "
                    "last_payload TEXT, PRIMARY KEY(mission_id, robot_id, role))"
                )
                self._connection.execute(
                    "CREATE TABLE IF NOT EXISTS resource_claim_resolutions ("
                    "resource_id TEXT NOT NULL, robot_id TEXT NOT NULL, mission_id TEXT NOT NULL, "
                    "evidence_id TEXT NOT NULL, PRIMARY KEY(resource_id,robot_id,mission_id,evidence_id))"
                )
                version = self._connection.execute(
                    "SELECT version FROM journal_schema WHERE name='payload_evidence'"
                ).fetchone()
                if version is None or version[0] < 2:
                    # Task 13/early Task 14 journals need one ordered backfill.
                    # The marker and summaries commit together, so failed migration
                    # retries safely. Later restarts never revisit telemetry.
                    for row in self._connection.execute(
                        "SELECT robot_id, evidence_json FROM robot_observations "
                        "WHERE json_extract(evidence_json, '$.payload_state') IN ('LOADED', 'UNKNOWN') "
                        "ORDER BY sequence"
                    ):
                        self._merge_payload_evidence(
                            row["robot_id"], json.loads(row["evidence_json"])
                        )
                    self._connection.execute(
                        "INSERT INTO journal_schema VALUES ('payload_evidence', 2) "
                        "ON CONFLICT(name) DO UPDATE SET version=excluded.version"
                    )
                if (
                    self._connection.execute(
                        "SELECT version FROM journal_schema WHERE name='physical_claims'"
                    ).fetchone()
                    is None
                ):
                    for record in self.load_active():
                        mission_id = record.request.mission_id
                        for robot_id in self.recovery_carriers(mission_id):
                            self._physical_claim(mission_id, robot_id, "CARRIER")
                        # The mission/event indexes bound this one-time migration
                        # to unresolved work. Pose telemetry is not replayed.
                        for event in self.events(mission_id):
                            owner = event.detail.get("assigned_robot_id")
                            if owner and event.state in (
                                MissionState.ASSIGNED,
                                MissionState.EXECUTING,
                            ):
                                self._physical_claim(
                                    mission_id,
                                    owner,
                                    "EXECUTOR",
                                    evidence_id=f"executor-event-{event.sequence}",
                                )
                        if record.assigned_robot_id and record.state in (
                            MissionState.ASSIGNED,
                            MissionState.EXECUTING,
                        ):
                            self._physical_claim(
                                mission_id, record.assigned_robot_id, "EXECUTOR"
                            )
                    self._connection.execute(
                        "INSERT INTO journal_schema VALUES ('physical_claims', 1)"
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
        cursor = self._connection.execute(
            "INSERT INTO mission_events "
            "(mission_id, previous_state, state, detail_json, timestamp) "
            "VALUES (?, ?, ?, ?, ?)",
            (mission_id, previous_state, state, detail_json, now),
        )
        return cursor.lastrowid

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
        if record.assigned_robot_id and record.state in (
            MissionState.ASSIGNED,
            MissionState.EXECUTING,
        ):
            self._physical_claim(
                record.request.mission_id, record.assigned_robot_id, "EXECUTOR"
            )
        sequence = self._event(
            record.request.mission_id, record.state, target, detail_json, now
        )
        if assigned_robot_id and target in (
            MissionState.ASSIGNED,
            MissionState.EXECUTING,
        ):
            new_execution = (
                record.assigned_robot_id != assigned_robot_id
                or record.state not in (MissionState.ASSIGNED, MissionState.EXECUTING)
            )
            self._physical_claim(
                record.request.mission_id,
                assigned_robot_id,
                "EXECUTOR",
                evidence_id=f"executor-event-{sequence}" if new_execution else None,
            )
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
            "SELECT * FROM missions WHERE state NOT IN ('COMPLETED', 'FAILED', 'CANCELLED') "
            "ORDER BY created_at, mission_id"
        ).fetchall()
        return tuple(_record(row) for row in rows)

    def load_recovery(self) -> tuple[MissionRecord, ...]:
        """Merge durable correlated carrying evidence, never replay pose authority.

        An observation can commit before its corresponding mission transition
        fails. EMPTY is not an ordered delivery proof and cannot erase carrying
        evidence. Completed delivery rows are excluded by load_active().
        """
        recovered = []
        for record in self.load_active():
            evidence = self._connection.execute(
                "SELECT * FROM payload_evidence WHERE mission_id = ?",
                (record.request.mission_id,),
            ).fetchone()
            if evidence is not None:
                carrier = evidence["carrier_robot_id"]
                strong_owner = (
                    record.assigned_robot_id is not None
                    and record.payload_ownership
                    in (PayloadOwnership.PICKED_UP, PayloadOwnership.UNKNOWN)
                )
                conflict = evidence["conflicted"] or (
                    strong_owner and record.assigned_robot_id != carrier
                )
                ownership = (
                    PayloadOwnership.UNKNOWN
                    if conflict or record.payload_ownership == PayloadOwnership.UNKNOWN
                    else PayloadOwnership(evidence["ownership"])
                )
                record = replace(
                    record,
                    assigned_robot_id=record.assigned_robot_id
                    if strong_owner
                    else carrier,
                    payload_ownership=ownership,
                )
            recovered.append(record)
        return tuple(recovered)

    def recovery_carriers(self, mission_id):
        """Return compact unresolved carrier identities, never physical authority."""
        return tuple(
            row[0]
            for row in self._connection.execute(
                "SELECT c.robot_id FROM payload_carrier_claims c JOIN missions m USING(mission_id) "
                "WHERE c.mission_id = ? AND m.state NOT IN ('COMPLETED','FAILED','CANCELLED') "
                "ORDER BY c.robot_id",
                (mission_id,),
            )
        )

    def _physical_claim(
        self, mission_id, robot_id, role, evidence_id=None, payload=None
    ):
        row = self._connection.execute(
            "SELECT evidence_id,last_payload FROM recovery_physical_claims "
            "WHERE mission_id=? AND robot_id=? AND role=?",
            (mission_id, robot_id, role),
        ).fetchone()
        if evidence_id is None:
            evidence_id = (
                row[0]
                if row is not None
                else "restart-work-" + secrets.token_urlsafe(32)
            )
        if (
            role == "CARRIER"
            and payload in ("LOADED", "UNKNOWN")
            and row is not None
            and row[1] == "EMPTY"
        ):
            evidence_id = "restart-work-" + secrets.token_urlsafe(32)
        self._connection.execute(
            "INSERT INTO recovery_physical_claims VALUES (?,?,?,?,?) "
            "ON CONFLICT(mission_id,robot_id,role) DO UPDATE SET "
            "evidence_id=excluded.evidence_id,last_payload=COALESCE(excluded.last_payload,last_payload)",
            (mission_id, robot_id, role, evidence_id, payload),
        )

    def recovery_physical_claims(self, mission_id):
        """Separate physical executors/observed owners from payload carrier choice."""
        return tuple(
            (row[0], row[1])
            for row in self._connection.execute(
                "SELECT c.robot_id,c.evidence_id FROM recovery_physical_claims c "
                "JOIN missions m USING(mission_id) WHERE c.mission_id=? "
                "AND m.state NOT IN ('COMPLETED','FAILED','CANCELLED') ORDER BY c.robot_id,c.role",
                (mission_id,),
            )
        )

    def _merge_payload_evidence(self, robot_id, evidence):
        """Fold ordered carrying claims into one durable summary per mission.

        EMPTY cannot prove delivery. The first carrier is retained and any
        conflicting carrier makes uncertainty sticky, including later LOADED
        claims from the first carrier. Only mission delivery transitions close
        work; telemetry never manufactures a delivery or terminal transition.
        """
        payload = evidence.get("payload_state")
        mission_id = evidence.get("mission_id")
        if payload == "EMPTY" and mission_id:
            self._connection.execute(
                "UPDATE recovery_physical_claims SET last_payload='EMPTY' "
                "WHERE mission_id=? AND robot_id=? AND role='CARRIER'",
                (mission_id, robot_id),
            )
        if payload not in ("LOADED", "UNKNOWN") or not mission_id:
            return
        record = self._find(mission_id)
        if record is None:
            return
        row = self._connection.execute(
            "SELECT * FROM payload_evidence WHERE mission_id = ?", (mission_id,)
        ).fetchone()
        carrier = (
            row["carrier_robot_id"]
            if row is not None
            else record.assigned_robot_id
            if record.assigned_robot_id is not None
            and record.payload_ownership
            in (PayloadOwnership.PICKED_UP, PayloadOwnership.UNKNOWN)
            else robot_id
        )
        conflict = carrier != robot_id or (row is not None and row["conflicted"])
        uncertain = (
            conflict
            or payload == "UNKNOWN"
            or record.payload_ownership == PayloadOwnership.UNKNOWN
            or row is not None
            and row["ownership"] == "UNKNOWN"
        )
        ownership = (
            PayloadOwnership.UNKNOWN if uncertain else PayloadOwnership.PICKED_UP
        )
        self._connection.execute(
            "INSERT INTO payload_evidence VALUES (?, ?, ?, ?) "
            "ON CONFLICT(mission_id) DO UPDATE SET ownership=excluded.ownership, conflicted=excluded.conflicted",
            (mission_id, carrier, ownership, int(bool(conflict))),
        )
        for owner in sorted({robot_id, carrier}):
            self._connection.execute(
                "INSERT INTO payload_carrier_claims VALUES (?, ?) ON CONFLICT DO NOTHING",
                (mission_id, owner),
            )
            self._physical_claim(
                mission_id,
                owner,
                "CARRIER",
                payload=payload if owner == robot_id else None,
            )

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

    def claim_resolved(self, key):
        return (
            self._connection.execute(
                "SELECT 1 FROM resource_claim_resolutions WHERE resource_id=? AND robot_id=? "
                "AND mission_id=? AND evidence_id=?",
                (key.resource_id, key.robot_id, key.mission_id, key.lease_id),
            ).fetchone()
            is not None
        )

    def _save_clearances(self, clearances):
        for key in clearances:
            self._connection.execute(
                "INSERT INTO resource_claim_resolutions VALUES (?,?,?,?) ON CONFLICT DO NOTHING",
                (key.resource_id, key.robot_id, key.mission_id, key.lease_id),
            )

    def _save_resources(self, snapshots, now):
        from fleet_manager.resources import Lease

        for snapshot in snapshots:
            evidence = _json(asdict(snapshot))
            row = self._connection.execute(
                "SELECT evidence_json FROM resource_snapshots WHERE resource_id = ?",
                (snapshot.resource_id,),
            ).fetchone()
            if row is not None and row[0] == evidence:
                continue
            if row is not None:
                previous = json.loads(row[0])
                old_claims = previous.get("former_leases") or (
                    (previous["former_lease"],) if previous.get("former_lease") else ()
                )
                fields = ("robot_id", "mission_id", "resource_id", "lease_id")
                current_claims = snapshot.former_leases or (
                    (snapshot.former_lease,)
                    if snapshot.former_lease is not None
                    else ()
                )
                remaining = {
                    tuple(getattr(lease, field) for field in fields)
                    for lease in current_claims
                }
                self._save_clearances(
                    Lease(**claim)
                    for claim in old_claims
                    if tuple(claim[field] for field in fields) not in remaining
                )
            self._connection.execute(
                "INSERT INTO resource_snapshots VALUES (?, ?) "
                "ON CONFLICT(resource_id) DO UPDATE SET evidence_json=excluded.evidence_json",
                (snapshot.resource_id, evidence),
            )
            self._connection.execute(
                "INSERT INTO resource_events(resource_id, evidence_json, timestamp) VALUES (?, ?, ?)",
                (snapshot.resource_id, evidence, now),
            )

    def save_resources(self, snapshots, now, clearances=()):
        """Persist authority evidence before a service can report a grant."""
        now = _timestamp(now)
        with self._transaction():
            self._save_resources(snapshots, now)
            self._save_clearances(clearances)

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
            value["former_leases"] = tuple(
                Lease(**item) for item in value.get("former_leases", ())
            )
            values.append(ResourceSnapshot(**value))
        return tuple(values)

    def record_observation(self, robot, received_at, source_time_ns=None):
        evidence = _json(asdict(robot))
        with self._transaction():
            cursor = self._connection.execute(
                "INSERT INTO robot_observations(robot_id, evidence_json, source_time_ns, received_at) "
                "VALUES (?, ?, ?, ?)",
                (robot.robot_id, evidence, source_time_ns, _timestamp(received_at)),
            )
            self._merge_payload_evidence(robot.robot_id, json.loads(evidence))
            if robot.mission_id and self._find(robot.mission_id) is not None:
                self._physical_claim(
                    robot.mission_id,
                    robot.robot_id,
                    "OBSERVED",
                    evidence_id=f"observation-{cursor.lastrowid}",
                )

    def record_contact(self, robot_id, contact, received_at, source_time_ns):
        with self._transaction():
            self._connection.execute(
                "INSERT INTO contact_observations(robot_id, contact, source_time_ns, received_at) VALUES (?, ?, ?, ?)",
                (robot_id, int(contact), source_time_ns, _timestamp(received_at)),
            )

    def reconcile(self, decisions, snapshots, now, clearances=()):
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
            self._save_clearances(clearances)

    def close(self) -> None:
        self._connection.close()
