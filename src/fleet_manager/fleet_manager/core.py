# SPDX-License-Identifier: Apache-2.0
"""Durable, ROS-free orchestration of fleet missions."""

from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from typing import Mapping

from fleet_manager.dispatcher import Dispatcher
from fleet_manager.journal import (
    MissionEvent,
    MissionJournal,
    MissionRecord,
    MissionState,
    PayloadOwnership,
)
from fleet_manager.models import (
    AssignmentDecision,
    CostEstimate,
    MissionRequest,
    RobotHealth,
    RobotSnapshot,
    robot_is_ready,
)
from fleet_manager.registry import RobotRegistry
from fleet_manager.resources import ResourceManager


@dataclass(frozen=True)
class RobotMissionFeedback:
    robot_id: str
    assignment_id: int
    payload_ownership: PayloadOwnership = PayloadOwnership.NOT_PICKED_UP
    stage: str = ""


@dataclass(frozen=True)
class RobotMissionResult:
    robot_id: str
    assignment_id: int
    success: bool
    error_code: str = ""
    message: str = ""
    payload_ownership: PayloadOwnership | None = None
    cancelled: bool = False
    recovery_required: bool = False


@dataclass(frozen=True)
class RobotCancellationAck:
    robot_id: str
    assignment_id: int
    accepted: bool


@dataclass(frozen=True)
class ChargingRequest:
    robot_id: str
    dock_id: str
    target_percent: float
    generation: int
    state: str = "CHARGE_QUEUED"


@dataclass(frozen=True)
class ReconciliationSnapshot:
    robots: tuple[RobotSnapshot, ...]


@dataclass(frozen=True)
class ReconcileResult:
    records: tuple[MissionRecord, ...]
    schedulable_mission_ids: tuple[str, ...]
    recovery_required_mission_ids: tuple[str, ...]


_CLOSED = frozenset(
    (
        MissionState.COMPLETED,
        MissionState.FAILED,
        MissionState.CANCELLED,
        MissionState.RECOVERY_REQUIRED,
    )
)
_RUNNING = frozenset((MissionState.ASSIGNED, MissionState.EXECUTING))
_SCHEDULABLE = frozenset((MissionState.QUEUED, MissionState.REASSIGNING))


def _ownership(record: MissionRecord, observed: PayloadOwnership) -> PayloadOwnership:
    observed = PayloadOwnership(observed)
    if record.payload_ownership in (
        PayloadOwnership.PICKED_UP,
        PayloadOwnership.DELIVERED,
    ):
        return record.payload_ownership
    if (
        record.payload_ownership == PayloadOwnership.UNKNOWN
        and observed == PayloadOwnership.NOT_PICKED_UP
    ):
        return PayloadOwnership.UNKNOWN
    return observed


def _time(now: float) -> None:
    if type(now) not in (int, float) or not math.isfinite(now):
        raise ValueError("now must be a finite number")


class FleetCore:
    """Serialize calls on the journal's owning thread, including node callbacks.

    Only returned assignment decisions authorize sending a new robot goal.
    The event sequence is an internal goal-generation fence. Adapters attach it
    through goal callback closures; it does not require a ROS interface field.
    Running cancellation requests and acknowledgements are durable audit events,
    not terminal execution outcomes. Fenced robot results or offline recovery
    finalize them. Duplicate operations return snapshots without issuing goals.
    """

    def __init__(
        self,
        dispatcher: Dispatcher,
        registry: RobotRegistry,
        resources: ResourceManager,
        journal: MissionJournal,
    ):
        self._dispatcher = dispatcher
        self._registry = registry
        self._resources = resources
        self._journal = journal
        self._charging = {}
        self._charge_generation = 0

    def charging_snapshot(self):
        return tuple(self._charging.values())

    def queue_charging(self, policy, now, reserved=()):
        """Reserve low idle robots before any mission assignment in this tick.

        CHARGE_QUEUED is a central scheduling state, not a public RobotMode.
        Fresh DOCKING/CHARGING observations after restart remain ineligible;
        this process never reuses their former dock authority.
        """
        _time(now)
        occupied = set(reserved) | {
            record.assigned_robot_id
            for record in self._journal.load_active()
            if record.assigned_robot_id is not None
        }
        queued = []
        from fleet_manager.energy import EnergyPolicy

        energy = EnergyPolicy(policy)
        for robot in sorted(
            self._registry.eligible(now), key=lambda item: item.robot_id
        ):
            if (
                robot.robot_id in occupied
                or robot.robot_id in self._charging
                or not robot_is_ready(robot)
                or robot.payload_state != "EMPTY"
                or robot.mission_id
                or not energy.should_charge(robot)
            ):
                continue
            self._charge_generation += 1
            request = ChargingRequest(
                robot.robot_id,
                policy.dock_id,
                policy.charge_until_percent,
                self._charge_generation,
            )
            self._charging[robot.robot_id] = request
            queued.append(request)
        return tuple(queued)

    def charging_feedback(self, robot_id, generation, state):
        current = self._charging.get(robot_id)
        if current is not None and current.generation == generation:
            mode = "CHARGING" if state == "CHARGING" else "DOCKING"
            if current.state != "RECOVERY_REQUIRED":
                self._charging[robot_id] = replace(current, state=mode)

    def charging_result(self, robot_id, generation, success, *, cancelled=False):
        current = self._charging.get(robot_id)
        if current is not None and current.generation == generation:
            if success or cancelled:
                del self._charging[robot_id]
            else:
                self._charging[robot_id] = replace(current, state="RECOVERY_REQUIRED")

    def submit(self, request: MissionRequest, now: float) -> MissionRecord:
        _time(now)
        canonical = json.dumps(
            asdict(request), sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        payload_hash = hashlib.sha256(canonical).hexdigest()
        return self._journal.register(request, payload_hash, now).record

    def assign(
        self, mission_id: str, estimates: Mapping[str, CostEstimate], now: float
    ) -> AssignmentDecision:
        _time(now)
        record = self._journal.get(mission_id)
        if record.state not in _SCHEDULABLE:
            return AssignmentDecision(None, "mission is not schedulable")
        if record.state == MissionState.REASSIGNING:
            events = self._journal.events(mission_id)
            assignment = next(
                (
                    event
                    for event in reversed(events)
                    if event.state == MissionState.ASSIGNED
                    and event.detail.get("assigned_robot_id")
                ),
                None,
            )
            lost = next(
                (
                    event
                    for event in reversed(events)
                    if event.state == MissionState.REASSIGNING
                ),
                None,
            )
            if assignment is None or lost is None:
                return AssignmentDecision(
                    None, "former assignment requires reconciliation"
                )
            former = assignment.detail["assigned_robot_id"]
            old = self._registry.get(former, now)
            if not (
                self._registry.observed_after(former, lost.timestamp)
                and robot_is_ready(old)
                and old.payload_state == "EMPTY"
                and not old.mission_id
            ):
                return AssignmentDecision(
                    None, "former robot stop and empty-payload proof required"
                )
            for resource in self._resources.snapshot(now):
                if resource.reconciliation_required and (
                    resource.resource_id
                    in (record.request.pickup_station, record.request.dropoff_station)
                    or any(
                        lease.robot_id == former or lease.mission_id == mission_id
                        for lease in resource.former_leases
                    )
                ):
                    return AssignmentDecision(
                        None, "former resource clearance required"
                    )
        reserved = {
            active.assigned_robot_id
            for active in self._journal.load_active()
            if active.assigned_robot_id is not None
        }
        robots = tuple(
            robot
            for robot in self._registry.eligible(now)
            if robot.robot_id not in reserved and robot.robot_id not in self._charging
        )
        decision = self._dispatcher.choose(record.request, robots, estimates)
        if decision.robot_id is None:
            return decision
        if record.state == MissionState.QUEUED:
            self._journal.transition(
                mission_id, record.state, MissionState.ASSIGNING, {}, now
            )
        self._journal.assign(mission_id, decision.robot_id, now)
        assignment_id = self._journal.events(mission_id)[-1].sequence
        return AssignmentDecision(decision.robot_id, decision.reason, assignment_id)

    def _assignment(self, record: MissionRecord) -> MissionEvent | None:
        assignment = next(
            (
                event
                for event in reversed(self._journal.events(record.request.mission_id))
                if event.state == MissionState.ASSIGNED
                and event.previous_state != MissionState.ASSIGNED
            ),
            None,
        )
        if (
            assignment is None
            or assignment.previous_state
            not in (MissionState.ASSIGNING, MissionState.REASSIGNING)
            or assignment.detail.get("assigned_robot_id") != record.assigned_robot_id
        ):
            return None
        return assignment

    def _matches(
        self,
        record: MissionRecord,
        callback: RobotMissionFeedback | RobotMissionResult | RobotCancellationAck,
    ) -> bool:
        if (
            record.state not in _RUNNING
            or record.assigned_robot_id != callback.robot_id
        ):
            return False
        assignment = self._assignment(record)
        return assignment is not None and assignment.sequence == callback.assignment_id

    def _cancellation_requested(self, record: MissionRecord) -> bool:
        return any(
            event.detail.get("cancellation_requested") is True
            for event in self._journal.events(record.request.mission_id)
        )

    def request_cancellation(self, mission_id: str, now: float) -> MissionRecord:
        """Persist intent before transport cancellation, retaining payload evidence."""
        _time(now)
        record = self._journal.get(mission_id)
        if record.state not in _RUNNING:
            return self.cancel(mission_id, now)
        if self._cancellation_requested(record):
            return record
        return self._journal.transition(
            mission_id,
            record.state,
            record.state,
            {"cancellation_requested": True},
            now,
        )

    def record_cancellation_acknowledgement(
        self, mission_id: str, acknowledgement: RobotCancellationAck, now: float
    ) -> MissionRecord:
        """An accepted CancelGoal request does not prove execution has stopped."""
        _time(now)
        record = self._journal.get(mission_id)
        if not self._matches(
            record, acknowledgement
        ) or not self._cancellation_requested(record):
            return record
        previous = next(
            (
                event.detail["cancellation_acknowledged"]
                for event in reversed(self._journal.events(mission_id))
                if "cancellation_acknowledged" in event.detail
                and event.detail.get("assignment_id") == acknowledgement.assignment_id
            ),
            None,
        )
        if previous is acknowledgement.accepted:
            return record
        return self._journal.transition(
            mission_id,
            record.state,
            record.state,
            {
                "cancellation_acknowledged": acknowledgement.accepted,
                "assignment_id": acknowledgement.assignment_id,
            },
            now,
        )

    def record_robot_feedback(
        self, mission_id: str, feedback: RobotMissionFeedback, now: float
    ) -> MissionRecord:
        _time(now)
        record = self._journal.get(mission_id)
        if not self._matches(record, feedback):
            return record
        ownership = _ownership(record, feedback.payload_ownership)
        if (
            record.state == MissionState.EXECUTING
            and ownership == record.payload_ownership
        ):
            return record
        return self._journal.transition(
            mission_id,
            record.state,
            MissionState.EXECUTING,
            {"payload_ownership": ownership, "stage": feedback.stage},
            now,
        )

    @staticmethod
    def _result(record, target, success, error_code, message):
        return {
            "success": success,
            "final_state": target.value,
            "assigned_robot_id": record.assigned_robot_id,
            "error_code": error_code,
            "message": message,
        }

    def record_robot_result(
        self, mission_id: str, result: RobotMissionResult, now: float
    ) -> MissionRecord:
        _time(now)
        record = self._journal.get(mission_id)
        if not self._matches(record, result):
            return record
        ownership = record.payload_ownership
        if result.payload_ownership is not None:
            ownership = _ownership(record, result.payload_ownership)
        if (
            not result.success
            and not result.recovery_required
            and result.error_code == "ROBOT_OFFLINE"
            and self._registry.get(result.robot_id, now).health == RobotHealth.OFFLINE
        ):
            if ownership != record.payload_ownership:
                record = self._journal.transition(
                    mission_id,
                    record.state,
                    record.state,
                    {"payload_ownership": ownership},
                    now,
                )
            self.handle_robot_offline(result.robot_id, now)
            return self._journal.get(mission_id)
        success = result.success and not result.recovery_required
        target = (
            MissionState.RECOVERY_REQUIRED
            if result.recovery_required
            else MissionState.COMPLETED
            if success
            else MissionState.FAILED
        )
        if (
            not success
            and not result.recovery_required
            and self._cancellation_requested(record)
        ):
            if ownership != PayloadOwnership.NOT_PICKED_UP:
                target = MissionState.RECOVERY_REQUIRED
            elif result.cancelled:
                target = MissionState.CANCELLED
        if success:
            ownership = PayloadOwnership.DELIVERED
        return self._journal.transition(
            mission_id,
            record.state,
            target,
            {
                "payload_ownership": ownership,
                "result": self._result(
                    record,
                    target,
                    success,
                    result.error_code or ("CANCELLED" if result.cancelled else ""),
                    result.message,
                ),
            },
            now,
        )

    def cancel(self, mission_id: str, now: float) -> MissionRecord:
        """Finalize cancellation when no running transport outcome is outstanding.

        Transport adapters use request_cancellation for a live assignment and
        finalize it through a fenced robot result or offline recovery instead.
        """
        _time(now)
        record = self._journal.get(mission_id)
        if record.state in _CLOSED:
            return record
        target = (
            MissionState.CANCELLED
            if record.payload_ownership == PayloadOwnership.NOT_PICKED_UP
            else MissionState.RECOVERY_REQUIRED
        )
        return self._journal.transition(
            mission_id,
            record.state,
            target,
            {
                "result": self._result(
                    record, target, False, "CANCELLED", "Cancellation requested"
                )
            },
            now,
        )

    def reject(
        self, mission_id: str, error_code: str, message: str, now: float
    ) -> MissionRecord:
        """Commit an explicit scheduling rejection before returning its result."""
        _time(now)
        record = self._journal.get(mission_id)
        if record.state not in _SCHEDULABLE:
            return record
        return self._journal.transition(
            mission_id,
            record.state,
            MissionState.FAILED,
            {
                "result": self._result(
                    record, MissionState.FAILED, False, error_code, message
                )
            },
            now,
        )

    def handle_robot_offline(
        self, robot_id: str, now: float
    ) -> tuple[MissionRecord, ...]:
        _time(now)
        records = []
        for record in self._journal.load_active():
            if record.assigned_robot_id != robot_id or record.state not in _RUNNING:
                continue
            target = (
                MissionState.REASSIGNING
                if record.payload_ownership == PayloadOwnership.NOT_PICKED_UP
                else MissionState.RECOVERY_REQUIRED
            )
            detail = {"reason": "robot offline"}
            if self._cancellation_requested(record):
                # Loss cannot confirm that a pending cancellation stopped the
                # robot or that its last empty payload observation is still true.
                target = MissionState.RECOVERY_REQUIRED
                if record.payload_ownership == PayloadOwnership.NOT_PICKED_UP:
                    detail["payload_ownership"] = PayloadOwnership.UNKNOWN
                detail["result"] = self._result(
                    record,
                    target,
                    False,
                    "ROBOT_OFFLINE",
                    "Robot offline during cancellation",
                )
            elif target == MissionState.REASSIGNING:
                detail["assigned_robot_id"] = None
            records.append(
                self._journal.transition(
                    record.request.mission_id, record.state, target, detail, now
                )
            )
        # Journal commits precede resource cleanup. A failed write cannot emit a goal.
        self._resources.release_owner(robot_id, now)
        return tuple(records)

    def reconcile(
        self, observations: ReconciliationSnapshot, now: float
    ) -> ReconcileResult:
        _time(now)
        robots = {robot.robot_id: robot for robot in observations.robots}
        duplicates = {
            robot_id
            for robot_id in robots
            if sum(robot.robot_id == robot_id for robot in observations.robots) > 1
        }
        for record in self._journal.load_active():
            mission_id = record.request.mission_id
            scheduling = record.state in _SCHEDULABLE or record.state in (
                MissionState.RECEIVED,
                MissionState.ASSIGNING,
            )
            claims = tuple(
                robot for robot in observations.robots if robot.mission_id == mission_id
            )
            if scheduling and (
                record.payload_ownership != PayloadOwnership.NOT_PICKED_UP
                or record.assigned_robot_id is not None
                or claims
            ):
                self._journal.transition(
                    mission_id,
                    record.state,
                    MissionState.RECOVERY_REQUIRED,
                    {"reason": "uncertain scheduling evidence"},
                    now,
                )
            elif record.state in (MissionState.RECEIVED, MissionState.ASSIGNING):
                self._journal.transition(
                    mission_id,
                    record.state,
                    MissionState.QUEUED,
                    {"assigned_robot_id": None, "reason": "interrupted scheduling"},
                    now,
                )
            elif record.state in _RUNNING:
                robot = robots.get(record.assigned_robot_id)
                payload = {
                    PayloadOwnership.NOT_PICKED_UP: "EMPTY",
                    PayloadOwnership.PICKED_UP: "LOADED",
                }.get(record.payload_ownership)
                certain = (
                    robot is not None
                    and robot.health == RobotHealth.ONLINE
                    and robot.mission_id == mission_id
                    and robot.mode in ("RESERVED", "EXECUTING", "WAITING_FOR_RESOURCE")
                    and payload is not None
                    and robot.payload_state == payload
                    and not robot.fault
                    and robot.robot_id not in duplicates
                    and len(claims) == 1
                    and self._assignment(record) is not None
                )
                if not certain:
                    self._journal.transition(
                        mission_id,
                        record.state,
                        MissionState.RECOVERY_REQUIRED,
                        {"reason": "uncertain restart evidence"},
                        now,
                    )
        records = self._journal.load_active()
        return ReconcileResult(
            records,
            tuple(
                record.request.mission_id
                for record in records
                if record.state in _SCHEDULABLE
            ),
            tuple(
                record.request.mission_id
                for record in records
                if record.state == MissionState.RECOVERY_REQUIRED
            ),
        )
