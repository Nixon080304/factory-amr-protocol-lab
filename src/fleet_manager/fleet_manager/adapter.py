# SPDX-License-Identifier: Apache-2.0
"""Serialized fleet orchestration with asynchronous transport boundaries.

Transport callbacks only enqueue work. The owning executor thread drains that
work in tick(), keeping the journal and FleetCore on their creating thread.
"""

from dataclasses import dataclass, field, replace
from functools import wraps
import math
import queue
import secrets
import sqlite3
import time

from fleet_manager.core import (
    FleetCore,
    RobotCancellationAck,
    RobotMissionFeedback,
    RobotMissionResult,
)
from fleet_manager.config import Pose2D
from fleet_manager.dispatcher import Dispatcher
from fleet_manager.energy import EnergyPolicy
from fleet_manager.journal import MissionState, PayloadOwnership
from fleet_manager.models import RobotHealth, RobotMode, RobotSnapshot, robot_is_ready
from fleet_manager.registry import RobotRegistry
from fleet_manager.recovery import RecoveryPlanner
from fleet_manager.resources import Lease, LeaseKey, LeaseRequest, ResourceManager

PATH_PENDING_GRACE_SEC = 3.0


def _storage_boundary(method):
    """Fence a failed journal mutation without changing its error semantics."""

    @wraps(method)
    def guarded(self, *args, **kwargs):
        try:
            return method(self, *args, **kwargs)
        except sqlite3.Error:
            self.state = "STORAGE_FAILED"
            raise

    return guarded


@dataclass(frozen=True)
class RobotFeedback:
    state: str
    detail: str = ""
    progress: float = 0.0


@dataclass(frozen=True)
class RobotReply:
    success: bool
    error_code: str = ""
    message: str = ""
    cancelled: bool = False
    recovery_required: bool = False


@dataclass
class _Round:
    deadline: float
    pending: set[str]
    estimates: dict = field(default_factory=dict)
    cleanups: dict = field(default_factory=dict)


@dataclass
class _Flight:
    robot_id: str
    assignment_id: int
    handle: object = None
    cancelling: bool = False
    retired: bool = False


def robot_endpoints(robot):
    base = robot.namespace.rstrip("/") + "/factory/"
    return {
        "action": base + "execute_mission",
        "cost": base + "estimate_mission_cost",
        "state": base + "robot_state",
        "dock": base + "dock_robot",
    }


def robot_snapshot(robot_id, message):
    if message.robot_id != robot_id:
        raise ValueError("robot state identity does not match its configured topic")
    pose, q = message.pose, message.pose.orientation
    valid = (
        bool(message.frame_id)
        and all(
            math.isfinite(value)
            for value in (
                pose.position.x,
                pose.position.y,
                q.x,
                q.y,
                q.z,
                q.w,
                message.battery_percent,
            )
        )
        and abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) <= 0.01
        and 0 <= message.battery_percent <= 100
    )
    try:
        mode = RobotMode(message.mode)
    except ValueError:
        mode, valid = RobotMode.UNHEALTHY, False
    yaw = (
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if valid
        else 0.0
    )
    health = RobotHealth.ONLINE
    if mode == RobotMode.OFFLINE:
        health = RobotHealth.OFFLINE
    elif not valid or message.health_detail or mode == RobotMode.UNHEALTHY:
        health = RobotHealth.UNHEALTHY
    return RobotSnapshot(
        robot_id,
        mode,
        Pose2D(pose.position.x, pose.position.y, yaw) if valid else None,
        message.battery_percent if valid else None,
        message.payload_state,
        message.mission_id or None,
        health,
        health_detail=message.health_detail,
    )


class FleetAdapter:
    """Reserve robot goals until execution completion or offline recovery.

    The reservation remains separate from journal terminal state: cancelling a
    mission must not make its still-running robot available for another goal.
    """

    def __init__(
        self,
        config,
        journal,
        transport,
        *,
        clock=time.monotonic,
        reconciliation_timeout=5.0,
    ):
        self.config, self.journal, self.transport, self.clock = (
            config,
            journal,
            transport,
            clock,
        )
        self.registry = RobotRegistry(tuple(robot.robot_id for robot in config.robots))
        self.resources = ResourceManager(config.resources)
        self.resources.restore_evidence(journal.load_resources())
        self.core = FleetCore(
            Dispatcher(EnergyPolicy(config.energy)),
            self.registry,
            self.resources,
            journal,
        )
        self._callbacks = queue.SimpleQueue()
        self._rounds = {}
        self._retry_at = {}
        self._path_pending_since = {}
        self._flights = {}
        self._dock_flights = {}
        self._dock_seen = set()
        self._dock_restart_unsafe = set()
        self._dock_completed = set()
        if not math.isfinite(reconciliation_timeout) or reconciliation_timeout <= 0:
            raise ValueError("reconciliation_timeout must be positive and finite")
        self.state = "RECONCILING"
        self.startup_wall_ns = time.time_ns()
        self._reconciliation_deadline = self.clock() + reconciliation_timeout
        self._startup_observed = set()
        self._source_times = {}
        self._contacts = {}

    def _reconcile_startup(self, now):
        if self.state != "RECONCILING":
            return
        if (
            len(self._startup_observed) < len(self.config.robots)
            and now < self._reconciliation_deadline
        ):
            return
        policy = RecoveryPlanner(self.config)
        robots = {
            robot.robot_id: self.registry.get(robot.robot_id, now)
            for robot in self.config.robots
        }
        self._fence_observed_resources(policy, robots, now)
        self._clear_resources(policy, robots, now)
        snapshots = self.resources.snapshot(now)
        for resource in snapshots:
            if resource.resource_id not in self.config.docks:
                continue
            for robot_id, robot in tuple(robots.items()):
                dock = self.config.docks[resource.resource_id]
                inside = (
                    robot.pose is not None
                    and math.hypot(
                        robot.pose.x - dock.charging_pose.x,
                        robot.pose.y - dock.charging_pose.y,
                    )
                    <= dock.arrival_tolerance
                )
                if inside or robot.mode in (RobotMode.DOCKING, RobotMode.CHARGING):
                    contact, receipt, _ = self._contacts.get(
                        robot_id, (None, -math.inf, 0)
                    )
                    mode = policy.dock_state(
                        robot,
                        resource,
                        contact=contact if now - receipt < 1.0 else None,
                    )
                    robots[robot_id] = replace(robot, mode=mode)
                    self.registry.observe(robots[robot_id], now)
        decisions = policy.plan(self.journal.load_active(), robots, snapshots)
        self.journal.reconcile(decisions, snapshots, now)
        self.state = "RUNNING"
        for decision in decisions:
            self._emit(self.journal.get(decision.mission_id))

    def _fence_observed_resources(self, policy, robots, now):
        for resource in self.resources.snapshot(now):
            if resource.lease is not None or resource.former_lease is not None:
                continue
            geometry = (
                resource.resource_id in self.config.traffic_bounds
                or resource.resource_id in self.config.resource_bounds
                or resource.resource_id in self.config.docks
            )
            for robot in robots.values():
                evidence = Lease(
                    robot.robot_id,
                    robot.mission_id or "restart-observation",
                    resource.resource_id,
                    secrets.token_urlsafe(32),
                    now,
                )
                if (
                    not policy.valid_pose(robot)
                    or geometry
                    and not policy.proves_outside(evidence, robot)
                ):
                    self.resources.quarantine_evidence(evidence)
                    break

    def _clear_resources(self, policy, robots, now):
        for resource in self.resources.snapshot(now):
            former = resource.former_lease
            if (
                former is not None
                and policy.proves_outside(former, robots.get(former.robot_id))
                and all(
                    policy.proves_outside(
                        replace(former, robot_id=robot.robot_id), robot
                    )
                    for robot in robots.values()
                )
            ):
                self.resources.clear_reconciliation(
                    LeaseKey(
                        former.robot_id,
                        former.mission_id,
                        former.resource_id,
                        former.lease_id,
                    ),
                    now,
                )
        for dock_id in tuple(self._dock_restart_unsafe):
            dock = self.config.docks[dock_id]
            margin = 2 * dock.robot_radius + dock.arrival_tolerance
            if all(
                policy.valid_pose(robot)
                and robot.mode == RobotMode.AVAILABLE
                and not robot.mission_id
                and robot.payload_state == "EMPTY"
                and all(
                    math.hypot(robot.pose.x - pose.x, robot.pose.y - pose.y) > margin
                    for pose in (dock.staging_pose, dock.charging_pose)
                )
                for robot in robots.values()
            ):
                self._dock_restart_unsafe.discard(dock_id)

    def submit(self, request):
        if self.state == "STORAGE_FAILED":
            raise sqlite3.OperationalError("fleet storage failed")
        if request.requested_robot_id and request.requested_robot_id not in {
            robot.robot_id for robot in self.config.robots
        }:
            raise ValueError("requested robot is not configured")
        try:
            record = self.core.submit(request, self.clock())
        except sqlite3.Error:
            self.state = "STORAGE_FAILED"
            raise
        self._emit(record)
        return record

    def observe_contact(self, robot_id, contact, *, source_time_ns):
        if robot_id not in {robot.robot_id for robot in self.config.robots}:
            raise ValueError("contact owner is not configured")
        if type(contact) is not bool:
            raise ValueError("dock contact must be bool")
        previous = self._contacts.get(robot_id, (None, 0, 0))[2]
        if (
            source_time_ns <= self.startup_wall_ns
            or source_time_ns <= previous
            or self.state == "STORAGE_FAILED"
        ):
            return
        now = self.clock()
        try:
            self.journal.record_contact(robot_id, contact, now, source_time_ns)
        except sqlite3.Error:
            self.state = "STORAGE_FAILED"
            return
        self._contacts[robot_id] = (contact, now, source_time_ns)

    @_storage_boundary
    def observe(self, snapshot, *, source_time_ns=None):
        now = self.clock()
        if source_time_ns is not None:
            if (
                source_time_ns <= self.startup_wall_ns
                or source_time_ns <= self._source_times.get(snapshot.robot_id, 0)
            ):
                return
            self._source_times[snapshot.robot_id] = source_time_ns
        if self.state == "STORAGE_FAILED":
            return
        try:
            self.journal.record_observation(snapshot, now, source_time_ns)
        except sqlite3.Error:
            self.state = "STORAGE_FAILED"
            return
        if (
            snapshot.robot_id in self._dock_flights
            and snapshot.mode == RobotMode.CHARGING
        ):
            for charge in self.core.charging_snapshot():
                if charge.robot_id != snapshot.robot_id:
                    continue
                resource = next(
                    value
                    for value in self.resources.snapshot(now)
                    if value.resource_id == charge.dock_id
                )
                contact, receipt, _ = self._contacts.get(
                    snapshot.robot_id, (None, -math.inf, 0)
                )
                mode = RecoveryPlanner(self.config).dock_state(
                    snapshot,
                    resource,
                    contact=contact if now - receipt < 1.0 else None,
                )
                snapshot = replace(snapshot, mode=mode)
        self.registry.observe(snapshot, now)
        self._startup_observed.add(snapshot.robot_id)
        if snapshot.pose is not None:
            self._dock_seen.add(snapshot.robot_id)
        if snapshot.robot_id not in self._dock_flights:
            for dock_id, dock in self.config.docks.items():
                inside = (
                    snapshot.pose is not None
                    and math.hypot(
                        snapshot.pose.x - dock.charging_pose.x,
                        snapshot.pose.y - dock.charging_pose.y,
                    )
                    <= dock.arrival_tolerance
                )
                delayed_completion = (
                    snapshot.robot_id in self._dock_completed
                    and snapshot.mode in (RobotMode.DOCKING, RobotMode.CHARGING)
                )
                if not delayed_completion and (
                    inside or snapshot.mode in (RobotMode.DOCKING, RobotMode.CHARGING)
                ):
                    # Observed former activity is evidence, never a fresh lease.
                    # Task 14's explicit reconciliation owns clearance.
                    self._dock_restart_unsafe.add(dock_id)
                    if inside and snapshot.mode == RobotMode.AVAILABLE:
                        snapshot = replace(snapshot, mode=RobotMode.RECOVERY_REQUIRED)
                        self.registry.observe(snapshot, now)
        if snapshot.payload_state in ("LOADED", "UNKNOWN"):
            ownership = (
                PayloadOwnership.PICKED_UP
                if snapshot.payload_state == "LOADED"
                else PayloadOwnership.UNKNOWN
            )
            for mission_id, flight in tuple(self._flights.items()):
                if (
                    flight.robot_id == snapshot.robot_id
                    and snapshot.mission_id == mission_id
                ):
                    record = self.core.record_robot_feedback(
                        mission_id,
                        RobotMissionFeedback(
                            flight.robot_id, flight.assignment_id, ownership
                        ),
                        now,
                    )
                    self._emit(record)
        try:
            self._reconcile_startup(now)
        except sqlite3.Error:
            self.state = "STORAGE_FAILED"

    def _enqueue(self, callback, *args):
        self._callbacks.put((callback, args))

    def _emit(self, record, state=None, detail="", progress=0.0):
        self.transport.publish(record, state or record.state.value, detail, progress)

    def _drain_callbacks(self, limit):
        for _ in range(limit):
            try:
                callback, args = self._callbacks.get_nowait()
            except queue.Empty:
                break
            callback(*args)

    def tick(self):
        if self.state == "STORAGE_FAILED":
            return
        try:
            self._reconcile_startup(self.clock())
            if self.state != "RUNNING":
                return
            self._tick()
            self.journal.save_resources(
                self.resources.snapshot(self.clock()), self.clock()
            )
        except sqlite3.Error:
            self.state = "STORAGE_FAILED"
            raise

    def _tick(self):
        self._drain_callbacks(200)
        now = self.clock()
        self.resources.expire(now)
        for robot in self.config.robots:
            if self.registry.get(robot.robot_id, now).health == RobotHealth.OFFLINE:
                for record in self.core.handle_robot_offline(robot.robot_id, now):
                    self._emit(record)
                for mission_id, flight in tuple(self._flights.items()):
                    if flight.robot_id == robot.robot_id:
                        flight.retired = True
                        del self._flights[mission_id]
                charge = self._dock_flights.get(robot.robot_id)
                if charge is not None:
                    self.core.charging_result(
                        robot.robot_id, charge.assignment_id, False
                    )
                    charge.retired = True
        self._clear_resources(
            RecoveryPlanner(self.config),
            {
                robot.robot_id: self.registry.get(robot.robot_id, now)
                for robot in self.config.robots
            },
            now,
        )
        self.journal.save_resources(self.resources.snapshot(now), now)
        reserved = {flight.robot_id for flight in self._flights.values()}
        self.core.queue_charging(self.config.energy, now, reserved)
        previous = {}
        for charge in self.core.charging_snapshot():
            predecessor = previous.get(charge.dock_id)
            if (
                charge.robot_id not in self._dock_flights
                and charge.state == "CHARGE_QUEUED"
                and predecessor is None
            ):
                # Serialize the whole cycle, including verified exit and release.
                # Reserving all queued robots still fences mission assignment.
                self._start_charging(charge)
            previous[charge.dock_id] = charge
        for record in self.journal.load_active():
            if record.state not in (MissionState.QUEUED, MissionState.REASSIGNING):
                continue
            mission_id = record.request.mission_id
            round_ = self._rounds.get(mission_id)
            if round_ is not None:
                if not round_.pending or now >= round_.deadline:
                    self._close_round(mission_id)
                    reserved = {flight.robot_id for flight in self._flights.values()}
                    estimates = {
                        robot_id: estimate
                        for robot_id, estimate in round_.estimates.items()
                        if robot_id not in reserved
                    }
                    decision = self.core.assign(mission_id, estimates, now)
                    if decision.robot_id is None:
                        if record.request.requested_robot_id:
                            pin = record.request.requested_robot_id
                            estimate = estimates.get(pin)
                            pending = (
                                estimate is not None
                                and not estimate.feasible
                                and estimate.reason == "path pending"
                                and robot_is_ready(self.registry.get(pin, now))
                            )
                            if (
                                pending
                                and now
                                - self._path_pending_since.setdefault(mission_id, now)
                                < PATH_PENDING_GRACE_SEC
                            ):
                                # Only this exact transient outcome gets a retry.
                                # Repeated replies never renew the original bound.
                                self._retry_at[mission_id] = now + 1.0
                                self._emit(
                                    self.journal.get(mission_id), detail="path pending"
                                )
                                continue
                            rejected = self.core.reject(
                                mission_id,
                                "REQUESTED_ROBOT_UNAVAILABLE",
                                f"Requested robot {record.request.requested_robot_id}: {decision.reason}",
                                now,
                            )
                            self._retry_at.pop(mission_id, None)
                            self._path_pending_since.pop(mission_id, None)
                            self._remove_waiters(mission_id)
                            self._emit(rejected)
                        else:
                            self._retry_at[mission_id] = now + 1.0
                            self._emit(
                                self.journal.get(mission_id), detail=decision.reason
                            )
                    else:
                        self._start(record.request, decision)
                continue
            if now < self._retry_at.get(mission_id, 0.0):
                continue
            reserved = {flight.robot_id for flight in self._flights.values()}
            candidates = [
                robot.robot_id
                for robot in self.registry.eligible(now)
                if robot.robot_id not in reserved
                and robot.robot_id
                not in {charge.robot_id for charge in self.core.charging_snapshot()}
                and (
                    not record.request.requested_robot_id
                    or record.request.requested_robot_id == robot.robot_id
                )
            ]
            round_ = _Round(now + 1.0, set(candidates))
            self._rounds[mission_id] = round_
            for robot_id in candidates:

                def callback(
                    estimate, robot_id=robot_id, round_=round_, mission_id=mission_id
                ):
                    self._enqueue(
                        self._estimated,
                        mission_id,
                        round_,
                        robot_id,
                        estimate,
                        self.clock(),
                    )

                try:
                    cleanup = self.transport.estimate(
                        robot_id, record.request, callback
                    )
                    if cleanup is not None:
                        round_.cleanups[robot_id] = cleanup
                except Exception:
                    callback(None)

    def _start_charging(self, charge):
        self._dock_completed.discard(charge.robot_id)
        flight = _Flight(charge.robot_id, charge.generation)
        self._dock_flights[charge.robot_id] = flight

        def accepted(handle, error):
            self._enqueue(self._dock_accepted, charge, flight, handle, error)

        try:
            self.transport.send_dock_goal(
                charge.robot_id,
                charge,
                lambda feedback: self._enqueue(
                    self._dock_feedback, charge, flight, feedback
                ),
                lambda reply: self._enqueue(self._dock_result, charge, flight, reply),
                accepted,
            )
        except Exception as error:
            accepted(None, str(error))

    def _dock_current(self, charge, flight):
        return self._dock_flights.get(charge.robot_id) is flight and not flight.retired

    def _dock_accepted(self, charge, flight, handle, error):
        flight.handle = handle
        if not self._dock_current(charge, flight):
            if handle is not None:
                self.transport.cancel(handle, lambda _: None)
            return
        if error or handle is None:
            self.core.charging_result(charge.robot_id, charge.generation, False)
            flight.retired = True

    def _dock_feedback(self, charge, flight, feedback):
        if self._dock_current(charge, flight):
            self.core.charging_feedback(
                charge.robot_id, charge.generation, feedback.state
            )

    def _dock_result(self, charge, flight, reply):
        if not self._dock_current(charge, flight):
            return
        # Terminal dock success is emitted only after verified exit and release.
        self.core.charging_result(
            charge.robot_id,
            charge.generation,
            reply.success,
            cancelled=reply.error_code == "CANCELLED",
        )
        if reply.success or reply.error_code == "CANCELLED":
            self._dock_completed.add(charge.robot_id)
        flight.retired = True
        self._dock_flights.pop(charge.robot_id, None)

    def _estimated(self, mission_id, round_, robot_id, estimate, received_at):
        if self._rounds.get(mission_id) is not round_ or received_at >= round_.deadline:
            return  # A response after timeout belongs to an obsolete round.
        round_.pending.discard(robot_id)
        if estimate is not None:
            round_.estimates[robot_id] = estimate

    def _close_round(self, mission_id):
        round_ = self._rounds.pop(mission_id, None)
        if round_ is not None:
            for robot_id in round_.pending:
                cleanup = round_.cleanups.get(robot_id)
                if cleanup is not None:
                    cleanup()

    def _start(self, request, decision):
        self._retry_at.pop(request.mission_id, None)
        self._path_pending_since.pop(request.mission_id, None)
        flight = _Flight(decision.robot_id, decision.assignment_id)
        self._flights[request.mission_id] = flight
        self._emit(self.journal.get(request.mission_id))
        try:
            self.transport.send_goal(
                decision.robot_id,
                request,
                lambda feedback: self._enqueue(
                    self._feedback, request.mission_id, flight, feedback
                ),
                lambda result: self._enqueue(
                    self._result, request.mission_id, flight, result
                ),
                lambda handle, error: self._enqueue(
                    self._accepted, request.mission_id, flight, handle, error
                ),
            )
        except Exception as error:
            self._accepted(request.mission_id, flight, None, str(error))

    def _current(self, mission_id, flight):
        return self._flights.get(mission_id) is flight and not flight.retired

    def _accepted(self, mission_id, flight, handle, error):
        flight.handle = handle
        if not self._current(mission_id, flight):
            if handle is not None:
                self.transport.cancel(handle, lambda acknowledged: None)
            return
        if error:
            # Delivery may have reached the robot. Fence uncertain ownership and
            # retain its reservation until offline recovery or acknowledged cancel.
            self.core.record_robot_feedback(
                mission_id,
                RobotMissionFeedback(
                    flight.robot_id, flight.assignment_id, PayloadOwnership.UNKNOWN
                ),
                self.clock(),
            )
            self.cancel(mission_id)
        elif handle is None:
            self._result(
                mission_id,
                flight,
                RobotReply(False, "ROBOT_BUSY", "Robot rejected mission"),
            )
        elif flight.cancelling:
            self._cancel_goal(mission_id, flight)

    def _feedback(self, mission_id, flight, feedback):
        if not self._current(mission_id, flight):
            return
        ownership = PayloadOwnership.NOT_PICKED_UP
        if feedback.state == "LOADING":
            ownership = PayloadOwnership.UNKNOWN
        elif feedback.state in (
            "NAVIGATING_TO_DROPOFF",
            "VERIFYING_DROPOFF",
            "UNLOADING",
            "COMPLETED",
        ):
            ownership = PayloadOwnership.PICKED_UP
        record = self.core.record_robot_feedback(
            mission_id,
            RobotMissionFeedback(
                flight.robot_id, flight.assignment_id, ownership, feedback.state
            ),
            self.clock(),
        )
        if record.state == MissionState.EXECUTING:
            self._emit(record, feedback.state, feedback.detail, feedback.progress)

    def _result(self, mission_id, flight, reply):
        if not self._current(mission_id, flight):
            return
        record = self.core.record_robot_result(
            mission_id,
            RobotMissionResult(
                flight.robot_id,
                flight.assignment_id,
                reply.success,
                reply.error_code,
                reply.message,
                cancelled=reply.cancelled,
                recovery_required=reply.recovery_required,
            ),
            self.clock(),
        )
        flight.retired = True
        del self._flights[mission_id]
        self._remove_waiters(mission_id)
        self._emit(record)

    @_storage_boundary
    def cancel(self, mission_id):
        if self.state == "STORAGE_FAILED":
            raise sqlite3.OperationalError("fleet storage failed")
        # Consume already-received pickup evidence before deciding whether a
        # cancellation is safe or requires physical payload recovery.
        self._drain_callbacks(self._callbacks.qsize())
        record = self.core.request_cancellation(mission_id, self.clock())
        self._close_round(mission_id)
        self._retry_at.pop(mission_id, None)
        self._path_pending_since.pop(mission_id, None)
        self._remove_waiters(mission_id)
        flight = self._flights.get(mission_id)
        if flight is not None and not flight.cancelling:
            flight.cancelling = True
            if flight.handle is not None:
                self._cancel_goal(mission_id, flight)
        self._emit(record)
        return record

    def _remove_waiters(self, mission_id):
        now = self.clock()
        for resource in self.resources.snapshot(now):
            for waiter in resource.waiters:
                if waiter.mission_id == mission_id:
                    self.resources.cancel_waiter(waiter, now)

    def _cancel_goal(self, mission_id, flight):
        try:
            self.transport.cancel(
                flight.handle,
                lambda acknowledged: self._enqueue(
                    self._cancelled, mission_id, flight, acknowledged
                ),
            )
        except Exception:
            pass  # The reservation stays until robot loss establishes recovery.

    def _cancelled(self, mission_id, flight, acknowledged):
        if self._current(mission_id, flight):
            self.core.record_cancellation_acknowledgement(
                mission_id,
                RobotCancellationAck(
                    flight.robot_id, flight.assignment_id, acknowledged
                ),
                self.clock(),
            )

    def resource(self, operation, request):
        if self.state == "RECONCILING":
            try:
                self._reconcile_startup(self.clock())
            except sqlite3.Error:
                self.state = "STORAGE_FAILED"
        if self.state != "RUNNING":
            return self._resource_denial(
                operation,
                "fleet reconciling"
                if self.state == "RECONCILING"
                else "fleet storage failed",
            )
        try:
            response = self._resource(operation, request)
            self.journal.save_resources(
                self.resources.snapshot(self.clock()), self.clock()
            )
            return response
        except sqlite3.Error:
            self.state = "STORAGE_FAILED"
            return self._resource_denial(operation, "fleet storage failed")

    @staticmethod
    def _resource_denial(operation, reason):
        if operation == "acquire":
            return {
                "granted": False,
                "lease_id": "",
                "lease_ttl_sec": 0.0,
                "current_owner": "",
                "reason": reason,
            }
        if operation == "renew":
            return {"renewed": False, "lease_ttl_sec": 0.0, "reason": reason}
        if operation == "release":
            return {"released": False, "reason": reason}
        return {
            "cancelled": False,
            "lease_id": "",
            "lease_ttl_sec": 0.0,
            "reconciliation_required": True,
            "reason": reason,
        }

    def _resource(self, operation, request):
        if operation not in ("acquire", "renew", "release", "cancel_wait"):
            raise ValueError("unsupported resource operation")
        if request.robot_id not in {robot.robot_id for robot in self.config.robots}:
            raise ValueError("resource owner is not configured")
        now = self.clock()
        identity = (request.robot_id, request.mission_id, request.resource_id)
        if operation == "acquire" and request.resource_id in self.config.docks:
            queue_ = [
                charge
                for charge in self.core.charging_snapshot()
                if charge.dock_id == request.resource_id
            ]
            if not queue_:
                return {
                    "granted": False,
                    "lease_id": "",
                    "lease_ttl_sec": 0.0,
                    "current_owner": "",
                    "reason": "dock charging not authorized",
                }
            if request.resource_id in self._dock_restart_unsafe or len(
                self._dock_seen
            ) != len(self.config.robots):
                return {
                    "granted": False,
                    "lease_id": "",
                    "lease_ttl_sec": 0.0,
                    "current_owner": "",
                    "reason": "dock reconciliation required",
                }
            if queue_[0].robot_id != request.robot_id:
                return {
                    "granted": False,
                    "lease_id": "",
                    "lease_ttl_sec": 0.0,
                    "current_owner": queue_[0].robot_id,
                    "reason": "charge queued",
                }
        if operation == "cancel_wait":
            resolution = self.resources.resolve_waiter(LeaseRequest(*identity), now)
            return {
                "cancelled": resolution.cancelled,
                "reason": resolution.reason,
                "lease_id": resolution.lease.lease_id if resolution.lease else "",
                "lease_ttl_sec": resolution.lease.expires_at - now
                if resolution.lease
                else 0.0,
                "reconciliation_required": resolution.reconciliation_required,
            }
        if (
            operation != "release"
            and self.registry.get(request.robot_id, now).health != RobotHealth.ONLINE
        ):
            response = {
                "granted" if operation == "acquire" else "renewed": False,
                "lease_ttl_sec": 0.0,
                "reason": "robot unavailable",
            }
            if operation == "acquire":
                response.update(lease_id="", current_owner="")
            return response
        if operation == "release":
            released = self.resources.release(
                LeaseKey(*identity, request.lease_id), now
            )
            return {
                "released": released,
                "reason": "released" if released else "lease mismatch",
            }
        decision = (
            self.resources.acquire(LeaseRequest(*identity), now)
            if operation == "acquire"
            else self.resources.renew(LeaseKey(*identity, request.lease_id), now)
        )
        response = {
            "granted" if operation == "acquire" else "renewed": decision.granted,
            "lease_ttl_sec": decision.lease.expires_at - now if decision.lease else 0.0,
            "reason": decision.reason,
        }
        if operation == "acquire":
            response.update(
                lease_id=decision.lease.lease_id if decision.lease else "",
                current_owner=decision.current_owner or "",
            )
        return response
