# SPDX-License-Identifier: Apache-2.0
"""Serialized fleet orchestration with asynchronous transport boundaries.

Transport callbacks only enqueue work. The owning executor thread drains that
work in tick(), keeping the journal and FleetCore on their creating thread.
"""

from dataclasses import dataclass, field
import math
import queue
import time

from fleet_manager.core import (
    FleetCore,
    ReconciliationSnapshot,
    RobotCancellationAck,
    RobotMissionFeedback,
    RobotMissionResult,
)
from fleet_manager.config import Pose2D
from fleet_manager.dispatcher import Dispatcher
from fleet_manager.energy import EnergyPolicy
from fleet_manager.journal import MissionState, PayloadOwnership
from fleet_manager.models import RobotHealth, RobotMode, RobotSnapshot
from fleet_manager.registry import RobotRegistry
from fleet_manager.resources import LeaseKey, LeaseRequest, ResourceManager


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

    def __init__(self, config, journal, transport, *, clock=time.monotonic):
        self.config, self.journal, self.transport, self.clock = (
            config,
            journal,
            transport,
            clock,
        )
        self.registry = RobotRegistry(tuple(robot.robot_id for robot in config.robots))
        self.resources = ResourceManager(config.resources)
        self.core = FleetCore(
            Dispatcher(EnergyPolicy(config.energy)),
            self.registry,
            self.resources,
            journal,
        )
        self._callbacks = queue.SimpleQueue()
        self._rounds = {}
        self._retry_at = {}
        self._flights = {}
        # Unknown running goals after restart are fenced before scheduling starts.
        self.core.reconcile(ReconciliationSnapshot(()), self.clock())

    def submit(self, request):
        if request.requested_robot_id and request.requested_robot_id not in {
            robot.robot_id for robot in self.config.robots
        }:
            raise ValueError("requested robot is not configured")
        record = self.core.submit(request, self.clock())
        self._emit(record)
        return record

    def observe(self, snapshot):
        now = self.clock()
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
                            rejected = self.core.reject(
                                mission_id,
                                "REQUESTED_ROBOT_UNAVAILABLE",
                                f"Requested robot {record.request.requested_robot_id}: {decision.reason}",
                                now,
                            )
                            self._retry_at.pop(mission_id, None)
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

    def cancel(self, mission_id):
        # Consume already-received pickup evidence before deciding whether a
        # cancellation is safe or requires physical payload recovery.
        self._drain_callbacks(self._callbacks.qsize())
        record = self.core.request_cancellation(mission_id, self.clock())
        self._close_round(mission_id)
        self._retry_at.pop(mission_id, None)
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
        if operation not in ("acquire", "renew", "release", "cancel_wait"):
            raise ValueError("unsupported resource operation")
        if request.robot_id not in {robot.robot_id for robot in self.config.robots}:
            raise ValueError("resource owner is not configured")
        now = self.clock()
        identity = (request.robot_id, request.mission_id, request.resource_id)
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
