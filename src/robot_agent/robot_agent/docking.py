# SPDX-License-Identifier: Apache-2.0
"""Nonblocking docking with physical arrival gates and exact lease identities."""

from dataclasses import dataclass, replace
import math
import time
import uuid

from robot_agent.adapter import finite_pose, identifier
from robot_agent.energy_model import percentage


@dataclass(frozen=True)
class DockKey:
    robot_id: str
    mission_id: str
    resource_id: str
    lease_id: str = ""


@dataclass(frozen=True)
class DockFeedback:
    state: str
    battery_percent: float
    detail: str = ""


@dataclass(frozen=True)
class DockResult:
    success: bool
    error_code: str = ""
    message: str = ""


class DockingController:
    """One executor owns transitions; callbacks never wait for transport.

    Navigation success and current localization must both confirm arrival.
    Cancellation resolves outstanding acquisitions before reporting availability.
    Occupancy uncertainty retains the dock for explicit recovery, including lease
    loss, contact loss, a failed exit, and process restart inside the dock.
    """

    def __init__(
        self,
        agent,
        transport,
        dock_id,
        staging_pose,
        charging_pose,
        *,
        clock=time.monotonic,
        tolerance=0.15,
        yaw_tolerance=0.2,
        contact_timeout=1.0,
        navigation_timeout=60.0,
        service_timeout=1.0,
    ):
        if (
            not identifier(dock_id)
            or not finite_pose(staging_pose)
            or not finite_pose(charging_pose)
        ):
            raise ValueError("dock requires an identity and finite poses")
        bounds = (
            tolerance,
            yaw_tolerance,
            contact_timeout,
            navigation_timeout,
            service_timeout,
        )
        if any(
            type(value) not in (int, float) or not math.isfinite(value) or value <= 0
            for value in bounds
        ):
            raise ValueError("dock bounds must be finite and positive")
        if (
            math.hypot(
                staging_pose[0] - charging_pose[0], staging_pose[1] - charging_pose[1]
            )
            <= 2 * tolerance
        ):
            raise ValueError("dock staging must be outside charging tolerance")
        self.agent, self.transport, self.clock = agent, transport, clock
        self.dock_id, self.staging, self.charging = (
            dock_id,
            tuple(staging_pose),
            tuple(charging_pose),
        )
        self.tolerance, self.yaw_tolerance = tolerance, yaw_tolerance
        self.contact_timeout, self.nav_timeout, self.service_timeout = (
            contact_timeout,
            navigation_timeout,
            service_timeout,
        )
        self.state, self.active = "IDLE", False
        self.key, self.lease_until, self.renew_at = None, 0.0, 0.0
        self._generation = 0
        self._pending = None
        self._acquiring = set()
        self._cleanup = set()
        self._released = set()
        self._resolved = False
        self._cancelled = False
        self._nav_cancel, self._nav_done = None, False
        self._nav_sequence = 0
        self._contact, self._contact_until = False, 0.0
        self.exit_pose = None
        self._next = 0.0
        self._feedback, self._result = None, None
        agent.configure_dock(dock_id, self.charging, tolerance, yaw_tolerance)

    def at(self, pose):
        current = self.agent.pose
        return (
            current is not None
            and not self.agent._health(self.clock())
            and math.hypot(current[0] - pose[0], current[1] - pose[1]) <= self.tolerance
            and abs(math.remainder(current[2] - pose[2], 2 * math.pi))
            <= self.yaw_tolerance
        )

    def can_start(self):
        state = self.agent.heartbeat()
        return (
            not self.active
            and state.mode == "AVAILABLE"
            and state.payload_state == "EMPTY"
            and not state.mission_id
            and not state.health_detail
            and state.pose is not None
        )

    def start(self, target, feedback, result):
        target = percentage(target, "target_percent")
        if not self.can_start():
            return False
        if self.at(self.charging):
            self.agent.mode = "RECOVERY_REQUIRED"
            return False
        if (
            math.hypot(
                self.agent.pose[0] - self.staging[0],
                self.agent.pose[1] - self.staging[1],
            )
            <= 2 * self.tolerance
        ):
            # A return pose inside shared staging cannot clear the next arrival.
            self.agent.mode = "RECOVERY_REQUIRED"
            return False
        self.exit_pose = tuple(self.agent.pose)
        self._generation += 1
        self.active, self._cancelled, self._resolved = True, False, False
        self._feedback, self._result, self.target = feedback, result, target
        self.key = DockKey(
            self.agent.robot_id, "dock-" + uuid.uuid4().hex, self.dock_id
        )
        self.lease_until, self.renew_at = 0.0, 0.0
        self._pending, self._acquiring, self._cleanup = None, set(), set()
        self._released = set()
        self._contact, self._contact_until = False, 0.0
        self.agent.begin_docking(self.key)
        self._navigate("STAGING", self.staging)
        return True

    def _state(self, state):
        self.state = state
        self.agent._advance()
        self.agent.mode = "CHARGING" if state == "CHARGING" else "DOCKING"
        self._authorize()
        if self._feedback:
            self._feedback(DockFeedback(state, self.agent.energy.battery_percent))

    def _authorize(self):
        self.agent.set_charge_authorization(
            lease_until=self.lease_until,
            contact=self._contact,
            lease_key=self.key,
            contact_until=self._contact_until,
        )

    def _navigate(self, state, pose):
        self._state(state)
        self._nav_sequence += 1
        sequence, generation = self._nav_sequence, self._generation
        self._nav_done, self._nav_cancel = False, None
        self._nav_deadline = self.clock() + self.nav_timeout

        def completed(success, reason, terminal=True):
            if (
                not self.active
                or generation != self._generation
                or sequence != self._nav_sequence
                or self._nav_done
            ):
                return
            self._nav_done = True
            if terminal:
                self._nav_cancel = None
            if not terminal:
                self._finish(False, "NAVIGATION_UNKNOWN", reason, recovery=True)
            elif self._cancelled and state == "STAGING":
                self._clear_staging()
            elif self._cancelled and state == "ENTERING":
                self._exit()
            elif not success:
                self._finish(
                    False, "NAVIGATION_FAILED", reason, recovery=state != "STAGING"
                )
            else:
                self.tick()

        try:
            cancel = self.transport.navigate(
                pose, self.agent.frame_prefix + "map", completed
            )
            if self.active and sequence == self._nav_sequence and not self._nav_done:
                self._nav_cancel = cancel
        except Exception as error:
            completed(False, str(error), False)

    @staticmethod
    def _ttl(reply, sent):
        ttl = getattr(reply, "lease_ttl_sec", 0.0)
        if type(ttl) not in (int, float) or not math.isfinite(ttl) or ttl <= 0:
            return 0.0
        return sent + ttl

    def _send(self, operation):
        if operation == "acquire" and len(self._acquiring) >= 8:
            self._finish(False, "ACQUIRE_UNRESOLVED", recovery=True)
            return
        token = object()
        sent, generation, key = self.clock(), self._generation, self.key
        self._pending = (token, operation, sent)
        if operation == "acquire":
            self._acquiring.add(token)

        def received(reply):
            if operation == "acquire" and token not in self._acquiring:
                return
            if operation == "acquire":
                self._acquiring.discard(token)
            current = self._pending is not None and self._pending[0] is token
            if current:
                self._pending = None
            if generation != self._generation:
                return
            if operation == "acquire" and self._cancelled:
                if getattr(reply, "granted", False) and getattr(reply, "lease_id", ""):
                    self._release_key(replace(key, lease_id=reply.lease_id))
                elif self._cancelled:
                    self._cleanup_complete()
                return
            if not self.active or not current:
                return
            self._next = self.clock() + 0.25
            if operation == "acquire":
                if not getattr(reply, "granted", False):
                    return
                deadline = self._ttl(reply, sent)
                if not getattr(reply, "lease_id", "") or self.clock() >= deadline:
                    self._finish(False, "LEASE_LOST", recovery=True)
                    return
                self.key = replace(key, lease_id=reply.lease_id)
                self.agent.begin_docking(self.key)
                self.lease_until = deadline
                self.renew_at = sent + (deadline - sent) / 2
                if (
                    self.agent._health(self.clock())
                    or self.agent.mission_id
                    or self.agent.payload_state != "EMPTY"
                ):
                    self._finish(False, "ROBOT_UNAVAILABLE", recovery=True)
                    return
                self._navigate("ENTERING", self.charging)
            elif operation == "renew":
                deadline = self._ttl(reply, sent)
                if (
                    not getattr(reply, "renewed", False)
                    or self.clock() >= self.lease_until
                    or self.clock() >= deadline
                    or key != self.key
                ):
                    self._finish(False, "LEASE_LOST", recovery=True)
                else:
                    self.lease_until, self.renew_at = (
                        deadline,
                        sent + (deadline - sent) / 2,
                    )
                    self._authorize()
            elif operation == "cancel_wait":
                if getattr(reply, "reconciliation_required", True):
                    self._finish(False, "CLEANUP_FAILED", recovery=True)
                    return
                self._resolved = True
                if getattr(reply, "lease_id", ""):
                    self._release_key(replace(key, lease_id=reply.lease_id))
                self._cleanup_complete()

        try:
            self.transport.resource(operation, key, received)
        except Exception:
            received(type("Unavailable", (), {})())

    def _release_key(self, key):
        if key.lease_id in self._released:
            self._cleanup_complete()
            return
        if key.lease_id in self._cleanup:
            return
        self._cleanup.add(key.lease_id)
        generation = self._generation

        def released(reply):
            if generation != self._generation or key.lease_id not in self._cleanup:
                return
            if not getattr(reply, "released", False):
                self._finish(False, "RELEASE_FAILED", recovery=True)
                return
            self._cleanup.discard(key.lease_id)
            self._released.add(key.lease_id)
            self._cleanup_complete()

        try:
            self.transport.resource("release", key, released)
        except Exception:
            self._finish(False, "RELEASE_FAILED", recovery=True)

    def _cleanup_complete(self):
        if self.active and self._resolved and not self._acquiring and not self._cleanup:
            if self._cancelled:
                self._clear_staging()
                return
            self._finish(
                not self._cancelled,
                "CANCELLED" if self._cancelled else "",
                recovery=False,
            )

    def _clear_staging(self):
        if self.at(self.exit_pose):
            self._finish(False, "CANCELLED", recovery=False)
        elif self.state != "CLEARING":
            # Reconciled pre-entry cancellation needs no dock lease, but must
            # vacate the shared approach before allowing another action.
            self._navigate("CLEARING", self.exit_pose)

    def _exit(self):
        if self.clock() >= self.lease_until:
            self._finish(False, "LEASE_LOST", recovery=True)
        else:
            self._navigate("EXITING", self.exit_pose)

    def renew(self):
        if self.active and self.key.lease_id and self._pending is None:
            self._send("renew")

    def contact(self, confirmed):
        if type(confirmed) is not bool:
            return
        self.agent._advance()
        self._contact = confirmed
        self._contact_until = self.clock() + self.contact_timeout if confirmed else 0.0
        if self.active:
            self._authorize()
            self.tick()

    def cancel(self):
        if not self.active or self._cancelled:
            return
        self.agent._advance()
        self._cancelled = True
        self._contact = False
        self._authorize()
        if self.state == "STAGING":
            if self._nav_done:
                self._clear_staging()
            elif self._nav_cancel:
                self._request_stop()
        elif self.state == "WAITING_FOR_LEASE":
            self._state("CANCELLING")
            self._cleanup_deadline = self.clock() + 5.0
            self._send("cancel_wait")
        elif self.state == "ENTERING" and not self._nav_done:
            if self._nav_cancel:
                self._request_stop()
        elif self.state != "EXITING":
            self._exit()

    def _request_stop(self):
        try:
            self._nav_cancel()
        except Exception as error:
            self._nav_cancel = None
            self._finish(False, "CANCEL_FAILED", str(error), recovery=True)

    def _finish(self, success, error_code="", message="", *, recovery):
        if not self.active:
            return
        self.agent._advance()
        self.active = False
        self._contact = False
        self.lease_until = 0.0
        self._authorize()
        if self._nav_cancel:
            try:
                self._nav_cancel()
            except Exception as error:
                success, recovery = False, True
                error_code = error_code or "CANCEL_FAILED"
                message = (message + "; " if message else "") + str(error)
            self._nav_cancel = None
        self.agent.end_docking(recovery)
        self.state = (
            "RECOVERY_REQUIRED"
            if recovery
            else "COMPLETED"
            if success
            else "CANCELLED"
            if error_code == "CANCELLED"
            else "FAILED"
        )
        self._result(DockResult(success, error_code, message))

    def shutdown(self):
        if self.active:
            # A stopped process cannot confirm exit or authorize a release.
            self._finish(False, "AGENT_SHUTDOWN", recovery=True)

    def tick(self):
        self.agent.tick()
        if not self.active:
            if self.agent.mode == "AVAILABLE" and self.at(self.charging):
                self.agent.mode = "RECOVERY_REQUIRED"
            return
        now = self.clock()
        if (
            self.agent._health(now)
            or self.agent.mission_id
            or self.agent.payload_state != "EMPTY"
        ):
            self._finish(False, "ROBOT_UNAVAILABLE", recovery=True)
            return
        if self.key.lease_id and now >= self.lease_until:
            self._finish(False, "LEASE_LOST", recovery=True)
            return
        if self._pending and now - self._pending[2] >= self.service_timeout:
            operation = self._pending[1]
            self._pending = None
            if operation != "acquire":
                self._finish(
                    False,
                    "LEASE_LOST" if operation == "renew" else "CLEANUP_FAILED",
                    recovery=True,
                )
                return
        if self.state in ("STAGING", "ENTERING", "EXITING", "CLEARING"):
            if now >= self._nav_deadline:
                self._finish(False, "NAVIGATION_TIMEOUT", recovery=True)
                return
            pose = {
                "ENTERING": self.charging,
                "STAGING": self.staging,
                "EXITING": self.exit_pose,
                "CLEARING": self.exit_pose,
            }[self.state]
            if self._nav_done and self.at(pose):
                if self.state == "STAGING":
                    self._state("WAITING_FOR_LEASE")
                    self._send("acquire")
                elif self.state == "ENTERING":
                    self._state("WAITING_FOR_CONTACT")
                    self._contact_deadline = now + 5.0
                elif self.state == "CLEARING":
                    self._finish(False, "CANCELLED", recovery=False)
                else:
                    self._state("RELEASING")
                    self._resolved = True
                    self._cleanup_deadline = now + 5.0
                    self._release_key(self.key)
        if (
            self.state == "WAITING_FOR_LEASE"
            and self._pending is None
            and now >= self._next
        ):
            self._send("acquire")
        if self.state == "WAITING_FOR_CONTACT":
            if self._contact and now < self._contact_until and self.at(self.charging):
                self._state("CHARGING")
            elif now >= self._contact_deadline:
                self._finish(False, "CONTACT_TIMEOUT", recovery=True)
                return
        if self.state == "CHARGING":
            if (
                not self._contact
                or now >= self._contact_until
                or not self.at(self.charging)
            ):
                self._finish(
                    False,
                    "CONTACT_LOST" if self.at(self.charging) else "POSE_LOST",
                    recovery=True,
                )
                return
            if self.agent.energy.battery_percent >= self.target:
                self._exit()
        if self.state in ("CANCELLING", "RELEASING") and now >= self._cleanup_deadline:
            self._finish(False, "CLEANUP_FAILED", recovery=True)
            return
        if (
            self.active
            and self.key.lease_id
            and self._pending is None
            and now >= self.renew_at
        ):
            self.renew()
