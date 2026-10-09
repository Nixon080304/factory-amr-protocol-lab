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
        robot_radius=0.15,
        yaw_tolerance=0.2,
        contact_timeout=1.0,
        navigation_timeout=60.0,
        service_timeout=1.0,
        exit_pose=None,
        diagnostic=None,
    ):
        if (
            not identifier(dock_id)
            or not finite_pose(staging_pose)
            or not finite_pose(charging_pose)
            or (exit_pose is not None and not finite_pose(exit_pose))
        ):
            raise ValueError("dock requires an identity and finite poses")
        bounds = (
            tolerance,
            robot_radius,
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
        self._diagnostic = diagnostic
        self.dock_id, self.staging, self.charging = (
            dock_id,
            tuple(staging_pose),
            tuple(charging_pose),
        )
        self.tolerance, self.yaw_tolerance = tolerance, yaw_tolerance
        # V1 leaves the exit unset and returns to the session's initial pose.
        self.configured_exit = None if exit_pose is None else tuple(exit_pose)
        self.robot_radius = robot_radius
        self.contact_timeout, self.nav_timeout, self.service_timeout = (
            contact_timeout,
            navigation_timeout,
            service_timeout,
        )
        self.state, self.active = "IDLE", False
        self.key, self.lease_until, self.renew_at = None, 0.0, 0.0
        self._generation = 0
        self._last_start_fence = None
        self._pending = None
        self._acquiring = set()
        self._cleanup = set()
        self._released = set()
        self._resolved = False
        self._cancelled = False
        self._nav_cancel, self._nav_done = None, False
        self._nav_sequence = 0
        self._refresh = None
        self._contact, self._contact_until = False, 0.0
        self.exit_pose = None
        self._next = 0.0
        self._feedback, self._result = None, None
        agent.configure_dock(dock_id, self.charging, tolerance, yaw_tolerance)

    def diagnose(self, event, **fields):
        """Observer-only bounded evidence; a failed logger cannot alter docking."""
        if self._diagnostic is not None:
            try:
                self._diagnostic(
                    dict(
                        event=event,
                        robot_id=self.agent.robot_id,
                        generation=self._generation,
                        state=self.state,
                        **fields,
                    )
                )
            except Exception:
                pass

    def evidence(self):
        now = self.clock()

        def age(receipt):
            return None if receipt is None else now - receipt

        return dict(
            mode=self.agent.mode,
            payload_state=self.agent.payload_state,
            mission_present=bool(self.agent.mission_id),
            active=self.active,
            health_detail=self.agent._health(now),
            pose=self.agent.pose,
            pose_source_stamp=self.agent._pose_stamp,
            pose_receipt_age_sec=age(self.agent._pose_receipt),
            odom_receipt_age_sec=age(self.agent._odom_receipt),
            battery_percent=self.agent.energy.battery_percent,
            lease_present=bool(self.key and self.key.lease_id),
        )

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

    def _clear_for_handoff(self, pose, arrival_error=0.0):
        """Exclude both footprints and every permitted dock arrival region."""
        if pose is None:
            return False
        minimum = 2 * self.robot_radius + self.tolerance + arrival_error
        for target in (self.staging, self.charging):
            distance = math.hypot(pose[0] - target[0], pose[1] - target[1])
            if distance <= minimum or math.isclose(distance, minimum):
                return False
        return True

    def start(self, target, feedback, result):
        target = percentage(target, "target_percent")
        if not self.can_start():
            self._start_fence("robot_unavailable")
            return False
        if self.at(self.charging):
            self.agent.mode = "RECOVERY_REQUIRED"
            self._start_fence("already_at_charging")
            return False
        if not self._clear_for_handoff(self.agent.pose, self.tolerance):
            # The return arrival region must clear the next dock cycle.
            self.agent.mode = "RECOVERY_REQUIRED"
            self._start_fence("initial_clearance")
            return False
        self.exit_pose = self.configured_exit or tuple(self.agent.pose)
        if not self._clear_for_handoff(self.exit_pose, self.tolerance):
            self.agent.mode = "RECOVERY_REQUIRED"
            self._start_fence("exit_clearance")
            return False
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

    def _start_fence(self, reason):
        signature = (self._generation, reason)
        if signature != self._last_start_fence:
            self._last_start_fence = signature
            self.diagnose(
                "dock_start_fence",
                reason=reason,
                exit_pose=self.configured_exit,
                **self.evidence(),
            )

    def _state(self, state):
        previous = self.state
        self.state = state
        self.agent._advance()
        self.agent.mode = "CHARGING" if state == "CHARGING" else "DOCKING"
        self._authorize()
        if previous != state:
            self.diagnose("dock_state", previous_state=previous, **self.evidence())
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
        self._drop_refresh()
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
                self._request_localization()

        try:
            cancel = self.transport.navigate(
                pose,
                self.agent.frame_prefix + "map",
                completed,
                precise=state == "ENTERING",
            )
            if self.active and sequence == self._nav_sequence and not self._nav_done:
                self._nav_cancel = cancel
        except Exception as error:
            completed(False, str(error), False)

    def _drop_refresh(self):
        refresh, self._refresh = self._refresh, None
        if refresh is not None and refresh["cancel"] is not None:
            try:
                refresh["cancel"]()
            except Exception:
                # The generation/token fence still rejects late replies.
                pass

    def _request_localization(self):
        """One bounded no-motion update per terminal navigation result.

        A service acknowledgement does not prove arrival. Only an accepted pose
        with a newer source stamp at or after the request's ROS epoch, received
        after this navigation result, can pass the physical arrival gates.
        """
        generation, sequence = self._generation, self._nav_sequence
        refresh = {
            "stamp": self.agent._pose_stamp,
            "sent": self.clock(),
            "epoch": None,
            "ready": False,
            "cancel": None,
            "reported": set(),
        }
        self._refresh = refresh

        def received(success):
            if (
                not self.active
                or generation != self._generation
                or sequence != self._nav_sequence
                or self._refresh is not refresh
            ):
                return
            if not success:
                self._finish(False, "LOCALIZATION_REFRESH_FAILED", recovery=True)
                return
            refresh["ready"] = True
            self.diagnose("dock_refresh_response", success=True)
            self.tick()

        try:
            epoch = self.transport.localization_epoch()
            if type(epoch) not in (int, float) or not math.isfinite(epoch) or epoch < 0:
                raise ValueError("localization refresh requires a valid ROS epoch")
            refresh["epoch"] = epoch
            self.diagnose(
                "dock_refresh_request",
                baseline_stamp=refresh["stamp"],
                request_epoch=epoch,
            )
            cancel = self.transport.request_localization(received)
            if self._refresh is refresh:
                refresh["cancel"] = cancel
            elif cancel is not None:
                cancel()
        except Exception as error:
            self._finish(
                False, "LOCALIZATION_REFRESH_FAILED", str(error), recovery=True
            )

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
            if self.state == "RELEASING" and not (
                self.at(self.exit_pose) and self._clear_for_handoff(self.agent.pose)
            ):
                self._finish(False, "CLEARANCE_FAILED", recovery=True)
                return
            if self._cancelled:
                self._clear_staging()
                return
            self._finish(
                not self._cancelled,
                "CANCELLED" if self._cancelled else "",
                recovery=False,
            )

    def _clear_staging(self):
        if self.at(self.exit_pose) and self._clear_for_handoff(self.agent.pose):
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
                # A retained pre-turn pose cannot prove a cancelled approach is
                # clear. Return navigation also requires a fresh observation.
                self._navigate("CLEARING", self.exit_pose)
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
        self._drop_refresh()
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
        if self._refresh and now - self._refresh["sent"] >= self.service_timeout:
            self._finish(False, "LOCALIZATION_REFRESH_TIMEOUT", recovery=True)
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
            refreshed = (
                self._refresh is not None
                and self._refresh["ready"]
                and self.agent._pose_stamp > self._refresh["stamp"]
                and self.agent._pose_stamp >= self._refresh["epoch"]
                and self.agent._pose_receipt is not None
                and self.agent._pose_receipt >= self._refresh["sent"]
            )
            if self._nav_done and self._refresh is not None:
                refresh = self._refresh
                reason = (
                    "service_pending"
                    if not refresh["ready"]
                    else "source_not_newer"
                    if self.agent._pose_stamp <= refresh["stamp"]
                    else "source_before_request"
                    if self.agent._pose_stamp < refresh["epoch"]
                    else "receipt_before_request"
                    if self.agent._pose_receipt is None
                    or self.agent._pose_receipt < refresh["sent"]
                    else "physical_arrival_pending"
                    if not self.at(pose)
                    else "verified"
                )
                if reason not in refresh["reported"]:
                    refresh["reported"].add(reason)
                    self.diagnose(
                        "dock_refresh_fence",
                        reason=reason,
                        baseline_stamp=refresh["stamp"],
                        request_epoch=refresh["epoch"],
                        refresh_age_sec=now - refresh["sent"],
                        **self.evidence(),
                    )
            if self._nav_done and refreshed and self.at(pose):
                self._drop_refresh()
                if self.state in (
                    "EXITING",
                    "CLEARING",
                ) and not self._clear_for_handoff(self.agent.pose):
                    self._finish(False, "CLEARANCE_FAILED", recovery=True)
                    return
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
