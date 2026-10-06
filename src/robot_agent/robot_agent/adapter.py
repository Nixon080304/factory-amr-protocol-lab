# SPDX-License-Identifier: Apache-2.0
"""Robot policy driven by wall receipts and asynchronous production paths."""

from dataclasses import dataclass
from collections import OrderedDict, deque
import json
import math
import re
import time

from robot_agent.cost_model import CostEstimate, CostModel, PathLengths
from robot_agent.energy_model import EnergyModel, nonnegative


def identifier(value):
    return (
        isinstance(value, str)
        and re.fullmatch(r"[A-Za-z0-9_-]{1,64}", value) is not None
    )


def finite_pose(values):
    return len(values) == 3 and all(
        type(v) in (int, float) and math.isfinite(v) for v in values
    )


@dataclass(frozen=True)
class AgentState:
    robot_id: str
    mode: str
    pose: tuple | None
    frame_id: str
    battery_percent: float
    payload_state: str
    mission_id: str
    health_detail: str


class AgentAdapter:
    """Serialized planning with generation fences and fail-closed ownership.

    Odometry supplies distance only. Localization supplies map coordinates only.
    ROS stamps are ordering fences, never health or battery clocks.
    """

    def __init__(
        self,
        robot_id,
        frame_prefix,
        stations,
        paths,
        *,
        battery_percent=100,
        energy_config=None,
        clock=time.monotonic,
        stale_sec=3.0,
        estimate_timeout_sec=0.75,
        max_odom_step_m=10.0,
        cache_max_age_sec=2.0,
        cache_move_tolerance_m=0.1,
        localization_move_tolerance_m=0.5,
        cache_max_entries=16,
    ):
        if not isinstance(robot_id, str) or not re.fullmatch(
            r"[A-Za-z_][A-Za-z0-9_]*", robot_id
        ):
            raise ValueError("invalid robot_id")
        if not isinstance(frame_prefix, str) or (
            frame_prefix
            and not re.fullmatch(r"(?:[A-Za-z_][A-Za-z0-9_]*/)+", frame_prefix)
        ):
            raise ValueError("invalid frame_prefix")
        if not stations or any(
            not identifier(k) or not finite_pose(v) for k, v in stations.items()
        ):
            raise ValueError("stations require finite map poses")
        self.robot_id, self.frame_prefix = robot_id, frame_prefix
        self.stations, self.paths, self.clock = dict(stations), paths, clock
        self.stale_sec = nonnegative(stale_sec, "stale_sec")
        self.timeout = nonnegative(estimate_timeout_sec, "estimate_timeout_sec")
        self.max_step = nonnegative(max_odom_step_m, "max_odom_step_m")
        self.cache_age = nonnegative(cache_max_age_sec, "cache_max_age_sec")
        self.cache_movement = nonnegative(
            cache_move_tolerance_m, "cache_move_tolerance_m"
        )
        self.localization_movement = nonnegative(
            localization_move_tolerance_m, "localization_move_tolerance_m"
        )
        if not self.stale_sec or not 0 < self.timeout < 1 or not self.max_step:
            raise ValueError(
                "health and path bounds must be positive; estimate timeout must be below 1 s"
            )
        if self.cache_age <= 1.0:
            raise ValueError("cache_max_age_sec must exceed the fleet's 1 s retry")
        if type(cache_max_entries) is not int or not 2 <= cache_max_entries <= 256:
            raise ValueError("cache_max_entries must be an integer from 2 to 256")
        self.cache_limit = cache_max_entries
        self.energy = EnergyModel(battery_percent, energy_config)
        self.cost = CostModel(self.energy.config)
        self.mode, self.payload_state, self.mission_id = "AVAILABLE", "UNKNOWN", ""
        self.pose = None
        self._pose_stamp = self._odom_stamp = -1
        self._pose_receipt = self._odom_receipt = None
        self._odom = None
        self._distance = 0.0
        self._localized_distance = 0.0
        self._wall = self.clock()
        self._generation = 0
        self._estimate = None
        self._queued_estimates = deque()
        self._pumping = False
        self._closed = False
        self._transfer = None
        self._terminal = False
        self._retired_missions = set()
        self._phase = ""
        self._lease_until = 0.0
        self._dock_contact = False
        self._dock_geometry = None
        self._dock_key = None
        self._dock_contact_until = float("inf")
        self._cache = OrderedDict()

    def configure_dock(self, dock_id, pose, tolerance, yaw_tolerance):
        self._dock_geometry = (dock_id, tuple(pose), tolerance, yaw_tolerance)

    def begin_docking(self, key):
        self._advance()
        self._dock_key = key
        self.mode = "DOCKING"

    def end_docking(self, recovery):
        self._advance()
        self._dock_key = None
        self.mode = "RECOVERY_REQUIRED" if recovery else "AVAILABLE"

    def set_charge_authorization(
        self, *, lease_until, contact, lease_key=None, contact_until=float("inf")
    ):
        deadline = nonnegative(lease_until, "lease_until")
        if type(contact) is not bool:
            raise ValueError("contact must be bool")
        self._advance()
        if self._dock_geometry:
            matches = (
                self._dock_key is not None
                and lease_key is not None
                and lease_key.robot_id == self.robot_id
                and lease_key.mission_id == self._dock_key.mission_id
                and lease_key.resource_id == self._dock_geometry[0]
                and lease_key.lease_id == self._dock_key.lease_id
                and bool(lease_key.lease_id)
            )
            if not matches:
                deadline, contact = 0.0, False
        self._lease_until = deadline
        self._dock_contact = contact
        self._dock_contact_until = contact_until

    def charge_authorized(self):
        if not self._dock_geometry:
            return False
        _, target, tolerance, yaw_tolerance = self._dock_geometry
        return (
            self._dock_contact
            and self.pose is not None
            and math.hypot(self.pose[0] - target[0], self.pose[1] - target[1])
            <= tolerance
            and abs(math.remainder(self.pose[2] - target[2], 2 * math.pi))
            <= yaw_tolerance
        )

    def _advance(self):
        now = self.clock()
        elapsed = max(0.0, now - self._wall)
        # Split at authorization expiry so a long timer gap cannot charge past it.
        deadline = min(self._lease_until, self._dock_contact_until)
        if self._dock_geometry and self._odom_receipt is not None:
            deadline = min(deadline, self._odom_receipt + self.stale_sec)
        authorized = min(elapsed, max(0.0, deadline - self._wall))
        self.energy.advance(
            authorized, 0, self.mode, lease_valid=True, contact=self.charge_authorized()
        )
        self.energy.advance(elapsed - authorized, 0, self.mode)
        self._wall = max(self._wall, now)
        return now

    def odometry(self, stamp, x, y, frame):
        self._advance()
        if (
            not finite_pose((stamp, x, y))
            or stamp < 0
            or frame != self.frame_prefix + "odom"
        ):
            self._odom_receipt = None
            return False
        if stamp <= self._odom_stamp:
            return False
        self._odom_stamp = stamp
        distance = (
            0
            if self._odom is None
            else math.hypot(x - self._odom[0], y - self._odom[1])
        )
        self._odom = (x, y)
        if distance > self.max_step:
            self._odom_receipt = None
            return False
        self._odom_receipt = self.clock()
        self._distance += distance
        self.energy.advance(0, distance, self.mode)
        return True

    def localization(self, stamp, x, y, yaw, frame):
        self._advance()
        if (
            not finite_pose((x, y, yaw))
            or type(stamp) not in (int, float)
            or not math.isfinite(stamp)
            or stamp < 0
            or frame != self.frame_prefix + "map"
        ):
            self.pose = None
            self._pose_receipt = None
            return False
        if stamp <= self._pose_stamp:
            return False
        self._pose_stamp = stamp
        self.pose = (float(x), float(y), float(yaw))
        self._pose_receipt = self.clock()
        self._localized_distance = self._distance
        return True

    def _health(self, now):
        errors = []
        for name, receipt in (
            ("odometry", self._odom_receipt),
            ("localization", self._pose_receipt),
        ):
            if receipt is None:
                errors.append(f"unknown {name}")
            elif name == "odometry" and now - receipt >= self.stale_sec:
                errors.append(f"stale {name}")
            elif (
                name == "localization"
                and now - receipt >= self.stale_sec
                and self._distance - self._localized_distance
                > self.localization_movement
            ):
                errors.append("stale localization after movement")
        return "; ".join(errors)

    def heartbeat(self):
        now = self._advance()
        detail = self._health(now)
        mode = self.mode
        if detail and mode != "RECOVERY_REQUIRED":
            mode = "UNHEALTHY"
        return AgentState(
            self.robot_id,
            mode,
            self.pose if not detail else None,
            self.frame_prefix + "map" if not detail else "",
            self.energy.battery_percent,
            self.payload_state,
            self.mission_id,
            detail,
        )

    def mission_state(self, state, *, correlated=False):
        if self._terminal:
            return False
        active = (
            "RECEIVED",
            "NAVIGATING_TO_PICKUP",
            "VERIFYING_PICKUP",
            "LOADING",
            "NAVIGATING_TO_DROPOFF",
            "VERIFYING_DROPOFF",
            "UNLOADING",
        )
        self._advance()
        if state == "WAITING_FOR_RESOURCE":
            self.mode = state
        elif state in active or state == "RECOVERING":
            if (
                state in active
                and self._phase in active
                and active.index(state) < active.index(self._phase)
            ):
                return False
            self.mode = "EXECUTING"
            if (
                state in ("LOADING", "UNLOADING")
                and state != self._phase
                and (correlated or not self.mission_id)
            ):
                confirmed = (
                    self._transfer
                    and self._transfer[0] == self.mission_id
                    and (self._transfer[1] == state or self._transfer[1] == "UNLOADING")
                )
                if not confirmed:
                    self.payload_state = "UNKNOWN"
            # The local String topic has no mission identity. It can report
            # activity, but cannot advance a correlated mission's payload/phase.
            if correlated and state != "RECOVERING":
                self._phase = state
        else:
            return False
        return True

    def protocol(self, message):
        if (
            message.robot_id != self.robot_id
            or message.protocol != "ROS"
            or not identifier(message.mission_id)
        ):
            return False
        if message.event == "state_changed":
            if message.detail not in {
                "RECEIVED",
                "NAVIGATING_TO_PICKUP",
                "VERIFYING_PICKUP",
                "LOADING",
                "NAVIGATING_TO_DROPOFF",
                "VERIFYING_DROPOFF",
                "UNLOADING",
                "RECOVERING",
                "WAITING_FOR_RESOURCE",
            }:
                return False
            if message.mission_id in self._retired_missions:
                return False
            if self.mission_id and self.mission_id != message.mission_id:
                return False
            if self._terminal:
                if (
                    self.mode != "AVAILABLE"
                    or self.mission_id
                    or self.payload_state != "EMPTY"
                ):
                    return False
                # A valid, non-retired correlated identity is the only evidence
                # that can reopen the terminal fence for another mission.
                self._terminal = False
            if not self.mission_id:
                self.mission_id = message.mission_id
            return self.mission_state(message.detail, correlated=True)
        if (
            message.event == "mission_finished"
            and message.mission_id == self.mission_id
            and message.outcome in ("COMPLETED", "FAILED", "RECOVERY_REQUIRED")
        ):
            self._advance()
            self._retired_missions.add(self.mission_id)
            self._terminal = True
            if message.outcome == "RECOVERY_REQUIRED" or self.payload_state != "EMPTY":
                self.mode = "RECOVERY_REQUIRED"
            else:
                self.mode = "AVAILABLE"
                self.mission_id = ""
                self._transfer = None
                self._phase = ""
            return True
        return False

    def payload(self, data):
        try:
            fields = json.loads(data)
        except (ValueError, TypeError, RecursionError):
            return False
        if not isinstance(fields, dict) or fields.get("robot_id") != self.robot_id:
            return False
        mission, state = fields.get("mission_id"), fields.get("state")
        kind, counter = fields.get("transfer_kind"), fields.get("cycle_counter")
        if not all(isinstance(item, str) for item in (mission, state, kind)):
            return False
        if (
            mission == ""
            and not self.mission_id
            and state == "AT_ASSEMBLY"
            and kind == ""
            and counter is None
        ):
            self.payload_state = "EMPTY"
            return True
        if (
            mission != self.mission_id
            or not identifier(mission)
            or type(counter) is not int
            or not 0 <= counter <= 65535
        ):
            return False
        target = {
            ("IN_TRANSIT", "LOADING"): "LOADED",
            ("AT_INSPECTION", "UNLOADING"): "EMPTY",
        }.get((state, kind))
        if target is None or self._transfer == (mission, kind, counter):
            return False
        if kind == "UNLOADING" and (
            self._transfer is None or self._transfer[1] != "LOADING"
        ):
            return False
        if kind == "LOADING" and self._transfer is not None:
            return False
        self._advance()
        self.energy.operation()
        self._transfer = (mission, kind, counter)
        self.payload_state = target
        return True

    def _finish(self, generation, estimate):
        pending = self._estimate
        if pending is None or pending["generation"] != generation:
            return
        self._estimate = None
        for cancel in pending["cancels"]:
            try:
                cancel()
            except Exception:
                # Cleanup cannot prevent the bounded service response. Planning
                # has no physical effect; generation fencing still drops replies.
                pass
        pending["callback"](estimate)
        self._pump()

    def estimate(self, mission, callback):
        now = self._advance()
        self._generation += 1
        generation = self._generation
        reason = (
            "agent shutdown" if self._closed else self._mission_reason(mission, now)
        )
        if len(self._queued_estimates) + bool(self._estimate) >= self.cache_limit:
            reason = "planning capacity exceeded"
        if reason:
            callback(CostEstimate(False, reason=reason))
            return generation
        pending = {
            "generation": generation,
            "deadline": now + self.timeout,
            "lengths": {},
            "cancels": [],
            "callback": callback,
            "mission": mission,
            "leg": -1,
        }
        self._queued_estimates.append(pending)
        self._pump()
        return generation

    def _pump(self):
        if self._pumping or self._closed:
            return
        self._pumping = True
        try:
            while self._estimate is None and self._queued_estimates:
                pending = self._queued_estimates.popleft()
                reason = self._mission_reason(pending["mission"], self.clock())
                if self.clock() >= pending["deadline"]:
                    reason = "path timeout"
                if reason:
                    pending["callback"](CostEstimate(False, reason=reason))
                    continue
                self._estimate = pending
                self._send_leg(pending, 0)
        finally:
            self._pumping = False

    def _send_leg(self, pending, index):
        generation, mission = pending["generation"], pending["mission"]
        pickup, dropoff = (
            self.stations[mission.pickup_station],
            self.stations[mission.dropoff_station],
        )
        pending["leg"] = index

        def received(length, reason):
            if self._estimate is not pending or pending["leg"] != index:
                return
            if self.clock() >= pending["deadline"]:
                self._finish(generation, CostEstimate(False, reason="path timeout"))
                return
            health = self._health(self._advance())
            if health or self.mode != "AVAILABLE" or self.payload_state != "EMPTY":
                self._finish(
                    generation,
                    CostEstimate(False, reason=health or "robot unavailable"),
                )
                return
            if reason or length is None:
                self._finish(
                    generation, CostEstimate(False, reason=reason or "path failed")
                )
                return
            try:
                pending["lengths"][index] = nonnegative(length, "path length")
                if len(pending["lengths"]) == 2:
                    result = self.cost.estimate(
                        PathLengths(pending["lengths"][0], pending["lengths"][1]),
                        self.energy.battery_percent,
                    )
                    self._finish(generation, result)
                else:
                    # Humble Nav2 has one current planner goal. Only dispatch the
                    # next leg after successful completion of the previous goal.
                    self._send_leg(pending, 1)
            except ValueError as error:
                self._finish(generation, CostEstimate(False, reason=str(error)))

        # Nav2 obtains its current map pose for the first leg. An idle AMCL
        # publisher need not republish a static pose; odometry receipts prove
        # sensor liveness without treating odom coordinates as map coordinates.
        start, goal = (None, pickup) if index == 0 else (pickup, dropoff)
        try:
            cancel = self.paths.compute(
                start, goal, self.frame_prefix + "map", received
            )
            if self._estimate is pending:
                pending["cancels"].append(cancel)
            elif cancel:
                cancel()
        except Exception as error:
            self._finish(
                generation, CostEstimate(False, reason=f"path unavailable: {error}")
            )

    def _cancel_estimate(self, generation, reason):
        if self._estimate and self._estimate["generation"] == generation:
            self._finish(generation, CostEstimate(False, reason=reason))
        else:
            for pending in tuple(self._queued_estimates):
                if pending["generation"] == generation:
                    self._queued_estimates.remove(pending)
                    pending["callback"](CostEstimate(False, reason=reason))
                    break

    def shutdown(self):
        self._closed = True
        for pending in tuple(self._queued_estimates):
            self._cancel_estimate(pending["generation"], "agent shutdown")
        if self._estimate:
            self._finish(
                self._estimate["generation"],
                CostEstimate(False, reason="agent shutdown"),
            )

    def _mission_reason(self, mission, now):
        if (
            not identifier(mission.mission_id)
            or not identifier(mission.part)
            or mission.pickup_station not in self.stations
            or mission.dropoff_station not in self.stations
            or mission.pickup_station == mission.dropoff_station
        ):
            return "invalid mission"
        health = self._health(now)
        if health or self.mode != "AVAILABLE" or self.payload_state != "EMPTY":
            return health or "robot unavailable"
        return ""

    def _cache_fresh(self, entry, now):
        return entry is not None and (
            now - entry["started"] < self.cache_age
            and self._distance - entry["distance"] <= self.cache_movement
            and entry["pose"] is not None
            and math.hypot(
                self.pose[0] - entry["pose"][0], self.pose[1] - entry["pose"][1]
            )
            <= self.cache_movement
            and abs(math.remainder(self.pose[2] - entry["pose"][2], 2 * math.pi)) <= 0.1
        )

    def cached_estimate(self, mission):
        """Return immediately; external planning callbacks fill a bounded LRU.

        The fleet retries pending candidates. No service callback waits for Nav2,
        and each successful read recomputes energy against the current battery.
        """
        self.tick()
        now = self.clock()
        reason = self._mission_reason(mission, now)
        if reason:
            return CostEstimate(False, reason=reason)
        key = (mission.pickup_station, mission.dropoff_station, mission.part)
        entry = self._cache.get(key)
        if self._cache_fresh(entry, now):
            self._cache.move_to_end(key)
            result = entry["result"]
            if result is None:
                return CostEstimate(False, reason="path pending")
            entry["consumed"] = True
            return (
                self.cost.estimate(result.path_lengths, self.energy.battery_percent)
                if result.path_lengths
                else result
            )
        if entry:
            del self._cache[key]
            self._cancel_estimate(entry["generation"], "stale route snapshot")
        if len(self._cache) >= self.cache_limit:
            # Do not discard a fresh result before its waiting fleet request has
            # read it. Overflow fails explicitly instead of causing cache churn.
            victim = next(
                (
                    k
                    for k, v in self._cache.items()
                    if v["result"] is not None
                    and (v["consumed"] or not self._cache_fresh(v, now))
                ),
                None,
            )
            if victim is None:
                return CostEstimate(False, reason="planning capacity exceeded")
            del self._cache[victim]
        entry = dict(
            started=now,
            distance=self._distance,
            pose=self.pose,
            result=None,
            generation=None,
            consumed=False,
        )
        self._cache[key] = entry

        def completed(result):
            if self._cache.get(key) is not entry:
                return
            entry["result"] = result

        entry["generation"] = self.estimate(mission, completed)
        return entry["result"] or CostEstimate(False, reason="path pending")

    def tick(self):
        self._advance()
        for pending in tuple(self._queued_estimates):
            if self.clock() >= pending["deadline"]:
                self._cancel_estimate(pending["generation"], "path timeout")
        if self._estimate and self.clock() >= self._estimate["deadline"]:
            self._finish(
                self._estimate["generation"], CostEstimate(False, reason="path timeout")
            )
