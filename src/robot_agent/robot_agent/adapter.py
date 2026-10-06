# SPDX-License-Identifier: Apache-2.0
"""Robot policy driven by wall receipts and asynchronous production paths."""

from dataclasses import dataclass
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
    """One current estimate with generation fences and fail-closed ownership.

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
        if not self.stale_sec or not 0 < self.timeout < 1 or not self.max_step:
            raise ValueError(
                "health and path bounds must be positive; estimate timeout must be below 1 s"
            )
        if self.cache_age <= 1.0:
            raise ValueError("cache_max_age_sec must exceed the fleet's 1 s retry")
        self.energy = EnergyModel(battery_percent, energy_config)
        self.cost = CostModel(self.energy.config)
        self.mode, self.payload_state, self.mission_id = "AVAILABLE", "UNKNOWN", ""
        self.pose = None
        self._pose_stamp = self._odom_stamp = -1
        self._pose_receipt = self._odom_receipt = None
        self._odom = None
        self._distance = 0.0
        self._wall = self.clock()
        self._generation = 0
        self._estimate = None
        self._transfer = None
        self._terminal = False
        self._retired_missions = set()
        self._phase = ""
        self._lease_until = 0.0
        self._dock_contact = False
        self._cache = None
        self._cache_generation = 0
        self._cache_pending_key = None
        self._cache_started = 0.0
        self._cache_distance = 0.0
        self._cache_pose = None

    def set_charge_authorization(self, *, lease_until, contact):
        deadline = nonnegative(lease_until, "lease_until")
        if type(contact) is not bool:
            raise ValueError("contact must be bool")
        self._advance()
        self._lease_until = deadline
        self._dock_contact = contact

    def _advance(self):
        now = self.clock()
        elapsed = max(0.0, now - self._wall)
        # Split at authorization expiry so a long timer gap cannot charge past it.
        authorized = min(elapsed, max(0.0, self._lease_until - self._wall))
        self.energy.advance(
            authorized, 0, self.mode, lease_valid=True, contact=self._dock_contact
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

    def mission_state(self, state):
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
            if state in ("LOADING", "UNLOADING") and state != self._phase:
                self.payload_state = "UNKNOWN"
            if state != "RECOVERING":
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
                return False
            if not self.mission_id:
                self.mission_id = message.mission_id
            return self.mission_state(message.detail)
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
                self._terminal = False
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

    def estimate(self, mission, callback):
        now = self._advance()
        if self._estimate is not None:
            self._finish(
                self._generation, CostEstimate(False, reason="superseded request")
            )
        self._generation += 1
        generation = self._generation
        reason = self._mission_reason(mission, now)
        if reason:
            callback(CostEstimate(False, reason=reason))
            return
        pending = {
            "generation": generation,
            "deadline": now + self.timeout,
            "lengths": {},
            "cancels": [],
            "callback": callback,
        }
        self._estimate = pending
        pickup, dropoff = (
            self.stations[mission.pickup_station],
            self.stations[mission.dropoff_station],
        )

        def received(index, length, reason):
            if self._estimate is not pending:
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
            except ValueError as error:
                self._finish(generation, CostEstimate(False, reason=str(error)))

        # Nav2 obtains its current map pose for the first leg. An idle AMCL
        # publisher need not republish a static pose; odometry receipts prove
        # sensor liveness without treating odom coordinates as map coordinates.
        for index, (start, goal) in enumerate(((None, pickup), (pickup, dropoff))):
            if self._estimate is not pending:
                break
            try:
                cancel = self.paths.compute(
                    start,
                    goal,
                    self.frame_prefix + "map",
                    lambda length, reason="", index=index: received(
                        index, length, reason
                    ),
                )
                if self._estimate is pending:
                    pending["cancels"].append(cancel)
                elif cancel:
                    cancel()
            except Exception as error:
                self._finish(
                    generation, CostEstimate(False, reason=f"path unavailable: {error}")
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

    def cached_estimate(self, mission):
        """Return immediately; external planning callbacks fill one bounded cache.

        The fleet retries pending candidates. No service callback waits for Nav2,
        and each successful read recomputes energy against the current battery.
        """
        self.tick()
        now = self.clock()
        reason = self._mission_reason(mission, now)
        if reason:
            return CostEstimate(False, reason=reason)
        key = (mission.pickup_station, mission.dropoff_station, mission.part)
        fresh = (
            now - self._cache_started < self.cache_age
            and self._distance - self._cache_distance <= self.cache_movement
            and self._cache_pose is not None
            and math.hypot(
                self.pose[0] - self._cache_pose[0], self.pose[1] - self._cache_pose[1]
            )
            <= self.cache_movement
            and abs(math.remainder(self.pose[2] - self._cache_pose[2], 2 * math.pi))
            <= 0.1
        )
        if self._cache is not None and self._cache[0] == key and fresh:
            result = self._cache[1]
            return (
                self.cost.estimate(result.path_lengths, self.energy.battery_percent)
                if result.path_lengths
                else result
            )
        if self._cache_pending_key == key and self._estimate is not None and fresh:
            return CostEstimate(False, reason="path pending")
        self._cache_generation += 1
        token = self._cache_generation
        self._cache, self._cache_pending_key = None, key
        self._cache_started, self._cache_distance = now, self._distance
        self._cache_pose = self.pose
        started_distance = self._distance

        def completed(result):
            if token != self._cache_generation:
                return
            self._cache_pending_key = None
            if self._distance - started_distance <= self.cache_movement:
                self._cache = (key, result)

        self.estimate(mission, completed)
        if self._cache is not None:
            return self._cache[1]
        return CostEstimate(False, reason="path pending")

    def tick(self):
        self._advance()
        if self._estimate and self.clock() >= self._estimate["deadline"]:
            self._finish(self._generation, CostEstimate(False, reason="path timeout"))
