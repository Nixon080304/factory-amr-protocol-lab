# SPDX-License-Identifier: Apache-2.0
"""Track configured robots and age their health using monotonic receipts."""

from dataclasses import replace
import math
from typing import Sequence

from fleet_manager.models import RobotHealth, RobotSnapshot, robot_is_ready


class RobotRegistry:
    def __init__(self, robot_ids: Sequence[str]):
        self._states = {
            robot_id: RobotSnapshot(robot_id, health=RobotHealth.OFFLINE)
            for robot_id in robot_ids
        }
        self._receipts: dict[str, float] = {}

    def observe(self, state: RobotSnapshot, received_at: float) -> None:
        if state.robot_id not in self._states:
            raise KeyError(state.robot_id)
        if not math.isfinite(received_at):
            raise ValueError("received_at must be finite monotonic time")
        previous = self._receipts.get(state.robot_id)
        if previous is not None and received_at < previous:
            return
        self._states[state.robot_id] = state
        self._receipts[state.robot_id] = received_at

    def get(self, robot_id: str, now: float) -> RobotSnapshot:
        """Project heartbeat health while preserving reported operational mode."""
        state = self._states[robot_id]
        if not math.isfinite(now):
            raise ValueError("now must be finite monotonic time")
        received_at = self._receipts.get(robot_id)
        if (
            state.health == RobotHealth.OFFLINE
            or received_at is None
            or now - received_at >= 5.0
        ):
            return replace(state, health=RobotHealth.OFFLINE)
        if now - received_at >= 3.0:
            return replace(state, health=RobotHealth.UNHEALTHY)
        return state

    def eligible(self, now: float) -> tuple[RobotSnapshot, ...]:
        states = (self.get(robot_id, now) for robot_id in self._states)
        return tuple(state for state in states if robot_is_ready(state))
