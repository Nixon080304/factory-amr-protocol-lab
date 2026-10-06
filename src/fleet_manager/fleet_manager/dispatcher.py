# SPDX-License-Identifier: Apache-2.0
"""Choose one eligible robot deterministically from precomputed estimates."""

import math
from typing import Mapping, Sequence

from fleet_manager.energy import EnergyPolicy
from fleet_manager.models import (
    AssignmentDecision,
    CostEstimate,
    MissionRequest,
    RobotSnapshot,
    robot_is_ready,
)


class Dispatcher:
    def __init__(self, energy_policy: EnergyPolicy):
        self._energy_policy = energy_policy

    def choose(
        self,
        mission: MissionRequest,
        robots: Sequence[RobotSnapshot],
        estimates: Mapping[str, CostEstimate],
    ) -> AssignmentDecision:
        candidates = []
        for robot in robots:
            if (
                mission.requested_robot_id
                and robot.robot_id != mission.requested_robot_id
            ):
                continue
            estimate = estimates.get(robot.robot_id)
            if (
                robot_is_ready(robot)
                and estimate is not None
                and math.isfinite(estimate.path_cost)
                and estimate.path_cost >= 0.0
                and self._energy_policy.mission_is_safe(estimate)
            ):
                candidates.append((estimate.path_cost, robot.robot_id))
        if not candidates:
            reason = (
                "requested robot unavailable"
                if mission.requested_robot_id
                else "no eligible robot"
            )
            return AssignmentDecision(None, reason)
        _, robot_id = min(candidates)
        return AssignmentDecision(robot_id, "assigned")
