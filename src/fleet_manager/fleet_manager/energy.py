# SPDX-License-Identifier: Apache-2.0
"""Apply configured reserve and charging boundaries without ROS dependencies."""

import math

from fleet_manager.config import EnergyPolicyConfig
from fleet_manager.models import CostEstimate, RobotSnapshot


def _valid_percent(value: float | None) -> bool:
    return value is not None and math.isfinite(value) and 0.0 <= value <= 100.0


class EnergyPolicy:
    def __init__(self, config: EnergyPolicyConfig):
        self._config = config

    def mission_is_safe(self, estimate: CostEstimate) -> bool:
        return (
            estimate.feasible
            and _valid_percent(estimate.predicted_final_battery)
            and estimate.predicted_final_battery >= self._config.reserve_percent
        )

    def should_charge(self, state: RobotSnapshot) -> bool:
        return (
            _valid_percent(state.battery_percent)
            and state.battery_percent < self._config.charge_below_percent
        )

    def charge_complete(self, state: RobotSnapshot) -> bool:
        return (
            _valid_percent(state.battery_percent)
            and state.battery_percent >= self._config.charge_until_percent
        )
