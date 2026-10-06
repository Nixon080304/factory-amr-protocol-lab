# SPDX-License-Identifier: Apache-2.0
"""Predict both mission legs and a conservative configured route to the dock."""

from dataclasses import dataclass

from robot_agent.energy_model import EnergyConfig, EnergyModel, nonnegative


@dataclass(frozen=True)
class PathLengths:
    to_pickup: float
    pickup_to_dropoff: float

    def __post_init__(self):
        nonnegative(self.to_pickup, "to_pickup")
        nonnegative(self.pickup_to_dropoff, "pickup_to_dropoff")


@dataclass(frozen=True)
class CostEstimate:
    feasible: bool
    path_cost: float = 0.0
    predicted_final_battery: float = 0.0
    reason: str = ""
    path_lengths: PathLengths | None = None


class CostModel:
    def __init__(self, config=None):
        self.config = config or EnergyConfig()

    def estimate(self, path_lengths, battery_percent):
        energy = EnergyModel(battery_percent, self.config)
        total = (
            path_lengths.to_pickup
            + path_lengths.pickup_to_dropoff
            + self.config.dock_allowance_m
        )
        predicted = energy.predict(total, 2)
        feasible = predicted >= self.config.reserve_percent
        return CostEstimate(
            feasible,
            path_lengths.to_pickup,
            predicted,
            "" if feasible else "insufficient reserve",
            path_lengths,
        )
