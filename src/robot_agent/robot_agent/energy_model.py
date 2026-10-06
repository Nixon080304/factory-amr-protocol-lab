# SPDX-License-Identifier: Apache-2.0
"""Deterministic simulation percentages; no ROS or transport dependencies."""

from dataclasses import dataclass
import math


def nonnegative(value, name):
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be finite and nonnegative")
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} must be finite and nonnegative") from error
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return number


def percentage(value, name="battery_percent"):
    value = nonnegative(value, name)
    if value > 100:
        raise ValueError(f"{name} must be at most 100")
    return value


@dataclass(frozen=True)
class EnergyConfig:
    idle_percent_per_sec: float = 0.001
    move_percent_per_m: float = 0.1
    operation_percent: float = 0.5
    charge_percent_per_sec: float = 1.0
    reserve_percent: float = 20.0
    dock_allowance_m: float = 10.0

    def __post_init__(self):
        for name, value in vars(self).items():
            nonnegative(value, name)
        percentage(self.reserve_percent, "reserve_percent")


class EnergyModel:
    def __init__(self, battery_percent, config=None):
        self.battery_percent = percentage(battery_percent)
        self.config = config or EnergyConfig()

    def advance(
        self, elapsed_sec, distance_m, mode, *, lease_valid=False, contact=False
    ):
        elapsed = nonnegative(elapsed_sec, "elapsed_sec")
        distance = nonnegative(distance_m, "distance_m")
        change = -elapsed * self.config.idle_percent_per_sec
        change -= distance * self.config.move_percent_per_m
        if mode == "CHARGING" and lease_valid is True and contact is True:
            change += elapsed * self.config.charge_percent_per_sec
        self.battery_percent = min(100.0, max(0.0, self.battery_percent + change))
        return self.battery_percent

    def operation(self):
        self.battery_percent = max(
            0.0, self.battery_percent - self.config.operation_percent
        )
        return self.battery_percent

    def predict(self, path_cost, operation_count):
        path = nonnegative(path_cost, "path_cost")
        if type(operation_count) is not int or operation_count < 0:
            raise ValueError("operation_count must be a nonnegative integer")
        return max(
            0.0,
            self.battery_percent
            - path * self.config.move_percent_per_m
            - operation_count * self.config.operation_percent,
        )
