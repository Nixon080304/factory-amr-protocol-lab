# SPDX-License-Identifier: Apache-2.0
"""Energy boundaries use independently specified battery outcomes."""

import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def energy_api():
    try:
        return (
            importlib.import_module("fleet_manager.models"),
            importlib.import_module("fleet_manager.energy").EnergyPolicy,
            importlib.import_module("fleet_manager.config").EnergyPolicyConfig,
        )
    except ModuleNotFoundError:
        pytest.fail("energy eligibility policy is not implemented")


@pytest.mark.parametrize(
    "feasible,battery,safe",
    [
        (True, 19.9, False),
        (True, 20.0, True),
        (True, 100.0, True),
        (False, 80.0, False),
        (True, float("nan"), False),
        (True, float("inf"), False),
        (True, -1.0, False),
        (True, 101.0, False),
    ],
)
def test_mission_requires_feasible_estimate_and_reserve(feasible, battery, safe):
    models, policy_type, config_type = energy_api()
    policy = policy_type(config_type(20.0, 30.0, 80.0, "dock_01"))
    estimate = models.CostEstimate(feasible, 10.0, battery)
    assert policy.mission_is_safe(estimate) is safe


@pytest.mark.parametrize(
    "battery,should_charge,complete",
    [
        (29.9, True, False),
        (30.0, False, False),
        (79.9, False, False),
        (80.0, False, True),
        (100.0, False, True),
        (None, False, False),
        (float("nan"), False, False),
        (float("inf"), False, False),
        (-1.0, False, False),
        (101.0, False, False),
    ],
)
def test_charging_starts_below_threshold_and_finishes_at_target(
    battery, should_charge, complete
):
    models, policy_type, config_type = energy_api()
    policy = policy_type(config_type(20.0, 30.0, 80.0, "dock_01"))
    state = models.RobotSnapshot("amr_01", battery_percent=battery)
    assert policy.should_charge(state) is should_charge
    assert policy.charge_complete(state) is complete


def test_energy_policy_uses_validated_configuration_thresholds():
    models, policy_type, config_type = energy_api()
    policy = policy_type(config_type(25.0, 35.0, 85.0, "dock_01"))
    assert policy.mission_is_safe(models.CostEstimate(True, 1.0, 24.9)) is False
    assert policy.mission_is_safe(models.CostEstimate(True, 1.0, 25.0)) is True
    assert policy.should_charge(models.RobotSnapshot("amr_01", battery_percent=34.9))
    assert not policy.should_charge(
        models.RobotSnapshot("amr_01", battery_percent=35.0)
    )
    assert not policy.charge_complete(
        models.RobotSnapshot("amr_01", battery_percent=84.9)
    )
    assert policy.charge_complete(models.RobotSnapshot("amr_01", battery_percent=85.0))
