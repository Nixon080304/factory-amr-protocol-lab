"""Catch omitted drain, unauthorized charge, and invalid energy inputs."""

import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def api():
    try:
        return importlib.import_module("robot_agent.energy_model")
    except ModuleNotFoundError:
        pytest.fail("EnergyModel behavior is missing")


def test_idle_movement_operations_and_prediction_are_independent():
    module = api()
    energy = module.EnergyModel(80, module.EnergyConfig(0.1, 2, 3, 4, 20, 10))
    assert energy.advance(5, 2, "AVAILABLE") == 75.5
    assert energy.operation() == 72.5
    assert energy.predict(7, 2) == 52.5
    assert energy.battery_percent == 72.5


@pytest.mark.parametrize(
    "mode,lease,contact,want",
    [
        ("CHARGING", True, True, 54),
        ("CHARGING", False, True, 49),
        ("CHARGING", True, False, 49),
        ("AVAILABLE", True, True, 49),
    ],
)
def test_charge_requires_mode_lease_and_contact(mode, lease, contact, want):
    energy = api().EnergyModel(50, api().EnergyConfig(1, 0, 0, 5, 20, 10))
    assert energy.advance(1, 0, mode, lease_valid=lease, contact=contact) == want


def test_charge_and_drain_clamp_to_physical_percent():
    energy = api().EnergyModel(99)
    assert energy.advance(1000, 0, "CHARGING", lease_valid=True, contact=True) == 100
    assert energy.advance(0, 100000, "EXECUTING") == 0
    assert energy.operation() == 0
    assert energy.predict(10000, 2) == 0


@pytest.mark.parametrize("value", [-1, float("nan"), float("inf"), True])
def test_invalid_energy_inputs_are_rejected_without_mutation(value):
    energy = api().EnergyModel(50)
    for call in (
        lambda: energy.advance(value, 0, "AVAILABLE"),
        lambda: energy.advance(0, value, "AVAILABLE"),
        lambda: energy.predict(value, 2),
    ):
        with pytest.raises(ValueError):
            call()
        assert energy.battery_percent == 50


@pytest.mark.parametrize(
    "field",
    [
        "idle_percent_per_sec",
        "move_percent_per_m",
        "operation_percent",
        "charge_percent_per_sec",
        "reserve_percent",
        "dock_allowance_m",
    ],
)
def test_invalid_config_is_rejected(field):
    with pytest.raises(ValueError):
        api().EnergyConfig(**{field: float("nan")})


@pytest.mark.parametrize("value", [-1, 101, float("nan"), True])
def test_invalid_initial_battery_is_rejected(value):
    with pytest.raises(ValueError):
        api().EnergyModel(value)


@pytest.mark.parametrize("count", [-1, 1.5, True])
def test_prediction_rejects_noninteger_operations(count):
    with pytest.raises(ValueError):
        api().EnergyModel(50).predict(1, count)


def test_overflowing_numeric_input_is_validation_error():
    with pytest.raises(ValueError):
        api().EnergyModel(50).advance(10**1000, 0, "AVAILABLE")
