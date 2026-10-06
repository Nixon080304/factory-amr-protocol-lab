"""Catch incomplete mission prediction and reserve-boundary errors."""

import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def api():
    try:
        return importlib.import_module("robot_agent.cost_model")
    except ModuleNotFoundError:
        pytest.fail("CostModel behavior is missing")


@pytest.mark.parametrize("battery,want", [(51.9, False), (52, True), (52.1, True)])
def test_prediction_counts_both_paths_operations_dock_and_exact_reserve(battery, want):
    module = api()
    config = module.EnergyConfig(0, 2, 3, 1, 20, 5)
    estimate = module.CostModel(config).estimate(module.PathLengths(3, 5), battery)
    assert estimate.path_cost == 3
    assert estimate.predicted_final_battery == pytest.approx(battery - 32)
    assert estimate.feasible is want
    assert bool(estimate.reason) is not want


@pytest.mark.parametrize(
    "pickup,dropoff,battery",
    [(-1, 2, 80), (1, float("nan"), 80), (1, 2, 101), (1, 2, True)],
)
def test_invalid_estimate_never_becomes_authoritative_success(pickup, dropoff, battery):
    with pytest.raises(ValueError):
        module = api()
        module.CostModel().estimate(module.PathLengths(pickup, dropoff), battery)
