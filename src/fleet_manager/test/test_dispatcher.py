# SPDX-License-Identifier: Apache-2.0
"""Selection rejects unsafe candidates and honors explicit robot pins."""

from dataclasses import replace
import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def dispatch_api():
    try:
        models = importlib.import_module("fleet_manager.models")
        policy_type = importlib.import_module("fleet_manager.energy").EnergyPolicy
        config_type = importlib.import_module("fleet_manager.config").EnergyPolicyConfig
        dispatcher_type = importlib.import_module("fleet_manager.dispatcher").Dispatcher
    except ModuleNotFoundError:
        pytest.fail("deterministic fleet dispatcher is not implemented")
    return models, dispatcher_type(
        policy_type(config_type(20.0, 30.0, 80.0, "dock_01"))
    )


def robot(models, robot_id, **changes):
    return replace(
        models.RobotSnapshot(
            robot_id=robot_id,
            mode=models.RobotMode.AVAILABLE,
            pose=models.Pose2D(0.0, -3.0, 0.0),
            battery_percent=60.0,
            payload_state="EMPTY",
        ),
        **changes,
    )


def mission(models, requested_robot_id=None):
    return models.MissionRequest(
        "mission_01", "assembly", "inspection", "gear", requested_robot_id
    )


@pytest.mark.parametrize("order", [("amr_01", "amr_02"), ("amr_02", "amr_01")])
@pytest.mark.parametrize(
    "cost_01,cost_02,expected_robot",
    [(4.0, 3.0, "amr_02"), (3.0, 4.0, "amr_01"), (3.0, 3.0, "amr_01")],
)
def test_automatic_assignment_orders_by_cost_then_robot_id(
    order, cost_01, cost_02, expected_robot
):
    models, dispatcher = dispatch_api()
    robots = tuple(robot(models, robot_id) for robot_id in order)
    estimates = {
        "amr_01": models.CostEstimate(True, cost_01, 40.0),
        "amr_02": models.CostEstimate(True, cost_02, 40.0),
    }
    decision = dispatcher.choose(mission(models), robots, estimates)
    assert decision.robot_id == expected_robot
    assert decision.reason == "assigned"


def test_pinned_available_robot_is_chosen_even_with_higher_cost():
    models, dispatcher = dispatch_api()
    robots = (robot(models, "amr_01"), robot(models, "amr_02"))
    estimates = {
        "amr_01": models.CostEstimate(True, 1.0, 80.0),
        "amr_02": models.CostEstimate(True, 10.0, 20.0),
    }
    assert (
        dispatcher.choose(mission(models, "amr_02"), robots, estimates).robot_id
        == "amr_02"
    )


@pytest.mark.parametrize(
    "changes",
    [
        {"mode": "RESERVED"},
        {"mode": "EXECUTING"},
        {"mode": "WAITING_FOR_RESOURCE"},
        {"mode": "DOCKING"},
        {"mode": "CHARGING"},
        {"mode": "UNHEALTHY"},
        {"mode": "OFFLINE"},
        {"mode": "RECOVERY_REQUIRED"},
        {"health": "UNHEALTHY"},
        {"health": "OFFLINE"},
        {"pose": None},
        {"payload_state": "UNKNOWN"},
        {"fault": "drive_fault"},
    ],
)
def test_pinned_unavailable_robot_never_falls_back(changes):
    models, dispatcher = dispatch_api()
    robots = (robot(models, "amr_01"), robot(models, "amr_02", **changes))
    estimates = {
        "amr_01": models.CostEstimate(True, 1.0, 80.0),
        "amr_02": models.CostEstimate(True, 10.0, 40.0),
    }
    decision = dispatcher.choose(mission(models, "amr_02"), robots, estimates)
    assert decision.robot_id is None
    assert decision.reason == "requested robot unavailable"


@pytest.mark.parametrize("requested_robot_id", [None, "amr_01"])
@pytest.mark.parametrize(
    "feasible,cost,battery",
    [
        (False, 1.0, 80.0),
        (True, 1.0, 19.9),
        (True, float("nan"), 80.0),
        (True, float("inf"), 80.0),
        (True, -1.0, 80.0),
    ],
)
def test_unsafe_estimate_cannot_authorize_assignment(
    requested_robot_id, feasible, cost, battery
):
    models, dispatcher = dispatch_api()
    decision = dispatcher.choose(
        mission(models, requested_robot_id),
        (robot(models, "amr_01"),),
        {"amr_01": models.CostEstimate(feasible, cost, battery)},
    )
    assert decision.robot_id is None


def test_exact_reserve_and_zero_path_cost_are_eligible():
    models, dispatcher = dispatch_api()
    decision = dispatcher.choose(
        mission(models),
        (robot(models, "amr_01"),),
        {"amr_01": models.CostEstimate(True, 0.0, 20.0)},
    )
    assert decision.robot_id == "amr_01"


@pytest.mark.parametrize("requested_robot_id", ["amr_01", "amr_missing"])
def test_missing_pinned_estimate_or_robot_never_falls_back(requested_robot_id):
    models, dispatcher = dispatch_api()
    decision = dispatcher.choose(
        mission(models, requested_robot_id),
        (robot(models, "amr_01"), robot(models, "amr_02")),
        {"amr_02": models.CostEstimate(True, 1.0, 80.0)},
    )
    assert decision.robot_id is None
    assert decision.reason == "requested robot unavailable"


def test_missing_cost_estimate_excludes_only_that_candidate():
    models, dispatcher = dispatch_api()
    decision = dispatcher.choose(
        mission(models),
        (robot(models, "amr_01"), robot(models, "amr_02")),
        {"amr_02": models.CostEstimate(True, 10.0, 80.0)},
    )
    assert decision.robot_id == "amr_02"


def test_no_eligible_robot_returns_queued_assignment_reason():
    models, dispatcher = dispatch_api()
    decision = dispatcher.choose(mission(models), (), {})
    assert decision.robot_id is None
    assert decision.reason == "no eligible robot"


def test_automatic_assignment_excludes_lower_cost_unhealthy_robot():
    models, dispatcher = dispatch_api()
    decision = dispatcher.choose(
        mission(models),
        (robot(models, "amr_01", health="UNHEALTHY"), robot(models, "amr_02")),
        {
            "amr_01": models.CostEstimate(True, 1.0, 80.0),
            "amr_02": models.CostEstimate(True, 10.0, 80.0),
        },
    )
    assert decision.robot_id == "amr_02"
