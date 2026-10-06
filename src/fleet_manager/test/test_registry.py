# SPDX-License-Identifier: Apache-2.0
"""Receipt time, not telemetry time, governs robot assignment readiness."""

from dataclasses import replace
import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def registry_api():
    try:
        return (
            importlib.import_module("fleet_manager.models"),
            importlib.import_module("fleet_manager.registry").RobotRegistry,
        )
    except ModuleNotFoundError:
        pytest.fail("robot registry and snapshot models are not implemented")


def ready_robot(models, robot_id="amr_01", **changes):
    return replace(
        models.RobotSnapshot(
            robot_id=robot_id,
            mode=models.RobotMode.AVAILABLE,
            pose=models.Pose2D(0.0, -3.0, 0.0),
            battery_percent=60.0,
            payload_state="EMPTY",
            stamp=10.0,
        ),
        **changes,
    )


@pytest.mark.parametrize(
    "now,health",
    [
        (102.999, "ONLINE"),
        (103.0, "UNHEALTHY"),
        (104.999, "UNHEALTHY"),
        (105.0, "OFFLINE"),
    ],
)
def test_health_expires_at_monotonic_deadlines(now, health):
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    registry.observe(ready_robot(models), received_at=100.0)
    state = registry.get("amr_01", now=now)
    assert state.health == health
    assert state.mode == "AVAILABLE"


@pytest.mark.parametrize("stamp", [10.0, 9.0])
def test_paused_or_out_of_order_ros_stamp_does_not_age_heartbeat(stamp):
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    registry.observe(ready_robot(models), received_at=100.0)
    registry.observe(ready_robot(models, stamp=stamp), received_at=104.0)
    state = registry.get("amr_01", now=106.0)
    assert state.health == "ONLINE"
    assert state.mode == "AVAILABLE"
    assert state.stamp == stamp
    assert tuple(robot.robot_id for robot in registry.eligible(106.0)) == ("amr_01",)


def test_fresh_heartbeat_recovers_health_without_mutating_observation():
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    observation = ready_robot(models)
    registry.observe(observation, received_at=100.0)
    assert registry.get("amr_01", now=105.0).health == "OFFLINE"
    assert observation.mode == "AVAILABLE"
    registry.observe(ready_robot(models, stamp=8.0), received_at=106.0)
    assert registry.get("amr_01", now=106.5).health == "ONLINE"


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
        {"pose": None},
        {"payload_state": "UNKNOWN"},
        {"payload_state": ""},
        {"payload_state": None},
        {"fault": "drive_fault"},
        {"health": "UNHEALTHY"},
        {"health": "OFFLINE"},
    ],
)
def test_registry_excludes_robot_missing_assignment_readiness(changes):
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    registry.observe(ready_robot(models, **changes), received_at=100.0)
    assert registry.eligible(now=101.0) == ()


def test_registry_excludes_stale_robot_and_keeps_fresh_robot():
    models, registry_type = registry_api()
    registry = registry_type(("amr_01", "amr_02"))
    registry.observe(ready_robot(models), received_at=100.0)
    registry.observe(ready_robot(models, "amr_02"), received_at=102.0)
    assert tuple(robot.robot_id for robot in registry.eligible(103.0)) == ("amr_02",)


def test_unknown_robot_is_not_invented():
    _, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    with pytest.raises(KeyError, match="amr_missing"):
        registry.get("amr_missing", now=100.0)


def test_configured_robot_without_heartbeat_is_offline():
    _, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    assert registry.get("amr_01", now=100.0).health == "OFFLINE"
    assert registry.eligible(now=100.0) == ()


def test_unconfigured_observation_is_rejected():
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    with pytest.raises(KeyError, match="amr_missing"):
        registry.observe(ready_robot(models, "amr_missing"), received_at=100.0)


def test_older_receipt_does_not_replace_newer_robot_observation():
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    registry.observe(ready_robot(models, stamp=5.0), received_at=104.0)
    registry.observe(
        ready_robot(models, mode="EXECUTING", stamp=50.0), received_at=100.0
    )
    state = registry.get("amr_01", now=106.0)
    assert state.mode == "AVAILABLE"
    assert state.health == "ONLINE"
    assert state.stamp == 5.0


def test_heartbeat_aging_cannot_downgrade_reported_offline_health():
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    registry.observe(ready_robot(models, health="OFFLINE"), received_at=100.0)
    assert registry.get("amr_01", now=103.0).health == "OFFLINE"


@pytest.mark.parametrize("received_at", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_receipt_cannot_keep_robot_online_forever(received_at):
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    with pytest.raises(ValueError, match="finite"):
        registry.observe(ready_robot(models), received_at=received_at)
    assert registry.get("amr_01", now=100.0).health == "OFFLINE"


@pytest.mark.parametrize("now", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_health_clock_is_rejected(now):
    models, registry_type = registry_api()
    registry = registry_type(("amr_01",))
    registry.observe(ready_robot(models), received_at=100.0)
    with pytest.raises(ValueError, match="finite"):
        registry.get("amr_01", now=now)
