# SPDX-License-Identifier: Apache-2.0
"""Validate fleet inputs before consumers can launch or assign robots."""

from copy import deepcopy
from dataclasses import FrozenInstanceError
import importlib
from pathlib import Path
import sys

import pytest
import yaml


def test_configurable_energy_rates_drive_fleet_agent_predictions(tmp_path):
    # Ignoring simulation rates would make cost and live drain disagree.
    source = Path(__file__).resolve().parents[2] / "factory_bringup/config/fleet.yaml"
    data = yaml.safe_load(source.read_text())
    data["energy"].update(
        idle_percent_per_sec=0.02,
        move_percent_per_m=0.4,
        operation_percent=1.5,
        charge_percent_per_sec=2.0,
        dock_allowance_m=15.0,
    )
    path = tmp_path / "fleet.yaml"
    path.write_text(yaml.safe_dump(data))
    config = loader()(path)
    assert config.energy.idle_percent_per_sec == 0.02
    assert config.energy.move_percent_per_m == 0.4
    assert config.energy.operation_percent == 1.5
    assert config.energy.charge_percent_per_sec == 2.0
    assert config.energy.dock_allowance_m == 15.0


@pytest.mark.parametrize("value", [-1, float("nan"), float("inf"), True])
def test_energy_rate_validation_rejects_invalid_prediction_inputs(tmp_path, value):
    source = Path(__file__).resolve().parents[2] / "factory_bringup/config/fleet.yaml"
    data = yaml.safe_load(source.read_text())
    data["energy"]["move_percent_per_m"] = value
    path = tmp_path / "fleet.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="move_percent_per_m"):
        loader()(path)


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))


def terminal_bay_data():
    data = yaml.safe_load(
        (PACKAGE.parent / "factory_bringup/config/fleet.yaml").read_text()
    )
    data.pop("station_exit", None)
    # Station-boundary cases isolate parking validation. Dock cases add their
    # own complete exit identities and actual charging-origin paths below.
    for dock in data["docks"].values():
        dock.pop("exit_poses", None)
        dock.pop("departure_stations", None)
    data["station_approach"] = {
        "assembly": [-3.0, 0.8, 1.5707963267948966],
        "inspection": [3.0, 0.8, 1.5707963267948966],
    }
    data["station_exit_poses"] = {
        "assembly": {
            "amr_01": [-4.2, 1, 3.141592653589793],
            "amr_02": [-4.2, 0, 3.141592653589793],
        },
        "inspection": {"amr_01": [4.2, 0, 0], "amr_02": [5.0, 2.4, 0]},
    }
    return data


def test_distinct_robot_terminal_bays_are_immutable_and_complete(tmp_path):
    config = load_data(tmp_path, terminal_bay_data())
    assert config.station_approach["inspection"].x == 3.0
    assert set(config.station_exit_poses["inspection"]) == {"amr_01", "amr_02"}
    assert config.station_exit_poses["inspection"]["amr_02"].y == 2.4
    with pytest.raises(TypeError):
        config.station_exit_poses["inspection"]["extra"] = None


def dock_exit_data():
    data = terminal_bay_data()
    data["robots"][0]["spawn"] = [-1.1, -3.4, 0.0]
    data["robots"][1]["spawn"] = [5.2, -3.4, 3.141592653589793]
    data["robots"][1]["battery_start_percent"] = 25.0
    data["docks"]["dock_01"]["staging_pose"] = [5.4, -1.4, 3.141592653589793]
    data["docks"]["dock_01"]["charging_pose"] = [4.0, -2.0, 3.141592653589793]
    data["docks"]["dock_01"]["departure_stations"] = ["inspection"]
    data["docks"]["dock_01"]["exit_poses"] = {
        "amr_01": [4.2, 0.0, 0.0],
        "amr_02": [5.2, -3.4, 3.141592653589793],
    }
    return data


@pytest.mark.parametrize(
    "spawn,renamed,accepted",
    [
        ([1.0, -3.4, 0.0], False, False),
        ([-1.0, -3.4, 0.0], False, False),
        ([-1.1, -3.4, 0.0], False, True),
        ([1.0, -3.4, 0.0], True, False),
    ],
)
def test_initial_route_excludes_other_idle_startup_footprints(
    tmp_path, spawn, renamed, accepted
):
    # A leased robot must not drive through a queued robot's idle startup bay.
    # x=1 crosses the first approach; x=-1 clears it but not staging-to-exit.
    data = dock_exit_data()
    data["robots"][0]["spawn"] = spawn
    if renamed:
        identities = {"amr_01": "cart_worker", "amr_02": "cart_charger"}
        for robot in data["robots"]:
            robot_id = identities[robot["robot_id"]]
            robot.update(
                robot_id=robot_id, namespace="/" + robot_id, frame_prefix=robot_id + "/"
            )
        for poses in data["station_exit_poses"].values():
            for previous, current in identities.items():
                poses[current] = poses.pop(previous)
        poses = data["docks"]["dock_01"]["exit_poses"]
        for previous, current in identities.items():
            poses[current] = poses.pop(previous)
    if accepted:
        assert load_data(tmp_path, data).robots[0].spawn.x == -1.1
    else:
        with pytest.raises(ValueError, match="initial route.*idle startup bays"):
            load_data(tmp_path, data)


@pytest.mark.parametrize(
    "stations", [["unknown"], ["inspection", "inspection"], "inspection"]
)
def test_dock_departure_stations_require_distinct_configured_terminal_bays(
    tmp_path, stations
):
    data = dock_exit_data()
    data["docks"]["dock_01"]["departure_stations"] = stations
    with pytest.raises(ValueError, match="departure_stations"):
        load_data(tmp_path, data)


@pytest.mark.parametrize(
    "start,end,bounds,expected",
    [
        ([-2, 0], [2, 0], [-1, -1, 1, 1], 0),
        ([-2, 2], [2, 2], [-1, -1, 1, 1], 1),
        ([-2, 1], [2, 1], [-1, -1, 1, 1], 0),
        ([-2, 2], [-2, 2], [-1, -1, 1, 1], 2**0.5),
        ([-5, -2.2], [0, -3.5], [-0.8, -2.65, 0.8, -1.75], 3.21 / 26.69**0.5),
        ([0, -3.5], [-5, -2.2], [-0.8, -2.65, 0.8, -1.75], 3.21 / 26.69**0.5),
    ],
)
def test_segment_rectangle_clearance_is_exact_not_bounding_envelope(
    start, end, bounds, expected
):
    from fleet_manager.config import Pose2D
    from fleet_manager import config

    assert config._segment_bounds_distance(
        Pose2D(*start, 0), Pose2D(*end, 0), bounds
    ) == pytest.approx(expected, abs=1e-12)


def test_dock_exit_bays_are_complete_immutable_and_clear_all_travel_paths(tmp_path):
    config = load_data(tmp_path, dock_exit_data())
    bays = config.docks["dock_01"].exit_poses
    assert set(bays) == {"amr_01", "amr_02"}
    assert bays["amr_01"].x == 4.2 and bays["amr_02"].y == -3.4
    with pytest.raises(TypeError):
        bays["unknown"] = bays["amr_01"]
    sys.path.insert(0, str(PACKAGE.parent / "factory_bringup"))
    from factory_bringup.fleet_launch import agent_parameters

    for robot in config.robots:
        pose = bays[robot.robot_id]
        assert agent_parameters(config, robot, {})["dock.exit_pose"] == [
            pose.x,
            pose.y,
            pose.yaw,
        ]


@pytest.mark.parametrize(
    "case",
    [
        "missing",
        "unknown",
        "too_close",
        "charging_path",
        "parking_path",
        "dock_staging",
        "spawn_path",
        "staging_path",
    ],
)
def test_dock_exit_bays_reject_unsafe_identity_or_travel_paths(tmp_path, case):
    data = dock_exit_data()
    bays = data["docks"]["dock_01"]["exit_poses"]
    if case == "missing":
        del bays["amr_02"]
    elif case == "unknown":
        bays["unknown"] = [5.2, 0, 0]
    elif case == "too_close":
        bays["amr_01"] = [5.2, -3.0, 0]
    elif case == "charging_path":
        bays["amr_01"] = [5.8, -0.2, 0]
    elif case == "parking_path":
        data["station_exit_poses"]["inspection"]["amr_02"] = [4.2, 1.0, 0]
    elif case == "dock_staging":
        data["docks"]["dock_01"]["staging_pose"] = [4.0, -2.0, 0]
    elif case == "spawn_path":
        data["robots"][1]["spawn"] = [-5.0, -1.0, 0]
    else:
        data["docks"]["dock_01"]["staging_pose"] = [2.3, -2.2, 0]
    expected = {
        "missing": "requires every configured robot exactly once",
        "unknown": "requires every configured robot exactly once",
        "too_close": "dock exit footprint must clear holding poses",
        "charging_path": "charging approach must clear holding poses",
        "parking_path": "station dock approach must clear holding poses",
        "dock_staging": "dock staging must clear charger",
        "spawn_path": "dock approach crosses resource bounds",
        "staging_path": "station dock approach crosses resource bounds",
    }[case]
    with pytest.raises(ValueError, match=expected):
        load_data(tmp_path, data)


def test_normal_worker_start_does_not_claim_general_dock_path_reservation(tmp_path):
    # Only low-battery startup paths execute. Idle poses rely on Nav2 avoidance,
    # not a fleet-wide reservation of every hypothetical future straight line.
    config = load_data(tmp_path, dock_exit_data())
    assert config.robots[0].battery_start_percent >= config.energy.charge_below_percent
    assert config.robots[1].battery_start_percent < config.energy.charge_below_percent
    assert config.robots[1].spawn == config.docks["dock_01"].exit_poses["amr_02"]


def test_dock_exit_cannot_reuse_another_robots_terminal_bay(tmp_path):
    data = dock_exit_data()
    data["docks"]["dock_01"]["exit_poses"]["amr_01"] = [5.0, 2.4, 0]
    with pytest.raises(
        ValueError, match="dock exit footprint must clear holding poses"
    ):
        load_data(tmp_path, data)


def test_dock_exit_can_reuse_own_terminal_bay_with_a_different_heading(tmp_path):
    data = dock_exit_data()
    data["docks"]["dock_01"]["exit_poses"]["amr_01"] = [4.2, 0, 1.5707963267948966]
    config = load_data(tmp_path, data)
    assert config.docks["dock_01"].exit_poses["amr_01"].yaw == 1.5707963267948966


@pytest.mark.parametrize(
    "case", ["missing", "unknown", "too_close", "blocked_path", "foreign_approach"]
)
def test_terminal_bays_reject_missing_identity_or_unsafe_path(tmp_path, case):
    data = terminal_bay_data()
    bays = data["station_exit_poses"]["inspection"]
    if case == "missing":
        del bays["amr_02"]
    elif case == "unknown":
        bays["unknown"] = [4.2, 2, 0]
    elif case == "too_close":
        bays["amr_02"] = [4.2, 0.5, 0]
    elif case == "blocked_path":
        bays["amr_02"] = [5.2, -0.5, 0]
    else:
        data["station_approach"]["inspection"] = [-3.0, 0.8, 0]
    expected = {
        "missing": "requires every configured robot exactly once",
        "unknown": "requires every configured robot exactly once",
        "too_close": "parking footprint must clear all holding poses",
        "blocked_path": "parking approach must clear other terminal bays",
        "foreign_approach": "approach must lie inside station bounds",
    }[case]
    with pytest.raises(ValueError, match=expected):
        load_data(tmp_path, data)


def test_duplicate_robot_terminal_bay_yaml_is_rejected(tmp_path):
    path = tmp_path / "duplicate.yaml"
    text = yaml.safe_dump(terminal_bay_data())
    text = text.replace("    amr_02:\n", "    amr_01:\n", 1)
    path.write_text(text)
    with pytest.raises(ValueError, match="duplicate key 'amr_01'"):
        loader()(path)


@pytest.mark.parametrize(
    "spawn", [[-1, -3, 0], [0, -3.4, 0], [1.1, -3.4, 0], [-2.3, -2.2, 0], [3.5, -2, 0]]
)
def test_initial_spawn_rejects_resource_and_other_holding_envelopes(tmp_path, spawn):
    source = PACKAGE.parent / "factory_bringup/config/fleet.yaml"
    data = yaml.safe_load(source.read_text())
    data["robots"][0]["spawn"] = spawn
    data["robots"][1]["spawn"] = [1.0, -3.4, 0.0]
    with pytest.raises(ValueError, match="spawn"):
        load_data(tmp_path, data)


@pytest.mark.parametrize(
    "pose",
    [[0, -2.2, 0], [-2.3, -2.2, 0], [-1.5, -2.2, 0], [-3, 0.8, 0], [3.2, -2.2, 0]],
)
def test_station_terminal_parking_rejects_occupancy_and_cross_aisle(tmp_path, pose):
    data = yaml.safe_load(
        (PACKAGE.parent / "factory_bringup/config/fleet.yaml").read_text()
    )
    data["station_exit_poses"]["assembly"]["amr_01"] = pose
    with pytest.raises(ValueError, match="station_exit_poses"):
        load_data(tmp_path, data)


def test_station_terminal_parking_is_explicit_and_distinct():
    config = loader()(PACKAGE.parent / "factory_bringup/config/fleet.yaml")
    assert config.station_exit_poses["assembly"]["amr_01"].x == -4.2
    assert config.station_exit_poses["inspection"]["amr_01"].x == 4.2
    assert config.station_exit_poses["inspection"]["amr_02"].y == 2.4


def test_terminal_parking_rejects_dock_staging_envelope(tmp_path):
    data = yaml.safe_load(
        (PACKAGE.parent / "factory_bringup/config/fleet.yaml").read_text()
    )
    for dock in data["docks"].values():
        dock.pop("exit_poses", None)
        dock.pop("departure_stations", None)
    data["station_exit_poses"]["inspection"]["amr_01"] = [5.4, -1.4, 0.0]
    with pytest.raises(
        ValueError, match="parking footprint must clear all holding poses"
    ):
        load_data(tmp_path, data)


@pytest.mark.parametrize("second", ["second_aisle", "central_aisle"])
def test_route_rejects_multiple_traffic_segments_until_handoff_bays_exist(
    tmp_path, second
):
    data = yaml.safe_load(
        (PACKAGE.parent / "factory_bringup/config/fleet.yaml").read_text()
    )
    data["resources"].append(
        {"resource_id": "second_aisle", "kind": "traffic_zone", "capacity": 1}
    )
    data["routes"]["to_assembly"] = ["central_aisle", second, "assembly"]
    data["traffic_bounds"]["second_aisle"] = [2.0, -5.0, 3.0, -4.0]
    data["route_segments"]["to_assembly"].append(
        {
            "resource_id": second,
            "staging_pose": [1.0, -4.5, 0.0],
            "exit_pose": [4.0, -4.5, 0.0],
        }
    )
    with pytest.raises(ValueError, match="one traffic segment"):
        load_data(tmp_path, data)


def test_station_waiting_poses_are_distinct_from_route_clearance(tmp_path):
    source = PACKAGE.parent / "factory_bringup/config/fleet.yaml"
    data = yaml.safe_load(source.read_text())
    data["station_staging"] = {
        "assembly": [-2.3, -2.2, 1.5707963267948966],
        "inspection": [2.3, -2.2, 1.5707963267948966],
    }
    config = load_data(tmp_path, data)
    assert (
        config.station_staging["assembly"].x,
        config.station_staging["assembly"].y,
    ) == (-2.3, -2.2)


@pytest.mark.parametrize(
    "pose", [[-1.5, -2.2, 0.0], [-1.8, -2.2, 0.0], [-3.0, 0.8, 0.0]]
)
def test_station_waiting_rejects_shared_route_pose_or_station_occupancy(tmp_path, pose):
    source = PACKAGE.parent / "factory_bringup/config/fleet.yaml"
    data = yaml.safe_load(source.read_text())
    data["station_staging"] = {
        "assembly": pose,
        "inspection": [2.3, -2.2, 1.5707963267948966],
    }
    with pytest.raises(ValueError, match="station_staging.assembly"):
        load_data(tmp_path, data)


def loader():
    try:
        return importlib.import_module("fleet_manager.config").load_fleet_config
    except ModuleNotFoundError:
        pytest.fail("fleet configuration loader is not implemented")


@pytest.fixture
def data():
    return {
        "robots": [
            {
                "robot_id": "amr_01",
                "namespace": "/amr_01",
                "frame_prefix": "amr_01/",
                "spawn": [0.0, -3.0, 0.0],
                "battery_start_percent": 100.0,
            },
            {
                "robot_id": "amr_02",
                "namespace": "/amr_02",
                "frame_prefix": "amr_02/",
                "spawn": [1.0, -3.0, 0.0],
                "battery_start_percent": 60.0,
            },
        ],
        "resources": [
            {"resource_id": "assembly", "kind": "station", "capacity": 1},
            {"resource_id": "inspection", "kind": "station", "capacity": 1},
            {"resource_id": "central_aisle", "kind": "traffic_zone", "capacity": 1},
            {"resource_id": "dock_01", "kind": "dock", "capacity": 1},
        ],
        "routes": {
            "assembly_to_inspection": ["assembly", "central_aisle", "inspection"]
        },
        "docks": {
            "dock_01": {
                "staging_pose": [3.5, -2.0, 0.0],
                "charging_pose": [4.0, -2.0, 0.0],
            }
        },
        "energy": {
            "reserve_percent": 20.0,
            "charge_below_percent": 30.0,
            "charge_until_percent": 80.0,
            "dock_id": "dock_01",
        },
    }


def load_data(tmp_path, data):
    path = tmp_path / "fleet.yaml"
    path.write_text(yaml.safe_dump(data))
    return loader()(path)


def test_two_robot_configuration_is_ready_for_consumers(tmp_path, data):
    config = load_data(tmp_path, data)
    assert tuple(robot.robot_id for robot in config.robots) == ("amr_01", "amr_02")
    assert config.robots[0].namespace == "/amr_01"
    assert config.robots[0].frame_prefix == "amr_01/"
    assert (
        config.robots[1].spawn.x,
        config.robots[1].spawn.y,
        config.robots[1].spawn.yaw,
    ) == (1.0, -3.0, 0.0)
    assert config.robots[1].battery_start_percent == 60.0
    assert config.resources[2].resource_id == "central_aisle"
    assert config.resources[2].kind == "traffic_zone"
    assert config.resources[2].capacity == 1
    assert config.routes["assembly_to_inspection"] == (
        "assembly",
        "central_aisle",
        "inspection",
    )
    assert config.docks["dock_01"].staging_pose.x == 3.5
    assert config.docks["dock_01"].charging_pose.x == 4.0
    assert config.energy.reserve_percent == 20.0
    assert config.energy.charge_below_percent == 30.0
    assert config.energy.charge_until_percent == 80.0
    assert config.energy.dock_id == "dock_01"


def test_committed_fleet_file_loads():
    config = loader()(PACKAGE.parent / "factory_bringup/config/fleet.yaml")
    assert tuple(robot.robot_id for robot in config.robots) == ("amr_01", "amr_02")
    assert {resource.resource_id for resource in config.resources} == {
        "assembly",
        "inspection",
        "central_aisle",
        "dock_01",
    }


@pytest.mark.parametrize("field", ["robot_radius", "arrival_tolerance"])
@pytest.mark.parametrize("value", [0, -0.1, True, float("nan"), "0.15"])
def test_dock_clearance_parameters_reject_invalid_numbers(tmp_path, data, field, value):
    data["docks"]["dock_01"][field] = value
    with pytest.raises(ValueError, match="docks.dock_01." + field):
        load_data(tmp_path, data)


def test_configured_dock_clearance_reaches_agent_launch(tmp_path, data):
    from factory_bringup.fleet_launch import agent_parameters

    data["docks"]["dock_01"].update(robot_radius=0.25, arrival_tolerance=0.10)
    config = load_data(tmp_path, data)
    values = agent_parameters(
        config, config.robots[0], {"assembly": dict(x=0, y=0, yaw=0)}
    )
    assert values["dock.robot_radius"] == 0.25
    assert values["dock.arrival_tolerance"] == 0.10


def test_route_geometry_reaches_coordinator_without_changing_resource_order(
    tmp_path, data
):
    # Dropping or swapping staging/exit coordinates would authorize entry at the wrong boundary.
    data["route_segments"] = {
        "assembly_to_inspection": [
            {
                "resource_id": "central_aisle",
                "staging_pose": [-1.5, 0.8, 0],
                "exit_pose": [1.5, 0.8, 0],
            }
        ]
    }
    data["traffic_bounds"] = {"central_aisle": [-0.8, 0.3, 0.8, 1.3]}
    config = load_data(tmp_path, data)
    assert config.routes["assembly_to_inspection"] == (
        "assembly",
        "central_aisle",
        "inspection",
    )
    segment = config.route_segments["assembly_to_inspection"][0]
    assert (segment.resource_id, segment.staging_pose.x, segment.exit_pose.x) == (
        "central_aisle",
        -1.5,
        1.5,
    )
    assert config.traffic_bounds["central_aisle"] == (-0.8, 0.3, 0.8, 1.3)


def test_route_exit_inside_zone_cannot_authorize_release(tmp_path, data):
    data["traffic_bounds"] = {"central_aisle": [-0.8, 0.3, 0.8, 1.3]}
    data["route_segments"] = {
        "assembly_to_inspection": [
            {
                "resource_id": "central_aisle",
                "staging_pose": [-1.5, 0.8, 0],
                "exit_pose": [0.5, 0.8, 0],
            }
        ]
    }
    with pytest.raises(ValueError, match="outside traffic bounds"):
        load_data(tmp_path, data)


@pytest.mark.parametrize(
    "change", ["unknown_route", "wrong_resource", "missing_exit", "nan_pose"]
)
def test_route_geometry_rejects_unusable_boundaries(tmp_path, data, change):
    segment = {
        "resource_id": "central_aisle",
        "staging_pose": [-1.5, 0.8, 0],
        "exit_pose": [1.5, 0.8, 0],
    }
    data["route_segments"] = {"assembly_to_inspection": [segment]}
    if change == "unknown_route":
        data["route_segments"] = {"missing": [segment]}
    elif change == "wrong_resource":
        segment["resource_id"] = "assembly"
    elif change == "missing_exit":
        del segment["exit_pose"]
    else:
        segment["exit_pose"][0] = float("nan")
    with pytest.raises(ValueError, match="route_segments"):
        load_data(tmp_path, data)


@pytest.mark.parametrize("field", ["robot_id", "namespace", "frame_prefix", "spawn"])
def test_duplicate_robot_identity_or_spawn_is_rejected(tmp_path, data, field):
    data["robots"][1][field] = deepcopy(data["robots"][0][field])
    with pytest.raises(ValueError, match=rf"robots\[1\]\.{field}:.*duplicate"):
        load_data(tmp_path, data)


def test_same_spawn_position_with_different_yaw_is_rejected(tmp_path, data):
    data["robots"][1]["spawn"] = [0.0, -3.0, 1.57]
    with pytest.raises(ValueError, match=r"robots\[1\]\.spawn:"):
        load_data(tmp_path, data)


def test_unknown_route_resource_is_rejected(tmp_path, data):
    data["routes"]["assembly_to_inspection"][1] = "missing"
    with pytest.raises(
        ValueError, match=r"routes\.assembly_to_inspection\[1\]:.*unknown"
    ):
        load_data(tmp_path, data)


@pytest.mark.parametrize("capacity", [0, 2, -1, 1.0, True, "1"])
def test_only_integer_capacity_one_is_supported(tmp_path, data, capacity):
    data["resources"][0]["capacity"] = capacity
    with pytest.raises(ValueError, match=r"resources\[0\]\.capacity:"):
        load_data(tmp_path, data)


@pytest.mark.parametrize(
    "field,value",
    [
        ("reserve_percent", -1),
        ("reserve_percent", 31),
        ("charge_below_percent", 20),
        ("charge_until_percent", 30),
        ("charge_until_percent", 101),
        ("charge_until_percent", float("nan")),
        ("reserve_percent", True),
        ("reserve_percent", "20"),
    ],
)
def test_invalid_energy_thresholds_are_rejected(tmp_path, data, field, value):
    data["energy"][field] = value
    with pytest.raises(ValueError, match=r"energy\."):
        load_data(tmp_path, data)


def test_unknown_energy_dock_is_rejected(tmp_path, data):
    data["energy"]["dock_id"] = "missing"
    with pytest.raises(ValueError, match=r"energy\.dock_id:.*unknown"):
        load_data(tmp_path, data)


def test_dock_without_resource_is_rejected(tmp_path, data):
    data["docks"]["missing"] = data["docks"].pop("dock_01")
    with pytest.raises(ValueError, match=r"docks\.missing:.*unknown"):
        load_data(tmp_path, data)


def test_dock_resource_without_poses_is_rejected(tmp_path, data):
    data["docks"] = {}
    with pytest.raises(ValueError, match=r"docks\.dock_01:"):
        load_data(tmp_path, data)


def test_non_dock_resource_cannot_define_dock_poses(tmp_path, data):
    data["docks"]["assembly"] = deepcopy(data["docks"]["dock_01"])
    with pytest.raises(ValueError, match=r"docks\.assembly:"):
        load_data(tmp_path, data)


def test_duplicate_resource_id_is_rejected(tmp_path, data):
    data["resources"][1]["resource_id"] = "assembly"
    with pytest.raises(ValueError, match=r"resources\[1\]\.resource_id:.*duplicate"):
        load_data(tmp_path, data)


@pytest.mark.parametrize(
    "section", ["robots", "resources", "routes", "energy", "docks"]
)
def test_missing_sections_are_rejected(tmp_path, data, section):
    del data[section]
    with pytest.raises(ValueError, match=rf"{section}:"):
        load_data(tmp_path, data)


@pytest.mark.parametrize(
    "field,value",
    [
        ("namespace", "amr_01"),
        ("namespace", "/"),
        ("namespace", "/amr-01"),
        ("frame_prefix", ""),
        ("frame_prefix", "/amr_01/"),
        ("robot_id", ""),
        ("spawn", [0, 0]),
        ("spawn", [0, float("inf"), 0]),
        ("battery_start_percent", -1),
        ("battery_start_percent", 101),
    ],
)
def test_invalid_robot_fields_are_rejected(tmp_path, data, field, value):
    data["robots"][0][field] = value
    with pytest.raises(ValueError, match=rf"robots\[0\]\.{field}(?:\[\d+\])?:"):
        load_data(tmp_path, data)


def test_unknown_fields_are_rejected_instead_of_silently_ignored(tmp_path, data):
    data["robots"][0]["battery_start_precent"] = 10
    with pytest.raises(ValueError, match=r"robots\[0\]\.battery_start_precent:"):
        load_data(tmp_path, data)


@pytest.mark.parametrize(
    "section,value",
    [
        ("robots", []),
        ("robots", {}),
        ("routes", {"empty": []}),
        ("resources", [{"resource_id": "wrong", "kind": "unknown", "capacity": 1}]),
    ],
)
def test_malformed_sections_are_rejected(tmp_path, data, section, value):
    data[section] = value
    with pytest.raises(ValueError, match=rf"{section}"):
        load_data(tmp_path, data)


def test_missing_dock_pose_is_rejected(tmp_path, data):
    del data["docks"]["dock_01"]["charging_pose"]
    with pytest.raises(ValueError, match=r"docks\.dock_01\.charging_pose:"):
        load_data(tmp_path, data)


def test_config_cannot_be_mutated_after_validation(tmp_path, data):
    config = load_data(tmp_path, data)
    with pytest.raises(FrozenInstanceError):
        config.robots[0].battery_start_percent = 0
    with pytest.raises(FrozenInstanceError):
        config.robots[0].spawn.x = 999
    with pytest.raises(FrozenInstanceError):
        config.resources[0].capacity = 2
    with pytest.raises(FrozenInstanceError):
        config.energy.reserve_percent = 0
    with pytest.raises(FrozenInstanceError):
        config.docks["dock_01"].charging_pose.x = 0
    with pytest.raises(TypeError):
        config.routes["assembly_to_inspection"] = ("missing",)
    with pytest.raises(TypeError):
        config.docks["dock_02"] = config.docks["dock_01"]


@pytest.mark.parametrize("text", ["", "[]", "robots: [", "robots: []\nrobots: []"])
def test_invalid_yaml_reports_file_context(tmp_path, text):
    path = tmp_path / "fleet.yaml"
    path.write_text(text)
    with pytest.raises(ValueError, match="fleet.yaml"):
        loader()(path)


def test_missing_file_reports_file_context(tmp_path):
    with pytest.raises(ValueError, match="missing.yaml"):
        loader()(tmp_path / "missing.yaml")
