# SPDX-License-Identifier: Apache-2.0
"""Protect the shared ROS wire contracts consumed by the other packages."""

from pathlib import Path

import pytest


INTERFACE_ROOT = Path(__file__).resolve().parents[2] / "src" / "factory_interfaces"


def interface_lines(relative_path):
    path = INTERFACE_ROOT / relative_path
    assert path.is_file(), f"Missing shared ROS interface: {path}"
    return [
        field
        for line in path.read_text(encoding="utf-8").splitlines()
        if (field := line.partition("#")[0].strip())
    ]


@pytest.mark.parametrize(
    ("relative_path", "expected"),
    [
        (
            "action/ExecuteFactoryMission.action",
            [
                "string mission_id",
                "string robot_id",
                "string pickup_station",
                "string dropoff_station",
                "string part",
                "---",
                "bool success",
                "string final_state",
                "string error_code",
                "string message",
                "---",
                "string state",
                "string station",
                "string detail",
                "float32 progress",
            ],
        ),
        (
            "action/ExecuteFleetMission.action",
            [
                "string mission_id",
                "string requested_robot_id",
                "string pickup_station",
                "string dropoff_station",
                "string part",
                "---",
                "bool success",
                "string final_state",
                "string assigned_robot_id",
                "string error_code",
                "string message",
                "---",
                "string state",
                "string assigned_robot_id",
                "string detail",
                "float32 progress",
            ],
        ),
        (
            "action/DockRobot.action",
            [
                "string dock_id",
                "geometry_msgs/PoseStamped staging_pose",
                "geometry_msgs/PoseStamped charging_pose",
                "float32 target_percent",
                "---",
                "bool success",
                "string error_code",
                "string message",
                "---",
                "string state",
                "float32 battery_percent",
                "string detail",
            ],
        ),
        (
            "msg/RobotState.msg",
            [
                "builtin_interfaces/Time stamp",
                "string robot_id",
                "string mode",
                "geometry_msgs/Pose pose",
                "string frame_id",
                "float32 battery_percent",
                "string payload_state",
                "string mission_id",
                "string health_detail",
            ],
        ),
        (
            "msg/StationDetection.msg",
            [
                "std_msgs/Header header",
                "string station_id",
                "int32 marker_id",
                "float32 confidence",
            ],
        ),
        (
            "msg/ProtocolEvent.msg",
            [
                "builtin_interfaces/Time stamp",
                "string mission_id",
                "string robot_id",
                "string protocol",
                "string direction",
                "string event",
                "string outcome",
                "float32 latency_ms",
                "string detail",
            ],
        ),
        (
            "srv/TransferPart.srv",
            [
                "string mission_id",
                "string robot_id",
                "string station_id",
                "string part",
                "---",
                "bool accepted",
                "string error_code",
                "string message",
            ],
        ),
        (
            "msg/FaultCommand.msg",
            [
                "uint64 command_id",
                "bool acknowledged",
                "string owner",
                "float64 ack_timeout_sec",
                "bool reset",
                "string name",
                "string mission_id",
                "string robot_id",
                "string station",
                "string activation_point",
                "float64 duration",
                "bool one_shot",
                "uint16 fault_code",
            ],
        ),
        (
            "srv/EstimateMissionCost.srv",
            [
                "string mission_id",
                "string pickup_station",
                "string dropoff_station",
                "string part",
                "---",
                "bool feasible",
                "float64 path_cost",
                "float32 predicted_final_battery",
                "string reason",
            ],
        ),
        (
            "srv/AcquireResource.srv",
            [
                "string robot_id",
                "string mission_id",
                "string resource_id",
                "---",
                "bool granted",
                "string lease_id",
                "float64 lease_ttl_sec",
                "string current_owner",
                "string reason",
            ],
        ),
        (
            "srv/RenewResource.srv",
            [
                "string robot_id",
                "string mission_id",
                "string resource_id",
                "string lease_id",
                "---",
                "bool renewed",
                "float64 lease_ttl_sec",
                "string reason",
            ],
        ),
        (
            "srv/ReleaseResource.srv",
            [
                "string robot_id",
                "string mission_id",
                "string resource_id",
                "string lease_id",
                "---",
                "bool released",
                "string reason",
            ],
        ),
        (
            "srv/CancelResourceWait.srv",
            [
                "string robot_id",
                "string mission_id",
                "string resource_id",
                "---",
                "bool cancelled",
                "string reason",
            ],
        ),
    ],
)
def test_interface_preserves_ordered_wire_fields(relative_path, expected):
    assert interface_lines(relative_path) == expected


@pytest.mark.parametrize(
    ("relative_path", "separator_count"),
    [
        ("action/ExecuteFactoryMission.action", 2),
        ("action/ExecuteFleetMission.action", 2),
        ("action/DockRobot.action", 2),
        ("srv/TransferPart.srv", 1),
        ("srv/EstimateMissionCost.srv", 1),
        ("srv/AcquireResource.srv", 1),
        ("srv/RenewResource.srv", 1),
        ("srv/ReleaseResource.srv", 1),
        ("srv/CancelResourceWait.srv", 1),
    ],
)
def test_interface_preserves_section_boundaries(relative_path, separator_count):
    assert interface_lines(relative_path).count("---") == separator_count
