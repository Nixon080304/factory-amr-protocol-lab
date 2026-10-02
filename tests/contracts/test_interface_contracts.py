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
                "string station_id",
                "string part",
                "---",
                "bool accepted",
                "string error_code",
                "string message",
            ],
        ),
    ],
)
def test_interface_preserves_ordered_wire_fields(relative_path, expected):
    assert interface_lines(relative_path) == expected


@pytest.mark.parametrize(
    ("relative_path", "separator_count"),
    [("action/ExecuteFactoryMission.action", 2), ("srv/TransferPart.srv", 1)],
)
def test_interface_preserves_section_boundaries(relative_path, separator_count):
    assert interface_lines(relative_path).count("---") == separator_count
