# SPDX-License-Identifier: Apache-2.0
"""Exercise request rejection at the untrusted MQTT boundary."""

from dataclasses import FrozenInstanceError

import pytest

from mqtt_gateway.models import MissionPayload
from mqtt_gateway.validator import MissionValidationError, MissionValidator


VALID = b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}'


def test_accepts_mission_as_immutable_payload():
    mission = MissionValidator().validate(VALID)
    assert mission == MissionPayload(
        "M-001", "amr_01", "assembly", "inspection", "motor"
    )
    with pytest.raises(FrozenInstanceError):
        mission.mission_id = "changed"


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        (
            b'{"mission_id":"m1","pickup":"assembly","dropoff":"inspection","part":"gear"}',
            MissionPayload("m1", None, "assembly", "inspection", "gear"),
        ),
        (
            b'{"mission_id":"m2","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"gear"}',
            MissionPayload("m2", "amr_01", "assembly", "inspection", "gear"),
        ),
    ],
)
def test_accepts_automatic_and_explicit_assignment(raw, expected):
    assert MissionValidator().validate(raw) == expected


def test_structural_parse_normalizes_omitted_robot_to_none():
    raw = (
        b'{"mission_id":"m1","pickup":"assembly","dropoff":"inspection","part":"motor"}'
    )
    mission = MissionValidator().parse_structure(raw)
    assert mission.robot_id is None
    MissionValidator().validate_configuration(mission)


@pytest.mark.parametrize("robot_id", ["amr_02", "A_" + "9" * 62])
def test_accepts_topic_safe_robot_id_without_static_registry(robot_id):
    raw = VALID.replace(b"amr_01", robot_id.encode())
    assert MissionValidator().validate(raw).robot_id == robot_id


@pytest.mark.parametrize("robot_id", ["amr/02", "amr+02", "amr#02", "amr\n", "amré"])
def test_rejects_robot_ids_unsafe_for_topic_correlation(robot_id):
    import json

    payload = json.loads(VALID)
    payload["robot_id"] = robot_id
    with pytest.raises(MissionValidationError):
        MissionValidator().validate(json.dumps(payload).encode())


@pytest.mark.parametrize(
    "raw",
    [
        b"{",
        b"\xff",
        b"null",
        b"[]",
        b'{"mission_id":"M-001"}',
        b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor","extra":1}',
        b'{"mission_id":1,"robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"unknown","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"unknown","part":"motor"}',
        b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"inspection","dropoff":"assembly","part":"motor"}',
        b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"assembly","part":"motor"}',
        b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"inspection","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"unknown"}',
        b'{"mission_id":"M-001","robot_id":null,"pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M-001","robot_id":"","pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M/001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M+001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M#001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M\\n001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M-001\\n","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
        b'{"mission_id":"M\\u00e9","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    ],
)
def test_rejects_invalid_requests_with_boundary_error(raw):
    with pytest.raises(MissionValidationError) as error:
        MissionValidator().validate(raw)
    assert error.value.error_code == "INVALID_MISSION"
    assert error.value.reason


@pytest.mark.parametrize(
    "field", ["mission_id", "robot_id", "pickup", "dropoff", "part"]
)
def test_rejects_strings_above_64_characters(field):
    import json

    payload = json.loads(VALID)
    payload[field] = "x" * 65
    with pytest.raises(MissionValidationError):
        MissionValidator().validate(json.dumps(payload).encode())


def test_accepts_topic_safe_id_at_64_character_boundary():
    raw = VALID.replace(b"M-001", b"A_" + b"9" * 62)
    assert MissionValidator().validate(raw).mission_id == "A_" + "9" * 62


def test_enforces_raw_byte_limit_before_decode():
    validator = MissionValidator()
    assert validator.validate(VALID + b" " * (4096 - len(VALID))).mission_id == "M-001"
    with pytest.raises(MissionValidationError) as error:
        validator.validate(b"\xff" * 4097)
    assert "4096" in error.value.reason


def test_structural_parse_keeps_configuration_validation_explicit():
    validator = MissionValidator()
    parsed = validator.parse_structure(VALID.replace(b'"motor"', b'"unknown"'))
    assert parsed.part == "unknown"
    with pytest.raises(MissionValidationError):
        validator.validate_configuration(parsed)
    with pytest.raises(MissionValidationError):
        validator.parse_structure(VALID.replace(b'"motor"', b"123"))
