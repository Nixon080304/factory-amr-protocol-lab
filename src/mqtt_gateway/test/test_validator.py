# SPDX-License-Identifier: Apache-2.0
"""Exercise request rejection at the untrusted MQTT boundary."""

from dataclasses import FrozenInstanceError

import pytest

from mqtt_gateway.models import MissionPayload
from mqtt_gateway.validator import MissionValidationError, MissionValidator


VALID = b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}'


def test_accepts_mission_as_immutable_payload():
    mission = MissionValidator().validate(VALID)
    assert mission == MissionPayload("M-001", "amr_01", "assembly", "inspection", "motor")
    with pytest.raises(FrozenInstanceError):
        mission.mission_id = "changed"


@pytest.mark.parametrize("raw", [
    b'{',
    b'\xff',
    b'null',
    b'[]',
    b'{"mission_id":"M-001"}',
    b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor","extra":1}',
    b'{"mission_id":1,"robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M-001","robot_id":"amr_02","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"unknown","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"unknown","part":"motor"}',
    b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"inspection","dropoff":"assembly","part":"motor"}',
    b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"assembly","part":"motor"}',
    b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"inspection","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M-001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"gear"}',
    b'{"mission_id":"","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M/001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M+001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M#001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M\\n001","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M-001\\n","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
    b'{"mission_id":"M\\u00e9","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"motor"}',
])
def test_rejects_invalid_requests_with_boundary_error(raw):
    with pytest.raises(MissionValidationError) as error:
        MissionValidator().validate(raw)
    assert error.value.error_code == "INVALID_MISSION"
    assert error.value.reason


@pytest.mark.parametrize("field", ["mission_id", "robot_id", "pickup", "dropoff", "part"])
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
