# SPDX-License-Identifier: Apache-2.0
"""Protect canonical deduplication and saved mission state."""

from dataclasses import replace

import pytest

from mqtt_gateway.mission_registry import MissionConflictError, MissionRegistry
from mqtt_gateway.models import MissionPayload
from mqtt_gateway.validator import MissionValidator


MISSION = MissionPayload("M-001", "amr_01", "assembly", "inspection", "motor")


def test_registers_new_ids_and_deduplicates_canonical_fields():
    registry = MissionRegistry()
    assert registry.register(MISSION) == "new"
    reordered = b'{ "part": "motor", "dropoff": "inspection", "pickup": "assembly", "robot_id": "amr_01", "mission_id": "M-001" }'
    assert registry.register(MissionValidator().validate(reordered)) == "duplicate"
    assert registry.register(replace(MISSION, mission_id="M-002")) == "new"


@pytest.mark.parametrize(("field", "value"), [
    ("robot_id", "amr_02"), ("pickup", "inspection"),
    ("dropoff", "assembly"), ("part", "gear"),
])
def test_rejects_changed_fields_without_replacing_identity(field, value):
    registry = MissionRegistry()
    registry.register(MISSION)
    with pytest.raises(MissionConflictError) as error:
        registry.register(replace(MISSION, **{field: value}))
    assert error.value.error_code == "MISSION_ID_CONFLICT"
    assert registry.register(MISSION) == "duplicate"


def test_retains_current_and_final_state_across_duplicate_requests():
    registry = MissionRegistry()
    assert registry.state_for("unknown") is None
    registry.register(MISSION)
    assert registry.state_for("M-001") is None
    registry.update_state("M-001", {"state": "NAVIGATING_TO_PICKUP"})
    assert registry.state_for("M-001") == {"state": "NAVIGATING_TO_PICKUP"}
    registry.update_state("M-001", {"state": "COMPLETED"})
    assert registry.register(MISSION) == "duplicate"
    assert registry.state_for("M-001") == {"state": "COMPLETED"}


def test_saved_state_is_independent_of_callers_nested_mutations():
    registry = MissionRegistry()
    registry.register(MISSION)
    state = {"detail": {"events": ["accepted"]}}
    registry.update_state("M-001", state)
    state["detail"]["events"].append("caller change")
    retrieved = registry.state_for("M-001")
    retrieved["detail"]["events"].clear()
    assert registry.state_for("M-001") == {"detail": {"events": ["accepted"]}}


def test_rejects_state_update_for_unregistered_mission():
    with pytest.raises(KeyError):
        MissionRegistry().update_state("unknown", {"state": "COMPLETED"})


def test_payload_lookup_returns_only_registered_immutable_identity():
    registry = MissionRegistry()
    assert registry.payload_for('M-001') is None
    registry.register(MISSION)
    assert registry.payload_for('M-001') == MISSION
