# SPDX-License-Identifier: Apache-2.0
"""Reject invalid or replayed transfer completions without moving the payload."""

import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from payload_simulator.state_machine import PayloadState, PayloadStateMachine


def complete(machine, *, mission_id="mission_1", protocol="MODBUS",
             event="modbus_pickup_finished", outcome="SUCCEEDED", detail=None):
    if detail is None:
        detail = json.dumps({"station_id": "assembly", "transfer_kind": "LOADING", "cycle_counter": 1})
    return machine.apply_protocol_event(mission_id, protocol, event, outcome, detail)


def test_successful_load_then_unload_moves_one_part_once():
    machine = PayloadStateMachine()
    assert machine.state == PayloadState.AT_ASSEMBLY
    loaded = machine.apply_transfer("mission_1", "LOADING", 65535)
    assert loaded.applied
    assert loaded.previous_state == PayloadState.AT_ASSEMBLY
    assert loaded.state == machine.state == PayloadState.IN_TRANSIT
    unloaded = machine.apply_transfer("mission_1", "UNLOADING", 0)
    assert unloaded.applied
    assert unloaded.previous_state == PayloadState.IN_TRANSIT
    assert unloaded.state == machine.state == PayloadState.AT_INSPECTION


def test_unload_before_load_does_not_consume_completion():
    machine = PayloadStateMachine()
    assert not machine.apply_transfer("mission_1", "UNLOADING", 7).applied
    assert machine.state == PayloadState.AT_ASSEMBLY
    assert machine.apply_transfer("mission_1", "LOADING", 6).applied
    assert machine.apply_transfer("mission_1", "UNLOADING", 7).applied


def test_duplicate_completion_never_replays_transition():
    machine = PayloadStateMachine()
    machine.apply_transfer("mission_1", "LOADING", 1)
    replay = machine.apply_transfer("mission_1", "LOADING", 1)
    assert not replay.applied
    assert replay.reason == "duplicate"
    assert machine.state == PayloadState.IN_TRANSIT
    machine.apply_transfer("mission_1", "UNLOADING", 1)
    assert not machine.apply_transfer("mission_1", "UNLOADING", 1).applied
    assert not machine.apply_transfer("mission_1", "LOADING", 1).applied
    assert machine.state == PayloadState.AT_INSPECTION


def test_changed_mission_cannot_unload_part_in_transit():
    machine = PayloadStateMachine()
    machine.apply_transfer("mission_1", "LOADING", 1)
    assert not machine.apply_transfer("mission_2", "UNLOADING", 2).applied
    assert machine.mission_id == "mission_1"
    assert machine.state == PayloadState.IN_TRANSIT
    assert machine.apply_transfer("mission_1", "UNLOADING", 2).applied


def test_single_part_does_not_reset_for_next_mission():
    machine = PayloadStateMachine()
    machine.apply_transfer("mission_1", "LOADING", 1)
    machine.apply_transfer("mission_1", "UNLOADING", 2)
    assert not machine.apply_transfer("mission_2", "LOADING", 3).applied
    assert machine.state == PayloadState.AT_INSPECTION


@pytest.mark.parametrize("overrides", [
    {"outcome": "FAILED"}, {"outcome": "COMPLETED"}, {"protocol": "MQTT"},
    {"event": "modbus_pickup_started"}, {"event": "modbus_dropoff_finished"},
    {"mission_id": ""}, {"mission_id": "bad/id"}, {"mission_id": "x" * 65},
    {"detail": "not JSON"}, {"detail": "[]"}, {"detail": "null"},
    {"detail": "[" * 1100 + "0" + "]" * 1100},
    {"detail": '{"station_id":"inspection","transfer_kind":"LOADING","cycle_counter":1}'},
    {"detail": '{"station_id":"assembly","transfer_kind":"UNLOADING","cycle_counter":1}'},
    {"detail": '{"station_id":"assembly","transfer_kind":"LOADING"}'},
])
def test_failed_or_malformed_event_leaves_state_unchanged(overrides):
    machine = PayloadStateMachine()
    assert not complete(machine, **overrides).applied
    assert machine.state == PayloadState.AT_ASSEMBLY
    assert machine.mission_id == ""
    assert complete(machine).applied


@pytest.mark.parametrize("counter", [True, False, -1, 65536, 1.5, "1", None])
def test_cycle_counter_requires_uint16_integer(counter):
    machine = PayloadStateMachine()
    detail = json.dumps({"station_id": "assembly", "transfer_kind": "LOADING", "cycle_counter": counter})
    assert not complete(machine, detail=detail).applied
    assert not machine.apply_transfer("mission_1", "LOADING", counter).applied
    assert machine.state == PayloadState.AT_ASSEMBLY


def test_success_event_validates_inspection_unload():
    machine = PayloadStateMachine()
    assert complete(machine).applied
    detail = '{"station_id":"inspection","transfer_kind":"UNLOADING","cycle_counter":8}'
    assert complete(machine, event="modbus_dropoff_finished", detail=detail).applied
    assert machine.state == PayloadState.AT_INSPECTION
