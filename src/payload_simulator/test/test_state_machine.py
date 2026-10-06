# SPDX-License-Identifier: Apache-2.0
"""Reject invalid or replayed transfer completions without moving the payload."""

import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from payload_simulator.state_machine import PayloadState, PayloadStateMachine


def complete(
    machine,
    *,
    mission_id="mission_1",
    robot_id="amr_01",
    protocol="MODBUS",
    event="modbus_pickup_finished",
    outcome="SUCCEEDED",
    detail=None,
):
    if detail is None:
        detail = json.dumps(
            {
                "station_id": "assembly",
                "transfer_kind": "LOADING",
                "cycle_counter": 1,
                "part": "motor",
            }
        )
    return machine.apply_protocol_event(
        mission_id, protocol, event, outcome, detail, robot_id=robot_id
    )


def test_successful_load_then_unload_moves_one_part_once():
    machine = PayloadStateMachine()
    assert machine.state == PayloadState.AT_ASSEMBLY
    loaded = machine.apply_transfer(
        "mission_1", "LOADING", 65535, robot_id="amr_01", part="motor"
    )
    assert loaded.applied
    assert loaded.previous_state == PayloadState.AT_ASSEMBLY
    assert loaded.state == machine.state == PayloadState.IN_TRANSIT
    unloaded = machine.apply_transfer(
        "mission_1", "UNLOADING", 0, robot_id="amr_01", part="motor"
    )
    assert unloaded.applied
    assert unloaded.previous_state == PayloadState.IN_TRANSIT
    assert unloaded.state == machine.state == PayloadState.AT_INSPECTION


def test_unload_before_load_does_not_consume_completion():
    machine = PayloadStateMachine()
    assert not machine.apply_transfer(
        "mission_1", "UNLOADING", 7, robot_id="amr_01", part="motor"
    ).applied
    assert machine.state == PayloadState.AT_ASSEMBLY
    assert machine.apply_transfer(
        "mission_1", "LOADING", 6, robot_id="amr_01", part="motor"
    ).applied
    assert machine.apply_transfer(
        "mission_1", "UNLOADING", 7, robot_id="amr_01", part="motor"
    ).applied


def test_duplicate_completion_never_replays_transition():
    machine = PayloadStateMachine()
    machine.apply_transfer("mission_1", "LOADING", 1, robot_id="amr_01", part="motor")
    replay = machine.apply_transfer(
        "mission_1", "LOADING", 1, robot_id="amr_01", part="motor"
    )
    assert not replay.applied
    assert replay.reason == "duplicate"
    assert machine.state == PayloadState.IN_TRANSIT
    machine.apply_transfer("mission_1", "UNLOADING", 1, robot_id="amr_01", part="motor")
    assert not machine.apply_transfer(
        "mission_1", "UNLOADING", 1, robot_id="amr_01", part="motor"
    ).applied
    assert not machine.apply_transfer(
        "mission_1", "LOADING", 1, robot_id="amr_01", part="motor"
    ).applied
    assert machine.state == PayloadState.AT_INSPECTION


def test_changed_mission_cannot_unload_part_in_transit():
    machine = PayloadStateMachine()
    machine.apply_transfer("mission_1", "LOADING", 1, robot_id="amr_01", part="motor")
    assert not machine.apply_transfer(
        "mission_2", "UNLOADING", 2, robot_id="amr_01", part="motor"
    ).applied
    assert machine.mission_id == "mission_1"
    assert machine.state == PayloadState.IN_TRANSIT
    assert machine.apply_transfer(
        "mission_1", "UNLOADING", 2, robot_id="amr_01", part="motor"
    ).applied


def test_single_part_does_not_reset_for_next_mission():
    machine = PayloadStateMachine()
    machine.apply_transfer("mission_1", "LOADING", 1, robot_id="amr_01", part="motor")
    machine.apply_transfer("mission_1", "UNLOADING", 2, robot_id="amr_01", part="motor")
    assert not machine.apply_transfer(
        "mission_2", "LOADING", 3, robot_id="amr_01", part="motor"
    ).applied
    assert machine.state == PayloadState.AT_INSPECTION


@pytest.mark.parametrize(
    "overrides",
    [
        {"outcome": "FAILED"},
        {"outcome": "COMPLETED"},
        {"protocol": "MQTT"},
        {"event": "modbus_pickup_started"},
        {"event": "modbus_dropoff_finished"},
        {"mission_id": ""},
        {"mission_id": "bad/id"},
        {"mission_id": "x" * 65},
        {"detail": "not JSON"},
        {"detail": "[]"},
        {"detail": "null"},
        {"detail": "[" * 1100 + "0" + "]" * 1100},
        {
            "detail": '{"station_id":"inspection","transfer_kind":"LOADING","cycle_counter":1,"part":"motor"}'
        },
        {
            "detail": '{"station_id":"assembly","transfer_kind":"UNLOADING","cycle_counter":1,"part":"motor"}'
        },
        {
            "detail": '{"station_id":"assembly","transfer_kind":"LOADING","part":"motor"}'
        },
    ],
)
def test_failed_or_malformed_event_leaves_state_unchanged(overrides):
    machine = PayloadStateMachine()
    assert not complete(machine, **overrides).applied
    assert machine.state == PayloadState.AT_ASSEMBLY
    assert machine.mission_id == ""
    assert complete(machine).applied


@pytest.mark.parametrize("counter", [True, False, -1, 65536, 1.5, "1", None])
def test_cycle_counter_requires_uint16_integer(counter):
    machine = PayloadStateMachine()
    detail = json.dumps(
        {
            "station_id": "assembly",
            "transfer_kind": "LOADING",
            "part": "motor",
            "cycle_counter": counter,
        }
    )
    assert not complete(machine, detail=detail).applied
    assert not machine.apply_transfer(
        "mission_1", "LOADING", counter, robot_id="amr_01", part="motor"
    ).applied
    assert machine.state == PayloadState.AT_ASSEMBLY


def test_success_event_validates_inspection_unload():
    machine = PayloadStateMachine()
    assert complete(machine).applied
    detail = '{"station_id":"inspection","transfer_kind":"UNLOADING","cycle_counter":8,"part":"motor"}'
    assert complete(machine, event="modbus_dropoff_finished", detail=detail).applied
    assert machine.state == PayloadState.AT_INSPECTION


@pytest.mark.parametrize(
    "physical_outcome, moves", [("COMPLETED", True), ("UNKNOWN", False)]
)
def test_cleanup_failure_moves_only_confirmed_physical_cycle(physical_outcome, moves):
    machine = PayloadStateMachine()
    detail = json.dumps(
        {
            "station_id": "assembly",
            "transfer_kind": "LOADING",
            "cycle_counter": 1,
            "part": "motor",
            "transfer_outcome": physical_outcome,
            "error_code": "PLC_TIMEOUT_TRANSFER_" + physical_outcome,
            "message": "cleanup acknowledgment lost",
        }
    )
    transition = complete(machine, outcome="FAILED", detail=detail)
    assert transition.applied == moves
    assert machine.state == (
        PayloadState.IN_TRANSIT if moves else PayloadState.AT_ASSEMBLY
    )
    assert not complete(machine, outcome="FAILED", detail=detail).applied


def test_interleaved_payloads_remain_owned_by_exact_robot_and_mission():
    machine = PayloadStateMachine()
    assert machine.apply_transfer(
        "M-1", "LOADING", 1, robot_id="amr_01", part="motor"
    ).applied
    assert machine.apply_transfer(
        "M-2", "LOADING", 2, robot_id="amr_02", part="motor"
    ).applied
    assert not machine.apply_transfer(
        "M-1", "UNLOADING", 3, robot_id="amr_02", part="motor"
    ).applied
    assert not machine.apply_transfer(
        "M-2", "UNLOADING", 3, robot_id="amr_01", part="motor"
    ).applied
    assert machine.apply_transfer(
        "M-1", "UNLOADING", 3, robot_id="amr_01", part="motor"
    ).applied
    assert machine.state_for("amr_02", "M-2") == PayloadState.IN_TRANSIT
    assert not machine.apply_transfer(
        "M-1", "LOADING", 1, robot_id="amr_01", part="motor"
    ).applied
    assert machine.apply_transfer(
        "M-2", "UNLOADING", 4, robot_id="amr_02", part="motor"
    ).applied
    assert machine.state_for("amr_01", "M-1") == PayloadState.AT_INSPECTION


@pytest.mark.parametrize("robot_id", [None, "", "bad/id", "x" * 65])
def test_missing_or_invalid_robot_cannot_attach_payload(robot_id):
    machine = PayloadStateMachine()
    assert not machine.apply_transfer(
        "M-1", "LOADING", 1, robot_id=robot_id, part="motor"
    ).applied


def test_confirmed_mission_payload_cannot_reattach_after_robot_reassignment():
    machine = PayloadStateMachine()
    assert machine.apply_transfer(
        "M-1", "LOADING", 1, robot_id="amr_01", part="motor"
    ).applied
    assert not machine.apply_transfer(
        "M-1", "LOADING", 2, robot_id="amr_02", part="motor"
    ).applied
    assert not machine.apply_transfer(
        "M-1", "UNLOADING", 3, robot_id="amr_02", part="motor"
    ).applied
    assert machine.state_for("amr_01", "M-1") == PayloadState.IN_TRANSIT
    assert machine.apply_transfer(
        "M-1", "UNLOADING", 3, robot_id="amr_01", part="motor"
    ).applied
    assert not machine.apply_transfer(
        "M-1", "LOADING", 4, robot_id="amr_02", part="motor"
    ).applied


@pytest.mark.parametrize("part", [None, "gear", ""])
@pytest.mark.parametrize("kind", ["LOADING", "UNLOADING"])
def test_review_missing_or_conflicting_part_cannot_move_payload(part, kind):
    machine = PayloadStateMachine()
    if kind == "UNLOADING":
        assert complete(
            machine,
            detail=json.dumps(
                dict(
                    station_id="assembly",
                    transfer_kind="LOADING",
                    cycle_counter=1,
                    part="motor",
                )
            ),
        ).applied
    fields = dict(
        station_id="assembly" if kind == "LOADING" else "inspection",
        transfer_kind=kind,
        cycle_counter=2,
    )
    if part is not None:
        fields["part"] = part
    transition = complete(
        machine,
        event="modbus_pickup_finished"
        if kind == "LOADING"
        else "modbus_dropoff_finished",
        detail=json.dumps(fields),
    )
    assert not transition.applied
    assert machine.state == (
        PayloadState.AT_ASSEMBLY if kind == "LOADING" else PayloadState.IN_TRANSIT
    )
