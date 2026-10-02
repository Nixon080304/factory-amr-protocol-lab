# SPDX-License-Identifier: Apache-2.0
"""Replay independently specified owner order and source-time counterexamples."""

from pathlib import Path
import runpy
from types import SimpleNamespace

import pytest


CHECK = runpy.run_path(
    str(Path(__file__).resolve().parents[1] / "system/trace_assertions.py")
)["assert_successful_transfer_phases"]


def event(name, protocol, seconds, nanoseconds=0, detail="", outcome=""):
    return SimpleNamespace(
        event=name,
        protocol=protocol,
        stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds),
        detail=detail,
        outcome=outcome,
    )


@pytest.fixture
def phase_events():
    # Source clock ties are real; successful callbacks retain their owner order.
    coordinator = [
        event("perception_pickup_finished", "ROS", 33, 500000000, outcome="SUCCEEDED"),
        event("state_changed", "ROS", 33, 500000000, detail="LOADING"),
        event("transfer_result", "ROS", 33, 700000000, outcome="SUCCEEDED"),
        event("perception_dropoff_finished", "ROS", 74, 700000000, outcome="SUCCEEDED"),
        event("state_changed", "ROS", 74, 700000000, detail="UNLOADING"),
        event("transfer_result", "ROS", 74, 900000000, outcome="SUCCEEDED"),
    ]
    gateway = [
        event("modbus_pickup_started", "MODBUS", 33, 500000000),
        event("modbus_pickup_finished", "MODBUS", 33, 700000000, outcome="SUCCEEDED"),
        event("modbus_dropoff_started", "MODBUS", 74, 700000000),
        event("modbus_dropoff_finished", "MODBUS", 74, 900000000, outcome="SUCCEEDED"),
    ]
    return coordinator, gateway


def test_successful_phase_evidence_accepts_cross_writer_reordering(phase_events):
    coordinator, gateway = phase_events
    CHECK(gateway + coordinator)
    CHECK(coordinator + gateway)
    CHECK(gateway[:2] + coordinator[:3] + gateway[2:] + coordinator[3:])


@pytest.mark.parametrize("left,right", [(0, 1), (1, 2), (3, 4), (4, 5)])
def test_successful_phase_evidence_rejects_coordinator_inversion(
    phase_events, left, right
):
    coordinator, gateway = phase_events
    coordinator[left], coordinator[right] = coordinator[right], coordinator[left]
    with pytest.raises(AssertionError):
        CHECK(coordinator + gateway)


@pytest.mark.parametrize("left,right", [(0, 1), (1, 2), (2, 3)])
def test_successful_phase_evidence_rejects_gateway_inversion(phase_events, left, right):
    coordinator, gateway = phase_events
    gateway[left], gateway[right] = gateway[right], gateway[left]
    with pytest.raises(AssertionError):
        CHECK(coordinator + gateway)


@pytest.mark.parametrize("index", range(10))
def test_successful_phase_evidence_rejects_missing_phase(phase_events, index):
    coordinator, gateway = phase_events
    messages = coordinator + gateway
    del messages[index]
    with pytest.raises(AssertionError):
        CHECK(messages)


@pytest.mark.parametrize("index", range(10))
def test_successful_phase_evidence_rejects_duplicate_phase(phase_events, index):
    coordinator, gateway = phase_events
    messages = coordinator + gateway
    messages.insert(index, messages[index])
    with pytest.raises(AssertionError):
        CHECK(messages)


@pytest.mark.parametrize("index", range(10))
def test_successful_phase_evidence_rejects_wrong_protocol_owner(phase_events, index):
    coordinator, gateway = phase_events
    messages = coordinator + gateway
    messages[index].protocol = "MQTT"
    with pytest.raises(AssertionError):
        CHECK(messages)


def test_successful_phase_evidence_rejects_early_transfer_source(phase_events):
    coordinator, gateway = phase_events
    gateway[0].stamp.nanosec = 499999999
    with pytest.raises(AssertionError):
        CHECK(coordinator + gateway)


def test_successful_phase_evidence_rejects_result_from_wrong_leg(phase_events):
    coordinator, gateway = phase_events
    coordinator[2], coordinator[5] = coordinator[5], coordinator[2]
    with pytest.raises(AssertionError):
        CHECK(coordinator + gateway)


def test_successful_phase_evidence_rejects_owner_source_time_reversal(phase_events):
    coordinator, gateway = phase_events
    coordinator[1].stamp.nanosec = 499999999
    with pytest.raises(AssertionError):
        CHECK(coordinator + gateway)
