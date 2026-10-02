"""A controller must never activate outside the requested boundary."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fault_injector.controller import FaultController
from fault_injector.models import FaultRequest


def test_target_activation_and_one_shot_consumption():
    events = []
    controller = FaultController(on_event=lambda *args: events.append(args))
    request = FaultRequest("modbus_timeout", "M-1", "assembly", "transfer_start", 0.2)
    controller.enable(request)
    assert controller.query() == (request,)
    assert (
        controller.consume("modbus_timeout", "M-2", "assembly", "transfer_start")
        is None
    )
    assert (
        controller.consume("modbus_timeout", "M-1", "inspection", "transfer_start")
        is None
    )
    assert controller.consume("modbus_timeout", "M-1", "assembly", "request") is None
    assert (
        controller.consume("modbus_timeout", "M-1", "assembly", "transfer_start")
        == request
    )
    assert controller.query() == ()
    assert (
        controller.consume("modbus_timeout", "M-1", "assembly", "transfer_start")
        is None
    )
    assert [entry[0] for entry in events] == ["fault_activated", "fault_consumed"]


def test_persistent_duration_starts_at_activation_and_optional_station_matches():
    now = [10.0]
    controller = FaultController(clock=lambda: now[0])
    request = FaultRequest("mqtt_duplicate", "M-1", None, "request", 2.0, False)
    controller.enable(request)
    now[0] = 100.0
    assert controller.consume("mqtt_duplicate", "M-1", "assembly", "request") == request
    now[0] = 101.9
    assert (
        controller.consume("mqtt_duplicate", "M-1", "inspection", "request") == request
    )
    now[0] = 102.0
    assert controller.consume("mqtt_duplicate", "M-1", "assembly", "request") is None
    assert controller.query() == ()


def test_reset_removes_all_faults_and_emits_correlated_evidence():
    events = []
    controller = FaultController(on_event=lambda *args: events.append(args))
    for name in ("mqtt_disconnect", "plc_fault"):
        controller.enable(FaultRequest(name, "M-1", duration=1.0))
    controller.reset()
    assert controller.query() == ()
    assert [entry[0] for entry in events] == ["fault_reset", "fault_reset"]
    assert all(entry[1].mission_id == "M-1" for entry in events)


@pytest.mark.parametrize(
    "changes",
    [
        {"name": "unknown"},
        {"mission_id": ""},
        {"station": "unknown"},
        {"duration": -1},
        {"duration": float("nan")},
        {"duration": float("inf")},
        {"one_shot": "yes"},
        {"activation_point": ""},
    ],
)
def test_invalid_controls_are_rejected_without_enabling(changes):
    values = dict(name="plc_fault", mission_id="M-1", duration=1.0)
    values.update(changes)
    with pytest.raises(ValueError):
        FaultRequest(**values)


def test_disabled_controller_has_no_events_or_effects():
    events = []
    controller = FaultController(on_event=lambda *args: events.append(args))
    assert controller.consume("mqtt_duplicate", "M-1", "assembly", "request") is None
    controller.reset()
    assert events == []
