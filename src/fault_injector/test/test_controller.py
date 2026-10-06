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


def test_robot_station_and_global_scopes_match_only_intended_consumers():
    controller = FaultController()
    robot = FaultRequest(
        "nav_reject_once", "M-1", activation_point="navigation_start", robot_id="amr_02"
    )
    controller.enable(robot)
    assert (
        controller.consume(
            "nav_reject_once", "M-1", "assembly", "navigation_start", robot_id="amr_01"
        )
        is None
    )
    assert (
        controller.consume(
            "nav_reject_once", "M-1", "assembly", "navigation_start", robot_id=""
        )
        is None
    )
    assert (
        controller.consume(
            "nav_reject_once", "M-1", "assembly", "navigation_start", robot_id="amr_02"
        )
        == robot
    )
    station = FaultRequest("plc_fault", "M-1", station="inspection")
    controller.enable(station)
    assert (
        controller.consume(
            "plc_fault", "M-1", "assembly", "transfer_start", robot_id="amr_01"
        )
        is None
    )
    assert (
        controller.consume(
            "plc_fault", "M-1", "inspection", "transfer_start", robot_id="amr_02"
        )
        == station
    )
    global_fault = FaultRequest(
        "nav_reject_once", "M-1", activation_point="navigation_start", one_shot=False
    )
    controller.enable(global_fault)
    for identifier in ("amr_01", "amr_02"):
        assert (
            controller.consume(
                "nav_reject_once",
                "M-1",
                "assembly",
                "navigation_start",
                robot_id=identifier,
            )
            == global_fault
        )


@pytest.mark.parametrize("robot_id", ["", "bad/id", "x" * 65])
def test_invalid_explicit_robot_scope_is_rejected(robot_id):
    with pytest.raises(ValueError, match="robot"):
        FaultRequest("nav_reject_once", "M-1", robot_id=robot_id)


@pytest.mark.parametrize("name", ["mqtt_disconnect", "wrong_marker"])
def test_shared_physical_effect_rejects_robot_scope_instead_of_affecting_fleet(name):
    with pytest.raises(ValueError, match="shared"):
        FaultRequest(name, "M-1", robot_id="amr_02")


@pytest.mark.parametrize("station", ["assembly", "inspection"])
def test_shared_disconnect_rejects_station_scope(station):
    with pytest.raises(ValueError, match="global"):
        FaultRequest("mqtt_disconnect", "M-1", station=station)


def test_review_mqtt_and_marker_effect_events_keep_original_trigger_robot():
    from concurrent.futures import Future
    from types import SimpleNamespace
    from rclpy.time import Time
    from factory_interfaces.msg import ProtocolEvent
    from fault_injector.node import attach_controls, SimulationFaults

    events = []
    node = SimpleNamespace(
        ack_group=None,
        events=SimpleNamespace(publish=events.append),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=1)),
        get_logger=lambda: SimpleNamespace(error=lambda *args: None),
        create_publisher=lambda *args: SimpleNamespace(publish=lambda *args: None),
        create_subscription=lambda *args, **kwargs: None,
        create_timer=lambda *args, **kwargs: SimpleNamespace(cancel=lambda: None),
    )
    controller, _ = attach_controls(node, owner="mqtt_gateway")
    controller.enable(
        FaultRequest("mqtt_duplicate", "M-mqtt", activation_point="request")
    )
    effect = controller.consume(
        "mqtt_duplicate", "M-mqtt", "assembly", "request", robot_id="amr_02"
    )
    controller.finish(effect)
    assert [event.robot_id for event in events] == ["amr_02"] * 3

    def send(request):
        future = Future()
        future.set_result(SimpleNamespace(success=True))
        return future

    node.create_client = lambda *args, **kwargs: SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=send,
    )
    simulation = SimulationFaults(node)
    simulation.controller.enable(
        FaultRequest(
            "wrong_marker",
            "M-marker",
            activation_point="navigation_start",
        )
    )
    simulation.observe(
        ProtocolEvent(
            mission_id="M-marker",
            robot_id="amr_01",
            event="navigation_pickup_started",
        )
    )
    assert simulation.reset()()
    marker = [event for event in events if event.mission_id == "M-marker"]
    assert all(event.robot_id == "amr_01" for event in marker)


def test_fault_service_preserves_explicit_robot_target_at_ros_boundary():
    import asyncio
    import json
    from types import SimpleNamespace
    from fault_injector.node import FaultInjectorNode

    node = FaultInjectorNode.__new__(FaultInjectorNode)
    node.owners = ["mission_coordinator"]
    node.robot_ids = frozenset(("amr_01", "amr_02"))
    delivered = []

    async def deliver(command, owners):
        delivered.append(command)
        return True, "acknowledged"

    node._deliver = deliver
    request = SimpleNamespace(
        json=json.dumps(
            dict(
                name="nav_reject_once",
                mission_id="M-1",
                robot_id="amr_02",
                activation_point="navigation_start",
            )
        )
    )
    response = asyncio.run(node._set(request, SimpleNamespace()))
    assert response.accepted
    assert delivered[0].robot_id == "amr_02"
    request.json = json.dumps(
        dict(
            name="nav_reject_once",
            mission_id="M-1",
            robot_id="unknown",
            activation_point="navigation_start",
        )
    )
    response = asyncio.run(node._set(request, SimpleNamespace()))
    assert not response.accepted
    assert len(delivered) == 1


def test_robot_fault_ack_requires_exact_target_and_all_global_consumers():
    import threading
    from types import SimpleNamespace
    from factory_interfaces.msg import FaultCommand
    from fault_injector.node import FaultInjectorNode

    node = FaultInjectorNode.__new__(FaultInjectorNode)
    node._lock = threading.RLock()
    node._pending = {}
    node._command_id = 0
    node.ack_timeout = 2.0
    node.robot_ids = frozenset(("amr_01", "amr_02"))
    published = []
    node.commands = SimpleNamespace(publish=published.append)
    delivery = node._deliver(
        FaultCommand(mission_id="M-1", robot_id="amr_02"), ("mission_coordinator",)
    )
    delivery.send(None)
    identifier = published[-1].command_id
    future = node._pending[identifier][0]
    node._ack(
        FaultCommand(
            command_id=identifier,
            owner="mission_coordinator",
            acknowledged=True,
            robot_id="amr_01",
            mission_id="M-1",
        )
    )
    assert not future.done()
    node._ack(
        FaultCommand(
            command_id=identifier,
            owner="mission_coordinator",
            acknowledged=True,
            robot_id="amr_02",
            mission_id="M-1",
        )
    )
    assert future.done()
    with pytest.raises(StopIteration) as finished:
        delivery.send(None)
    assert finished.value.value[0] is True
    delivery = node._deliver(FaultCommand(mission_id="M-2"), ("mission_coordinator",))
    delivery.send(None)
    identifier = published[-1].command_id
    future = node._pending[identifier][0]
    for robot_id in ("amr_01", "amr_02"):
        node._ack(
            FaultCommand(
                command_id=identifier,
                owner="mission_coordinator",
                acknowledged=True,
                robot_id=robot_id,
                mission_id="M-2",
            )
        )
        assert future.done() == (robot_id == "amr_02")
    with pytest.raises(StopIteration):
        delivery.send(None)
