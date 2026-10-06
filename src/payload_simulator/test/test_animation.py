# SPDX-License-Identifier: Apache-2.0
"""The visible part travels on each conveyor before its final placement."""

import pytest

from payload_simulator.node import animation_steps


@pytest.mark.parametrize(
    "kind, x, expected_y, final, final_frame",
    [
        (
            "LOADING",
            -3.0,
            [2.0, 1.95, 1.9, 1.85, 1.8],
            (-0.03, 0.0, 0.28),
            "factory_amr",
        ),
        (
            "UNLOADING",
            3.0,
            [1.8, 1.85, 1.9, 1.95, 2.0],
            (3.0, 2.0, 0.65),
            "world",
        ),
    ],
)
def test_part_travels_in_bounded_steps_before_final_placement(
    kind, x, expected_y, final, final_frame
):
    steps = list(animation_steps(kind))
    assert all(len(step) == 3 for step in steps), "every pose needs a reference frame"
    parts = [
        (target.position, reference)
        for name, target, reference in steps
        if name == "factory_part"
    ]
    visible = parts[:-1]
    assert [position.y for position, _ in visible] == pytest.approx(expected_y)
    assert all(
        position.x == x and position.z == 0.65 and reference == "world"
        for position, reference in visible
    )
    final_position, reference = parts[-1]
    assert (final_position.x, final_position.y, final_position.z) == final
    assert reference == final_frame
    belts = [
        target.position for name, target, reference in steps if name != "factory_part"
    ]
    assert len(belts) == 4
    assert belts[-1].y == 2.0
    assert all(position.x == x and position.z == 0.603 for position in belts)


def test_robot_visual_uses_configured_payload_and_entity_for_future_robot():
    from payload_simulator.node import carried_part_step

    steps = list(
        animation_steps(
            "LOADING",
            part_entity="payload_17",
            robot_entity="robot_17::amr_17_base_link",
        )
    )
    assert steps[-1][0] == "payload_17"
    assert steps[-1][2] == "robot_17::amr_17_base_link"
    assert all(name != "factory_part" for name, _, _ in steps)
    assert carried_part_step("payload_17", "robot_17::amr_17_base_link")[::2] == (
        "payload_17",
        "robot_17::amr_17_base_link",
    )


def test_interleaved_node_events_publish_and_animate_only_correct_robot(monkeypatch):
    import json
    from types import SimpleNamespace
    from concurrent.futures import Future
    from rclpy.node import Node
    from rclpy.time import Time
    from factory_interfaces.msg import ProtocolEvent
    from payload_simulator.node import PayloadSimulatorNode

    clock = [0]
    states, poses, warnings = [], [], []
    transport_failed = [False]
    parameters = dict(
        robot_ids=["amr_01", "amr_02"],
        robot_entities=["amr_01::base_link", "amr_02::base_link"],
        part_entities=["part_01", "part_02"],
    )
    monkeypatch.setattr(Node, "__init__", lambda *args, **kwargs: None)
    monkeypatch.setattr(Node, "has_parameter", lambda *args: False)
    monkeypatch.setattr(
        Node,
        "declare_parameter",
        lambda self, name, value: SimpleNamespace(value=parameters.get(name, value)),
    )
    monkeypatch.setattr(
        Node,
        "create_publisher",
        lambda *args: SimpleNamespace(
            publish=lambda message: states.append(json.loads(message.data))
        ),
    )
    monkeypatch.setattr(Node, "create_subscription", lambda *args: None)
    monkeypatch.setattr(Node, "create_timer", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        Node,
        "get_clock",
        lambda *args: SimpleNamespace(now=lambda: Time(nanoseconds=clock[0])),
    )
    monkeypatch.setattr(
        Node, "get_logger", lambda *args: SimpleNamespace(warning=warnings.append)
    )

    def send(request):
        if transport_failed[0]:
            raise RuntimeError("carried-pose transport unavailable")
        poses.append((request.state.name, request.state.reference_frame))
        future = Future()
        future.set_result(SimpleNamespace(success=True))
        return future

    monkeypatch.setattr(
        Node,
        "create_client",
        lambda *args: SimpleNamespace(service_is_ready=lambda: True, call_async=send),
    )
    node = PayloadSimulatorNode()

    def event(robot, mission, kind, counter):
        node._on_event(
            ProtocolEvent(
                robot_id=robot,
                mission_id=mission,
                protocol="MODBUS",
                outcome="SUCCEEDED",
                event="modbus_pickup_finished"
                if kind == "LOADING"
                else "modbus_dropoff_finished",
                detail=json.dumps(
                    dict(
                        station_id="assembly" if kind == "LOADING" else "inspection",
                        transfer_kind=kind,
                        cycle_counter=counter,
                    )
                ),
            )
        )

    def animate():
        for _ in range(100):
            clock[0] += 150_000_000
            node._tick()

    event("amr_01", "M-1", "LOADING", 1)
    animate()
    event("amr_02", "M-2", "LOADING", 2)
    poses.clear()
    for _ in range(20):
        clock[0] += 150_000_000
        node._tick()
    assert poses.count(("part_01", "amr_01::base_link")) >= 3
    animate()
    assert node._carrying == {"amr_01": "M-1", "amr_02": "M-2"}
    assert ("part_01", "amr_01::base_link") in poses
    assert ("part_02", "amr_02::base_link") in poses
    assert not any(
        name == "part_01" and frame == "amr_02::base_link" for name, frame in poses
    )
    event("amr_02", "M-1", "UNLOADING", 3)
    assert len(states) == 4
    event("amr_01", "M-1", "UNLOADING", 3)
    assert node._carrying == {"amr_02": "M-2"}
    animate()
    assert states[-1]["robot_id"] == "amr_01"
    assert states[-1]["state"] == "AT_INSPECTION"
    assert node._machine.state_for("amr_02", "M-2").value == "IN_TRANSIT"
    transport_failed[0] = True
    animate()
    assert not node._carrying
    assert node._machine.state_for("amr_02", "M-2").value == "IN_TRANSIT"
    assert any("carried-pose transport unavailable" in warning for warning in warnings)
    transport_failed[0] = False
    node = PayloadSimulatorNode()
    poses.clear()
    event("amr_01", "M-pending", "LOADING", 1)
    event("amr_01", "M-pending", "UNLOADING", 2)
    animate()
    assert ("part_01", "amr_01::base_link") not in poses
    assert not node._carrying


def test_missing_robot_event_cannot_publish_or_queue_visual(monkeypatch):
    from collections import deque
    from factory_interfaces.msg import ProtocolEvent
    from payload_simulator.node import PayloadSimulatorNode
    from payload_simulator.state_machine import PayloadStateMachine

    node = PayloadSimulatorNode.__new__(PayloadSimulatorNode)
    node._machine = PayloadStateMachine(("amr_01",))
    node._animations = deque()
    node._on_event(
        ProtocolEvent(
            mission_id="M-1",
            protocol="MODBUS",
            event="modbus_pickup_finished",
            outcome="SUCCEEDED",
            detail='{"station_id":"assembly","transfer_kind":"LOADING","cycle_counter":1}',
        )
    )
    assert not node._animations
    assert not node._machine.states
