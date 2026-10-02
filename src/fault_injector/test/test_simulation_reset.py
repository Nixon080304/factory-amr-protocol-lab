"""Real ROS pose/reset services retain an applied board until restoration ACK."""

import json
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.task import Future
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Trigger
from factory_interfaces.msg import ProtocolEvent
from fault_injector.models import FaultRequest
from fault_injector.node import FaultInjectorNode


@pytest.fixture
def scene():
    context = Context()
    rclpy.init(context=context, domain_id=118)
    node = FaultInjectorNode(
        context=context,
        parameter_overrides=[
            Parameter("fault_owners", value=["simulation"]),
            Parameter("ack_timeout_sec", value=0.3),
        ],
    )
    peer = rclpy.create_node("owned_pose_service", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(peer)
    requests, events = [], []
    gate = Future()
    hold = [False]

    async def pose(request, response):
        requests.append(request)
        if hold[0] and request.state.pose.position.z == -10.0:
            await gate
        response.success = True
        return response

    group = ReentrantCallbackGroup()
    server = [
        peer.create_service(
            SetEntityState, "/gazebo/set_entity_state", pose, callback_group=group
        )
    ]
    subscription = peer.create_subscription(
        ProtocolEvent, "/factory/protocol_events", events.append, 100
    )
    reset = peer.create_client(Trigger, "/factory/faults/reset")

    def wait(predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), "owned ROS boundary did not reach expected state"

    def apply(mission="old"):
        control = FaultRequest(
            "wrong_marker", mission, activation_point="navigation_start", duration=30
        )
        node.simulation.controller.enable(control)
        node.simulation.observe(
            ProtocolEvent(mission_id=mission, event="navigation_pickup_started")
        )
        wait(
            lambda: any(
                event.event == "wrong_marker_pose_applied"
                and event.mission_id == mission
                for event in events
            )
        )

    def expire():
        control, _ = node.simulation.active
        node.simulation.active = (control, time.monotonic() - 1)
        node.simulation.expire()

    def unavailable():
        peer.destroy_service(server[0])
        server[0] = None
        wait(lambda: not node.simulation.client.service_is_ready())

    def available():
        server[0] = peer.create_service(
            SetEntityState, "/gazebo/set_entity_state", pose, callback_group=group
        )
        wait(node.simulation.client.service_is_ready)

    wait(lambda: node.simulation.client.service_is_ready() and reset.service_is_ready())
    try:
        yield dict(
            node=node,
            wait=wait,
            apply=apply,
            expire=expire,
            unavailable=unavailable,
            available=available,
            reset=reset,
            requests=requests,
            events=events,
            hold=hold,
            gate=gate,
        )
    finally:
        if not gate.done():
            gate.set_result(True)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert executor.shutdown(timeout_sec=1)
        peer.destroy_subscription(subscription)
        node.destroy_node()
        peer.destroy_node()
        context.try_shutdown()


def test_expired_applied_board_survives_unavailable_restoration_and_recovers(scene):
    scene["apply"]()
    assert scene["events"][-1].outcome == "SUCCEEDED"
    assert scene["requests"][-1].state.pose.position.z == 0.35
    scene["unavailable"]()
    scene["expire"]()
    simulation = scene["node"].simulation
    assert simulation.active is not None
    scene["wait"](
        lambda: any(
            e.event == "wrong_marker_pose_restored" and e.outcome == "FAILED"
            for e in scene["events"]
        )
    )
    assert not any(e.event == "fault_reset" for e in scene["events"])
    reset = scene["reset"].call_async(Trigger.Request())
    scene["wait"](reset.done)
    assert not reset.result().success
    assert simulation.active is not None
    scene["available"]()
    scene["wait"](lambda: simulation.active is None)
    assert scene["requests"][-1].state.pose.position.z == -10.0
    reset = scene["reset"].call_async(Trigger.Request())
    scene["wait"](reset.done)
    assert reset.result().success
    scene["wait"](
        lambda: any(
            e.event == "wrong_marker_pose_restored" and e.outcome == "SUCCEEDED"
            for e in scene["events"]
        )
    )


def test_delayed_restoration_never_acknowledges_early_or_overlaps_new_control(scene):
    scene["apply"]()
    scene["hold"][0] = True
    scene["expire"]()
    scene["wait"](lambda: len(scene["requests"]) == 2)
    simulation = scene["node"].simulation
    assert simulation.active is not None
    reset = scene["reset"].call_async(Trigger.Request())
    # A newer control cannot move the board while its old restore is in flight.
    scene["apply"]("new")
    newer = [
        e
        for e in scene["events"]
        if e.event == "wrong_marker_pose_applied" and e.mission_id == "new"
    ]
    assert newer[-1].outcome == "FAILED"
    assert len(scene["requests"]) == 2
    assert not reset.done()
    scene["gate"].set_result(True)
    scene["wait"](reset.done)
    assert reset.result().success
    assert simulation.active is None
    assert all(
        json.loads(e.detail).get("pose") == [0.0, 0.0, -10.0]
        for e in scene["events"]
        if e.event == "wrong_marker_pose_restored" and e.outcome == "SUCCEEDED"
    )


def test_timed_out_reset_stays_failed_when_later_reset_recovers(scene):
    scene["apply"]()
    scene["hold"][0] = True
    scene["expire"]()
    scene["wait"](lambda: len(scene["requests"]) == 2)
    first = scene["reset"].call_async(Trigger.Request())
    scene["wait"](first.done)
    assert not first.result().success
    assert scene["node"].simulation.active is not None
    second = scene["reset"].call_async(Trigger.Request())
    scene["gate"].set_result(True)
    scene["wait"](second.done)
    assert second.result().success
    assert not first.result().success
    assert scene["node"].simulation.active is None


def test_never_dispatched_application_has_no_active_effect_but_reset_stays_owned(scene):
    scene["unavailable"]()
    scene["apply"]()
    simulation = scene["node"].simulation
    assert scene["requests"] == []
    assert scene["events"][-1].outcome == "FAILED"
    assert simulation.active is None
    reset = scene["reset"].call_async(Trigger.Request())
    scene["wait"](reset.done)
    assert not reset.result().success
    assert simulation.active is None
    assert simulation.restoration is not None
    scene["available"]()
    scene["wait"](lambda: simulation.restoration is None)
    assert scene["requests"][-1].state.pose.position.z == -10.0
    reset = scene["reset"].call_async(Trigger.Request())
    scene["wait"](reset.done)
    assert reset.result().success
