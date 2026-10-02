"""Real DDS incompatibility and recovery on a separate experiment topic."""

import json
import os
from pathlib import Path
import time

from factory_interfaces.msg import FaultCommand, ProtocolEvent
from factory_interfaces.srv import SetFault
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSPolicyKind
from std_srvs.srv import Trigger

from test_protocol_faults import ROOT, start_executor, shutdown_executor, wait
from fault_injector.models import FaultRequest
from fault_injector.node import FaultInjectorNode


def test_real_dds_mismatch_records_policies_and_recovers():
    FaultRequest(
        "qos_mismatch", "M-qos", activation_point="experiment_start", duration=2.0
    )
    context = Context()
    rclpy.init(
        context=context, domain_id=int(os.environ.get("FACTORY_SCENARIO_DOMAIN", "92"))
    )
    control = FaultInjectorNode(
        context=context,
        parameter_overrides=[
            Parameter("fault_owners", value=["qos_experiment"]),
            Parameter("use_sim_time", value=True),
        ],
    )
    peer = rclpy.create_node("qos_behavior_probe", context=context)
    events = []
    _subscription = peer.create_subscription(
        ProtocolEvent, "/factory/protocol_events", events.append, 100
    )
    configure = peer.create_client(SetFault, "/factory/faults/set")
    reset = peer.create_client(Trigger, "/factory/faults/reset")
    executor = SingleThreadedExecutor(context=context)
    for node in (control, peer):
        executor.add_node(node)
    runner, stop = start_executor(executor)
    try:
        wait(
            lambda: (
                configure.service_is_ready()
                and reset.service_is_ready()
                and control.events.get_subscription_count() > 0
            )
        )
        future = configure.call_async(
            SetFault.Request(
                json=json.dumps(
                    dict(
                        name="qos_mismatch",
                        mission_id="M-qos",
                        activation_point="experiment_start",
                        duration=2.0,
                    )
                )
            )
        )
        wait(future.done)
        assert future.result().accepted, future.result().message
        experiment = control.qos_experiment
        wait(lambda: experiment.offered and experiment.requested)
        assert experiment.offered[0].last_policy_kind == QoSPolicyKind.RELIABILITY
        assert experiment.requested[0].last_policy_kind == QoSPolicyKind.RELIABILITY
        assert experiment.publisher.get_subscription_count() == 0
        assert experiment.samples == []
        wait(lambda: any(event.event == "qos_mismatch_finished" for event in events))
        mismatch = json.loads(
            next(
                event.detail
                for event in events
                if event.event == "qos_mismatch_finished"
            )
        )
        assert mismatch["offered"] == {
            "reliability": "BEST_EFFORT",
            "durability": "VOLATILE",
        }
        assert mismatch["requested"] == {
            "reliability": "RELIABLE",
            "durability": "VOLATILE",
        }
        assert mismatch["match_count"] == mismatch["sample_count"] == 0
        assert 2.0 <= mismatch["elapsed_sec"] < 2.3
        assert control.get_clock().now().nanoseconds == 0, (
            "experiment must progress even with a stalled simulation clock"
        )
        wait(lambda: any(event.event == "qos_recovery_finished" for event in events))
        assert experiment.publisher.get_subscription_count() == 1 and experiment.samples
        recovery = json.loads(
            next(
                event.detail
                for event in events
                if event.event == "qos_recovery_finished"
            )
        )
        assert (
            recovery["requested"]["reliability"] == "BEST_EFFORT"
            and recovery["sample_count"] >= 1
        )
        names = [event.event for event in events]
        assert (
            names.index("qos_mismatch_started")
            < names.index("qos_incompatible_requested")
            < names.index("qos_mismatch_finished")
            < names.index("qos_recovery_finished")
        )
        assert (
            names.index("qos_mismatch_started")
            < names.index("qos_incompatible_offered")
            < names.index("qos_mismatch_finished")
        )
        assert experiment.topic.startswith("/factory/faults/qos_experiment/")
        assert peer.count_publishers("/factory/telemetry") == 0
        output = (
            Path(os.environ["FACTORY_SCENARIO_OUTPUT"]) / "qos-trace.json"
            if "FACTORY_SCENARIO_OUTPUT" in os.environ
            else ROOT
            / ".superpowers/sdd/2026-10-02-factory-amr-03-reliability-release/evidence/qos-trace.json"
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(
                [
                    dict(
                        event=event.event, protocol=event.protocol, detail=event.detail
                    )
                    for event in events
                ],
                indent=2,
            )
            + "\n"
        )
        future = reset.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success
        assert experiment.publisher is experiment.subscriber is None
        wait(
            lambda: (
                peer.count_publishers(experiment.topic)
                == peer.count_subscribers(experiment.topic)
                == 0
            )
        )
        print(
            f"DDS: no match/data for {mismatch['elapsed_sec']:.3f}s; both RELIABILITY callbacks; recovery samples={len(experiment.samples)}; trace={output}"
        )
    finally:
        shutdown_executor(executor, runner, stop)
        for node in (control, peer):
            node.destroy_node()
        context.shutdown()
        print("cleanup: owned QoS endpoints, executor and DDS context stopped")
    if "FACTORY_SCENARIO_OUTPUT" in os.environ:
        return dict(
            final_state="RECOVERED" if recovery["sample_count"] >= 1 else "NO_DATA",
            error_code=None,
            mission_id="M-qos",
            events=events,
            mismatch=mismatch,
            recovery=recovery,
        )


def test_simulation_reset_deadline_with_stalled_clock_and_missing_gazebo():
    context = Context()
    rclpy.init(context=context, domain_id=93)
    control = FaultInjectorNode(
        context=context,
        parameter_overrides=[
            Parameter("fault_owners", value=["simulation"]),
            Parameter("ack_timeout_sec", value=0.15),
            Parameter("use_sim_time", value=True),
        ],
    )
    peer = rclpy.create_node("stalled_fault_probe", context=context)
    reset = peer.create_client(Trigger, "/factory/faults/reset")
    executor = SingleThreadedExecutor(context=context)
    for node in (control, peer):
        executor.add_node(node)
    runner, stop = start_executor(executor)
    try:
        wait(
            lambda: (
                reset.service_is_ready()
                and control.commands.get_subscription_count() == 1
            )
        )
        started = time.monotonic()
        future = reset.call_async(Trigger.Request())
        wait(future.done, timeout=0.6)
        elapsed = time.monotonic() - started
        assert not future.result().success
        assert future.result().message == "Fault acknowledgement timed out: simulation"
        assert 0.15 <= elapsed < 0.6 and control.get_clock().now().nanoseconds == 0
        print(
            f"stalled /clock=0: missing Gazebo reset fails within {elapsed:.3f}s, configured deadline=0.15s"
        )
    finally:
        shutdown_executor(executor, runner, stop)
        for node in (control, peer):
            node.destroy_node()
        context.shutdown()


def test_wrong_marker_records_failed_application_when_pose_service_disappears():
    from gazebo_msgs.srv import SetEntityState

    context = Context()
    rclpy.init(context=context, domain_id=97)
    control = FaultInjectorNode(
        context=context,
        parameter_overrides=[
            Parameter("fault_owners", value=["simulation"]),
            Parameter("ack_timeout_sec", value=0.15),
            Parameter("use_sim_time", value=True),
        ],
    )
    peer = rclpy.create_node("unavailable_pose_probe", context=context)
    events = []
    _subscription = peer.create_subscription(
        ProtocolEvent, "/factory/protocol_events", events.append, 100
    )
    phase = peer.create_publisher(ProtocolEvent, "/factory/protocol_events", 100)
    configure = peer.create_client(SetFault, "/factory/faults/set")
    reset = peer.create_client(Trigger, "/factory/faults/reset")
    # Use an actual ROS service endpoint, then remove it after acknowledged arm.
    # No perception input or pose outcome is fabricated by this fixture.
    pose_service = peer.create_service(
        SetEntityState, "/gazebo/set_entity_state", lambda request, response: response
    )
    executor = SingleThreadedExecutor(context=context)
    for node in (control, peer):
        executor.add_node(node)
    runner, stop = start_executor(executor)
    try:
        wait(
            lambda: (
                configure.service_is_ready()
                and reset.service_is_ready()
                and control.simulation.client.service_is_ready()
                and control.commands.get_subscription_count() == 1
                and control.events.get_subscription_count() == 2
                and phase.get_subscription_count() == 2
            )
        )
        future = configure.call_async(
            SetFault.Request(
                json=json.dumps(
                    dict(
                        name="wrong_marker",
                        mission_id="M-unavailable",
                        station="assembly",
                        activation_point="navigation_start",
                        duration=10.0,
                    )
                )
            )
        )
        wait(future.done)
        assert future.result().accepted, future.result().message
        assert len(control.simulation.controller.query()) == 1
        peer.destroy_service(pose_service)
        wait(lambda: not control.simulation.client.service_is_ready())
        phase.publish(
            ProtocolEvent(
                mission_id="M-unavailable",
                protocol="ROS",
                event="navigation_pickup_started",
            )
        )
        wait(lambda: any(event.event == "fault_consumed" for event in events))
        wait(
            lambda: any(event.event == "wrong_marker_pose_applied" for event in events),
            timeout=0.6,
        )
        applied = [
            event for event in events if event.event == "wrong_marker_pose_applied"
        ]
        assert [
            (event.mission_id, event.protocol, event.outcome) for event in applied
        ] == [("M-unavailable", "SIMULATION", "FAILED")]
        assert json.loads(applied[0].detail) == {
            "entity": "wrong_marker_station",
            "pose": [-3.0, 1.57, 0.35],
        }
        assert [event.event for event in events if event.protocol == "FAULT"] == [
            "fault_activated",
            "fault_consumed",
        ]
        assert control.simulation.controller.query() == ()
        started = time.monotonic()
        future = reset.call_async(Trigger.Request())
        wait(future.done, timeout=0.6)
        assert not future.result().success
        assert future.result().message == "Fault acknowledgement timed out: simulation"
        assert 0.15 <= time.monotonic() - started < 0.6
        assert control.simulation.active is None
        print(
            "unavailable Gazebo activation: fault_activated, fault_consumed, wrong_marker_pose_applied/FAILED; no success; bounded failed reset"
        )
    finally:
        shutdown_executor(executor, runner, stop)
        for node in (control, peer):
            node.destroy_node()
        context.shutdown()
        assert not runner.is_alive()
        print("cleanup: owned pose endpoint, executor and DDS context stopped")


def test_excluded_owner_cannot_arm_fault_outside_reset_scope():
    context = Context()
    rclpy.init(context=context, domain_id=96)
    control = FaultInjectorNode(
        context=context,
        parameter_overrides=[Parameter("fault_owners", value=["qos_experiment"])],
    )
    peer = rclpy.create_node("excluded_owner_probe", context=context)
    acks = peer.create_publisher(FaultCommand, "/factory/faults/acknowledgements", 100)
    received = []

    def acknowledge(message):
        received.append(message)
        acks.publish(
            FaultCommand(
                command_id=message.command_id, owner=message.owner, acknowledged=True
            )
        )

    _subscription = peer.create_subscription(
        FaultCommand, "/factory/faults/commands", acknowledge, 100
    )
    configure = peer.create_client(SetFault, "/factory/faults/set")
    executor = SingleThreadedExecutor(context=context)
    for node in (control, peer):
        executor.add_node(node)
    runner, stop = start_executor(executor)
    try:
        wait(
            lambda: (
                configure.service_is_ready()
                and control.commands.get_subscription_count() == 2
                and acks.get_subscription_count() > 0
            )
        )
        unsupported = configure.call_async(
            SetFault.Request(
                json=json.dumps(
                    dict(
                        name="wrong_marker",
                        mission_id="M-disabled",
                        station="inspection",
                        activation_point="navigation_start",
                    )
                )
            )
        )
        wait(unsupported.done)
        assert not unsupported.result().accepted
        assert (
            unsupported.result().message
            == "wrong_marker supports the assembly station only"
        )
        future = configure.call_async(
            SetFault.Request(
                json=json.dumps(
                    dict(
                        name="nav_reject_once",
                        mission_id="M-disabled",
                        activation_point="navigation_start",
                    )
                )
            )
        )
        wait(future.done)
        assert not future.result().accepted, (
            "set must not arm an owner excluded from acknowledged reset"
        )
        assert future.result().message == "fault owner not enabled: mission_coordinator"
        assert not received
    finally:
        shutdown_executor(executor, runner, stop)
        for node in (control, peer):
            node.destroy_node()
        context.shutdown()
