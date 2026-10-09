"""MQTT adapter real ROS exchanges; only the broker transport is replaced."""

import json
import os
import queue
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from factory_interfaces.action import ExecuteFleetMission
from factory_interfaces.msg import FaultCommand, ProtocolEvent, RobotState
from rosgraph_msgs.msg import Clock
from mqtt_gateway.node import MqttGatewayNode


@pytest.fixture
def without_dds():
    """Use real adapter methods; replace only external broker/action boundaries."""
    from collections import deque
    import queue
    from types import SimpleNamespace
    from mqtt_gateway.mission_registry import MissionRegistry
    from mqtt_gateway.reconnect_queue import ReconnectQueue
    from mqtt_gateway.validator import MissionValidator
    from rclpy.task import Future

    node = object.__new__(MqttGatewayNode)
    broker = Broker()
    node.client, node.connected = broker, True
    node.registry, node.validator = MissionRegistry(), MissionValidator()
    node.reconnect = ReconnectQueue()
    node.sequence, node.pending, node.awaiting_acceptance = {}, {}, {}
    node.assigned_robots, node.missions, node.robot_states = {}, {}, {}
    node.robot_ids = {"amr_01", "amr_02"}
    node.telemetry_reconnect = {}
    node.incoming, node.completions = queue.Queue(), queue.SimpleQueue()
    node.current_mission_id, node.current_state = "", "IDLE"
    node._fault_request_transport, node._fault_disconnected = False, False
    node._pending_injections = deque()
    node._event = lambda *args: None
    node.get_logger = lambda: SimpleNamespace(
        info=lambda message: None, error=lambda message: None
    )
    node._timestamp = lambda: "2026-10-05T00:00:00Z"
    node.faults = SimpleNamespace(consume=lambda *args, **kwargs: None)
    goals = []

    def send(goal, feedback_callback):
        goals.append((goal, feedback_callback))
        return Future()

    node.action = SimpleNamespace(send_goal_async=send, server_is_ready=lambda: True)
    return node, broker, goals


def test_without_dds_automatic_request_uses_empty_fleet_pin(without_dds):
    node, _, goals = without_dds
    node._request(
        b'{"mission_id":"auto","pickup":"assembly","dropoff":"inspection","part":"motor"}'
    )
    node._drain()
    assert goals[0][0].requested_robot_id == ""


def test_without_dds_gateway_online_publication_uses_fleet_availability_topic(
    without_dds,
):
    node, broker, _ = without_dds
    node.incoming.put(("connection", True))
    node._drain()
    assert ("factory/fleet/availability", "online", 1, True) in broker.published


def test_without_dds_explicit_configured_robot_remains_pinned(without_dds):
    node, _, goals = without_dds
    node._request(
        b'{"mission_id":"pin","robot_id":"amr_02","pickup":"assembly","dropoff":"inspection","part":"motor"}'
    )
    node._drain()
    assert goals[0][0].requested_robot_id == "amr_02"


def test_without_dds_protocol_events_preserve_pin_and_reassignment(without_dds):
    from types import MethodType, SimpleNamespace
    from rclpy.time import Time
    from mqtt_gateway.models import MissionPayload

    node, _, _ = without_dds
    events = []
    node._event = MethodType(MqttGatewayNode._event, node)
    node.events = SimpleNamespace(publish=events.append)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=1))
    node.registry.register(
        MissionPayload("pin", "amr_02", "assembly", "inspection", "motor")
    )
    node.registry.register(
        MissionPayload("auto", None, "assembly", "inspection", "motor")
    )
    node._event("pin", "mqtt_acceptance_started")
    node._event("auto", "mqtt_acceptance_started")
    node.assigned_robots["auto"] = "amr_01"
    node._event("auto", "mqtt_duplicate")
    node.assigned_robots["auto"] = "amr_02"
    node._event("auto", "mqtt_duplicate")
    assert [event.robot_id for event in events] == ["amr_02", "", "amr_01", "amr_02"]


def test_without_dds_robot_scoped_mqtt_injection_matches_only_target(without_dds):
    from fault_injector.controller import FaultController
    from fault_injector.models import FaultRequest
    from mqtt_gateway.models import MissionPayload

    node, broker, _ = without_dds
    node.faults = FaultController()
    node._fault_request_transport = node._fault_subscription_ready = True
    node.faults.enable(
        FaultRequest(
            "mqtt_duplicate", "M-target", activation_point="request", robot_id="amr_02"
        )
    )
    node._inject_request_faults(
        MissionPayload("M-target", "amr_01", "assembly", "inspection", "motor")
    )
    assert not node._pending_injections
    node._inject_request_faults(
        MissionPayload("M-target", "amr_02", "assembly", "inspection", "motor")
    )
    assert len(node._pending_injections) == 1
    assert json.loads(node._pending_injections[0].payload)["robot_id"] == "amr_02"
    assert len(broker.published) == 1


def test_without_dds_unconfigured_pin_never_reaches_action(without_dds):
    node, broker, goals = without_dds
    node._request(
        b'{"mission_id":"pin","robot_id":"unconfigured","pickup":"assembly","dropoff":"inspection","part":"motor"}'
    )
    node._drain()
    assert goals == []
    assert statuses(broker)[-1]["error_code"] == "INVALID_MISSION"


def test_without_dds_status_omits_robot_until_assignment_and_keeps_it_after(
    without_dds,
):
    from factory_interfaces.action import ExecuteFleetMission

    node, broker, _ = without_dds
    node.publish_status("auto", "QUEUED")
    assert "robot_id" not in statuses(broker)[-1]
    node._feedback(
        "auto",
        ExecuteFleetMission.Feedback(
            state="ASSIGNED", assigned_robot_id="amr_02", detail="selected"
        ),
    )
    node.publish_status("auto", "EXECUTING")
    assert [payload["robot_id"] for payload in statuses(broker)[1:]] == [
        "amr_02",
        "amr_02",
    ]


def test_without_dds_terminal_result_propagates_assignment_and_error(without_dds):
    from factory_interfaces.action import ExecuteFleetMission
    from mqtt_gateway.models import MissionPayload
    from rclpy.task import Future
    from types import SimpleNamespace

    node, broker, _ = without_dds
    future = Future()
    future.set_result(
        SimpleNamespace(
            result=ExecuteFleetMission.Result(
                success=False,
                final_state="RECOVERY_REQUIRED",
                assigned_robot_id="amr_02",
                error_code="PAYLOAD_UNCERTAIN",
                message="manual recovery",
            )
        )
    )
    node._result(MissionPayload("m1", None, "assembly", "inspection", "motor"), future)
    assert statuses(broker)[-1]["robot_id"] == "amr_02"
    assert statuses(broker)[-1]["error_code"] == "PAYLOAD_UNCERTAIN"


def test_without_dds_reassignment_changes_all_following_status_robot_ids(without_dds):
    from factory_interfaces.action import ExecuteFleetMission

    node, broker, _ = without_dds
    for robot_id in ("amr_01", "amr_02"):
        node._feedback(
            "m1",
            ExecuteFleetMission.Feedback(state="ASSIGNED", assigned_robot_id=robot_id),
        )
        node.publish_status("m1", "EXECUTING")
    assert [s["robot_id"] for s in statuses(broker)] == [
        "amr_01",
        "amr_01",
        "amr_02",
        "amr_02",
    ]


def test_without_dds_telemetry_uses_each_robot_topic_and_retains_each_latest_sample(
    without_dds,
):
    node, broker, _ = without_dds
    node.connected = False
    for robot_id, x in (("amr_01", 1.0), ("amr_02", 2.0), ("amr_01", 3.0)):
        node.publish_telemetry({"robot_id": robot_id, "x": x})
    node.incoming.put(("connection", True))
    node._drain()
    telemetry = [
        (topic, payload["x"])
        for topic, payload, _, _ in broker.published
        if topic.endswith("/telemetry")
    ]
    assert sorted(telemetry) == [
        ("factory/robots/amr_01/telemetry", 3.0),
        ("factory/robots/amr_02/telemetry", 2.0),
    ]


def test_without_dds_vertical_automatic_mqtt_to_terminal_status(without_dds, tmp_path):
    """Bounded vertical driver crosses real MQTT conversion and durable core."""
    from pathlib import Path
    from types import SimpleNamespace
    from factory_interfaces.action import ExecuteFleetMission
    from fleet_manager.adapter import FleetAdapter, RobotFeedback, RobotReply
    from fleet_manager.config import Pose2D, load_fleet_config
    from fleet_manager.journal import MissionJournal
    from fleet_manager.models import CostEstimate, MissionRequest, RobotSnapshot
    from rclpy.task import Future

    node, broker, _ = without_dds
    journal = MissionJournal(tmp_path / "vertical.sqlite3")
    config = load_fleet_config(
        Path(__file__).resolve().parents[2] / "factory_bringup/config/fleet.yaml"
    )
    result_future = Future()
    feedback_sink = [None]
    robot_goals = []

    class Transport:
        def estimate(self, robot_id, mission, callback):
            callback(CostEstimate(True, 2 if robot_id == "amr_02" else 9, 50))

        def send_goal(self, robot_id, mission, feedback, result, accepted):
            assert journal.get(mission.mission_id).assigned_robot_id == robot_id
            robot_goals.append((robot_id, feedback, result))
            accepted(object(), "")

        def publish(self, record, state, detail, progress):
            assert journal.get(record.request.mission_id) == record
            if record.state == "COMPLETED":
                result_future.set_result(
                    SimpleNamespace(
                        result=ExecuteFleetMission.Result(
                            success=True,
                            final_state="COMPLETED",
                            assigned_robot_id=record.assigned_robot_id,
                            message="delivered",
                        )
                    )
                )
            else:
                feedback_sink[0](
                    SimpleNamespace(
                        feedback=ExecuteFleetMission.Feedback(
                            state=state,
                            assigned_robot_id=record.assigned_robot_id or "",
                            detail=detail,
                            progress=float(progress),
                        )
                    )
                )

    fleet = FleetAdapter(config, journal, Transport(), clock=lambda: 100.0)
    for robot_id in ("amr_01", "amr_02"):
        fleet.observe(
            RobotSnapshot(robot_id, "AVAILABLE", Pose2D(0, 0, 0), 80, "EMPTY")
        )

    def send_goal(goal, feedback_callback):
        # Replace DDS only; keep generated ROS types and both production adapters.
        feedback_sink[0] = feedback_callback
        fleet.submit(
            MissionRequest(
                goal.mission_id,
                goal.pickup_station,
                goal.dropoff_station,
                goal.part,
                goal.requested_robot_id or None,
            )
        )
        accepted = Future()
        accepted.set_result(
            SimpleNamespace(accepted=True, get_result_async=lambda: result_future)
        )
        return accepted

    node.action.send_goal_async = send_goal
    try:
        node._request(
            b'{"mission_id":"vertical","pickup":"assembly","dropoff":"inspection","part":"motor"}'
        )
        for _ in range(10):
            node._drain()
            fleet.tick()
            if robot_goals:
                break
        assert len(robot_goals) == 1 and robot_goals[0][0] == "amr_02"
        robot_goals[0][1](RobotFeedback("NAVIGATING_TO_DROPOFF", "loaded", 0.6))
        robot_goals[0][2](RobotReply(True, "", "delivered"))
        fleet.tick()
        node._drain()
        observed = statuses(broker)
        assert "robot_id" not in observed[0]
        assert any(
            status.get("robot_id") == "amr_02" and status["state"] == "ASSIGNED"
            for status in observed
        )
        assert (
            observed[-1]["state"] == "COMPLETED"
            and observed[-1]["robot_id"] == "amr_02"
        )
        assert journal.get("vertical").state == "COMPLETED"
    finally:
        journal.close()


class Broker:
    def __init__(self):
        self.published = []
        self.enabled_generations = []
        self.disabled_count = 0

    def set_handlers(self, message, connection):
        self.message, self.connection = message, connection

    def set_fault_handlers(self, message, connection):
        self.fault_message, self.fault_connection = message, connection

    def start(self):
        self.connection(True)

    def publish(self, topic, payload, qos=0, retain=False):
        self.published.append(
            (
                topic,
                json.loads(payload) if payload.startswith("{") else payload,
                qos,
                retain,
            )
        )
        return True

    def enable_fault_requests(self, generation):
        self.enabled_generations.append(generation)

    def disable_fault_requests(self):
        self.disabled_count += 1

    def close(self):
        pass


@pytest.fixture
def rig(request):
    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("ROS_DOMAIN_ID", "77")))
    broker = Broker()
    node = MqttGatewayNode(mqtt_client=broker, context=context)
    peer = rclpy.create_node("mqtt_action_peer", context=context)
    peer.declare_parameter("execute_delay_sec", 0.0)
    peer.declare_parameter("terminal_feedback_state", "")
    peer.terminal_gate = threading.Event()
    peer.terminal_gate.set()
    executed = []
    busy = [False]

    def execute(handle):
        executed.append(handle.request)
        handle.publish_feedback(
            ExecuteFleetMission.Feedback(
                state="LOADING", assigned_robot_id="amr_01", detail="confirmed"
            )
        )
        time.sleep(peer.get_parameter("execute_delay_sec").value)
        terminal = peer.get_parameter("terminal_feedback_state").value
        if terminal:
            handle.publish_feedback(
                ExecuteFleetMission.Feedback(
                    state=terminal,
                    assigned_robot_id="amr_01",
                    detail="terminal feedback",
                )
            )
            assert peer.terminal_gate.wait(timeout=4)
        if terminal == "FAILED":
            handle.abort()
            return ExecuteFleetMission.Result(
                success=False,
                final_state="FAILED",
                error_code="PLC_FAULT",
                message="actual result",
                assigned_robot_id="amr_01",
            )
        handle.succeed()
        return ExecuteFleetMission.Result(
            success=True,
            final_state="COMPLETED",
            message="actual result",
            assigned_robot_id="amr_01",
        )

    server = (
        None
        if getattr(request, "param", True) is False
        else ActionServer(
            peer,
            ExecuteFleetMission,
            "/factory/execute_fleet_mission",
            execute,
            goal_callback=lambda goal: (
                GoalResponse.REJECT if busy[0] else GoalResponse.ACCEPT
            ),
        )
    )
    executor = (
        SingleThreadedExecutor(context=context)
        if getattr(request, "param", "") == "result-transport"
        else MultiThreadedExecutor(num_threads=3, context=context)
    )
    executor.add_node(node)
    executor.add_node(peer)

    def wait(predicate):
        deadline = time.monotonic() + 4
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    yield broker, node, executed, busy, wait, peer
    if isinstance(executor, MultiThreadedExecutor):
        # Humble base shutdown does not join this fixture's owned worker pool.
        # Drain handlers and surface their errors while ROS entities are alive.
        executor._executor.shutdown(wait=True)
        for future in executor._futures:
            if future.done() and not future.cancelled():
                future.result()
        assert all(not worker.is_alive() for worker in executor._executor._threads)
    if server is not None:
        server.destroy()
    executor.shutdown()
    node.destroy_node()
    peer.destroy_node()
    context.shutdown()


def request(broker, mission_id="M-001", **changes):
    mission = dict(
        mission_id=mission_id,
        robot_id="amr_01",
        pickup="assembly",
        dropoff="inspection",
        part="motor",
    )
    mission.update(changes)
    thread = threading.Thread(
        target=lambda: broker.message(json.dumps(mission).encode())
    )
    thread.start()
    thread.join()


def statuses(broker):
    return [
        payload
        for topic, payload, _, _ in broker.published
        if topic.endswith("/status")
    ]


def arm_request_fault(rig, *, one_shot=True, duration=1.0):
    broker, node, _, _, wait, peer = rig
    acknowledgements = []
    publisher = peer.create_publisher(FaultCommand, "/factory/faults/commands", 100)
    peer.create_subscription(
        FaultCommand, "/factory/faults/acknowledgements", acknowledgements.append, 100
    )
    wait(lambda: publisher.get_subscription_count() > 0)
    publisher.publish(
        FaultCommand(
            command_id=123,
            owner="mqtt_gateway",
            name="mqtt_duplicate",
            mission_id="M-001",
            station="assembly",
            activation_point="request",
            duration=duration,
            one_shot=one_shot,
            fault_code=1,
            ack_timeout_sec=1.0,
        )
    )
    wait(lambda: bool(broker.enabled_generations))
    assert not acknowledgements, "set acknowledged before injection-topic SUBACK"
    broker.fault_connection((broker.enabled_generations[-1], True))
    wait(lambda: any(a.command_id == 123 for a in acknowledgements))
    return publisher, acknowledgements


def test_one_shot_pending_delivery_drains_without_recursive_injection(rig):
    broker, node, executed, _, wait, _ = rig
    arm_request_fault(rig)
    request(broker)
    wait(lambda: bool(node._pending_injections))
    assert node.faults.query() == ()
    assert node._fault_request_transport
    payload = node._pending_injections[0].payload
    broker.fault_message(payload)
    wait(
        lambda: (
            not node._fault_request_transport
            and any(s["state"] == "COMPLETED" for s in statuses(broker))
        )
    )
    assert broker.disabled_count == 1
    assert (
        len([p for p in broker.published if p[0] == "factory/faults/injected_request"])
        == 1
    )
    assert len(executed) == 1


@pytest.mark.parametrize("cancel", ["reset", "expiry"])
def test_pending_generated_packets_rejected_after_reset_or_expiry(rig, cancel):
    broker, node, executed, _, wait, peer = rig
    publisher, acknowledgements = arm_request_fault(rig, one_shot=False, duration=0.25)
    rejected = []
    peer.create_subscription(
        ProtocolEvent,
        "/factory/protocol_events",
        lambda event: (
            rejected.append(event)
            if event.event == "mqtt_fault_packet_rejected"
            else None
        ),
        100,
    )
    wait(lambda: node.events.get_subscription_count() > 0)
    request(broker)
    wait(lambda: bool(node._pending_injections))
    payload = node._pending_injections[0].payload
    # Unrelated packets on the enabled internal path never enter the registry.
    broker.fault_message(payload.replace(b"M-001", b"unrelated"))
    wait(lambda: len(rejected) == 1)
    assert node.registry.payload_for("unrelated") is None
    if cancel == "reset":
        publisher.publish(FaultCommand(command_id=124, reset=True, ack_timeout_sec=1.0))
        wait(lambda: any(a.command_id == 124 for a in acknowledgements))
    wait(lambda: not node._fault_request_transport)
    assert not node._pending_injections and node.faults.query() == ()
    assert broker.disabled_count == 1
    broker.fault_message(payload)
    wait(lambda: len(rejected) == 2)
    wait(lambda: any(s["state"] == "COMPLETED" for s in statuses(broker)))
    request(broker)
    wait(lambda: node.incoming.empty())
    assert (
        len([p for p in broker.published if p[0] == "factory/faults/injected_request"])
        == 1
    )
    assert len(executed) == 1


def test_real_action_conversion_status_schema_and_duplicate(rig):
    broker, node, executed, _, wait, _ = rig
    request(broker)
    wait(lambda: any(s["state"] == "COMPLETED" for s in statuses(broker)))
    assert len(executed) == 1
    assert executed[0].pickup_station == "assembly"
    assert executed[0].dropoff_station == "inspection"
    assert all(
        set(s) | {"robot_id"}
        == {
            "mission_id",
            "robot_id",
            "state",
            "station",
            "timestamp",
            "sequence",
            "detail",
            "error_code",
        }
        for s in statuses(broker)
    )
    count = len(statuses(broker))
    request(broker)
    wait(lambda: len(statuses(broker)) > count)
    assert len(executed) == 1


@pytest.mark.parametrize("terminal", ["COMPLETED", "FAILED"])
def test_action_result_owns_single_terminal_status(rig, terminal):
    broker, node, executed, _, wait, peer = rig
    observed = []
    original_feedback = node._feedback

    def record_feedback(mission_id, feedback):
        observed.append(feedback.state)
        original_feedback(mission_id, feedback)

    node._feedback = record_feedback
    peer.set_parameters(
        [rclpy.parameter.Parameter("terminal_feedback_state", value=terminal)]
    )
    peer.terminal_gate.clear()
    try:
        request(broker)
        wait(
            lambda: (
                terminal in observed
                and any(s["state"] == "RECEIVED" for s in statuses(broker))
            )
        )
        assert not [
            s for s in statuses(broker) if s["state"] in ("COMPLETED", "FAILED")
        ]
        peer.terminal_gate.set()
        wait(lambda: any(s["detail"] == "actual result" for s in statuses(broker)))
        final = [s for s in statuses(broker) if s["state"] in ("COMPLETED", "FAILED")]
        assert len(final) == 1
        assert final[0]["state"] == terminal
        assert final[0]["error_code"] == ("PLC_FAULT" if terminal == "FAILED" else None)
        assert len(executed) == 1
    finally:
        peer.terminal_gate.set()


def test_inflight_duplicate_never_overlaps_acceptance_phase(rig):
    broker, node, executed, _, wait, peer = rig
    events = []
    subscription = peer.create_subscription(
        ProtocolEvent, "/factory/protocol_events", events.append, 100
    )
    wait(lambda: node.events.get_subscription_count() > 0)
    request(broker, "duplicate")
    request(broker, "duplicate")
    wait(
        lambda: (
            any(e.event == "mqtt_acceptance_finished" for e in events)
            and any(s["state"] == "COMPLETED" for s in statuses(broker))
        )
    )
    phases = [e.event for e in events if e.event.startswith("mqtt_acceptance_")]
    assert phases == ["mqtt_acceptance_started", "mqtt_acceptance_finished"]
    assert len(executed) == 1
    peer.destroy_subscription(subscription)


@pytest.mark.parametrize(
    "changes",
    [
        {"part": "unsupported"},
        {"pickup": "inspection"},
        {"dropoff": "assembly"},
        {"robot_id": "not_configured"},
    ],
)
def test_known_id_conflicts_precede_configuration_validation(rig, changes):
    broker, node, executed, _, wait, _ = rig
    request(broker)
    wait(lambda: any(s["state"] == "COMPLETED" for s in statuses(broker)))
    saved = node.registry.state_for("M-001")
    request(broker, **changes)
    wait(
        lambda: any(s["error_code"] == "MISSION_ID_CONFLICT" for s in statuses(broker))
    )
    assert node.registry.state_for("M-001") == saved
    assert len(executed) == 1
    request(broker, "unknown", **changes)
    wait(
        lambda: any(
            s["mission_id"] == "unknown" and s["error_code"] == "INVALID_MISSION"
            for s in statuses(broker)
        )
    )
    assert len(executed) == 1
    broker.message(
        json.dumps(
            dict(
                mission_id="M-001",
                robot_id="amr_01",
                pickup="assembly",
                dropoff="inspection",
                part=12,
            )
        ).encode()
    )
    wait(
        lambda: any(
            s["mission_id"] == "invalid" and s["error_code"] == "INVALID_MISSION"
            for s in statuses(broker)
        )
    )
    assert node.registry.state_for("M-001") == saved


def test_supported_gear_conflicts_with_known_id_and_executes_for_new_id(rig):
    broker, node, executed, _, wait, _ = rig
    request(broker)
    wait(lambda: any(s["state"] == "COMPLETED" for s in statuses(broker)))
    saved = node.registry.state_for("M-001")
    request(broker, part="gear")
    wait(
        lambda: any(s["error_code"] == "MISSION_ID_CONFLICT" for s in statuses(broker))
    )
    assert node.registry.state_for("M-001") == saved
    assert len(executed) == 1
    request(broker, "new-gear", part="gear")
    wait(
        lambda: any(
            s["mission_id"] == "new-gear" and s["state"] == "COMPLETED"
            for s in statuses(broker)
        )
    )
    assert [(goal.mission_id, goal.part) for goal in executed] == [
        ("M-001", "motor"),
        ("new-gear", "gear"),
    ]
    assert node.registry.state_for("M-001") == saved


def test_validation_busy_and_bounded_reconnect_replay(rig):
    broker, node, executed, busy, wait, _ = rig
    busy[0] = True
    request(broker, "busy")
    wait(lambda: any(s["error_code"] == "ROBOT_BUSY" for s in statuses(broker)))
    assert [s["state"] for s in statuses(broker)] == ["FAILED"]
    broker.message(b"{invalid")
    wait(lambda: any(s["error_code"] == "INVALID_MISSION" for s in statuses(broker)))
    assert not executed
    broker.connection(False)
    wait(lambda: not node.connected)
    for index in range(105):
        node.publish_status("busy", "RECOVERING", detail=str(index))
    before = len(statuses(broker))
    broker.connection(True)
    wait(lambda: len(statuses(broker)) == before + 100)
    assert [s["detail"] for s in statuses(broker)[-100:]] == [
        str(i) for i in range(5, 105)
    ]


def test_real_robot_state_conversion_and_latest_telemetry(rig):
    broker, node, _, _, wait, peer = rig
    clock = peer.create_publisher(Clock, "/clock", 10)
    pose_publisher = peer.create_publisher(
        RobotState, "/amr_01/factory/robot_state", 10
    )
    wait(lambda: pose_publisher.get_subscription_count() > 0)
    pose = RobotState(
        robot_id="amr_01",
        mode="AVAILABLE",
        frame_id="map",
        battery_percent=80.0,
        payload_state="EMPTY",
    )
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = 3.0
    clock.publish(Clock(clock=rclpy.time.Time(seconds=100).to_msg()))
    pose_publisher.publish(pose)

    def telemetry():
        return [p for t, p, _, _ in broker.published if t.endswith("/telemetry")]

    wait(lambda: bool(telemetry()))
    sample = telemetry()[-1]
    assert sample["robot_id"] == "amr_01"
    assert sample["frame_id"] == "map" and sample["x"] == 3.0
    assert sample["battery_percent"] == 80.0 and sample["state"] == "AVAILABLE"
    assert sample["timestamp"].startswith("1970-01-01T00:01:40")
    broker.connection(False)
    wait(lambda: not node.connected)
    pose.pose.position.x = 4.0
    pose_publisher.publish(pose)
    wait(
        lambda: (
            "amr_01" in node.telemetry_reconnect
            and node.telemetry_reconnect["amr_01"]["x"] == 4.0
        )
    )
    pose.pose.position.x = 5.0
    pose_publisher.publish(pose)
    wait(lambda: node.telemetry_reconnect["amr_01"]["x"] == 5.0)
    count = len(telemetry())
    broker.connection(True)
    wait(lambda: len(telemetry()) > count)
    assert telemetry()[count]["x"] == 5.0


def test_goal_acceptance_preserves_feedback_received_before_response(rig):
    broker, node, _, _, wait, peer = rig
    peer.set_parameters([rclpy.parameter.Parameter("execute_delay_sec", value=0.15)])
    request(broker)
    wait(
        lambda: (
            any(s["state"] == "LOADING" for s in statuses(broker))
            and node.current_mission_id == "M-001"
        )
    )
    assert node.current_state == "LOADING"
    assert [s["state"] for s in statuses(broker)][:2] == ["RECEIVED", "LOADING"]
    wait(lambda: any(s["state"] == "COMPLETED" for s in statuses(broker)))


@pytest.mark.parametrize("rig", ["result-transport"], indirect=True)
def test_paused_protocol_consumption_preserves_received_then_feedback_order(rig):
    broker, node, _, _, wait, peer = rig
    wait(lambda: node.connected and node.action.server_is_ready())
    node.timer.cancel()
    peer.set_parameters([rclpy.parameter.Parameter("execute_delay_sec", value=0.15)])
    request(broker, "early-feedback")
    node._drain()
    # Poll the real ActionClient feedback while acceptance consumption is paused.
    wait(lambda: node.completions.qsize() >= 2)
    assert statuses(broker) == []
    assert node.registry.state_for("early-feedback") is None
    wait(lambda: not node.completions.empty())
    node._drain()
    assert [s["state"] for s in statuses(broker)][:2] == ["RECEIVED", "LOADING"]
    assert node.current_state == "LOADING"
    node.timer.reset()
    wait(lambda: any(s["state"] == "COMPLETED" for s in statuses(broker)))


@pytest.mark.parametrize("rig", [False], indirect=True)
def test_unavailable_server_does_not_publish_received(rig):
    broker, node, executed, _, wait, peer = rig
    events = []
    subscription = peer.create_subscription(
        ProtocolEvent, "/factory/protocol_events", events.append, 100
    )
    wait(lambda: node.events.get_subscription_count() > 0)
    request(broker, "unavailable")
    wait(lambda: "unavailable" in node.pending and bool(events))
    assert statuses(broker) == []
    assert "unavailable" in node.pending
    assert not executed
    assert [event.event for event in events] == ["mqtt_acceptance_started"]
    peer.destroy_subscription(subscription)


@pytest.mark.parametrize("rig", ["result-transport"], indirect=True)
def test_destroyed_result_client_finishes_acceptance_exactly_once(rig):
    broker, node, _, _, wait, peer = rig
    events = []
    subscription = peer.create_subscription(
        ProtocolEvent, "/factory/protocol_events", events.append, 100
    )
    wait(
        lambda: (
            node.events.get_subscription_count() > 0 and node.action.server_is_ready()
        )
    )
    # Hold adapter response consumption, not the real ROS goal exchange.
    node.timer.cancel()
    request(broker, "result-transport")
    node._drain()
    held_completions = []

    def acceptance_queued():
        # Feedback can precede the goal response. Hold both in original order
        # until the actual accepted response reaches the adapter boundary.
        while True:
            try:
                held_completions.append(node.completions.get_nowait())
            except queue.Empty:
                break
        return any(kind == "accepted" for kind, _, _ in held_completions)

    wait(acceptance_queued)
    acceptance = next(
        future for kind, _, future in held_completions if kind == "accepted"
    )
    assert acceptance.result().accepted
    for completion in held_completions:
        node.completions.put(completion)
    node.action.destroy()
    node._drain()
    # Restore an owned real client for normal node teardown and executor polling.
    node.action = ActionClient(
        node,
        ExecuteFleetMission,
        "/factory/execute_fleet_mission",
        callback_group=node.protocol_group,
    )
    node.events.publish(ProtocolEvent(event="test_delivery_barrier"))
    wait(lambda: any(e.event == "test_delivery_barrier" for e in events))
    wait(
        lambda: any(
            s["error_code"] == "MISSION_TRANSPORT_ERROR" for s in statuses(broker)
        )
    )
    assert [e.outcome for e in events if e.event == "mqtt_acceptance_finished"] == [
        "SUCCEEDED"
    ]
    assert statuses(broker)[0]["state"] == "RECEIVED"
    assert statuses(broker)[-1]["state"] == "FAILED"
    assert (
        len([s for s in statuses(broker) if s["state"] in ("COMPLETED", "FAILED")]) == 1
    )
    peer.destroy_subscription(subscription)
