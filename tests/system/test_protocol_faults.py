"""Bounded protocol scenarios with real MQTT, ROS and PLC transports.

The action driver replaces navigation only. Transfer results and payload state
come from the production Modbus gateway and payload state machine.
"""

import asyncio
from contextlib import contextmanager
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import uuid

import paho.mqtt.client as mqtt
import pytest
from pymodbus.client import ModbusTcpClient
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[2]
for package in (
    "fault_injector",
    "mqtt_gateway",
    "modbus_gateway",
    "plc_simulator",
    "payload_simulator",
):
    sys.path.insert(0, str(ROOT / "src" / package))

from factory_interfaces.action import ExecuteFactoryMission
from factory_interfaces.msg import FaultCommand, ProtocolEvent
from factory_interfaces.srv import SetFault, TransferPart
from fault_injector.node import FaultInjectorNode
from mqtt_gateway.node import MqttGatewayNode
from modbus_gateway.node import ModbusGatewayNode
from plc_simulator.server import PlcServer
from payload_simulator.state_machine import PayloadStateMachine


def free_port():
    with socket.socket() as connection:
        connection.bind(("127.0.0.1", 0))
        return connection.getsockname()[1]


def record_owned_resource(kind, value):
    """Publish only resources created by this case to its outer supervisor."""
    directory = os.environ.get("FACTORY_SCENARIO_OUTPUT")
    if not directory:
        return
    path = Path(directory) / "resources.json"
    data = (
        json.loads(path.read_text())
        if path.exists()
        else dict(containers=[], compose_projects=[], groups=[], ports=[])
    )
    data[kind].append(value)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data) + "\n")
    temporary.replace(path)


def record_owned_process(process):
    start = Path(f"/proc/{process.pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    record_owned_resource("groups", dict(pid=process.pid, start_ticks=start))


def wait(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert predicate(), "bounded readiness/outcome deadline exceeded"


def start_executor(executor):
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    runner = threading.Thread(target=spin, daemon=True)
    runner.start()
    return runner, stop


def shutdown_executor(executor, runner, stop):
    stop.set()
    executor.wake()
    runner.join(timeout=5)
    assert not runner.is_alive()
    if isinstance(executor, MultiThreadedExecutor):
        # Humble base shutdown destroys its guard before draining queued pool
        # handlers. Drain only our owned pool while ROS entities remain alive.
        executor._executor.shutdown(wait=True)
        for future in executor._futures:
            if future.done() and not future.cancelled():
                future.result()
        assert all(not worker.is_alive() for worker in executor._executor._threads)
    assert executor.shutdown(timeout_sec=8), "owned ROS callbacks did not finish"


@contextmanager
def services():
    broker_port = free_port()
    name = "factory-fault-" + uuid.uuid4().hex
    config = ROOT / "docker" / "mosquitto.conf"
    owner = uuid.uuid4().hex
    # Record before requesting creation: termination during docker run must not
    # lose a container that the daemon already created. Its label proves owner.
    record_owned_resource("containers", dict(name=name, owner=owner))
    labels = (
        ["--label", f"factory-amr.scenario-owner={owner}"]
        if "FACTORY_SCENARIO_OUTPUT" in os.environ
        else []
    )
    subprocess.run(
        [
            "docker",
            "run",
            "--detach",
            "--name",
            name,
            *labels,
            "-p",
            f"127.0.0.1:{broker_port}:1883",
            "-v",
            f"{config}:/mosquitto/config/mosquitto.conf:ro",
            "eclipse-mosquitto:2.0.22",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    plc = PlcServer(port=0, cycle_delay=0.04, fault_control_port=0)
    probe = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=name + "-probe")
    connected = threading.Event()
    probe.on_connect = lambda *args: connected.set()
    try:
        asyncio.run_coroutine_threadsafe(plc.start(), loop).result(timeout=5)
        # The optional listener must be enabled explicitly, even with an ephemeral port.
        asyncio.run_coroutine_threadsafe(plc.start_fault_control(), loop).result(
            timeout=5
        )
        for port in (broker_port, plc.port, plc.fault_control_port):
            record_owned_resource("ports", port)
        probe.connect_async("127.0.0.1", broker_port)
        probe.loop_start()
        wait(connected.is_set)
        with ModbusTcpClient("127.0.0.1", port=plc.port, timeout=0.5) as client:
            reply = client.read_holding_registers(0, count=4, device_id=1)
            assert not reply.isError() and reply.registers == [1, 0, 0, 0]
        yield broker_port, plc
    finally:
        probe.disconnect()
        probe.loop_stop()
        asyncio.run_coroutine_threadsafe(plc.stop(), loop).result(timeout=5)
        loop.call_soon_threadsafe(loop.stop)
        thread.join(timeout=5)
        loop.close()
        subprocess.run(
            ["docker", "rm", "-f", name],
            check=True,
            capture_output=True,
            text=True,
            timeout=15,
        )
        for port in (broker_port, plc.port, plc.fault_control_port):
            with socket.socket() as connection:
                assert connection.connect_ex(("127.0.0.1", port)) != 0
        assert not thread.is_alive()
        print(
            f"cleanup: owned broker {name}, PLC {plc.port}, control {plc.fault_control_port} stopped"
        )


@pytest.fixture
def rig():
    with services() as (broker_port, plc):
        context = Context()
        rclpy.init(
            context=context,
            domain_id=int(os.environ.get("FACTORY_SCENARIO_DOMAIN", "89")),
        )
        control = FaultInjectorNode(context=context)
        gateway = MqttGatewayNode(
            context=context,
            parameter_overrides=[Parameter("broker_port", value=broker_port)],
        )
        modbus = ModbusGatewayNode(
            context=context,
            parameter_overrides=[
                Parameter("plc_port", value=plc.port),
                Parameter("fault_control_port", value=plc.fault_control_port),
                Parameter("response_timeout", value=0.2),
                Parameter("transfer_timeout", value=0.35),
            ],
        )
        peer = rclpy.create_node("fault_scenario_driver", context=context)
        group = ReentrantCallbackGroup()
        transfer = peer.create_client(
            TransferPart, "/factory/transfer_part", callback_group=group
        )
        set_fault = peer.create_client(
            SetFault, "/factory/faults/set", callback_group=group
        )
        reset = peer.create_client(
            Trigger, "/factory/faults/reset", callback_group=group
        )
        events, executed, statuses, injected_packets = [], [], [], []
        payload = PayloadStateMachine()

        def observe(event):
            events.append(event)
            payload.apply_protocol_event(
                event.mission_id,
                event.protocol,
                event.event,
                event.outcome,
                event.detail,
            )

        sub = peer.create_subscription(
            ProtocolEvent, "/factory/protocol_events", observe, 100
        )
        local_done = threading.Event()
        release = threading.Event()
        release.set()

        def execute(handle):
            executed.append(handle.request.mission_id)
            handle.publish_feedback(
                ExecuteFactoryMission.Feedback(state="LOADING", station="assembly")
            )
            assert release.wait(timeout=6)
            future = transfer.call_async(
                TransferPart.Request(
                    mission_id=handle.request.mission_id,
                    station_id="assembly",
                    part="motor",
                )
            )
            wait(future.done)
            response = future.result()
            if response.accepted:
                handle.publish_feedback(
                    ExecuteFactoryMission.Feedback(
                        state="NAVIGATING_TO_DROPOFF", station="inspection"
                    )
                )
                handle.publish_feedback(
                    ExecuteFactoryMission.Feedback(
                        state="UNLOADING", station="inspection"
                    )
                )
                future = transfer.call_async(
                    TransferPart.Request(
                        mission_id=handle.request.mission_id,
                        station_id="inspection",
                        part="motor",
                    )
                )
                wait(future.done)
                response = future.result()
            if response.accepted:
                handle.succeed()
                result = ExecuteFactoryMission.Result(
                    success=True, final_state="COMPLETED"
                )
            else:
                handle.abort()
                result = ExecuteFactoryMission.Result(
                    success=False,
                    final_state="FAILED",
                    error_code=response.error_code,
                    message=response.message,
                )
            local_done.set()
            return result

        server = ActionServer(
            peer,
            ExecuteFactoryMission,
            "/factory/execute_mission",
            execute,
            callback_group=group,
        )
        executor = MultiThreadedExecutor(num_threads=6, context=context)
        for node in (control, gateway, modbus, peer):
            executor.add_node(node)
        runner, stop = start_executor(executor)
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id="scenario-" + uuid.uuid4().hex
        )
        subscribed = threading.Event()
        client.on_connect = lambda client, *args: client.subscribe(
            [("factory/missions/+/status", 1), ("factory/faults/injected_request", 1)]
        )
        client.on_subscribe = lambda *args: subscribed.set()

        def receive(client, userdata, message):
            if message.topic == "factory/faults/injected_request":
                injected_packets.append(bytes(message.payload))
            else:
                statuses.append(json.loads(message.payload))

        client.on_message = receive
        client.connect("127.0.0.1", broker_port)
        client.loop_start()

        def configure(
            name,
            mission="M-fault",
            station="assembly",
            activation=None,
            duration=1.0,
            **extra,
        ):
            values = dict(
                name=name,
                mission_id=mission,
                station=station,
                activation_point=activation
                or ("request" if name.startswith("mqtt_") else "transfer_start"),
                duration=duration,
                **extra,
            )
            future = set_fault.call_async(SetFault.Request(json=json.dumps(values)))
            wait(future.done)
            assert future.result().accepted, future.result().message
            # Success itself is a delivery barrier. Publish the mission immediately.

        def request(mission="M-fault"):
            raw = dict(
                mission_id=mission,
                robot_id="amr_01",
                pickup="assembly",
                dropoff="inspection",
                part="motor",
            )
            client.publish(
                "factory/missions/request", json.dumps(raw), qos=1
            ).wait_for_publish(timeout=2)

        try:
            wait(
                lambda: (
                    subscribed.is_set()
                    and gateway.connected
                    and gateway.action.server_is_ready()
                    and transfer.service_is_ready()
                    and set_fault.service_is_ready()
                    and reset.service_is_ready()
                    and control.commands.get_subscription_count() == 2
                    and gateway.events.get_subscription_count() > 0
                )
            )
            yield locals()
        finally:
            release.set()
            if executed:
                wait(local_done.is_set)
            client.disconnect()
            client.loop_stop()
            shutdown_executor(executor, runner, stop)
            server.destroy()
            for node in (gateway, modbus, control, peer):
                node.destroy_node()
            context.shutdown()
            assert not runner.is_alive()


@pytest.mark.parametrize(
    "name,error",
    [
        ("modbus_delay", None),
        ("modbus_timeout", "PLC_TIMEOUT"),
        ("modbus_stale_completion", "STALE_PLC_STATE"),
        ("plc_fault", "PLC_FAULT"),
    ],
)
def test_modbus_fault_outcome_cleanup_and_trace(rig, name, error):
    rig["configure"](
        name, duration=0.3 if name == "modbus_delay" else 2.0, fault_code=91
    )
    rig["request"]()
    wait(lambda: any(s["state"] in ("COMPLETED", "FAILED") for s in rig["statuses"]))
    final = rig["statuses"][-1]
    assert (final["state"], final["error_code"]) == (
        "FAILED" if error else "COMPLETED",
        error,
    )
    assert rig["executed"] == ["M-fault"]
    wait(lambda: rig["payload"].state == ("AT_ASSEMBLY" if error else "AT_INSPECTION"))
    with ModbusTcpClient("127.0.0.1", port=rig["plc"].port, timeout=0.5) as client:
        coils = client.read_coils(0, count=5, device_id=1).bits[:5]
        registers = client.read_holding_registers(0, count=4, device_id=1).registers
    assert coils[1:3] == [False, False]
    assert registers[2] == (0 if error else 1)
    if name == "plc_fault":
        assert "fault_code=91" in final["detail"]
    wait(lambda: any(e.event == "modbus_pickup_finished" for e in rig["events"]))
    finish = [e for e in rig["events"] if e.event == "modbus_pickup_finished"]
    assert len(finish) == 1 and finish[0].outcome == (
        "FAILED" if error else "SUCCEEDED"
    )
    if name == "modbus_delay":
        retries = [e for e in rig["events"] if e.event == "retry"]
        assert len(retries) == 1 and retries[0].mission_id == "M-fault"
        assert json.loads(retries[0].detail) == {"attempt": 1, "delay_sec": 0.5}
    wait(
        lambda: any(
            e.event == "fault_reset" and e.mission_id == "M-fault"
            for e in rig["events"]
        )
    )
    assert [
        (e.event, e.mission_id)
        for e in rig["events"]
        if e.protocol == "FAULT" and e.event != "fault_reset"
    ] == [("fault_activated", "M-fault"), ("fault_consumed", "M-fault")]
    print(
        f"{name}: {final['state']}/{final['error_code']}, counter={registers[2]}, robot_coils={coils[1:3]}"
    )


@pytest.mark.parametrize("name", ["mqtt_duplicate", "mqtt_conflict"])
def test_mqtt_duplicate_and_conflict_execute_once_and_trace(rig, name):
    rig["configure"](name)
    rig["request"]()
    wait(lambda: any(s["state"] == "COMPLETED" for s in rig["statuses"]))
    if name == "mqtt_conflict":
        wait(
            lambda: any(
                s["error_code"] == "MISSION_ID_CONFLICT" for s in rig["statuses"]
            )
        )
    before = len(rig["statuses"])
    rig["request"]()
    wait(lambda: len(rig["statuses"]) > before)
    assert rig["statuses"][-1]["state"] == "COMPLETED"
    assert rig["executed"] == ["M-fault"]
    assert rig["plc"].stations[1].registers[2] == 1
    wait(
        lambda: any(
            e.event
            == ("mqtt_rejected" if name == "mqtt_conflict" else "mqtt_duplicate")
            for e in rig["events"]
        )
    )
    assert [
        e.event
        for e in rig["events"]
        if e.protocol == "FAULT" and e.event != "fault_reset"
    ] == ["fault_activated", "fault_consumed"]
    wait(lambda: any(e.event == "fault_reset" for e in rig["events"]))


def test_disconnect_local_completion_and_ordered_replay(rig):
    rig["configure"]("mqtt_disconnect", activation="accepted", duration=1.0)
    rig["request"]()
    wait(lambda: rig["gateway"]._fault_disconnected)
    wait(rig["local_done"].is_set)
    assert not rig["gateway"].connected
    wait(lambda: any(s["state"] == "COMPLETED" for s in rig["statuses"]), timeout=8)
    assert [s["state"] for s in rig["statuses"]] == [
        "RECEIVED",
        "LOADING",
        "NAVIGATING_TO_DROPOFF",
        "UNLOADING",
        "COMPLETED",
    ]
    assert [s["sequence"] for s in rig["statuses"]] == [1, 2, 3, 4, 5]
    assert rig["executed"] == ["M-fault"] and rig["plc"].stations[1].registers[2] == 1
    wait(lambda: any(e.event == "fault_reset" for e in rig["events"]))


def test_mismatched_fault_and_reset_preserve_normal_transfer(rig):
    rig["configure"]("plc_fault", mission="other", station="inspection")
    rig["request"]()
    wait(lambda: any(s["state"] == "COMPLETED" for s in rig["statuses"]))
    assert not [e for e in rig["events"] if e.event == "fault_activated"]
    future = rig["reset"].call_async(Trigger.Request())
    wait(future.done)
    assert future.result().success
    assert rig["gateway"].faults.query() == rig["modbus"].faults.query() == ()
    wait(
        lambda: any(
            e.event == "fault_reset" and e.mission_id == "other" for e in rig["events"]
        )
    )


@pytest.mark.parametrize(
    "raw",
    [
        "{",
        "[]",
        '{"name":"unknown","mission_id":"M-fault"}',
        '{"name":"plc_fault","mission_id":"M-fault","duration":0}',
    ],
)
def test_invalid_ros_control_never_arms_fault(rig, raw):
    future = rig["set_fault"].call_async(SetFault.Request(json=raw))
    wait(future.done)
    assert not future.result().accepted
    assert rig["gateway"].faults.query() == rig["modbus"].faults.query() == ()
    rig["request"]()
    wait(lambda: any(s["state"] == "COMPLETED" for s in rig["statuses"]))
    assert not [e for e in rig["events"] if e.protocol == "FAULT"]


def test_reset_restores_disconnected_transport_before_duration(rig):
    rig["configure"]("mqtt_disconnect", activation="accepted", duration=30.0)
    rig["request"]()
    wait(lambda: rig["gateway"]._fault_disconnected)
    wait(rig["local_done"].is_set)
    future = rig["reset"].call_async(Trigger.Request())
    wait(future.done)
    assert future.result().success
    wait(lambda: any(s["state"] == "COMPLETED" for s in rig["statuses"]))
    assert rig["gateway"].connected and not rig["gateway"]._fault_disconnected
    assert [s["sequence"] for s in rig["statuses"]] == [1, 2, 3, 4, 5]
    wait(lambda: any(e.event == "fault_reset" for e in rig["events"]))


def test_control_listener_disabled_by_default_and_shutdown_closes_pending_client():
    async def scenario():
        server = PlcServer(port=0)
        await server.start()
        assert server._control_server is None
        await server.start_fault_control()
        reader, writer = await asyncio.open_connection(
            "127.0.0.1", server.fault_control_port
        )
        # A real client that never sends a request must not survive owned shutdown.
        deadline = asyncio.get_running_loop().time() + 1
        while (
            not server._control_tasks and asyncio.get_running_loop().time() < deadline
        ):
            await asyncio.sleep(0)
        assert server._control_tasks
        await asyncio.wait_for(server.stop(), 1)
        assert await asyncio.wait_for(reader.read(), 0.2) == b""
        writer.close()
        await writer.wait_closed()

    asyncio.run(scenario())


def test_set_and_reset_fail_without_owning_gateway_acknowledgements():
    context = Context()
    rclpy.init(context=context, domain_id=90)
    control = FaultInjectorNode(
        context=context, parameter_overrides=[Parameter("ack_timeout_sec", value=0.1)]
    )
    peer = rclpy.create_node("missing_ack_peer", context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(control)
    executor.add_node(peer)
    thread, stop = start_executor(executor)
    set_fault = peer.create_client(SetFault, "/factory/faults/set")
    reset = peer.create_client(Trigger, "/factory/faults/reset")
    try:
        wait(lambda: set_fault.service_is_ready() and reset.service_is_ready())
        future = set_fault.call_async(
            SetFault.Request(
                json=json.dumps(dict(name="plc_fault", mission_id="M-ack", duration=1))
            )
        )
        wait(future.done, timeout=2)
        assert not future.result().accepted
        assert "modbus_gateway" in future.result().message
        future = reset.call_async(Trigger.Request())
        wait(future.done, timeout=2)
        assert not future.result().success
        assert (
            "mqtt_gateway" in future.result().message
            and "modbus_gateway" in future.result().message
        )
    finally:
        shutdown_executor(executor, thread, stop)
        control.destroy_node()
        peer.destroy_node()
        context.shutdown()


@pytest.mark.parametrize("delay,accepted", [(0.04, True), (0.25, False)])
def test_bounded_acknowledgements_on_single_thread_executor(delay, accepted):
    context = Context()
    rclpy.init(context=context, domain_id=90)
    control = FaultInjectorNode(
        context=context, parameter_overrides=[Parameter("ack_timeout_sec", value=0.12)]
    )
    peer = rclpy.create_node("ack_delivery_peer", context=context)
    publisher = peer.create_publisher(
        FaultCommand, "/factory/faults/acknowledgements", 100
    )
    pending, delivered = [], []

    def command(message):
        pending.append((time.monotonic() + delay, message))
        # Wrong-owner acknowledgements never satisfy delivery.
        publisher.publish(
            FaultCommand(
                command_id=message.command_id, acknowledged=True, owner="unrelated"
            )
        )

    _subscription = peer.create_subscription(
        FaultCommand, "/factory/faults/commands", command, 100
    )

    def deliver():
        for deadline, message in tuple(pending):
            if time.monotonic() >= deadline:
                for owner in (
                    ("mqtt_gateway", "modbus_gateway")
                    if message.reset
                    else (message.owner,)
                ):
                    publisher.publish(
                        FaultCommand(
                            command_id=message.command_id,
                            acknowledged=True,
                            owner=owner,
                        )
                    )
                pending.remove((deadline, message))
                delivered.append(message.command_id)

    _timer = peer.create_timer(0.01, deliver)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(control)
    executor.add_node(peer)
    thread, stop = start_executor(executor)
    set_fault = peer.create_client(SetFault, "/factory/faults/set")
    reset = peer.create_client(Trigger, "/factory/faults/reset")
    try:
        wait(
            lambda: (
                set_fault.service_is_ready()
                and reset.service_is_ready()
                and control.commands.get_subscription_count() == 1
                and publisher.get_subscription_count() == 1
            )
        )
        future = set_fault.call_async(
            SetFault.Request(
                json=json.dumps(dict(name="plc_fault", mission_id="M-ack", duration=1))
            )
        )
        wait(future.done, timeout=2)
        assert future.result().accepted is accepted
        saved = future.result()
        wait(lambda: len(delivered) == 1, timeout=2)
        assert saved.accepted is accepted and not control._pending
        future = reset.call_async(Trigger.Request())
        wait(future.done, timeout=2)
        assert future.result().success is accepted
        if not accepted:
            assert (
                "mqtt_gateway" in future.result().message
                and "modbus_gateway" in future.result().message
            )
        wait(lambda: len(delivered) == 2, timeout=2)
        assert future.result().success is accepted and not control._pending
    finally:
        shutdown_executor(executor, thread, stop)
        control.destroy_node()
        peer.destroy_node()
        context.shutdown()


def test_inspection_target_keeps_pickup_safe_then_fails_dropoff(rig):
    rig["configure"]("plc_fault", station="inspection", fault_code=91)
    rig["request"]()
    wait(lambda: any(s["state"] == "FAILED" for s in rig["statuses"]))
    final = rig["statuses"][-1]
    assert final["error_code"] == "PLC_FAULT" and "fault_code=91" in final["detail"]
    wait(lambda: rig["payload"].state == "IN_TRANSIT")
    assert [rig["plc"].stations[unit].registers[2] for unit in (1, 2)] == [1, 0]
    assert all(
        rig["plc"].stations[unit].coils[1:3] == [False, False] for unit in (1, 2)
    )
    activation = [e for e in rig["events"] if e.event == "fault_activated"]
    assert (
        len(activation) == 1
        and json.loads(activation[0].detail)["station"] == "inspection"
    )


def test_owned_executor_drains_queued_handlers_before_node_destruction():
    context = Context()
    rclpy.init(context=context, domain_id=90)
    node = rclpy.create_node("queued_shutdown_probe", context=context)
    publisher = node.create_publisher(ProtocolEvent, "/factory/protocol_events", 10)
    executor = MultiThreadedExecutor(num_threads=1, context=context)
    executor.add_node(node)
    gate = threading.Event()
    blocking = executor._executor.submit(lambda: gate.wait(timeout=2))
    executed = []

    def callback():
        publisher.publish(ProtocolEvent(event="queued_owned_handler"))
        executed.append("published")

    _timer = node.create_timer(0.01, callback)
    runner, stop = start_executor(executor)
    release = threading.Timer(0.1, gate.set)
    try:
        wait(lambda: bool(executor._futures))
        # The real ROS timer handler is queued behind a bounded owned worker.
        release.start()
        shutdown_executor(executor, runner, stop)
        assert all(not worker.is_alive() for worker in executor._executor._threads)
        assert executed == ["published"]
        assert blocking.result(timeout=1)
    finally:
        gate.set()
        release.cancel()
        executor.shutdown(timeout_sec=2)
        runner.join(timeout=2)
        executor._executor.shutdown(wait=True)
        node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize("recover_before_deadline", [False, True])
def test_disconnect_reset_waits_for_broker_subscription(rig, recover_before_deadline):
    rig["configure"]("mqtt_disconnect", activation="accepted", duration=30.0)
    rig["request"]()
    wait(lambda: rig["gateway"]._fault_disconnected)
    wait(rig["local_done"].is_set)
    names = subprocess.run(
        [
            "docker",
            "ps",
            "--filter",
            f"publish={rig['broker_port']}",
            "--format",
            "{{.Names}}",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    ).stdout.splitlines()
    assert len(names) == 1 and names[0].startswith("factory-fault-")
    broker = names[0]
    paused = False
    try:
        subprocess.run(
            ["docker", "pause", broker],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        paused = True
        future = rig["reset"].call_async(Trigger.Request())
        if recover_before_deadline:
            wait(
                lambda: (
                    future.done()
                    or any(
                        owners == {"mqtt_gateway"}
                        for _, owners, _ in rig["control"]._pending.values()
                    )
                )
            )
            assert not future.done(), (
                "reset acknowledged before request subscription restored"
            )
            subprocess.run(
                ["docker", "unpause", broker],
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            )
            paused = False
            wait(future.done, timeout=3)
            assert future.result().success, future.result().message
            # Publish immediately after reset success, without another readiness wait.
            rig["request"]()
            wait(lambda: any(e.event == "mqtt_duplicate" for e in rig["events"]))
            assert rig["executed"] == ["M-fault"]
        else:
            wait(future.done, timeout=3)
            assert not future.result().success, (
                "unavailable broker must fail the reset readiness barrier"
            )
            assert "mqtt_gateway" in future.result().message
            assert not rig["gateway"].connected
            retry = rig["reset"].call_async(Trigger.Request())
            wait(retry.done, timeout=3)
            assert not retry.result().success, (
                "retry cannot bypass outstanding request SUBACK"
            )
            assert "mqtt_gateway" in retry.result().message
            subprocess.run(
                ["docker", "unpause", broker],
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            )
            paused = False
            recovered = rig["reset"].call_async(Trigger.Request())
            wait(recovered.done, timeout=3)
            assert recovered.result().success, recovered.result().message
            rig["request"]()
            wait(lambda: any(e.event == "mqtt_duplicate" for e in rig["events"]))
            assert rig["executed"] == ["M-fault"]
    finally:
        if paused:
            subprocess.run(
                ["docker", "unpause", broker],
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            )
        wait(lambda: rig["gateway"].connected, timeout=8)


@pytest.mark.parametrize("name", ["mqtt_duplicate", "mqtt_conflict"])
def test_persistent_mqtt_controls_repeat_without_generated_recursion(rig, name):
    rig["configure"](name, duration=2.0, one_shot=False)
    rig["release"].clear()
    rig["request"]()
    wait(lambda: any(e.event == "fault_reset" for e in rig["events"]))
    rig["request"]()
    # Each external request must activate exactly one generated delivery.
    wait(
        lambda: len([e for e in rig["events"] if e.event == "fault_activated"]) >= 2,
        timeout=3,
    )
    rig["release"].set()
    wait(lambda: any(s["state"] == "COMPLETED" for s in rig["statuses"]))
    wait(lambda: not rig["gateway"].faults.query(), timeout=3)
    before = len([e for e in rig["events"] if e.event == "mqtt_duplicate"])
    rig["request"]()
    wait(
        lambda: len([e for e in rig["events"] if e.event == "mqtt_duplicate"]) > before
    )
    assert len([e for e in rig["events"] if e.event == "fault_activated"]) == 2
    assert len(rig["injected_packets"]) == 2
    assert not rig["gateway"]._fault_request_transport
    assert not rig["gateway"]._pending_injections
    assert rig["executed"] == ["M-fault"]
    assert [rig["plc"].stations[unit].registers[2] for unit in (1, 2)] == [1, 1]
    if name == "mqtt_conflict":
        assert (
            len(
                [s for s in rig["statuses"] if s["error_code"] == "MISSION_ID_CONFLICT"]
            )
            == 2
        )


@pytest.mark.parametrize("recover_before_deadline", [False, True])
def test_set_request_fault_waits_for_internal_subscription(
    rig, recover_before_deadline
):
    names = subprocess.run(
        [
            "docker",
            "ps",
            "--filter",
            f"publish={rig['broker_port']}",
            "--format",
            "{{.Names}}",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    ).stdout.splitlines()
    assert len(names) == 1 and names[0].startswith("factory-fault-")
    broker, paused = names[0], False
    try:
        subprocess.run(
            ["docker", "pause", broker],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        paused = True
        values = dict(
            name="mqtt_duplicate",
            mission_id="M-fault",
            station="assembly",
            activation_point="request",
            duration=1.0,
        )
        future = rig["set_fault"].call_async(SetFault.Request(json=json.dumps(values)))
        wait(lambda: rig["gateway"]._fault_request_transport)
        assert not future.done() and not rig["gateway"]._fault_subscription_ready
        if recover_before_deadline:
            subprocess.run(
                ["docker", "unpause", broker],
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            )
            paused = False
            wait(future.done, timeout=3)
            assert future.result().accepted, future.result().message
            rig["request"]()
            wait(lambda: any(s["state"] == "COMPLETED" for s in rig["statuses"]))
            wait(lambda: any(e.event == "fault_consumed" for e in rig["events"]))
            assert len(rig["injected_packets"]) == 1
            assert rig["executed"] == ["M-fault"]
        else:
            wait(future.done, timeout=3)
            assert (
                not future.result().accepted
                and "mqtt_gateway" in future.result().message
            )
    finally:
        if paused:
            subprocess.run(
                ["docker", "unpause", broker],
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            )
        future = rig["reset"].call_async(Trigger.Request())
        wait(future.done, timeout=3)
        assert future.result().success, future.result().message
        assert (
            not rig["gateway"]._fault_request_transport
            and not rig["gateway"]._pending_injections
        )
