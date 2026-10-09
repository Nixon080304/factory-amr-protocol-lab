"""Modbus real transfer service; fake only the socket/client boundary."""

import asyncio
import json
import os
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from factory_interfaces.srv import TransferPart
from factory_interfaces.msg import ProtocolEvent
from payload_simulator.state_machine import PayloadStateMachine
from modbus_gateway.node import ModbusGatewayNode
from fault_injector.models import FaultRequest


@pytest.fixture
def pure_gateway(monkeypatch):
    from rclpy.node import Node
    from rclpy.time import Time

    events, acknowledgements = [], []
    monkeypatch.setattr(Node, "__init__", lambda *args, **kwargs: None)
    monkeypatch.setattr(Node, "set_parameters", lambda *args: [])
    monkeypatch.setattr(
        Node,
        "declare_parameter",
        lambda self, name, default: SimpleNamespace(
            value=["amr_01", "amr_02"] if name == "robot_ids" else default
        ),
    )
    monkeypatch.setattr(
        Node,
        "create_publisher",
        lambda self, kind, *args: SimpleNamespace(
            publish=lambda message: (
                events.append(message)
                if kind is ProtocolEvent
                else acknowledgements.append(message)
            )
        ),
    )
    endpoints = []

    class Endpoint:
        def __init__(self, group, callback):
            self.callback_group = group
            self.callback = callback
            group.add_entity(self)
            endpoints.append(self)

        def cancel(self):
            pass

        def reset(self):
            pass

    def endpoint(*args, callback_group, **kwargs):
        return Endpoint(
            callback_group, args[2] if isinstance(args[1], float) else args[3]
        )

    monkeypatch.setattr(Node, "create_service", endpoint)
    monkeypatch.setattr(Node, "create_subscription", endpoint)
    monkeypatch.setattr(Node, "create_timer", endpoint)
    monkeypatch.setattr(
        Node, "get_clock", lambda self: SimpleNamespace(now=lambda: Time(seconds=1))
    )
    monkeypatch.setattr(
        Node,
        "get_logger",
        lambda self: SimpleNamespace(info=lambda *args: None, error=lambda *args: None),
    )
    monkeypatch.setattr("modbus_gateway.station_client.AsyncModbusTcpClient", Socket)
    Socket.writes = []
    Socket.fault = Socket.unexpected_error = False
    Socket.connect_failures = 0
    node = ModbusGatewayNode()
    node.test_endpoints = endpoints
    node.test_acknowledgements = acknowledgements
    return node, events


def transfer(node, mission="M-1", robot="amr_01", station="assembly", part="motor"):
    return node._transfer(
        TransferPart.Request(
            mission_id=mission, robot_id=robot, station_id=station, part=part
        ),
        TransferPart.Response(),
    )


@pytest.mark.parametrize("robot", ["", "bad/id", "not_configured"])
def test_transfer_rejects_missing_invalid_or_unconfigured_robot(pure_gateway, robot):
    node, _ = pure_gateway
    response = transfer(node, robot=robot)
    assert not response.accepted
    assert response.error_code == "INVALID_MISSION"
    assert Socket.writes == []


def test_identical_duplicate_replays_without_second_plc_cycle(pure_gateway):
    node, events = pure_gateway
    assert transfer(node).accepted
    first_writes = list(Socket.writes)
    assert transfer(node).accepted
    assert Socket.writes == first_writes
    assert (
        len([event for event in events if event.event == "modbus_pickup_started"]) == 1
    )
    finish = next(event for event in events if event.event == "modbus_pickup_finished")
    assert (finish.robot_id, finish.mission_id) == ("amr_01", "M-1")
    assert json.loads(finish.detail)["part"] == "motor"


def test_fault_reset_and_readiness_serialize_without_blocking_transfers(pure_gateway):
    node, _ = pure_gateway
    command = node.fault_subscription
    readiness = node.test_endpoints[1]
    group = command.callback_group
    assert group.beginning_execution(command)
    try:
        assert not readiness.callback_group.can_execute(readiness)
        assert node.service.callback_group.can_execute(node.service)
    finally:
        group.ending_execution(command)
    assert readiness.callback_group.can_execute(readiness)


@pytest.mark.parametrize("robot, part", [("amr_02", "motor"), ("amr_01", "gear")])
def test_conflicting_duplicate_cannot_move_a_second_robot(pure_gateway, robot, part):
    node, _ = pure_gateway
    assert transfer(node).accepted
    response = transfer(node, robot=robot, part=part)
    assert not response.accepted
    assert response.error_code == "CONFLICTING_TRANSFER"


def test_station_fault_evidence_carries_actual_robot_operation(
    pure_gateway, monkeypatch
):
    node, events = pure_gateway
    node.faults.enable(FaultRequest("modbus_delay", "M-1", station="assembly"))
    monkeypatch.setattr(node, "_plc_control", lambda *args: None)
    assert transfer(node, robot="amr_02").accepted
    faults = [event for event in events if event.protocol == "FAULT"]
    assert [event.event for event in faults] == [
        "fault_activated",
        "fault_consumed",
        "fault_reset",
    ]
    assert all(
        (event.mission_id, event.robot_id) == ("M-1", "amr_02") for event in faults
    )


def test_review_expiring_fault_keeps_activation_robot_across_other_transfer(
    pure_gateway, monkeypatch
):
    node, events = pure_gateway
    now = [0.0]
    node.faults._clock = lambda: now[0]
    node.faults.enable(
        FaultRequest(
            "modbus_delay",
            "M-1",
            station="assembly",
            duration=0.2,
            one_shot=False,
        )
    )
    monkeypatch.setattr(node, "_plc_control", lambda *args: None)
    assert transfer(node, robot="amr_01").accepted
    now[0] = 1.0
    assert transfer(node, mission="M-2", robot="amr_02", station="inspection").accepted
    assert all(
        event.robot_id == "amr_01"
        for event in events
        if event.protocol == "FAULT" and event.mission_id == "M-1"
    )


def test_review_reset_waits_for_physical_restore_without_blocking_other_station(
    pure_gateway, monkeypatch
):
    import threading
    from concurrent.futures import ThreadPoolExecutor
    from factory_interfaces.msg import FaultCommand

    node, _ = pure_gateway
    entered, release_transfer, release_restore = (
        threading.Event(),
        threading.Event(),
        threading.Event(),
    )
    physical = [False]

    def control(unit, fault=None):
        if fault is None:
            if not release_restore.is_set():
                raise RuntimeError("physical restoration unavailable")
            physical[0] = False
        else:
            physical[0] = True

    class HeldSocket(Socket):
        async def read_coils(self, *args, **kwargs):
            if kwargs["device_id"] == 1 and not self.started:
                entered.set()
                deadline = asyncio.get_running_loop().time() + 3
                while (
                    not release_transfer.is_set()
                    and asyncio.get_running_loop().time() < deadline
                ):
                    await asyncio.sleep(0.005)
                assert release_transfer.is_set()
            return await super().read_coils(*args, **kwargs)

    monkeypatch.setattr(node, "_plc_control", control)
    monkeypatch.setattr(
        "modbus_gateway.station_client.AsyncModbusTcpClient", HeldSocket
    )
    node.faults.enable(FaultRequest("modbus_delay", "M-1", station="assembly"))
    with ThreadPoolExecutor(max_workers=1) as executor:
        assembly = executor.submit(transfer, node)
        try:
            assert entered.wait(1) and physical[0]
            old_reset = node._reset_controls()
            node.fault_subscription.callback(
                FaultCommand(
                    reset=True,
                    command_id=92,
                    ack_timeout_sec=2.0,
                )
            )
            assert not node.test_acknowledgements
            assert transfer(node, "M-2", "amr_02", "inspection").accepted
            release_restore.set()
            node.test_endpoints[1].callback()
            assert not physical[0]
            assert [ack.command_id for ack in node.test_acknowledgements] == [92]
        finally:
            release_transfer.set()
            release_restore.set()
        assert assembly.result(timeout=2).accepted
        entered.clear()
        release_transfer.clear()
        node.faults.enable(FaultRequest("modbus_delay", "M-3", station="assembly"))
        next_assembly = executor.submit(transfer, node, "M-3", "amr_02")
        try:
            assert entered.wait(1) and physical[0]
            # A late readiness poll for the old generation cannot erase the new
            # station effect, even though the old generation is already restored.
            assert old_reset()
            assert physical[0]
        finally:
            release_transfer.set()
        assert next_assembly.result(timeout=2).accepted
        assert not physical[0]


def test_review_explicit_v1_transfer_preserves_safe_reconnect(
    pure_gateway, monkeypatch
):
    node, _ = pure_gateway

    class ReconnectingSocket(Socket):
        async def read_coils(self, *args, **kwargs):
            result = await super().read_coils(*args, **kwargs)
            if not getattr(self, "disconnected_once", False):
                self.disconnected_once = True
                self.connected = False
            return result

    monkeypatch.setattr(
        "modbus_gateway.station_client.AsyncModbusTcpClient", ReconnectingSocket
    )
    assert transfer(node).accepted


def test_review2_reset_fences_consumed_but_unregistered_physical_fault(
    pure_gateway, monkeypatch
):
    import threading
    from concurrent.futures import ThreadPoolExecutor
    from factory_interfaces.msg import FaultCommand

    node, events = pure_gateway
    consumed, resume = threading.Event(), threading.Event()
    consume = node.faults.consume
    applied = []

    def pause_after_consume(*args, **kwargs):
        fault = consume(*args, **kwargs)
        if fault is not None:
            consumed.set()
            assert resume.wait(2)
        return fault

    def physical_control(unit, fault=None):
        if fault is not None:
            applied.append((unit, fault.mission_id))

    monkeypatch.setattr(node.faults, "consume", pause_after_consume)
    monkeypatch.setattr(node, "_plc_control", physical_control)
    node.faults.enable(FaultRequest("modbus_delay", "M-1", station="assembly"))
    with ThreadPoolExecutor(max_workers=1) as executor:
        assembly = executor.submit(transfer, node)
        try:
            assert consumed.wait(1)
            node.fault_subscription.callback(
                FaultCommand(reset=True, command_id=92, ack_timeout_sec=2.0)
            )
            acknowledged_before_resume = bool(node.test_acknowledgements)
            assert transfer(node, "M-2", "amr_02", "inspection").accepted
        finally:
            resume.set()
        assert assembly.result(timeout=2).accepted
    assert not (acknowledged_before_resume and applied)
    assert [ack.command_id for ack in node.test_acknowledgements] == [92]
    assert all(
        event.robot_id == "amr_01"
        for event in events
        if event.protocol == "FAULT" and event.mission_id == "M-1"
    )


def test_review_lost_boundary_result_never_claims_not_requested(
    pure_gateway, monkeypatch
):
    node, events = pure_gateway

    class CloseFailureSocket(Socket):
        def close(self):
            raise RuntimeError("connection close failed after physical work")

    monkeypatch.setattr(
        "modbus_gateway.station_client.AsyncModbusTcpClient", CloseFailureSocket
    )
    result = transfer(node)
    assert not result.accepted
    assert result.error_code == "PLC_TIMEOUT_TRANSFER_UNKNOWN"
    finish = next(event for event in events if event.event == "modbus_pickup_finished")
    assert json.loads(finish.detail)["transfer_outcome"] == "UNKNOWN"


def test_review_station_client_awaits_connection_hooks_before_close(pure_gateway):
    node, _ = pure_gateway
    stages = []

    async def connected(client):
        assert client.connected
        stages.append("claimed")

    async def finished(result):
        assert result.outcome == "COMPLETED"
        assert stages == ["claimed"]
        stages.append("released")
        return result

    result = asyncio.run(
        node.station.transfer(
            1,
            1,
            on_connected=connected,
            on_finished=finished,
        )
    )
    assert result.success
    assert stages == ["claimed", "released"]


def test_different_stations_progress_concurrently_without_event_cross_talk(
    pure_gateway, monkeypatch
):
    import threading
    from concurrent.futures import ThreadPoolExecutor

    node, events = pure_gateway
    entered, release = threading.Event(), threading.Event()

    class BlockingSocket(Socket):
        async def read_coils(self, *args, **kwargs):
            if kwargs["device_id"] == 1 and not self.started:
                entered.set()
                deadline = asyncio.get_running_loop().time() + 2
                while (
                    not release.is_set()
                    and asyncio.get_running_loop().time() < deadline
                ):
                    await asyncio.sleep(0.005)
                assert release.is_set()
            return await super().read_coils(*args, **kwargs)

    monkeypatch.setattr(
        "modbus_gateway.station_client.AsyncModbusTcpClient", BlockingSocket
    )
    with ThreadPoolExecutor(max_workers=2) as executor:
        assembly = executor.submit(transfer, node)
        assert entered.wait(1)
        inspection = executor.submit(transfer, node, "M-2", "amr_02", "inspection")
        try:
            assert inspection.result(timeout=1).accepted
            busy = transfer(node, "M-3", "amr_02", "assembly")
            assert not busy.accepted
            assert busy.error_code == "STATION_BUSY"
        finally:
            release.set()
        assert assembly.result(timeout=2).accepted
    assert {
        (event.mission_id, event.robot_id)
        for event in events
        if event.protocol == "MODBUS"
    } == {("M-1", "amr_01"), ("M-2", "amr_02")}


@pytest.mark.parametrize("lose_connection", [False, True])
def test_gateway_claims_correlated_plc_cycle_and_releases_exact_owner(
    pure_gateway, monkeypatch, lose_connection
):
    from plc_simulator.server import PlcServer

    node, events = pure_gateway
    node.fault_control_port = 1234
    server = PlcServer(cycle_delay=0)
    operations = []

    class ControlSocket:
        def settimeout(self, timeout):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def sendall(self, frame):
            operations.append(json.loads(frame[3:]))
            self.output = bytearray()

            async def handle():
                reader = asyncio.StreamReader()
                reader.feed_data(frame)
                reader.feed_eof()
                await server._control_request(reader, self)

            from concurrent.futures import ThreadPoolExecutor

            with ThreadPoolExecutor(max_workers=1) as executor:
                executor.submit(asyncio.run, handle()).result(timeout=1)

        def write(self, data):
            self.output.extend(data)

        async def drain(self):
            pass

        def recv(self, size):
            result = bytes(self.output[:size])
            del self.output[:size]
            return result

    tcp = []

    class PlcSocket:
        def __init__(self, *args, **kwargs):
            from plc_simulator.server import OwnershipModbusServer

            if not tcp:
                tcp.append(OwnershipModbusServer(server))
            self.handler = tcp[0].callback_new_connection()
            self.handler.transport = SimpleNamespace(
                get_extra_info=lambda name: ("127.0.0.1", 20101)
            )
            self.ctx = SimpleNamespace(transport=self.handler.transport)
            self.connected = False
            self.sequence = 0

        async def connect(self):
            self.handler.callback_connected()
            self.connected = True
            return True

        async def raw(self, pdu):
            self.sequence += 1
            pdu.transaction_id = self.sequence
            self.handler.last_pdu = pdu
            output = []
            self.handler.server_send = lambda response, address: output.append(response)
            await self.handler.handle_request()
            return output[-1]

        async def read_coils(self, address, *, count, device_id):
            from pymodbus.pdu.bit_message import ReadCoilsRequest

            return await self.raw(
                ReadCoilsRequest(dev_id=device_id, address=address, count=count)
            )

        async def read_holding_registers(self, address, *, count, device_id):
            from pymodbus.pdu.register_message import ReadHoldingRegistersRequest

            return await self.raw(
                ReadHoldingRegistersRequest(
                    dev_id=device_id, address=address, count=count
                )
            )

        async def write_register(self, address, value, *, device_id):
            from pymodbus.pdu.register_message import WriteSingleRegisterRequest

            return await self.raw(
                WriteSingleRegisterRequest(
                    dev_id=device_id, address=address, registers=[value]
                )
            )

        async def write_coil(self, address, value, *, device_id):
            from pymodbus.pdu.bit_message import WriteSingleCoilRequest

            response = await self.raw(
                WriteSingleCoilRequest(dev_id=device_id, address=address, bits=[value])
            )
            if lose_connection and address == 2 and value:
                self.connected = False
                self.handler.callback_disconnected(None)
            return response

        async def write_coils(self, address, values, *, device_id):
            from pymodbus.pdu.bit_message import WriteMultipleCoilsRequest

            return await self.raw(
                WriteMultipleCoilsRequest(
                    dev_id=device_id, address=address, bits=values
                )
            )

        def close(self):
            if self.connected:
                self.connected = False
                self.handler.callback_disconnected(None)

    monkeypatch.setattr(
        "modbus_gateway.node.socket.create_connection",
        lambda *args, **kwargs: ControlSocket(),
    )
    monkeypatch.setattr("modbus_gateway.station_client.AsyncModbusTcpClient", PlcSocket)
    response = transfer(node, robot="amr_02")
    if lose_connection:
        assert not response.accepted
        assert response.error_code == "PLC_TIMEOUT_TRANSFER_UNKNOWN"
        assert [operation["operation"] for operation in operations] == [
            "claim",
            "release",
        ]
        assert server.stations[1].owner == ("amr_02", "M-1", "motor")
        # A zero-delay PLC can complete before disconnect, but the gateway has
        # no matching observation and must retain UNKNOWN and the station owner.
        assert server.stations[1].last_completion == dict(
            robot_id="amr_02", mission_id="M-1", part="motor", cycle_counter=1
        )
        assert server.stations[1].coils[1:3] == [True, True]
        assert not server._sessions
        assert not server._connections
        assert len(tcp) == 1
        return
    assert response.accepted
    assert [operation["operation"] for operation in operations] == [
        "claim",
        "status",
        "release",
    ]
    assert all(
        (operation["robot_id"], operation["mission_id"], operation["part"])
        == ("amr_02", "M-1", "motor")
        for operation in operations
    )
    assert server.stations[1].owner is None
    assert server.stations[1].last_completion == dict(
        robot_id="amr_02", mission_id="M-1", part="motor", cycle_counter=1
    )
    assert (
        next(
            event for event in events if event.event == "modbus_pickup_finished"
        ).robot_id
        == "amr_02"
    )


@pytest.mark.parametrize(
    "failure, expected",
    [
        ("cleanup", "PLC_TIMEOUT_TRANSFER_COMPLETED"),
        ("configure", "PLC_TIMEOUT"),
    ],
)
def test_fault_side_channel_failure_preserves_transfer_outcome(
    monkeypatch, failure, expected
):
    monkeypatch.setattr("modbus_gateway.station_client.AsyncModbusTcpClient", Socket)
    Socket.fault = Socket.unexpected_error = False
    Socket.connect_failures = 0
    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("ROS_DOMAIN_ID", "78")))
    node = ModbusGatewayNode(context=context)
    node.faults.enable(FaultRequest("modbus_delay", "cleanup_test"))

    def control(unit, fault=None):
        if (fault is None) == (failure == "cleanup"):
            raise OSError("fault control acknowledgment lost")

    monkeypatch.setattr(node, "_plc_control", control)
    events = []
    monkeypatch.setattr(node.events, "publish", events.append)
    try:
        response = node._transfer(
            TransferPart.Request(
                mission_id="cleanup_test",
                robot_id="amr_01",
                station_id="assembly",
                part="motor",
            ),
            TransferPart.Response(),
        )
        assert not response.accepted
        assert response.error_code == expected
        if failure == "cleanup":
            finish = events[-1]
            name, outcome, detail = finish.event, finish.outcome, finish.detail
            assert (finish.mission_id, finish.robot_id) == ("cleanup_test", "amr_01")
            fields = json.loads(detail)
            assert fields["part"] == "motor"
            assert fields["station_id"] == "assembly"
            assert fields["transfer_kind"] == "LOADING"
            assert fields["cycle_counter"] == 11
            assert fields["transfer_outcome"] == "COMPLETED"
            assert fields["error_code"] == "PLC_TIMEOUT_TRANSFER_COMPLETED"
            assert "fault cleanup failed" in fields["message"]
            assert outcome == "FAILED"
            payload = PayloadStateMachine()
            assert payload.apply_protocol_event(
                "cleanup_test", "MODBUS", name, outcome, detail, robot_id="amr_01"
            ).applied
            assert payload.state == "IN_TRANSIT"
    finally:
        node.destroy_node()
        context.shutdown()


class Socket:
    writes = []
    fault = False
    connect_failures = 0
    unexpected_error = False

    def __init__(self, *args, **kwargs):
        self.connected = False
        self.started = False

    async def connect(self):
        if self.unexpected_error:
            raise RuntimeError("socket fixture failed")
        if type(self).connect_failures:
            type(self).connect_failures -= 1
            return False
        self.connected = True
        return True

    async def read_coils(self, *args, **kwargs):
        await asyncio.sleep(0.05)
        return SimpleNamespace(bits=[True, False, False, self.started, self.fault])

    async def read_holding_registers(self, *args, **kwargs):
        return SimpleNamespace(
            registers=[
                kwargs["device_id"],
                1,
                11 if self.started else 10,
                7 if self.fault else 0,
            ]
        )

    async def write_register(self, *args, **kwargs):
        self.writes.append(("part", args, kwargs))
        return True

    async def write_coil(self, address, value, **kwargs):
        self.started |= address == 2 and value
        return True

    async def write_coils(self, *args, **kwargs):
        self.writes.append(("cleanup", args, kwargs))
        return True

    def close(self):
        self.connected = False


def test_service_mapping_cycle_detail_failure_and_live_clock(monkeypatch):
    monkeypatch.setattr("modbus_gateway.station_client.AsyncModbusTcpClient", Socket)
    Socket.writes = []
    Socket.fault = False
    Socket.connect_failures = 1
    Socket.unexpected_error = False
    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("ROS_DOMAIN_ID", "78")))
    node = ModbusGatewayNode(context=context)
    peer = rclpy.create_node("modbus_service_peer", context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(node)
    executor.add_node(peer)
    client = peer.create_client(TransferPart, "/factory/transfer_part")
    events = []
    sub = peer.create_subscription(
        ProtocolEvent,
        "/factory/protocol_events",
        events.append,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
    )
    clock = peer.create_publisher(Clock, "/clock", 10)
    assert client.wait_for_service(timeout_sec=2)

    def call(station, mission="M-001"):
        future = client.call_async(
            TransferPart.Request(
                mission_id=mission, robot_id="amr_01", station_id=station, part="motor"
            )
        )
        deadline = time.monotonic() + 4
        stamp = 10
        while not future.done() and time.monotonic() < deadline:
            stamp += 1
            clock.publish(Clock(clock=rclpy.time.Time(seconds=stamp).to_msg()))
            executor.spin_once(timeout_sec=0.02)
        assert future.done()
        for _ in range(20):
            executor.spin_once(timeout_sec=0.01)
        return future.result()

    try:
        assert call("assembly").accepted
        finish = next(e for e in events if e.event == "modbus_pickup_finished")
        start = next(e for e in events if e.event == "modbus_pickup_started")
        assert json.loads(finish.detail) == {
            "station_id": "assembly",
            "transfer_kind": "LOADING",
            "cycle_counter": 11,
            "part": "motor",
        }
        assert finish.stamp.sec > 10
        assert finish.stamp.sec > start.stamp.sec
        assert any(e.event == "station_state_changed" for e in events)
        retries = [e for e in events if e.event == "retry"]
        assert len(retries) == 1
        assert json.loads(retries[0].detail) == {
            "attempt": 1,
            "delay_sec": 0.5,
            "station_id": "assembly",
            "part": "motor",
        }
        assert Socket.writes[0][1] == (1, 1)
        Socket.fault = True
        assert call("inspection").error_code == "PLC_FAULT"
        assert call("invalid").error_code == "INVALID_MISSION"
        Socket.unexpected_error = True
        assert call("assembly", "M-002").error_code == "PLC_TIMEOUT"
    finally:
        executor.shutdown()
        peer.destroy_subscription(sub)
        node.destroy_node()
        peer.destroy_node()
        context.shutdown()
        Socket.unexpected_error = False
