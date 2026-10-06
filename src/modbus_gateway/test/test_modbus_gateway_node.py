"""Modbus real transfer service; fake only the socket/client boundary."""

import asyncio
import json
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

    events = []
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
            publish=lambda message: events.append(message)
        ),
    )
    endpoints = []

    class Endpoint:
        def __init__(self, group):
            self.callback_group = group
            group.add_entity(self)
            endpoints.append(self)

        def cancel(self):
            pass

        def reset(self):
            pass

    def endpoint(*args, callback_group, **kwargs):
        return Endpoint(callback_group)

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


def test_gateway_claims_correlated_plc_cycle_and_releases_exact_owner(
    pure_gateway, monkeypatch
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

            asyncio.run(handle())

        def write(self, data):
            self.output.extend(data)

        async def drain(self):
            pass

        def recv(self, size):
            result = bytes(self.output[:size])
            del self.output[:size]
            return result

    class PlcSocket(Socket):
        async def read_coils(self, *args, **kwargs):
            station = server.stations[kwargs["device_id"]]
            station.advance(1)
            return SimpleNamespace(bits=list(station.coils))

        async def read_holding_registers(self, *args, **kwargs):
            return SimpleNamespace(
                registers=list(server.stations[kwargs["device_id"]].registers)
            )

        async def write_register(self, address, value, **kwargs):
            server.stations[kwargs["device_id"]].write_register(address, value)
            return True

        async def write_coil(self, address, value, **kwargs):
            server.stations[kwargs["device_id"]].write_coil(address, value, now=0)
            return True

        async def write_coils(self, address, values, **kwargs):
            for offset, value in enumerate(values):
                server.stations[kwargs["device_id"]].write_coil(
                    address + offset, value, now=1
                )
            return True

    monkeypatch.setattr(
        "modbus_gateway.node.socket.create_connection",
        lambda *args, **kwargs: ControlSocket(),
    )
    monkeypatch.setattr("modbus_gateway.station_client.AsyncModbusTcpClient", PlcSocket)
    assert transfer(node, robot="amr_02").accepted
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
    rclpy.init(context=context, domain_id=78)
    node = ModbusGatewayNode(context=context)
    node.faults.enable(FaultRequest("modbus_delay", "cleanup_test"))

    def control(unit, fault=None):
        if (fault is None) == (failure == "cleanup"):
            raise OSError("fault control acknowledgment lost")

    monkeypatch.setattr(node, "_plc_control", control)
    events = []
    monkeypatch.setattr(node, "_event", lambda *args: events.append(args))
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
            name, outcome, detail = events[-1]
            fields = json.loads(detail)
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
    rclpy.init(context=context, domain_id=78)
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
