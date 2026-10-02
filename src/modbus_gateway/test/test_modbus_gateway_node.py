"""Modbus real transfer service; fake only the socket/client boundary."""

import asyncio
import json
import time
from types import SimpleNamespace

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from factory_interfaces.srv import TransferPart
from factory_interfaces.msg import ProtocolEvent
from modbus_gateway.node import ModbusGatewayNode


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

    def call(station):
        future = client.call_async(
            TransferPart.Request(mission_id="M-001", station_id=station, part="motor")
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
        }
        assert finish.stamp.sec > 10
        assert finish.stamp.sec > start.stamp.sec
        assert any(e.event == "station_state_changed" for e in events)
        retries = [e for e in events if e.event == "retry"]
        assert len(retries) == 1
        assert json.loads(retries[0].detail) == {"attempt": 1, "delay_sec": 0.5}
        assert Socket.writes[0][1] == (1, 1)
        Socket.fault = True
        assert call("inspection").error_code == "PLC_FAULT"
        assert call("invalid").error_code == "INVALID_MISSION"
        Socket.unexpected_error = True
        assert call("assembly").error_code == "PLC_TIMEOUT"
    finally:
        executor.shutdown()
        peer.destroy_subscription(sub)
        node.destroy_node()
        peer.destroy_node()
        context.shutdown()
        Socket.unexpected_error = False
