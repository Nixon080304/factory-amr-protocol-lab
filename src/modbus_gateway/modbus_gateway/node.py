"""Typed transfer service with a serialized PLC boundary and live ROS clock."""

import asyncio
import json
import re
import socket
import struct

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from factory_interfaces.srv import TransferPart
from factory_interfaces.msg import ProtocolEvent
from .station_client import StationClient
from fault_injector.node import attach_controls


class ModbusGatewayNode(Node):
    def __init__(self, **kwargs):
        super().__init__("modbus_gateway", **kwargs)
        self.set_parameters([Parameter("use_sim_time", value=True)])
        host = self.declare_parameter("plc_host", "127.0.0.1").value
        port = self.declare_parameter("plc_port", 1502).value
        self.fault_control_port = self.declare_parameter("fault_control_port", 0).value
        self.motor_code = self.declare_parameter("motor_part_code", 1).value
        if type(self.motor_code) is not int or not 0 <= self.motor_code <= 65535:
            raise ValueError("motor_part_code must be a uint16")
        self.events = self.create_publisher(
            ProtocolEvent,
            "/factory/protocol_events",
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.protocol_group = MutuallyExclusiveCallbackGroup()
        self.service = self.create_service(
            TransferPart,
            "/factory/transfer_part",
            self._transfer,
            callback_group=self.protocol_group,
        )
        self.station = StationClient(
            host,
            port,
            on_retry=self._retry,
            on_state=self._state,
            response_timeout=self.declare_parameter("response_timeout", 2.0).value,
            transfer_timeout=self.declare_parameter("transfer_timeout", 2.0).value,
        )
        self.mission_id = ""
        self.faults, self.fault_subscription = attach_controls(
            self, owner="modbus_gateway", callback_group=self.protocol_group
        )

    def _plc_control(self, unit_id, fault=None):
        if not self.fault_control_port:
            raise RuntimeError("PLC fault control listener is not configured")
        with socket.create_connection(
            ("127.0.0.1", self.fault_control_port), timeout=1.0
        ) as connection:
            names = (
                "modbus_delay",
                "modbus_timeout",
                "modbus_stale_completion",
                "plc_fault",
            )
            connection.sendall(
                struct.pack(
                    "!BBdH",
                    names.index(fault.name) + 1 if fault else 0,
                    unit_id,
                    fault.duration if fault else 0.0,
                    fault.fault_code if fault else 0,
                )
            )
            if connection.recv(1) != b"\x01":
                raise RuntimeError("PLC fault control rejected")

    def _event(self, name, outcome="", detail=""):
        self.events.publish(
            ProtocolEvent(
                stamp=self.get_clock().now().to_msg(),
                mission_id=self.mission_id,
                protocol="MODBUS",
                direction="OUTBOUND",
                event=name,
                outcome=outcome,
                detail=detail,
            )
        )

    def _retry(self, attempt, delay):
        self._event(
            "retry", detail=json.dumps({"attempt": attempt, "delay_sec": delay})
        )

    def _state(self, unit_id, coils, registers):
        self._event(
            "station_state_changed",
            detail=json.dumps(
                {"unit_id": unit_id, "coils": coils, "registers": registers}
            ),
        )

    def _transfer(self, request, response):
        self.mission_id = request.mission_id
        if (
            not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", request.mission_id)
            or request.station_id not in ("assembly", "inspection")
            or request.part != "motor"
        ):
            response.error_code = "INVALID_MISSION"
            response.message = "Unsupported transfer request"
            return response
        pickup = request.station_id == "assembly"
        phase = "modbus_pickup" if pickup else "modbus_dropoff"
        self.get_logger().info(
            f"mission_id={request.mission_id} robot_id=amr_01 station={request.station_id} state={'LOADING' if pickup else 'UNLOADING'}"
        )
        self._event(phase + "_started")
        active = []
        try:
            for name in (
                "modbus_delay",
                "modbus_timeout",
                "modbus_stale_completion",
                "plc_fault",
            ):
                fault = self.faults.consume(
                    name, request.mission_id, request.station_id, "transfer_start"
                )
                if fault is not None:
                    active.append(fault)
                    self._plc_control(1 if pickup else 2, fault)
            result = asyncio.run(
                self.station.transfer(1 if pickup else 2, self.motor_code)
            )
            response.accepted, response.error_code, response.message = (
                result.success,
                result.error_code,
                result.message,
            )
            detail = (
                json.dumps(
                    dict(
                        station_id=request.station_id,
                        transfer_kind="LOADING" if pickup else "UNLOADING",
                        cycle_counter=result.cycle_counter,
                    )
                )
                if result.success
                else result.message
            )
        except Exception as error:
            response.accepted = False
            response.error_code = "PLC_TIMEOUT"
            response.message = str(error)
            detail = str(error)
            self.get_logger().error(
                f"Mission {request.mission_id}: PLC boundary failed: {error}"
            )
        finally:
            if active:
                try:
                    self._plc_control(1 if pickup else 2)
                except Exception as error:
                    response.accepted = False
                    response.error_code = "PLC_TIMEOUT"
                    response.message = (
                        f"{response.message}; fault cleanup failed: {error}"
                    )
                    detail = response.message
                for fault in active:
                    self.faults.finish(fault)
        self._event(
            phase + "_finished", "SUCCEEDED" if response.accepted else "FAILED", detail
        )
        self.get_logger().info(
            f"mission_id={request.mission_id} robot_id=amr_01 station={request.station_id} state=TRANSFER_FINISHED error_code={response.error_code}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModbusGatewayNode()
    # The service owns its group. The default group remains free for /clock.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
