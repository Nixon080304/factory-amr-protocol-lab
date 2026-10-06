"""Typed transfer service with a serialized PLC boundary and live ROS clock."""

import asyncio
from contextvars import ContextVar
from dataclasses import replace
import json
import re
import socket
import struct
import threading
import time

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
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
        self.robot_ids = frozenset(
            self.declare_parameter("robot_ids", ["amr_01"]).value
        )
        if not self.robot_ids or any(
            not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", identifier)
            for identifier in self.robot_ids
        ):
            raise ValueError("robot_ids must contain valid configured robot IDs")
        self._request = ContextVar("transfer_request", default=None)
        self._session = ContextVar("transfer_session", default=None)
        self._effect_locks = {unit: threading.RLock() for unit in (1, 2)}
        self._physical_lock = threading.RLock()
        self._physical_effects = {}
        self._station_locks = {
            station: threading.Lock() for station in ("assembly", "inspection")
        }
        self._requests = {}
        self._requests_lock = threading.Lock()
        if type(self.motor_code) is not int or not 0 <= self.motor_code <= 65535:
            raise ValueError("motor_part_code must be a uint16")
        self.events = self.create_publisher(
            ProtocolEvent,
            "/factory/protocol_events",
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.protocol_group = ReentrantCallbackGroup()
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
        self.fault_group = MutuallyExclusiveCallbackGroup()
        self.faults, self.fault_subscription = attach_controls(
            self,
            owner="modbus_gateway",
            callback_group=self.fault_group,
            on_reset=self._reset_controls,
        )

    def _reset_controls(self):
        with self._physical_lock:
            effects = tuple(self._physical_effects.values())

        def ready():
            restored = [self._restore_effect(effect) for effect in effects]
            return all(restored)

        return ready

    def _restore_effect(self, effect, *, wait=False):
        lock = self._effect_locks[effect["unit_id"]]
        if not lock.acquire(blocking=wait):
            return False
        try:
            if effect["restored"]:
                return True
            request_token = self._request.set(effect["request"])
            session_token = self._session.set(effect["session"])
            try:
                self._plc_control(effect["unit_id"])
            except Exception as error:
                effect["error"] = str(error)
                return False
            finally:
                self._session.reset(session_token)
                self._request.reset(request_token)
            effect["restored"] = True
            with self._physical_lock:
                if self._physical_effects.get(effect["unit_id"]) is effect:
                    del self._physical_effects[effect["unit_id"]]
            for fault in effect["faults"]:
                self.faults.finish(fault)
            return True
        finally:
            lock.release()

    def _plc_control(self, unit_id, fault=None):
        if self._session.get() is not None:
            if not self._plc_ownership(
                unit_id, "fault", self._request.get(), fault=fault
            )["accepted"]:
                raise RuntimeError("Authorized PLC fault control rejected")
            return
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
        request = self._request.get()
        if request is not None:
            try:
                fields = json.loads(detail) if detail else {}
            except (ValueError, TypeError):
                fields = {"message": detail}
            if not isinstance(fields, dict):
                fields = {"message": detail}
            fields.update(station_id=request.station_id, part=request.part)
            detail = json.dumps(fields)
        self.events.publish(
            ProtocolEvent(
                stamp=self.get_clock().now().to_msg(),
                mission_id=request.mission_id if request is not None else "",
                robot_id=request.robot_id if request is not None else "",
                protocol="MODBUS",
                direction="OUTBOUND",
                event=name,
                outcome=outcome,
                detail=detail,
            )
        )

    def _plc_ownership(
        self, unit_id, operation, request, *, endpoint=None, session=None, fault=None
    ):
        fields = dict(
            operation=operation,
            unit_id=unit_id,
            robot_id=request.robot_id,
            mission_id=request.mission_id,
            part=request.part,
        )
        if operation == "claim":
            fields["endpoint"] = list(endpoint)
            return self._plc_exchange(fields)
        session = session or self._session.get()
        if session is None:
            raise RuntimeError("PLC ownership session is unavailable")
        with session["lock"]:
            session["sequence"] += 1
            fields.update(session=session["token"], sequence=session["sequence"])
            if operation == "fault":
                fields.update(
                    name=fault.name if fault is not None else "",
                    duration=fault.duration if fault is not None else 0.0,
                    fault_code=fault.fault_code if fault is not None else 0,
                )
            return self._plc_exchange(fields)

    def _plc_exchange(self, fields):
        encoded = json.dumps(fields, allow_nan=False).encode()
        with socket.create_connection(
            ("127.0.0.1", self.fault_control_port), timeout=1.0
        ) as connection:
            deadline = time.monotonic() + 2.0

            def read_exact(size):
                result = bytearray()
                while len(result) < size:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("PLC ownership response deadline")
                    connection.settimeout(remaining)
                    chunk = connection.recv(size - len(result))
                    if not chunk:
                        raise OSError("PLC ownership response closed")
                    result.extend(chunk)
                return bytes(result)

            connection.sendall(b"\xff" + struct.pack("!H", len(encoded)) + encoded)
            header = read_exact(3)
            size = struct.unpack("!H", header[1:])[0]
            if header[:1] != b"\xff" or not 1 <= size <= 1024:
                raise ValueError("invalid PLC ownership response")
            reply = json.loads(read_exact(size))
            if not isinstance(reply, dict) or type(reply.get("accepted")) is not bool:
                raise ValueError("invalid PLC ownership result")
            return reply

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
        if (
            not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", request.mission_id)
            or request.robot_id not in self.robot_ids
            or request.station_id not in ("assembly", "inspection")
        ):
            response.error_code = "INVALID_MISSION"
            response.message = "Unsupported transfer request"
            return response
        key = (request.mission_id, request.station_id)
        identity = (request.robot_id, request.part)
        with self._requests_lock:
            previous = self._requests.get(key)
            if previous is not None:
                if previous[0] != identity:
                    response.error_code = "CONFLICTING_TRANSFER"
                    response.message = (
                        "Mission station transfer has another robot or part"
                    )
                elif previous[1] is None:
                    response.error_code = "STATION_BUSY"
                    response.message = "Identical transfer is still in progress"
                else:
                    response.accepted, response.error_code, response.message = previous[
                        1
                    ]
                return response
            if request.part != "motor":
                response.error_code = "INVALID_MISSION"
                response.message = "Unsupported transfer part"
                return response
            lock = self._station_locks[request.station_id]
            if not lock.acquire(blocking=False):
                response.error_code = "STATION_BUSY"
                response.message = "Station transfer is in progress"
                return response
            unit = 1 if request.station_id == "assembly" else 2
            with self._physical_lock:
                if unit in self._physical_effects:
                    lock.release()
                    response.error_code = "STATION_BUSY"
                    response.message = "Station physical fault restoration is pending"
                    return response
            self._requests[key] = (identity, None)
        token = self._request.set(request)
        try:
            return self._execute_transfer(request, response)
        finally:
            with self._requests_lock:
                self._requests[key] = (
                    identity,
                    (response.accepted, response.error_code, response.message),
                )
            self._request.reset(token)
            lock.release()

    def _execute_transfer(self, request, response):
        pickup = request.station_id == "assembly"
        unit_id = 1 if pickup else 2
        phase = "modbus_pickup" if pickup else "modbus_dropoff"
        self.get_logger().info(
            f"mission_id={request.mission_id} robot_id={request.robot_id} station={request.station_id} state={'LOADING' if pickup else 'UNLOADING'}"
        )
        self._event(phase + "_started")
        outcome = "NOT_REQUESTED"
        result = None
        claimed = False
        session = None
        physical_effect = None

        def connected(client):
            nonlocal claimed, session, physical_effect
            if self.fault_control_port:
                endpoint = client.ctx.transport.get_extra_info("sockname")[:2]
                ownership = self._plc_ownership(
                    unit_id, "claim", request, endpoint=endpoint
                )
                if not ownership["accepted"]:
                    response.error_code = "STATION_BUSY"
                    raise RuntimeError(
                        "PLC station or Modbus connection cannot be claimed"
                    )
                token = ownership.get("session")
                if not isinstance(token, str) or not re.fullmatch(
                    r"[A-Za-z0-9_-]{32,64}", token
                ):
                    raise RuntimeError("PLC claim lacks a valid connection session")
                session = dict(token=token, sequence=0, lock=threading.RLock())
                self._session.set(session)
                claimed = True
            for name in (
                "modbus_delay",
                "modbus_timeout",
                "modbus_stale_completion",
                "plc_fault",
            ):
                with self._effect_locks[unit_id]:
                    fault = self.faults.consume(
                        name,
                        request.mission_id,
                        request.station_id,
                        "transfer_start",
                        robot_id=request.robot_id,
                    )
                    if fault is not None:
                        with self._physical_lock:
                            current = self.faults.is_current(fault)
                            if current:
                                if (
                                    physical_effect is None
                                    or physical_effect["restored"]
                                ):
                                    physical_effect = dict(
                                        unit_id=unit_id,
                                        request=request,
                                        session=session,
                                        faults=[],
                                        restored=False,
                                        error="",
                                    )
                                self._physical_effects[unit_id] = physical_effect
                                physical_effect["faults"].append(fault)
                        if not current:
                            # Reset has already covered this consumed control.
                            # It must never acquire a physical effect after ACK.
                            self.faults.finish(fault)
                            continue
                        self._plc_control(unit_id, fault)

        def finished(transfer_result):
            # The actual Modbus connection is still open here, after raw cleanup.
            # Never use a replacement connection to authorize ambiguous work.
            if claimed and transfer_result.outcome == "COMPLETED":
                try:
                    ownership = self._plc_ownership(
                        unit_id, "status", request, session=session
                    )
                    expected = dict(
                        robot_id=request.robot_id,
                        mission_id=request.mission_id,
                        part=request.part,
                        cycle_counter=transfer_result.cycle_counter,
                    )
                    if (
                        not ownership["accepted"]
                        or ownership.get("last_completion") != expected
                    ):
                        raise RuntimeError(
                            "PLC completion does not match transfer connection owner"
                        )
                except Exception as error:
                    transfer_result = replace(
                        transfer_result,
                        success=False,
                        error_code="PLC_TIMEOUT",
                        message=str(error),
                        outcome="UNKNOWN",
                    )
            restored = physical_effect is None or self._restore_effect(
                physical_effect, wait=True
            )
            if not restored:
                transfer_result = replace(
                    transfer_result,
                    success=False,
                    error_code="PLC_TIMEOUT",
                    message=f"{transfer_result.message}; fault cleanup failed: {physical_effect['error']}",
                )
            if claimed and restored:
                try:
                    if not self._plc_ownership(
                        unit_id, "release", request, session=session
                    )["accepted"]:
                        raise RuntimeError("PLC connection ownership cleanup rejected")
                except Exception as error:
                    transfer_result = replace(
                        transfer_result,
                        success=False,
                        error_code=transfer_result.error_code or "PLC_TIMEOUT",
                        message=f"{transfer_result.message}; ownership cleanup failed: {error}",
                    )
            return transfer_result

        try:
            outcome = "UNKNOWN"
            result = asyncio.run(
                self.station.transfer(
                    unit_id,
                    self.motor_code,
                    on_connected=connected,
                    on_finished=finished,
                    connection_bound=bool(self.fault_control_port),
                )
            )
            outcome = result.outcome
            response.accepted = result.success
            response.error_code = response.error_code or result.error_code
            response.message = result.message
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
            response.error_code = response.error_code or "PLC_TIMEOUT"
            response.message = str(error)
            detail = str(error)
            self.get_logger().error(
                f"Mission {request.mission_id}: PLC boundary failed: {error}"
            )
        if not response.accepted and outcome != "NOT_REQUESTED":
            response.error_code += "_TRANSFER_" + outcome
            detail = json.dumps(
                dict(
                    station_id=request.station_id,
                    transfer_kind="LOADING" if pickup else "UNLOADING",
                    cycle_counter=result.cycle_counter if result is not None else None,
                    transfer_outcome=outcome,
                    error_code=response.error_code,
                    message=response.message,
                )
            )
        self._event(
            phase + "_finished", "SUCCEEDED" if response.accepted else "FAILED", detail
        )
        self.get_logger().info(
            f"mission_id={request.mission_id} robot_id={request.robot_id} station={request.station_id} state=TRANSFER_FINISHED error_code={response.error_code}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    # The service owns its group. The default group remains free for /clock.
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = ModbusGatewayNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError as error:
        # Humble can race signal shutdown while creating its next wait set.
        if rclpy.ok() or not any(
            message in str(error)
            for message in ("context is invalid", "context is not valid")
        ):
            raise
    finally:
        # Humble's base shutdown does not join queued pool callbacks. Keep
        # node entities alive until our owned pool has drained.
        try:
            executor._executor.shutdown(wait=True)
            for future in executor._futures:
                if future.done() and not future.cancelled():
                    future.result()
        finally:
            executor.shutdown()
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
