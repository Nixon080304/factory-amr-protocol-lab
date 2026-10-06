"""Serialized ROS/MQTT adapter with bounded reconnect state replay."""

from datetime import datetime, timezone
from collections import deque
import json
import math
import queue
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from factory_interfaces.action import ExecuteFleetMission
from factory_interfaces.msg import ProtocolEvent, RobotState
from fleet_manager.adapter import robot_endpoints
from fleet_manager.config import load_fleet_config
from .client import MqttClient, FAULT_REQUEST_TOPIC, FLEET_AVAILABILITY_TOPIC
from .validator import MissionValidator, MissionValidationError
from .mission_registry import MissionRegistry, MissionConflictError
from .reconnect_queue import ReconnectQueue
from fault_injector.node import attach_controls
from fault_injector.models import FaultRequest


@dataclass(frozen=True)
class PendingInjection:
    payload: bytes
    fault: FaultRequest
    deadline: float


class MqttGatewayNode(Node):
    def __init__(self, *, mqtt_client=None, **kwargs):
        super().__init__("mqtt_gateway", **kwargs)
        self.set_parameters([Parameter("use_sim_time", value=True)])
        host = self.declare_parameter("broker_host", "127.0.0.1").value
        port = self.declare_parameter("broker_port", 1883).value
        fleet_file = self.declare_parameter(
            "fleet_file",
            str(
                Path(get_package_share_directory("factory_bringup"))
                / "config/fleet.yaml"
            ),
        ).value
        fleet = load_fleet_config(fleet_file)
        self.robot_ids = {robot.robot_id for robot in fleet.robots}
        self.protocol_group = MutuallyExclusiveCallbackGroup()
        self.action = ActionClient(
            self,
            ExecuteFleetMission,
            "/factory/execute_fleet_mission",
            callback_group=self.protocol_group,
        )
        self.events = self.create_publisher(
            ProtocolEvent,
            "/factory/protocol_events",
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.robot_subscriptions = [
            self.create_subscription(
                RobotState,
                robot_endpoints(robot)["state"],
                lambda message, robot_id=robot.robot_id: self._robot_state(
                    robot_id, message
                ),
                qos_profile_sensor_data,
                callback_group=self.protocol_group,
            )
            for robot in fleet.robots
        ]
        self.robot_states = {}
        self.telemetry_reconnect = {}
        self.assigned_robots, self.missions = {}, {}
        self.current_mission_id, self.current_state = "", "IDLE"
        self.validator, self.registry, self.reconnect = (
            MissionValidator(),
            MissionRegistry(),
            ReconnectQueue(),
        )
        self.incoming = queue.Queue(maxsize=100)
        self.completions = queue.SimpleQueue()
        self.connected = False
        self.sequence = {}
        self.pending = {}
        self.awaiting_acceptance = {}
        self.client = mqtt_client or MqttClient(host, port)
        self._fault_disconnected = False
        self._disconnect_fault = None
        self._reconnect_at = 0.0
        self._fault_request_transport = False
        self._fault_subscription_ready = False
        self._fault_generation = 0
        self._pending_injections = deque()
        self.faults, self.fault_subscription = attach_controls(
            self,
            owner="mqtt_gateway",
            callback_group=self.protocol_group,
            on_reset=self._reset_controls,
            on_enable=self._enable_request_fault,
        )
        self.client.set_handlers(
            lambda raw: self._enqueue("request", raw[:4097]),
            lambda connected: self._enqueue("connection", connected),
        )
        self.client.set_fault_handlers(
            lambda raw: self._enqueue("injected_request", raw[:4097]),
            lambda ready: self._enqueue("fault_subscription", ready),
        )
        self.timer = self.create_timer(
            0.02,
            self._drain,
            callback_group=self.protocol_group,
            clock=rclpy.clock.Clock(),
        )
        self.telemetry_timer = self.create_timer(
            0.5,
            self._sample_telemetry,
            callback_group=self.protocol_group,
            clock=rclpy.clock.Clock(),
        )
        self.client.start()

    def _enqueue(self, kind, value):
        # Paho never touches registries, action clients, or ROS-owned state.
        try:
            self.incoming.put_nowait((kind, value))
        except queue.Full:
            self.get_logger().error("MQTT input queue full; request dropped")

    def _timestamp(self):
        return (
            datetime.fromtimestamp(
                self.get_clock().now().nanoseconds / 1e9, timezone.utc
            )
            .isoformat(timespec="milliseconds")
            .replace("+00:00", "Z")
        )

    def _event(self, mission_id, name, outcome="", detail=""):
        mission = self.registry.payload_for(mission_id)
        robot_id = self.assigned_robots.get(
            mission_id, mission.robot_id if mission is not None else ""
        )
        self.events.publish(
            ProtocolEvent(
                stamp=self.get_clock().now().to_msg(),
                mission_id=mission_id,
                robot_id=robot_id or "",
                protocol="MQTT",
                direction="INBOUND",
                event=name,
                outcome=outcome,
                detail=detail,
            )
        )

    def _send_status(self, payload):
        if self.connected:
            try:
                if self.client.publish(
                    f"factory/missions/{payload['mission_id']}/status",
                    json.dumps(payload),
                    qos=1,
                ):
                    return
            except Exception as error:
                self.get_logger().error(
                    f"Mission {payload['mission_id']}: MQTT publish failed: {error}"
                )
            self.connected = False
        self.reconnect.push_state(payload)

    def publish_status(
        self,
        mission_id,
        state,
        station="",
        detail="",
        error_code=None,
        *,
        remember=True,
    ):
        self.get_logger().info(
            f"mission_id={mission_id} robot_id={self.assigned_robots.get(mission_id, '')} station={station} state={state} error_code={error_code}"
        )
        self.sequence[mission_id] = self.sequence.get(mission_id, 0) + 1
        payload = dict(
            mission_id=mission_id,
            state=state,
            station=station,
            timestamp=self._timestamp(),
            sequence=self.sequence[mission_id],
            detail=detail,
            error_code=error_code,
        )
        if mission_id in self.assigned_robots:
            payload["robot_id"] = self.assigned_robots[mission_id]
        if remember:
            try:
                self.registry.update_state(mission_id, payload)
            except KeyError:
                pass
            if mission_id == self.current_mission_id:
                self.current_state = state
        self._send_status(payload)

    def publish_telemetry(self, payload):
        if self.connected:
            try:
                if self.client.publish(
                    f"factory/robots/{payload['robot_id']}/telemetry",
                    json.dumps(payload),
                    qos=0,
                ):
                    return
            except Exception as error:
                self.get_logger().error(f"MQTT telemetry failed: {error}")
            self.connected = False
        self.telemetry_reconnect[payload["robot_id"]] = dict(payload)

    @staticmethod
    def _pose_sample(pose, frame_id):
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
        if not frame_id or not all(math.isfinite(value) for value in values):
            return None
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) > 0.01:
            return None
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return dict(frame_id=frame_id, x=pose.position.x, y=pose.position.y, yaw=yaw)

    def _robot_state(self, robot_id, message):
        if message.robot_id != robot_id:
            return
        pose = self._pose_sample(message.pose, message.frame_id)
        if pose is None or not math.isfinite(message.battery_percent):
            return
        self.robot_states[robot_id] = dict(
            robot_id=robot_id,
            mission_id=message.mission_id,
            state=message.mode,
            battery_percent=message.battery_percent,
            payload_state=message.payload_state,
            **pose,
        )

    def _sample_telemetry(self):
        for payload in self.robot_states.values():
            self.publish_telemetry(dict(payload, timestamp=self._timestamp()))

    def _drain(self):
        if self._fault_request_transport:
            self._maintain_request_faults()
        if self._fault_disconnected and time.monotonic() >= self._reconnect_at:
            self._reset_fault_effects()
        # rclpy future callbacks are executor tasks, outside callback groups.
        # Consume their results here so composition with multiple threads is safe.
        for _ in range(100):
            try:
                kind, mission, future = self.completions.get_nowait()
            except queue.Empty:
                break
            if kind == "feedback":
                self._feedback(mission, future)
            elif kind == "accepted":
                self._accepted(mission, future)
            else:
                self._result(mission, future)
        for _ in range(100):
            try:
                kind, value = self.incoming.get_nowait()
            except queue.Empty:
                break
            if kind == "connection":
                self.connected = value
                if value:
                    try:
                        self.client.publish(
                            FLEET_AVAILABILITY_TOPIC,
                            "online",
                            qos=1,
                            retain=True,
                        )
                    except Exception as error:
                        self.get_logger().error(
                            f"MQTT connection publication failed: {error}"
                        )
                        self.connected = False
                    for payload in self.reconnect.drain_states():
                        self._send_status(payload)
                    telemetry_samples = list(self.telemetry_reconnect.values())
                    self.telemetry_reconnect.clear()
                    for telemetry in telemetry_samples:
                        self.publish_telemetry(telemetry)
            elif kind == "fault_subscription":
                generation, ready = value
                if (
                    self._fault_request_transport
                    and generation == self._fault_generation
                ):
                    self._fault_subscription_ready = ready
            elif kind == "injected_request":
                self._request(value, generated=True)
            else:
                self._request(value)
        # Discovery is asynchronous; do not block ROS while Nav2/bringup starts.
        for mission_id, mission in list(self.pending.items()):
            if self.action.server_is_ready():
                del self.pending[mission_id]
                self._dispatch(mission)

    def _request(self, raw, *, generated=False):
        if generated:
            self._maintain_request_faults()
            delivery = next(
                (entry for entry in self._pending_injections if entry.payload == raw),
                None,
            )
            if not self._fault_request_transport or delivery is None:
                try:
                    mission_id = self.validator.parse_structure(raw).mission_id
                except MissionValidationError:
                    mission_id = "invalid"
                self._event(
                    mission_id,
                    "mqtt_fault_packet_rejected",
                    "FAILED",
                    "No active injected delivery",
                )
                return
            self._pending_injections.remove(delivery)
            self.faults.finish(delivery.fault)
        mission_id = "invalid"
        try:
            mission = self.validator.parse_structure(raw)
            mission_id = mission.mission_id
            if self.registry.payload_for(mission_id) is None:
                self.validator.validate_configuration(mission)
                if (
                    mission.robot_id is not None
                    and mission.robot_id not in self.robot_ids
                ):
                    raise MissionValidationError("robot_id is not configured")
        except MissionValidationError as error:
            self._event(mission_id, "mqtt_acceptance_started")
            self._event(
                mission_id, "mqtt_acceptance_finished", "FAILED", error.error_code
            )
            self.publish_status(
                mission_id,
                "FAILED",
                detail=str(error),
                error_code=error.error_code,
                remember=False,
            )
            return
        try:
            disposition = self.registry.register(mission)
        except MissionConflictError as error:
            self._event(mission_id, "mqtt_rejected", "FAILED", error.error_code)
            self.publish_status(
                mission_id,
                "FAILED",
                detail=str(error),
                error_code=error.error_code,
                remember=False,
            )
            return
        if disposition == "duplicate":
            state = self.registry.state_for(mission_id)
            if state:
                self._send_status(state)
            self._event(mission_id, "mqtt_duplicate", "SUCCEEDED", "Duplicate mission")
            if not generated:
                self._inject_request_faults(mission)
            return
        self._event(mission_id, "mqtt_acceptance_started")
        self.missions[mission_id] = mission
        self.pending[mission_id] = mission
        if not generated:
            self._inject_request_faults(mission)

    def _inject_request_faults(self, mission):
        if not self._fault_request_transport or not self._fault_subscription_ready:
            return
        for name in ("mqtt_duplicate", "mqtt_conflict"):
            if len(self._pending_injections) >= 100:
                self._event(
                    mission.mission_id,
                    "mqtt_fault_delivery_rejected",
                    "FAILED",
                    "Pending injection limit reached",
                )
                return
            fault = self.faults.consume(
                name,
                mission.mission_id,
                mission.pickup,
                "request",
                robot_id=self.assigned_robots.get(mission.mission_id, mission.robot_id),
            )
            if fault is not None:
                duplicate = asdict(mission)
                if name == "mqtt_conflict":
                    duplicate["part"] = "gear"
                payload = json.dumps(duplicate)
                delivery = PendingInjection(
                    payload.encode(), fault, time.monotonic() + fault.duration
                )
                self._pending_injections.append(delivery)
                if not self.client.publish(FAULT_REQUEST_TOPIC, payload, qos=1):
                    self._pending_injections.remove(delivery)
                    self.faults.finish(fault)
                    self._event(
                        mission.mission_id,
                        "mqtt_fault_delivery_rejected",
                        "FAILED",
                        "MQTT publish failed",
                    )

    def _enable_request_fault(self, request):
        if request.name in ("mqtt_duplicate", "mqtt_conflict"):
            if not self._fault_request_transport:
                self._fault_generation += 1
                self._fault_subscription_ready = False
            self._fault_request_transport = True
            self.client.enable_fault_requests(self._fault_generation)
            generation = self._fault_generation
            return lambda: (
                self._fault_request_transport
                and self._fault_generation == generation
                and self.connected
                and self._fault_subscription_ready
                and request in self.faults.query()
            )

    def _disable_request_faults(self):
        for delivery in self._pending_injections:
            self.faults.finish(delivery.fault)
        self._pending_injections.clear()
        self._fault_request_transport = self._fault_subscription_ready = False
        self.client.disable_fault_requests()

    def _maintain_request_faults(self):
        active = self.faults.query()
        now = time.monotonic()
        for delivery in tuple(self._pending_injections):
            if now >= delivery.deadline or (
                not delivery.fault.one_shot and delivery.fault not in active
            ):
                self._pending_injections.remove(delivery)
                self.faults.finish(delivery.fault)
        if not self._pending_injections and not any(
            r.name in ("mqtt_duplicate", "mqtt_conflict") for r in active
        ):
            self._disable_request_faults()

    def _reset_controls(self):
        self._disable_request_faults()
        return self._reset_fault_effects()

    def _reset_fault_effects(self):
        if self._fault_disconnected:
            self._fault_disconnected = False
            self.client.resume()
            self.faults.finish(self._disconnect_fault)
            self._disconnect_fault = None
        # attach_controls polls this on the serialized protocol group. A SUBACK
        # callback reaches connected only after the ROS timer consumes it.
        return lambda: self.connected

    def _dispatch(self, mission):
        request = ExecuteFleetMission.Goal(
            mission_id=mission.mission_id,
            requested_robot_id=mission.robot_id or "",
            pickup_station=mission.pickup,
            dropoff_station=mission.dropoff,
            part=mission.part,
        )
        try:
            self.awaiting_acceptance[mission.mission_id] = deque(maxlen=100)
            future = self.action.send_goal_async(
                request,
                feedback_callback=lambda feedback: self.completions.put(
                    ("feedback", mission.mission_id, feedback.feedback)
                ),
            )
            future.add_done_callback(
                lambda result: self.completions.put(("accepted", mission, result))
            )
        except Exception as error:
            self._acceptance_failed(mission.mission_id, error)

    def _acceptance_failed(self, mission_id, error):
        self.awaiting_acceptance.pop(mission_id, None)
        self.get_logger().error(
            f"Mission {mission_id}: ROS action boundary failed: {error}"
        )
        self._event(
            mission_id, "mqtt_acceptance_finished", "FAILED", "MISSION_TRANSPORT_ERROR"
        )
        self.publish_status(
            mission_id,
            "FAILED",
            detail=str(error),
            error_code="MISSION_TRANSPORT_ERROR",
        )

    def _feedback(self, mission_id, feedback):
        # The action result owns terminal status and its final error code.
        # Terminal feedback can precede that authoritative result on DDS.
        if feedback.state in ("COMPLETED", "FAILED", "CANCELLED", "RECOVERY_REQUIRED"):
            return
        buffered = self.awaiting_acceptance.get(mission_id)
        if buffered is not None:
            # Keep observed feedback ordered after confirmed acceptance; bound memory.
            buffered.append(feedback)
            return
        if feedback.assigned_robot_id:
            self.assigned_robots[mission_id] = feedback.assigned_robot_id
        mission = self.missions.get(mission_id)
        station = ""
        if mission is not None:
            station = (
                mission.dropoff
                if feedback.state
                in ("NAVIGATING_TO_DROPOFF", "VERIFYING_DROPOFF", "UNLOADING")
                else mission.pickup
            )
        self.publish_status(mission_id, feedback.state, station, feedback.detail)

    def _accepted(self, mission, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.awaiting_acceptance.pop(mission.mission_id, None)
                self._event(
                    mission.mission_id,
                    "mqtt_acceptance_finished",
                    "FAILED",
                    "ROBOT_BUSY",
                )
                self.publish_status(
                    mission.mission_id,
                    "FAILED",
                    detail="Fleet rejected mission",
                    error_code="ROBOT_BUSY",
                )
                return
        except Exception as error:
            self._acceptance_failed(mission.mission_id, error)
            return
        self._event(mission.mission_id, "mqtt_acceptance_finished", "SUCCEEDED")
        self.current_mission_id = mission.mission_id
        fault = self.faults.consume(
            "mqtt_disconnect",
            mission.mission_id,
            mission.pickup,
            "accepted",
            robot_id=self.assigned_robots.get(mission.mission_id, mission.robot_id),
        )
        if fault is not None:
            self.client.pause()
            self.connected = False
            self._fault_disconnected = True
            self._disconnect_fault = fault
            self._reconnect_at = time.monotonic() + fault.duration
        self.publish_status(
            mission.mission_id, "RECEIVED", mission.pickup, "Mission accepted"
        )
        for feedback in self.awaiting_acceptance.pop(mission.mission_id, ()):
            self._feedback(mission.mission_id, feedback)
        try:
            handle.get_result_async().add_done_callback(
                lambda result: self.completions.put(("result", mission, result))
            )
        except Exception as error:
            self._result_failed(mission.mission_id, error)

    def _result_failed(self, mission_id, error):
        self.get_logger().error(f"Mission {mission_id}: ROS result failed: {error}")
        self.publish_status(
            mission_id,
            "FAILED",
            detail=str(error),
            error_code="MISSION_TRANSPORT_ERROR",
        )

    def _result(self, mission, future):
        try:
            result = future.result().result
            if result.assigned_robot_id:
                self.assigned_robots[mission.mission_id] = result.assigned_robot_id
            self.publish_status(
                mission.mission_id,
                result.final_state,
                mission.dropoff if result.success else "",
                result.message,
                result.error_code or None,
            )
        except Exception as error:
            self._result_failed(mission.mission_id, error)

    def destroy_node(self):
        try:
            self.client.close()
        except Exception as error:
            self.get_logger().error(f"MQTT shutdown failed: {error}")
        self.action.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = MqttGatewayNode()
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
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
