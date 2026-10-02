# SPDX-License-Identifier: Apache-2.0
"""Publish logical state immediately and animate Gazebo outside the mission path."""

from collections import deque
import json
import time

from factory_interfaces.msg import ProtocolEvent
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from payload_simulator.state_machine import PayloadStateMachine


def pose(x, y, z):
    result = Pose()
    result.position.x, result.position.y, result.position.z = (
        float(x),
        float(y),
        float(z),
    )
    result.orientation.w = 1.0
    return result


def animation_steps(transfer_kind):
    """Return bounded model poses; station bodies and markers never move.

    The part is placed on inspection before the unload conveyor starts. A static
    world include prevents gravity from changing its below-world storage pose.
    """
    loading = transfer_kind == "LOADING"
    x = -3.0 if loading else 3.0
    conveyor = "assembly_conveyor" if loading else "inspection_conveyor"
    steps = [("factory_part", pose(x, 2.0 if loading else 1.8, 0.65))]
    for offset in (0.06, 0.12, 0.06, 0.0):
        steps.append((conveyor, pose(x, 2.0 + offset, 0.603)))
    steps.append(("factory_part", pose(0, 0, -2) if loading else pose(3, 2, 0.65)))
    return deque(steps)


class PayloadSimulatorNode(Node):
    """Single-threaded callbacks with asynchronous, wall-time-bounded services."""

    def __init__(self, **kwargs):
        super().__init__("payload_simulator", **kwargs)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        elif not self.get_parameter("use_sim_time").value:
            self.set_parameters([Parameter("use_sim_time", value=True)])
        self._machine = PayloadStateMachine()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(String, "/factory/payload_state", qos)
        self._subscription = self.create_subscription(
            ProtocolEvent,
            "/factory/protocol_events",
            self._on_event,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self._animations = deque()
        self._steps = deque()
        self._future = None
        self._visual_deadline = 0.0
        self._service_deadline = 0.0
        self._next_step_sim_ns = 0
        # A steady timer still detects visual timeouts when simulation is paused.
        # Actual motion pacing uses the same simulation clock as protocol events.
        self._timer = self.create_timer(
            0.02, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self._publish_state("", "", None)

    def _publish_state(self, mission_id, transfer_kind, cycle_counter):
        message = String()
        message.data = json.dumps(
            {
                "state": self._machine.state.value,
                "mission_id": mission_id,
                "transfer_kind": transfer_kind,
                "cycle_counter": cycle_counter,
            }
        )
        self._publisher.publish(message)

    def _on_event(self, message):
        transition = self._machine.apply_protocol_event(
            message.mission_id,
            message.protocol,
            message.event,
            message.outcome,
            message.detail,
        )
        if not transition.applied:
            return
        self._publish_state(
            transition.mission_id, transition.transfer_kind, transition.cycle_counter
        )
        self._animations.append(transition.transfer_kind)

    def _visual_failed(self, reason):
        if self._future is not None:
            self._client.remove_pending_request(self._future)
            self._future.cancel()
        self._future = None
        self._steps.clear()
        self.get_logger().warning(
            f"Payload visual update failed: {reason}; logical state remains authoritative"
        )

    def _tick(self):
        wall_now = time.monotonic()
        if not self._steps and self._future is None:
            if not self._animations:
                return
            self._steps = animation_steps(self._animations.popleft())
            self._visual_deadline = wall_now + 5.0
            self._next_step_sim_ns = self.get_clock().now().nanoseconds
        if wall_now >= self._visual_deadline:
            self._visual_failed("5 s animation deadline")
            return
        if self._future is not None:
            if self._future.done():
                try:
                    success = self._future.result().success
                except Exception as error:
                    self._visual_failed(str(error))
                    return
                self._future = None
                if not success:
                    self._visual_failed("Gazebo rejected entity pose")
                    return
                self._next_step_sim_ns = (
                    self.get_clock().now().nanoseconds + 150_000_000
                )
            elif wall_now >= self._service_deadline:
                self._visual_failed("1 s service deadline")
            return
        if (
            not self._steps
            or self.get_clock().now().nanoseconds < self._next_step_sim_ns
        ):
            return
        if not self._client.service_is_ready():
            return
        name, target = self._steps.popleft()
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose = target
        request.state.reference_frame = "world"
        self._future = self._client.call_async(request)
        self._service_deadline = wall_now + 1.0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PayloadSimulatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
