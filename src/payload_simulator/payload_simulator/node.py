# SPDX-License-Identifier: Apache-2.0
"""Publish logical state immediately and animate Gazebo outside the mission path."""

from collections import deque
import json
import time

from factory_interfaces.msg import ProtocolEvent
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
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


def carried_part_step(part_entity="factory_part", robot_entity="factory_amr"):
    """Keep the payload visible above the robot while it is in transit."""
    return part_entity, pose(-0.03, 0, 0.28), robot_entity


def animation_steps(
    transfer_kind, *, part_entity="factory_part", robot_entity="factory_amr"
):
    """Return bounded model poses; station bodies and markers never move.

    The visible part travels 0.2 m along each belt before storage or placement.
    A kinematic, gravity-disabled link keeps commanded poses stable between steps.
    """
    loading = transfer_kind == "LOADING"
    x = -3.0 if loading else 3.0
    conveyor = "assembly_conveyor" if loading else "inspection_conveyor"
    steps = [(part_entity, pose(x, 2.0 if loading else 1.8, 0.65), "world")]
    for index, offset in enumerate((0.06, 0.12, 0.06, 0.0), start=1):
        steps.append((conveyor, pose(x, 2.0 + offset, 0.603), "world"))
        part_y = 2.0 - index * 0.05 if loading else 1.8 + index * 0.05
        steps.append((part_entity, pose(x, part_y, 0.65), "world"))
    steps.append(
        carried_part_step(part_entity, robot_entity)
        if loading
        else (part_entity, pose(3, 2, 0.65), "world")
    )
    return deque(steps)


class PayloadSimulatorNode(Node):
    """Single-threaded callbacks with asynchronous, wall-time-bounded services."""

    def __init__(self, **kwargs):
        super().__init__("payload_simulator", **kwargs)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        elif not self.get_parameter("use_sim_time").value:
            self.set_parameters([Parameter("use_sim_time", value=True)])
        robot_ids = self.declare_parameter("robot_ids", ["amr_01"]).value
        robot_entities = self.declare_parameter("robot_entities", ["factory_amr"]).value
        part_entities = self.declare_parameter("part_entities", ["factory_part"]).value
        if (
            not robot_ids
            or len(robot_ids) != len(set(robot_ids))
            or len(part_entities) != len(set(part_entities))
            or not len(robot_ids) == len(robot_entities) == len(part_entities)
            or not all(robot_entities)
            or not all(part_entities)
        ):
            raise ValueError(
                "payload mappings must have unique robot IDs and part entities with matching nonempty robot entities"
            )
        self._entities = dict(zip(robot_ids, zip(part_entities, robot_entities)))
        self._machine = PayloadStateMachine(robot_ids)
        qos = QoSProfile(
            depth=len(robot_ids),
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
        self._active_animation = None
        self._carrying = {}
        self._carry_updates = {}
        self._future = None
        self._visual_deadline = 0.0
        self._service_deadline = 0.0
        self._next_step_sim_ns = 0
        # A steady timer still detects visual timeouts when simulation is paused.
        # Actual motion pacing uses the same simulation clock as protocol events.
        self._timer = self.create_timer(
            0.02, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        for robot_id in robot_ids:
            self._publish_state("", "", None, robot_id)

    def _publish_state(self, mission_id, transfer_kind, cycle_counter, robot_id):
        message = String()
        message.data = json.dumps(
            {
                "state": self._machine.state_for(robot_id, mission_id).value,
                "mission_id": mission_id,
                "robot_id": robot_id,
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
            robot_id=message.robot_id,
        )
        if not transition.applied:
            return
        self._publish_state(
            transition.mission_id,
            transition.transfer_kind,
            transition.cycle_counter,
            transition.robot_id,
        )
        if transition.transfer_kind == "UNLOADING":
            self._carrying.pop(transition.robot_id, None)
            self._animations = deque(
                item
                for item in self._animations
                if item.robot_id != transition.robot_id
            )
            if (
                self._active_animation is not None
                and self._active_animation.robot_id == transition.robot_id
            ):
                self._steps.clear()
                self._active_animation = None
            pending = self._carry_updates.pop(transition.robot_id, None)
            if pending is not None and pending[0] is not None and not pending[0].done():
                self._client.remove_pending_request(pending[0])
                pending[0].cancel()
        self._animations.append(transition)

    def _refresh_carrying(self, wall_now):
        sim_now = self.get_clock().now().nanoseconds
        for robot_id in tuple(self._carrying):
            future, deadline, next_sim = self._carry_updates.get(
                robot_id, (None, 0.0, 0)
            )
            if future is not None:
                try:
                    if future.done():
                        if not future.result().success:
                            raise RuntimeError("Gazebo rejected carried payload pose")
                        self._carry_updates[robot_id] = (
                            None,
                            0.0,
                            sim_now + 150_000_000,
                        )
                    elif wall_now >= deadline:
                        self._client.remove_pending_request(future)
                        future.cancel()
                        raise TimeoutError("1 s carried payload service deadline")
                except Exception as error:
                    self._carrying.pop(robot_id, None)
                    self._carry_updates.pop(robot_id, None)
                    self.get_logger().warning(
                        f"Payload visual update failed for {robot_id}: {error}; logical state remains authoritative"
                    )
                continue
            if sim_now < next_sim or not self._client.service_is_ready():
                continue
            name, target, reference = carried_part_step(*self._entities[robot_id])
            request = SetEntityState.Request()
            request.state.name = name
            request.state.pose = target
            request.state.reference_frame = reference
            try:
                self._carry_updates[robot_id] = (
                    self._client.call_async(request),
                    wall_now + 1.0,
                    next_sim,
                )
            except Exception as error:
                self._carrying.pop(robot_id, None)
                self._carry_updates.pop(robot_id, None)
                self.get_logger().warning(
                    f"Payload visual update failed for {robot_id}: {error}; logical state remains authoritative"
                )

    def _visual_failed(self, reason):
        if self._future is not None:
            self._client.remove_pending_request(self._future)
            self._future.cancel()
        self._future = None
        self._steps.clear()
        if self._active_animation is not None:
            self._carrying.pop(self._active_animation.robot_id, None)
        self._active_animation = None
        self.get_logger().warning(
            f"Payload visual update failed: {reason}; logical state remains authoritative"
        )

    def _tick(self):
        wall_now = time.monotonic()
        self._refresh_carrying(wall_now)
        if not self._steps and self._future is None:
            if self._animations:
                self._active_animation = self._animations.popleft()
                part, robot = self._entities[self._active_animation.robot_id]
                self._steps = animation_steps(
                    self._active_animation.transfer_kind,
                    part_entity=part,
                    robot_entity=robot,
                )
            else:
                return
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
                if not self._steps and self._active_animation is not None:
                    transition = self._active_animation
                    if (
                        transition.transfer_kind == "LOADING"
                        and self._machine.state_for(
                            transition.robot_id, transition.mission_id
                        ).value
                        == "IN_TRANSIT"
                    ):
                        self._carrying[transition.robot_id] = transition.mission_id
                    self._active_animation = None
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
        name, target, reference_frame = self._steps.popleft()
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose = target
        request.state.reference_frame = reference_frame
        self._future = self._client.call_async(request)
        self._service_deadline = wall_now + 1.0


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = PayloadSimulatorNode()
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
