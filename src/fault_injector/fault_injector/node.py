"""JSON CLI services and typed, explicitly enabled ROS fault distribution."""
from dataclasses import asdict
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.task import Future
from factory_interfaces.msg import FaultCommand, ProtocolEvent
from factory_interfaces.srv import SetFault
from std_srvs.srv import Trigger

from .controller import FaultController
from .models import FaultRequest


def attach_controls(node, *, owner, callback_group=None, on_reset=None, on_enable=None):
    """Keep configuration callbacks serialized with the gateway protocol boundary."""
    def event(name, request):
        node.events.publish(ProtocolEvent(stamp=node.get_clock().now().to_msg(), mission_id=request.mission_id,
            protocol='FAULT', direction='INTERNAL', event=name, outcome='SUCCEEDED', detail=json.dumps(asdict(request))))

    controller = FaultController(on_event=event)
    acknowledgements = node.create_publisher(FaultCommand, '/factory/faults/acknowledgements', 100)
    pending_controls = {}
    def acknowledge(identifier):
        acknowledgements.publish(FaultCommand(command_id=identifier, acknowledged=True, owner=owner))
    def ready_controls():
        now = time.monotonic()
        for identifier, (ready, deadline) in tuple(pending_controls.items()):
            if now >= deadline:
                del pending_controls[identifier]
            elif ready():
                del pending_controls[identifier]
                acknowledge(identifier)
        if not pending_controls:
            readiness_timer.cancel()
    readiness_timer = node.create_timer(0.02, ready_controls, clock=Clock(), callback_group=callback_group)
    readiness_timer.cancel()
    def command(message):
        if message.owner and message.owner != owner:
            return
        ready = None
        if message.reset:
            pending_controls.clear()
            readiness_timer.cancel()
            controller.reset()
            if on_reset is not None:
                ready = on_reset()
        else:
            try:
                control = FaultRequest(message.name, message.mission_id, message.station or None,
                    message.activation_point, message.duration, message.one_shot, message.fault_code)
                controller.enable(control)
                if on_enable is not None:
                    ready = on_enable(control)
            except (TypeError, ValueError) as error:
                node.get_logger().error(f'Invalid fault command: {error}')
                return
        if ready is not None and not ready():
            timeout = message.ack_timeout_sec
            if not math.isfinite(timeout) or timeout <= 0:
                return
            pending_controls[message.command_id] = (ready, time.monotonic() + timeout)
            readiness_timer.reset()
            return
        acknowledge(message.command_id)
    subscription = node.create_subscription(FaultCommand, '/factory/faults/commands', command, 100,
                                            callback_group=callback_group)
    return controller, subscription


class FaultInjectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('fault_injector', **kwargs)
        self.commands = self.create_publisher(FaultCommand, '/factory/faults/commands', 100)
        self.owners = self.declare_parameter('fault_owners', ['mqtt_gateway', 'modbus_gateway']).value
        allowed = {'mqtt_gateway', 'modbus_gateway', 'mission_coordinator', 'simulation', 'qos_experiment'}
        if not self.owners or len(set(self.owners)) != len(self.owners) or not set(self.owners) <= allowed:
            raise ValueError('fault_owners must contain unique supported owners')
        self.events = self.create_publisher(ProtocolEvent, '/factory/protocol_events', 100)
        self.ack_timeout = self.declare_parameter('ack_timeout_sec', 1.0).value
        if type(self.ack_timeout) not in (float, int) or not math.isfinite(self.ack_timeout) or self.ack_timeout <= 0:
            raise ValueError('ack_timeout_sec must be finite and positive')
        self._lock = threading.RLock()
        self._command_id = time.time_ns()
        self._pending = {}
        # Services can await acknowledgement even on the single-thread executor.
        # Subscription and wall-clock expiry use a separate callback group.
        self.ack_group = MutuallyExclusiveCallbackGroup()
        self.acks = self.create_subscription(FaultCommand, '/factory/faults/acknowledgements', self._ack, 100,
                                             callback_group=self.ack_group)
        self.ack_timer = self.create_timer(0.02, self._expire, clock=Clock(), callback_group=self.ack_group)
        self.set_service = self.create_service(SetFault, '/factory/faults/set', self._set)
        self.reset_service = self.create_service(Trigger, '/factory/faults/reset', self._reset)
        self.simulation = SimulationFaults(self) if 'simulation' in self.owners else None
        self.qos_experiment = QosExperiment(self) if 'qos_experiment' in self.owners else None

    def _ack(self, message):
        with self._lock:
            pending = self._pending.get(message.command_id)
            if pending is None or not message.acknowledged or message.owner not in pending[1]:
                return
            future, owners, deadline = pending
            # A late acknowledgement must never turn a timed-out request green.
            if time.monotonic() >= deadline:
                return
            owners.remove(message.owner)
            if not owners:
                del self._pending[message.command_id]
                future.set_result((True, 'Gateway fault state acknowledged'))

    def _expire(self):
        with self._lock:
            for identifier, (future, owners, deadline) in tuple(self._pending.items()):
                if time.monotonic() >= deadline:
                    del self._pending[identifier]
                    future.set_result((False, 'Fault acknowledgement timed out: ' + ', '.join(sorted(owners))))

    async def _deliver(self, command, owners):
        future = Future()
        with self._lock:
            self._command_id += 1
            command.command_id = self._command_id
            command.ack_timeout_sec = float(self.ack_timeout)
            self._pending[command.command_id] = (future, set(owners), time.monotonic() + self.ack_timeout)
        self.commands.publish(command)
        return await future

    async def _set(self, request, response):
        try:
            if len(request.json) > 4096:
                raise ValueError('fault request exceeds 4096 characters')
            fields = json.loads(request.json)
            if not isinstance(fields, dict):
                raise ValueError('fault request must be an object')
            control = FaultRequest(**fields)
            owner = ('mqtt_gateway' if control.name.startswith('mqtt_') else
                     'mission_coordinator' if control.name.startswith('nav_reject_') else
                     'qos_experiment' if control.name == 'qos_mismatch' else
                     'simulation' if control.name == 'wrong_marker' else 'modbus_gateway')
            if owner not in self.owners:
                raise ValueError('fault owner not enabled: ' + owner)
            command = FaultCommand(**{**asdict(control), 'station': control.station or '', 'owner': owner})
            response.accepted, response.message = await self._deliver(command, (owner,))
        except (TypeError, ValueError, RecursionError) as error:
            response.accepted, response.message = False, str(error)
        return response

    async def _reset(self, request, response):
        response.success, response.message = await self._deliver(FaultCommand(reset=True),
                                                                 self.owners)
        return response


class SimulationFaults:
    """Move the real rendered board only at a matching navigation boundary."""
    def __init__(self, node):
        from gazebo_msgs.srv import SetEntityState
        self.node, self.service_type = node, SetEntityState
        self.client = node.create_client(SetEntityState, '/gazebo/set_entity_state', callback_group=node.ack_group)
        self.active = None
        self.generation = 0
        self.pending = None
        self.controller, self.commands = attach_controls(node, owner='simulation',
            callback_group=node.ack_group, on_reset=self.reset)
        self.subscription = node.create_subscription(ProtocolEvent, '/factory/protocol_events', self.observe, 100)
        self.timer = node.create_timer(0.02, self.expire, clock=Clock())

    def move(self, x, y, z, done):
        self.generation += 1
        generation = self.generation
        if not self.client.service_is_ready():
            return False
        request = self.service_type.Request()
        request.state.name = 'wrong_marker_station'
        request.state.reference_frame = 'world'
        request.state.pose.position.x, request.state.pose.position.y, request.state.pose.position.z = x, y, z
        request.state.pose.orientation.w = 1.0
        self.pending = self.client.call_async(request)
        def complete(future):
            if generation != self.generation:
                return
            try:
                success = future.result().success
            except Exception as error:
                self.node.get_logger().error(f'Wrong-marker pose update failed: {error}')
                success = False
            done(success)
        self.pending.add_done_callback(complete)
        return True

    def observe(self, event):
        if event.event != 'navigation_pickup_started':
            return
        control = self.controller.consume('wrong_marker', event.mission_id, 'assembly', 'navigation_start')
        if control is not None:
            self.active = (control, time.monotonic() + control.duration)
            def applied(success):
                self.node.events.publish(ProtocolEvent(stamp=self.node.get_clock().now().to_msg(), mission_id=control.mission_id,
                    protocol='SIMULATION', direction='INTERNAL', event='wrong_marker_pose_applied',
                    outcome='SUCCEEDED' if success else 'FAILED', detail=json.dumps(dict(entity='wrong_marker_station',
                    pose=[-3.0, 1.57, 0.35]))))
            self.move(-3.0, 1.57, 0.35, applied)

    def expire(self):
        if self.active is not None and time.monotonic() >= self.active[1]:
            self.reset()

    def reset(self):
        finished = []
        active, self.active = self.active, None
        def done(success):
            if success:
                if active is not None:
                    self.controller.finish(active[0])
                finished.append(True)
        self.move(0.0, 0.0, -10.0, done)
        return lambda: bool(finished)


class QosExperiment:
    """Measure DDS reliability incompatibility without touching production topics."""
    def __init__(self, node):
        self.node = node
        self.publisher = self.subscriber = None
        self.offered, self.requested, self.samples = [], [], []
        self.active = None
        self.generation = 0
        self.topic = ''
        self.timer = node.create_timer(0.05, self.tick, clock=Clock())
        self.timer.cancel()
        self.controller, self.commands = attach_controls(node, owner='qos_experiment',
            callback_group=node.ack_group, on_enable=self.enable, on_reset=self.reset)

    def profile(self, reliable):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        return QoSProfile(depth=10, reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
                          durability=DurabilityPolicy.VOLATILE)

    def event(self, name, outcome='SUCCEEDED', **extra):
        offered = dict(reliability='BEST_EFFORT', durability='VOLATILE')
        requested = dict(reliability='RELIABLE' if self.mismatch else 'BEST_EFFORT', durability='VOLATILE')
        detail = dict(topic=self.topic, offered=offered, requested=requested, **extra)
        self.node.events.publish(ProtocolEvent(stamp=self.node.get_clock().now().to_msg(), mission_id=self.active.mission_id,
            protocol='DDS', direction='INTERNAL', event=name, outcome=outcome, detail=json.dumps(detail)))

    def enable(self, control):
        from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
        from std_msgs.msg import String
        self.reset()
        self.active = self.controller.consume('qos_mismatch', control.mission_id, control.station, control.activation_point)
        self.topic = '/factory/faults/qos_experiment/mission_' + control.mission_id.replace('-', '_')
        self.offered, self.requested, self.samples = [], [], []
        self.mismatch = True
        self.started = time.monotonic()
        self.generation += 1
        generation = self.generation
        def incompatible(events, name, status):
            if generation == self.generation and self.active is not None:
                events.append(status)
                self.event(name, 'INCOMPATIBLE', last_policy_kind=int(status.last_policy_kind), total_count=status.total_count)
        self.publisher = self.node.create_publisher(String, self.topic, self.profile(False),
            event_callbacks=PublisherEventCallbacks(incompatible_qos=lambda status: incompatible(self.offered, 'qos_incompatible_offered', status)))
        self.subscriber = self.node.create_subscription(String, self.topic, self.receive, self.profile(True),
            event_callbacks=SubscriptionEventCallbacks(incompatible_qos=lambda status: incompatible(self.requested, 'qos_incompatible_requested', status)))
        self.event('qos_mismatch_started')
        self.timer.reset()

    def receive(self, message):
        if self.active is None:
            return
        self.samples.append(message.data)
        if not self.mismatch and not self.recovered:
            self.recovered = True
            self.event('qos_recovery_finished', sample_count=len(self.samples), match_count=self.publisher.get_subscription_count())

    def tick(self):
        from std_msgs.msg import String
        if self.active is None:
            return
        elapsed = time.monotonic() - self.started
        if self.mismatch and elapsed >= self.active.duration:
            self.event('qos_mismatch_finished', 'INCOMPATIBLE', elapsed_sec=elapsed,
                       sample_count=len(self.samples), match_count=self.publisher.get_subscription_count())
            self.node.destroy_subscription(self.subscriber)
            self.mismatch, self.recovered = False, False
            self.subscriber = self.node.create_subscription(String, self.topic, self.receive, self.profile(False))
        self.publisher.publish(String(data='isolated telemetry sample'))

    def reset(self):
        self.generation += 1
        self.timer.cancel()
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
            self.publisher = None
        if self.subscriber is not None:
            self.node.destroy_subscription(self.subscriber)
            self.subscriber = None
        if self.active is not None:
            self.controller.finish(self.active)
            self.active = None


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
