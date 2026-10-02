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


def attach_controls(node, *, owner, callback_group=None, on_reset=None):
    """Keep configuration callbacks serialized with the gateway protocol boundary."""
    def event(name, request):
        node.events.publish(ProtocolEvent(stamp=node.get_clock().now().to_msg(), mission_id=request.mission_id,
            protocol='FAULT', direction='INTERNAL', event=name, outcome='SUCCEEDED', detail=json.dumps(asdict(request))))

    controller = FaultController(on_event=event)
    acknowledgements = node.create_publisher(FaultCommand, '/factory/faults/acknowledgements', 100)
    def command(message):
        if message.owner and message.owner != owner:
            return
        if message.reset:
            controller.reset()
            if on_reset is not None:
                on_reset()
        else:
            try:
                controller.enable(FaultRequest(message.name, message.mission_id, message.station or None,
                    message.activation_point, message.duration, message.one_shot, message.fault_code))
            except (TypeError, ValueError) as error:
                node.get_logger().error(f'Invalid fault command: {error}')
                return
        acknowledgements.publish(FaultCommand(command_id=message.command_id, acknowledged=True, owner=owner))
    subscription = node.create_subscription(FaultCommand, '/factory/faults/commands', command, 100,
                                            callback_group=callback_group)
    return controller, subscription


class FaultInjectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('fault_injector', **kwargs)
        self.commands = self.create_publisher(FaultCommand, '/factory/faults/commands', 100)
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
            owner = 'mqtt_gateway' if control.name.startswith('mqtt_') else 'modbus_gateway'
            command = FaultCommand(**{**asdict(control), 'station': control.station or '', 'owner': owner})
            response.accepted, response.message = await self._deliver(command, (owner,))
        except (TypeError, ValueError, RecursionError) as error:
            response.accepted, response.message = False, str(error)
        return response

    async def _reset(self, request, response):
        response.success, response.message = await self._deliver(FaultCommand(reset=True),
                                                                 ('mqtt_gateway', 'modbus_gateway'))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
