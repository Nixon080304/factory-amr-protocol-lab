# SPDX-License-Identifier: Apache-2.0
"""Simulation-only dock proximity/contact sensor using Gazebo world evidence.

This intentionally models a virtual charging contact at configured geometry,
not a hardware electrical sensor. It has no dependency on Nav2 or docking state.
Missing, invalid, delayed or out-of-position simulator evidence fails closed.
"""

import math
import time

from gazebo_msgs.srv import GetEntityState
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool


class DockContactRuntime:
    def __init__(
        self,
        node,
        client,
        entity_name,
        charging_pose,
        *,
        clock=time.monotonic,
        stale_sec=0.5,
    ):
        if (
            not entity_name
            or len(charging_pose) != 3
            or not all(math.isfinite(value) for value in charging_pose)
        ):
            raise ValueError(
                "contact sensor requires entity identity and finite dock pose"
            )
        if not math.isfinite(stale_sec) or stale_sec <= 0:
            raise ValueError("contact evidence deadline must be positive")
        self.client, self.entity, self.target, self.clock = (
            client,
            entity_name,
            tuple(charging_pose),
            clock,
        )
        self.stale_sec, self._pending, self._receipt, self._confirmed = (
            stale_sec,
            None,
            None,
            False,
        )
        self.publisher = node.create_publisher(Bool, "factory/dock_contact", 10)
        self.timer = node.create_timer(
            0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.publish()

    def publish(self):
        valid = (
            self._receipt is not None and self.clock() < self._receipt + self.stale_sec
        )
        self.publisher.publish(Bool(data=bool(valid and self._confirmed)))

    def _at_contact(self, response):
        if not response.success or self._pending is None:
            return False
        request = self._pending[3]
        # gazebo_ros_state3.9 leaves state.name/reference_frame empty. Those
        # upstream defaults are permitted only for this current exact query;
        # a missing field or any conflicting nonempty identity fails closed.
        state = response.state
        if (
            request.name != self.entity
            or request.reference_frame != "world"
            or getattr(getattr(response, "header", None), "frame_id", None) != "world"
            or getattr(state, "name", None) not in ("", self.entity)
            or getattr(state, "reference_frame", None) not in ("", "world")
        ):
            return False
        p, q = response.state.pose.position, response.state.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return False
        if abs(sum(value * value for value in (q.x, q.y, q.z, q.w)) - 1.0) > 0.01:
            return False
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (
            math.hypot(p.x - self.target[0], p.y - self.target[1]) <= 0.15
            and abs(p.z) <= 0.15
            and abs(math.remainder(yaw - self.target[2], 2 * math.pi)) <= 0.2
        )

    def tick(self):
        now = self.clock()
        if not self.client.service_is_ready():
            self._forget_pending()
            self._pending, self._receipt, self._confirmed = None, None, False
            self.publish()
            return
        if self._pending is not None and now >= self._pending[1] + self.stale_sec:
            self._forget_pending()
            self._pending, self._receipt, self._confirmed = None, None, False
        self.publish()
        if self._pending is not None:
            return
        token = object()
        request = GetEntityState.Request(name=self.entity, reference_frame="world")
        self._pending = (token, now, None, request)

        def completed(future):
            if (
                self._pending is None
                or self._pending[0] is not token
                or self._pending[2] is not future
            ):
                return
            self._receipt = now
            try:
                self._confirmed = (
                    self.clock() < now + self.stale_sec
                    and self._at_contact(future.result())
                )
            except Exception:
                self._confirmed = False
            self._pending = None
            self.publish()

        try:
            future = self.client.call_async(request)
            self._pending = (token, now, future, request)
            future.add_done_callback(completed)
        except Exception:
            self._pending, self._receipt, self._confirmed = None, None, False
            self.publish()

    def _forget_pending(self):
        pending, self._pending = self._pending, None
        if pending is not None and pending[2] is not None:
            remove = getattr(self.client, "remove_pending_request", None)
            if remove is not None:
                try:
                    remove(pending[2])
                except Exception:
                    pass  # Unavailable transport still cannot confirm contact.

    def shutdown(self):
        self._forget_pending()
        self._pending, self._receipt, self._confirmed = None, None, False
        self.publish()


class DockContactNode(Node):
    def __init__(self, **kwargs):
        super().__init__("simulated_dock_contact", **kwargs)
        entity = self.declare_parameter("entity_name", "").value
        dock_id = self.declare_parameter("dock_id", "").value
        pose = self.declare_parameter("charging_pose", [0.0, 0.0, 0.0]).value
        if not dock_id:
            raise ValueError("simulation contact requires configured dock_id")
        self.client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.runtime = DockContactRuntime(self, self.client, entity, pose)

    def destroy_node(self):
        self.runtime.shutdown()
        return super().destroy_node()


def main():
    rclpy.init()
    node = DockContactNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
