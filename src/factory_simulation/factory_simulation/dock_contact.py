# SPDX-License-Identifier: Apache-2.0
"""Simulation-only dock proximity/contact sensor using Gazebo world evidence.

This intentionally models a virtual charging contact at configured geometry,
not a hardware electrical sensor. It has no dependency on Nav2 or docking state.
Missing, invalid, delayed or out-of-position simulator evidence fails closed.
"""

import json
import math
import time

from gazebo_msgs.srv import GetEntityState
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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
        self.node = node
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
        self._reason, self._measurements = "service", {}
        self._request_time, self._response_time = None, None
        self._published_confirmed = None
        self.publisher = node.create_publisher(Bool, "factory/dock_contact", 10)
        self.timer = node.create_timer(
            0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.publish()

    def publish(self):
        now = self.clock()
        valid = self._receipt is not None and now < self._receipt + self.stale_sec
        confirmed = bool(valid and self._confirmed)
        self.publisher.publish(Bool(data=confirmed))
        if confirmed != self._published_confirmed:
            self._published_confirmed = confirmed
            reason = "confirmed" if confirmed else self._reason
            if not valid and self._receipt is not None:
                reason = "stale"
            self.node.get_logger().info(
                json.dumps(
                    {
                        "event": "dock_contact_transition",
                        "entity_name": self.entity,
                        "confirmed": confirmed,
                        "reason": reason,
                        "monotonic_sec": now,
                        "measurements": self._measurements,
                        "thresholds": {
                            "xy_m": 0.15,
                            "yaw_rad": 0.2,
                            "z_m": 0.15,
                            "quaternion_norm_error": 0.01,
                            "stale_sec": self.stale_sec,
                        },
                        "request_age_sec": None
                        if self._request_time is None
                        else now - self._request_time,
                        "receipt_age_sec": None
                        if self._response_time is None
                        else now - self._response_time,
                    },
                    allow_nan=False,
                    sort_keys=True,
                )
            )

    def _at_contact(self, response):
        self._reason, self._measurements = "service", {}
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
            self._reason = "identity"
            self._measurements = {
                "header_frame": getattr(
                    getattr(response, "header", None), "frame_id", None
                ),
                "state_name": getattr(state, "name", None),
                "reference_frame": getattr(state, "reference_frame", None),
            }
            return False
        p, q = response.state.pose.position, response.state.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = sum(value * value for value in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        distance = math.hypot(p.x - self.target[0], p.y - self.target[1])
        yaw_error = abs(math.remainder(yaw - self.target[2], 2 * math.pi))
        self._measurements = {
            name: value if math.isfinite(value) else None
            for name, value in {
                "x_m": p.x,
                "y_m": p.y,
                "z_m": abs(p.z),
                "xy_m": distance,
                "yaw": yaw,
                "yaw_rad": yaw_error,
                "quaternion_norm_squared": norm,
            }.items()
        }
        if not all(math.isfinite(value) for value in values):
            self._reason = (
                "xy"
                if not all(math.isfinite(value) for value in (p.x, p.y))
                else "z"
                if not math.isfinite(p.z)
                else "quaternion"
            )
            return False
        if abs(norm - 1.0) > 0.01:
            self._reason = "quaternion"
            return False
        for reason, value, threshold in (
            ("xy", distance, 0.15),
            ("z", abs(p.z), 0.15),
            ("yaw", yaw_error, 0.2),
        ):
            if value > threshold:
                self._reason = reason
                return False
        self._reason = "confirmed"
        return True

    def tick(self):
        now = self.clock()
        if not self.client.service_is_ready():
            self._forget_pending()
            self._pending, self._receipt, self._confirmed = None, None, False
            self._reason = "service"
            self.publish()
            return
        if self._pending is not None and now >= self._pending[1] + self.stale_sec:
            self._forget_pending()
            self._pending, self._receipt, self._confirmed = None, None, False
            self._reason = "stale"
        self.publish()
        if self._pending is not None:
            return
        token = object()
        request = GetEntityState.Request(name=self.entity, reference_frame="world")
        self._pending = (token, now, None, request)
        self._request_time = now

        def completed(future):
            if (
                self._pending is None
                or self._pending[0] is not token
                or self._pending[2] is not future
            ):
                return
            self._receipt = now
            self._response_time = self.clock()
            try:
                if self.clock() >= now + self.stale_sec:
                    self._confirmed, self._reason = False, "stale"
                    self._measurements = {}
                else:
                    self._confirmed = self._at_contact(future.result())
            except Exception:
                self._confirmed, self._reason = False, "service"
                self._measurements = {}
            self._pending = None
            self.publish()

        try:
            future = self.client.call_async(request)
            self._pending = (token, now, future, request)
            future.add_done_callback(completed)
        except Exception:
            self._pending, self._receipt, self._confirmed = None, None, False
            self._reason = "service"
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
        self._reason = "service"
        context = getattr(self.node, "context", None)
        if context is None or context.ok():
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError as error:
        # SIGINT can invalidate the context before native WaitSet construction.
        # This exact shutdown race never licenses hiding a live transport error.
        if node.context.ok() or str(error) != (
            "failed to initialize wait set: the given context is not valid, either "
            "rcl_init() was not called or rcl_shutdown() was called., "
            "at ./src/rcl/wait.c:130"
        ):
            raise
    except SystemError as error:
        # rclpy's generated response conversion can wrap a SIGINT in SystemError.
        # Never hide an active-context failure or an unrelated conversion error.
        if node.context.ok() or not isinstance(error.__cause__, KeyboardInterrupt):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
