# SPDX-License-Identifier: Apache-2.0
"""Keep Gazebo entity service details inside the simulation package."""

import time

from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose
import rclpy


class WorldPoseProbe:
    """Observe one scenario through one owned, bounded DDS service client."""

    def __init__(self, node):
        self._node = node
        self._client = node.create_client(GetEntityState, "/gazebo/get_entity_state")
        self._ready = self._closed = False
        self._pending = None
        self._sequence = 0

    def pose(self, name, *, timeout_sec=1.0) -> Pose:
        if self._closed:
            raise RuntimeError("world pose probe is closed")
        if self._pending is not None:
            raise RuntimeError("world pose probe already has a pending request")
        deadline = time.monotonic() + timeout_sec
        if not self._ready:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self._client.wait_for_service(
                timeout_sec=remaining
            ):
                raise TimeoutError(
                    "Gazebo pose service unavailable within bounded timeout"
                )
            self._ready = True
        self._sequence += 1
        sequence = self._sequence
        request = GetEntityState.Request()
        request.name, request.reference_frame = name, "world"
        future = self._client.call_async(request)
        self._pending = future
        try:
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(
                    self._node,
                    timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
                )
            if self._closed or sequence != self._sequence:
                raise RuntimeError("world pose probe is closed or response retired")
            if not future.done():
                raise TimeoutError(
                    "Gazebo world pose did not arrive within bounded timeout"
                )
            response = future.result()
            if not response.success:
                raise RuntimeError(
                    f"Gazebo could not retrieve world pose for entity {name!r}"
                )
            return response.state.pose
        finally:
            if self._pending is future:
                self._pending = None
                if not future.done():
                    future.cancel()
                self._client.remove_pending_request(future)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._sequence += 1
        if self._pending is not None:
            future, self._pending = self._pending, None
            if not future.done():
                future.cancel()
            self._client.remove_pending_request(future)
        self._node.destroy_client(self._client)


def get_entity_pose(node, name, *, timeout_sec=5.0) -> Pose:
    """Return an independent world-frame pose within one wall-time budget.

    The caller supplies a node that this function may spin. No Gazebo message or
    service handle escapes this interface. Each call owns and releases its client.
    """
    deadline = time.monotonic() + timeout_sec
    client = node.create_client(GetEntityState, "/gazebo/get_entity_state")
    try:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
            raise TimeoutError("Gazebo pose service unavailable within bounded timeout")
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = "world"
        future = client.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic()))
            )
        if not future.done():
            future.cancel()
            raise TimeoutError(
                "Gazebo world pose did not arrive within bounded timeout"
            )
        response = future.result()
        if not response.success:
            raise RuntimeError(
                f"Gazebo could not retrieve world pose for entity {name!r}"
            )
        return response.state.pose
    finally:
        node.destroy_client(client)


def set_entity_pose(node, name, pose: Pose, *, timeout_sec=5.0) -> None:
    """Reposition an entity in world coordinates with zero twist and a deadline.

    This simulation-only adapter keeps Gazebo service types out of perception and
    mission packages. The caller supplies a node this function may spin.
    """
    deadline = time.monotonic() + timeout_sec
    client = node.create_client(SetEntityState, "/gazebo/set_entity_state")
    try:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
            raise TimeoutError(
                "Gazebo reposition service unavailable within bounded timeout"
            )
        request = SetEntityState.Request()
        request.state.name = name
        request.state.reference_frame = "world"
        request.state.pose = pose
        future = client.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic()))
            )
        if not future.done():
            future.cancel()
            raise TimeoutError(
                "Gazebo reposition did not finish within bounded timeout"
            )
        if not future.result().success:
            raise RuntimeError(f"Gazebo could not reposition entity {name!r}")
    finally:
        node.destroy_client(client)
