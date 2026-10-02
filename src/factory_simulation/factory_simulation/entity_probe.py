# SPDX-License-Identifier: Apache-2.0
"""Keep Gazebo entity service details inside the simulation package."""

import time

from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose
import rclpy


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
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
        if not future.done():
            future.cancel()
            raise TimeoutError("Gazebo world pose did not arrive within bounded timeout")
        response = future.result()
        if not response.success:
            raise RuntimeError(f"Gazebo could not retrieve world pose for entity {name!r}")
        return response.state.pose
    finally:
        node.destroy_client(client)
