# SPDX-License-Identifier: Apache-2.0
"""Keep the real DDS world observer persistent, bounded and response-fenced."""

import time
from pathlib import Path
import sys

from gazebo_msgs.srv import GetEntityState
import pytest
import rclpy
from rclpy.task import Future

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from factory_simulation import entity_probe

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tests/system"))
from fleet_isolation import Domain


@pytest.fixture
def node():
    domain, value = Domain(), None
    try:
        rclpy.init(domain_id=domain.id)
        value = rclpy.create_node("persistent_world_probe_test")
        yield value
    finally:
        if value is not None:
            value.destroy_node()
        rclpy.try_shutdown()
        domain.close()


def test_world_probe_reuses_one_real_client_without_retry(node, monkeypatch):
    requests, clients, waits = [], [], []
    original = node.create_client

    def create(*args, **kwargs):
        client = original(*args, **kwargs)
        clients.append(client)
        wait = client.wait_for_service

        def ready(*args, **kwargs):
            waits.append(kwargs["timeout_sec"])
            return wait(*args, **kwargs)

        monkeypatch.setattr(client, "wait_for_service", ready)
        return client

    monkeypatch.setattr(node, "create_client", create)

    def answer(request, response):
        requests.append((request.name, request.reference_frame))
        response.success = True
        response.state.pose.position.x = float(len(requests))
        response.state.pose.orientation.w = 1.0
        return response

    server = node.create_service(GetEntityState, "/gazebo/get_entity_state", answer)
    probe = entity_probe.WorldPoseProbe(node)
    try:
        for index in range(10):
            pose = probe.pose(f"robot_{index}", timeout_sec=1.0)
            assert pose.position.x == index + 1
        assert len(clients) == 1
        assert len(waits) == 1
        assert requests == [(f"robot_{index}", "world") for index in range(10)]
    finally:
        probe.close()
        node.destroy_service(server)


def test_closing_pending_world_probe_retires_response_and_client(node):
    pending = []
    probe = entity_probe.WorldPoseProbe(node)

    def answer(request, response):
        assert request.name == "closing"
        pending.append(probe._pending)
        probe.close()
        response.success = True
        return response

    server = node.create_service(GetEntityState, "/gazebo/get_entity_state", answer)
    try:
        with pytest.raises(RuntimeError, match="closed"):
            probe.pose("closing", timeout_sec=1.0)
        assert len(pending) == 1 and pending[0].cancelled()
        assert probe._pending is None
        assert not probe._client._pending_requests
    finally:
        probe.close()
        node.destroy_service(server)
    probe.close()
    with pytest.raises(RuntimeError, match="closed"):
        probe.pose("retired", timeout_sec=1.0)


def test_missing_world_reply_fails_one_second_and_fences_late_reply(node):
    requests = []
    barrier = Future()

    async def answer(request, response):
        requests.append(request.name)
        if request.name == "old":
            await barrier
            response.state.pose.position.x = 99.0
        else:
            response.state.pose.position.x = 2.0
        response.success = True
        response.state.pose.orientation.w = 1.0
        return response

    server = node.create_service(GetEntityState, "/gazebo/get_entity_state", answer)
    probe = entity_probe.WorldPoseProbe(node)
    try:
        started = time.monotonic()
        with pytest.raises(TimeoutError, match="bounded timeout"):
            probe.pose("old", timeout_sec=1.0)
        assert 1.0 <= time.monotonic() - started < 1.3
        assert requests == ["old"]
        barrier.set_result(None)
        assert probe.pose("new", timeout_sec=1.0).position.x == 2.0
        assert requests == ["old", "new"]
        assert not probe._client._pending_requests
    finally:
        probe.close()
        node.destroy_service(server)
