# SPDX-License-Identifier: Apache-2.0
"""Ingress cannot start with only one half of a robot's dispatch endpoints."""

import sys
import time
from pathlib import Path

from factory_interfaces.action import ExecuteFactoryMission
from factory_interfaces.srv import EstimateMissionCost
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fleet_driver
from fleet_isolation import Domain


@pytest.mark.parametrize("missing", ["cost", "action", None, "delayed_action"])
def test_startup_requires_both_real_dispatch_endpoints_before_ingress(missing):
    domain, context = Domain(), Context()
    rclpy.init(context=context, domain_id=domain.id)
    node = rclpy.create_node("dispatch_readiness_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    cost_name = "/cart/factory/estimate_mission_cost"
    action_name = "/cart/factory/execute_mission"
    cost = node.create_client(EstimateMissionCost, cost_name)
    action = ActionClient(node, ExecuteFactoryMission, action_name)
    service, server = None, None
    if missing != "cost":
        service = node.create_service(
            EstimateMissionCost, cost_name, lambda request, response: response
        )
    if missing not in ("action", "delayed_action"):
        server = ActionServer(
            node, ExecuteFactoryMission, action_name, lambda goal: None
        )
    endpoints = {cost_name: cost.service_is_ready, action_name: action.server_is_ready}
    started = time.monotonic()
    ticks = 0

    def spin():
        nonlocal server, ticks
        ticks += 1
        if missing == "delayed_action" and ticks == 3:
            server = ActionServer(
                node, ExecuteFactoryMission, action_name, lambda goal: None
            )
        executor.spin_once(timeout_sec=0.01)

    try:
        deadline = started + (0.3 if missing in ("cost", "action") else 2.0)
        if missing in ("cost", "action"):
            exact = cost_name if missing == "cost" else action_name
            with pytest.raises(AssertionError, match=exact):
                fleet_driver.wait_dispatch_endpoints(
                    endpoints, spin, lambda: None, deadline
                )
            assert time.monotonic() - started < 1.0
        else:
            assert fleet_driver.wait_dispatch_endpoints(
                endpoints, spin, lambda: None, deadline
            ) == {cost_name: True, action_name: True}
            if missing == "delayed_action":
                assert ticks >= 3
    finally:
        action.destroy()
        node.destroy_client(cost)
        if server is not None:
            server.destroy()
        if service is not None:
            node.destroy_service(service)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        domain.close()
