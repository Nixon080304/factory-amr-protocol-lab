# SPDX-License-Identifier: Apache-2.0
"""Robot loss through production DDS endpoints, with exact lease quarantine."""

import json
from pathlib import Path
import sys
import time

import rclpy
from factory_interfaces.action import ExecuteFleetMission
from factory_interfaces.msg import RobotState
from factory_interfaces.srv import AcquireResource
from fleet_manager.node import FleetManagerNode
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from fleet_isolation import Domain

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/fleet_manager/test"))


def scenario(output, picked_up):
    from fake_robot_agent import create_ros_robot

    output.mkdir()
    domain = Domain()
    context = Context()
    rclpy.init(context=context, domain_id=domain.id)
    executor = SingleThreadedExecutor(context=context)
    manager = FleetManagerNode(
        context=context,
        parameter_overrides=[
            Parameter(
                "fleet_file", value=str(ROOT / "src/factory_bringup/config/fleet.yaml")
            ),
            Parameter("journal_path", value=str(output / "fleet.sqlite3")),
        ],
    )
    agents = [
        create_ros_robot(context, robot, "/" + robot, cost)
        for robot, cost in (("amr_01", 1.0), ("amr_02", 2.0))
    ]
    former, replacement = agents
    former.finish = False
    former.stage = "NAVIGATING_TO_DROPOFF" if picked_up else "NAVIGATING_TO_PICKUP"
    peer = rclpy.create_node("failure_proof", context=context)
    nodes = [manager, *agents, peer]
    for node in nodes:
        executor.add_node(node)
    client = ActionClient(peer, ExecuteFleetMission, "/factory/execute_fleet_mission")
    acquire = peer.create_client(AcquireResource, "/factory/resources/acquire")
    started = time.monotonic()

    def wait(predicate, bound=12):
        end = min(started + 35, time.monotonic() + bound)
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "DDS failure proof deadline expired"

    try:
        wait(
            lambda: (
                client.server_is_ready()
                and acquire.service_is_ready()
                and len(manager.adapter.registry.eligible(time.monotonic())) == 2
            )
        )
        sent = client.send_goal_async(
            ExecuteFleetMission.Goal(
                mission_id="loss",
                pickup_station="assembly",
                dropoff_station="inspection",
                part="motor",
            )
        )
        wait(sent.done)
        result = sent.result().get_result_async()
        wait(lambda: len(former.goals) == 1)
        wait(lambda: manager.journal.get("loss").state == "EXECUTING")
        owned = acquire.call_async(
            AcquireResource.Request(
                robot_id="amr_01", mission_id="loss", resource_id="central_aisle"
            )
        )
        wait(owned.done)
        assert owned.result().granted and owned.result().lease_id
        lease_id = owned.result().lease_id
        former.heartbeat_timer.cancel()
        wait(
            lambda: (
                manager.journal.get("loss").state
                == ("RECOVERY_REQUIRED" if picked_up else "REASSIGNING")
            )
        )
        resource = next(
            r
            for r in manager.adapter.resources.snapshot(time.monotonic())
            if r.resource_id == "central_aisle"
        )
        assert resource.reconciliation_required
        assert resource.former_lease.lease_id == lease_id
        assert not replacement.goals
        if picked_up:
            record = manager.journal.get("loss")
            assert (
                record.payload_ownership == "PICKED_UP"
                and record.assigned_robot_id == "amr_01"
            )
            wait(result.done)
            assert result.result().result.final_state == "RECOVERY_REQUIRED"
            return dict(
                state=record.state,
                carrier=record.assigned_robot_id,
                retained_lease_id=lease_id,
                ros_domain_id=domain.id,
            )
        # No timer or last known pose can authorize replacement. Wait past a
        # scheduling round while the old action is still active and unknown.
        end = time.monotonic() + 1.3
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            assert not replacement.goals
        # The bounded driver explicitly stops its old execution, then publishes
        # a fresh empty/stopped world-frame observation outside all route zones.
        former.failed = True
        for handle in former.active.values():
            handle.execute()
        former.active.clear()
        for _ in range(20):
            executor.spin_once(timeout_sec=0.01)
        for service in tuple(former.services):
            former.destroy_service(service)
        publisher = next(p for p in former.publishers if p.msg_type is RobotState)
        stopped = RobotState(
            robot_id="amr_01",
            mode="AVAILABLE",
            frame_id="amr_01/map",
            battery_percent=80.0,
            payload_state="EMPTY",
        )
        stopped.pose.position.x, stopped.pose.position.y = -3.0, -3.0
        stopped.pose.orientation.w = 1.0
        stop_receipt = time.monotonic()
        end = stop_receipt + 10
        while not replacement.goals and time.monotonic() < end:
            publisher.publish(stopped)
            executor.spin_once(timeout_sec=0.02)
        assert len(replacement.goals) == 1
        assert manager.journal.get("loss").assigned_robot_id == "amr_02"
        resource = next(
            r
            for r in manager.adapter.resources.snapshot(time.monotonic())
            if r.resource_id == "central_aisle"
        )
        assert not resource.reconciliation_required and resource.former_lease is None
        wait(result.done)
        assert result.result().result.success
        return dict(
            state="COMPLETED",
            reassigned_robot="amr_02",
            blocked_before_stop_clear=True,
            cleared_lease_id=lease_id,
            stop_clear_monotonic=stop_receipt,
            ros_domain_id=domain.id,
        )
    finally:
        client.destroy()
        executor.shutdown(timeout_sec=2)
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown(context=context)
        domain.close()


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    before = scenario(output / "before-pickup", False)
    after = scenario(output / "after-pickup", True)
    receipt = dict(
        before_pickup=before["state"],
        blocked_before_stop_clear=before["blocked_before_stop_clear"],
        reassigned_robot=before["reassigned_robot"],
        after_pickup=after["state"],
        carrier=after["carrier"],
        evidence=[before, after],
    )
    (output / "failure-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt))


if __name__ == "__main__":
    run(Path(sys.argv[1]))
