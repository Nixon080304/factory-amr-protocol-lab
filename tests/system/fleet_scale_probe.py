# SPDX-License-Identifier: Apache-2.0
"""Real DDS scale proof; only navigation execution is a bounded synthetic peer."""

import hashlib
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from factory_interfaces.action import ExecuteFleetMission
from fleet_manager.node import FleetManagerNode
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
import yaml

from fleet_isolation import Domain

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/fleet_manager/test"))


def run(output):
    from fake_robot_agent import create_ros_robot

    output.mkdir(parents=True, exist_ok=True)
    source = ROOT / "src/factory_bringup/config/fleet.yaml"
    original = hashlib.sha256(source.read_bytes()).hexdigest()
    production = sorted((ROOT / "src/fleet_manager/fleet_manager").glob("*.py"))
    original_code = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in production
    }
    config = yaml.safe_load(source.read_text())
    identities = [f"synthetic_{i:02}" for i in range(10)]
    config["robots"] = [
        dict(
            robot_id=r,
            namespace="/scale/" + r,
            frame_prefix="scale/" + r + "/",
            spawn=[
                7.04 * math.cos(math.pi / 2 + (i - 4.5) * 0.09),
                -3.5 + 7.04 * math.sin(math.pi / 2 + (i - 4.5) * 0.09),
                0.0,
            ],
            battery_start_percent=80.0,
        )
        for i, r in enumerate(identities)
    ]
    # Synthetic DDS peers do not navigate station approaches. Do not fabricate
    # ten physical parking bays in the supplied two-robot Gazebo world.
    for field in ("station_staging", "station_approach", "station_exit_poses"):
        config.pop(field, None)
    for dock in config["docks"].values():
        dock.pop("exit_poses", None)
        dock.pop("departure_stations", None)
    fleet_file = output / "ten-agent-fleet.yaml"
    fleet_file.write_text(yaml.safe_dump(config))
    domain = Domain()
    context = Context()
    rclpy.init(context=context, domain_id=domain.id)
    executor = SingleThreadedExecutor(context=context)
    nodes = []
    started = time.monotonic()

    def wait(predicate, timeout=15):
        deadline = min(started + 50, time.monotonic() + timeout)
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "ten-agent DDS proof deadline expired"

    try:
        manager = FleetManagerNode(
            context=context,
            parameter_overrides=[
                Parameter("fleet_file", value=str(fleet_file)),
                Parameter("journal_path", value=str(output / "ten-agent.sqlite3")),
            ],
        )
        nodes.append(manager)
        agents = [create_ros_robot(context, r, "/scale/" + r, 1.0) for r in identities]
        for agent in agents:
            agent.finish = False
        nodes.extend(agents)
        peer = rclpy.create_node("scale_acceptance", context=context)
        nodes.append(peer)
        for node in nodes:
            executor.add_node(node)
        client = ActionClient(
            peer, ExecuteFleetMission, "/factory/execute_fleet_mission"
        )
        wait(
            lambda: (
                client.server_is_ready()
                and len(manager.adapter.registry.eligible(time.monotonic())) == 10
            )
        )
        results = []
        order = []
        for i in range(10):
            sent = client.send_goal_async(
                ExecuteFleetMission.Goal(
                    mission_id=f"scale-{i:02}",
                    pickup_station="assembly",
                    dropoff_station="inspection",
                    part="motor",
                )
            )
            wait(sent.done)
            assert sent.result().accepted
            results.append(sent.result().get_result_async())
            wait(lambda: sum(len(a.goals) for a in agents) == i + 1)
            assigned = manager.journal.get(f"scale-{i:02}").assigned_robot_id
            order.append(assigned)
            assert assigned == identities[i]
        for agent in agents:
            agent.finish = True
        wait(lambda: all(r.done() for r in results))
        assert all(
            r.result().result.success and r.result().result.final_state == "COMPLETED"
            for r in results
        )
        assert all(
            manager.journal.get(f"scale-{i:02}").payload_ownership == "DELIVERED"
            for i in range(10)
        )
        assert hashlib.sha256(source.read_bytes()).hexdigest() == original
        assert all(
            hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
            for path, digest in original_code.items()
        )
        assert all(agent.cost_requests for agent in agents)
        receipt = dict(
            registered=10,
            costed=sum(bool(agent.cost_requests) for agent in agents),
            cost_requests={
                identity: len(agent.cost_requests)
                for identity, agent in zip(identities, agents)
            },
            assigned=10,
            completed=10,
            assignment_order=order,
            production_config_unchanged=True,
            production_code_unchanged=True,
            ros_domain_id=domain.id,
            elapsed_sec=time.monotonic() - started,
            proof="production fleet manager, real DDS heartbeats/cost/action endpoints; bounded synthetic navigation",
        )
        (output / "ten-agent-receipt.json").write_text(
            json.dumps(receipt, indent=2) + "\n"
        )
        print(json.dumps(receipt), flush=True)
        client.destroy()
    finally:
        executor.shutdown(timeout_sec=2)
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown(context=context)
        domain.close()


if __name__ == "__main__":
    run(Path(sys.argv[1]))
