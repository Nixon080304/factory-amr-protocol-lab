"""Kill and restart the real fleet node against SQLite and real DDS fake agents."""

import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src/fleet_manager"))

from fleet_manager.config import load_fleet_config
from fleet_manager.journal import MissionJournal, MissionState
from fleet_manager.models import MissionRequest
from fleet_manager.resources import LeaseRequest, ResourceManager

ROOT = Path(__file__).resolve().parents[3]


MATRIX = [
    ("RECEIVED", "NOT_PICKED_UP", None, "AVAILABLE", (-3, -3), "QUEUED"),
    ("QUEUED", "NOT_PICKED_UP", None, "AVAILABLE", (-3, -3), "QUEUED"),
    ("ASSIGNING", "NOT_PICKED_UP", None, "AVAILABLE", (-3, -3), "QUEUED"),
    ("ASSIGNED", "NOT_PICKED_UP", None, "AVAILABLE", (-3, -3), "QUEUED"),
    ("EXECUTING", "NOT_PICKED_UP", None, "AVAILABLE", (-3, -3), "QUEUED"),
    ("REASSIGNING", "NOT_PICKED_UP", None, "AVAILABLE", (-3, -3), "QUEUED"),
    ("EXECUTING", "PICKED_UP", None, "EXECUTING", (-3, -3), "RECOVERY_REQUIRED"),
    (
        "EXECUTING",
        "UNKNOWN",
        "assembly",
        "WAITING_FOR_RESOURCE",
        (-3, 1),
        "RECOVERY_REQUIRED",
    ),
    (
        "EXECUTING",
        "NOT_PICKED_UP",
        "central_aisle",
        "WAITING_FOR_RESOURCE",
        (0, -2.2),
        "RECOVERY_REQUIRED",
    ),
    ("COMPLETED", "DELIVERED", None, "AVAILABLE", (-3, -3), "COMPLETED"),
    ("FAILED", "UNKNOWN", None, "AVAILABLE", (-3, -3), "FAILED"),
    ("CANCELLED", "NOT_PICKED_UP", None, "AVAILABLE", (-3, -3), "CANCELLED"),
    (
        "RECOVERY_REQUIRED",
        "PICKED_UP",
        None,
        "EXECUTING",
        (-3, -3),
        "RECOVERY_REQUIRED",
    ),
    ("QUEUED", "NOT_PICKED_UP", None, "DOCKING", (3.5, -2), "QUEUED"),
    ("QUEUED", "NOT_PICKED_UP", "dock_01", "CHARGING", (4, -2), "QUEUED"),
    ("EXECUTING", "NOT_PICKED_UP", None, "OFFLINE", (-3, -3), "RECOVERY_REQUIRED"),
    ("EXECUTING", "NOT_PICKED_UP", None, "LIVE_BEFORE", (-3, -3), "QUEUED"),
    ("EXECUTING", "PICKED_UP", None, "LIVE_AFTER", (-3, -3), "RECOVERY_REQUIRED"),
    ("EXECUTING", "PICKED_UP", None, "LIVE_WRITE_FAIL", (-3, -3), "RECOVERY_REQUIRED"),
    (
        "EXECUTING",
        "UNKNOWN",
        None,
        "WAITING_FOR_RESOURCE",
        (-3, 1),
        "RECOVERY_REQUIRED",
    ),
]


@pytest.mark.parametrize("state,ownership,held,mode,position,want", MATRIX)
def test_actual_kill_restart_failure_matrix(
    tmp_path, state, ownership, held, mode, position, want
):
    import rclpy
    from factory_interfaces.action import ExecuteFactoryMission
    from factory_interfaces.msg import RobotState
    from factory_interfaces.srv import (
        AcquireResource,
        EstimateMissionCost,
        RenewResource,
    )
    from rclpy.action import ActionServer
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    fleet_file = ROOT / "src/factory_bringup/config/fleet.yaml"
    if mode == "WAITING_FOR_RESOURCE" and held is None:
        # Migration from Task 13 has neither station geometry nor lease history.
        data = yaml.safe_load(fleet_file.read_text())
        data.pop("resource_bounds", None)
        fleet_file = tmp_path / "legacy-fleet.yaml"
        fleet_file.write_text(yaml.safe_dump(data))
    config = load_fleet_config(fleet_file)
    live = mode.startswith("LIVE_")
    seed_state = "QUEUED" if live else state
    path = tmp_path / "restart.sqlite3"
    journal = MissionJournal(path)
    journal.register(
        MissionRequest("old", "assembly", "inspection", "motor"), "old-hash", 1
    )
    assigned = (
        "amr_01"
        if seed_state not in ("RECEIVED", "QUEUED", "ASSIGNING", "REASSIGNING")
        else None
    )
    journal.transition(
        "old",
        MissionState.QUEUED,
        MissionState(seed_state),
        {
            "assigned_robot_id": assigned,
            "payload_ownership": "NOT_PICKED_UP" if live else ownership,
        },
        2,
    )
    old_lease = None
    if held:
        resources = ResourceManager(config.resources)
        old_lease = resources.acquire(
            LeaseRequest("amr_01", "dock-old" if held == "dock_01" else "old", held), 2
        ).lease
        journal.save_resources(resources.snapshot(2), 2)
    journal.close()
    context = Context()
    domain_id = 100 + MATRIX.index((state, ownership, held, mode, position, want))
    rclpy.init(context=context, domain_id=domain_id)
    peer = rclpy.create_node("restart_agents", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    goals, costs = [], []
    handles = []
    phase = [0]
    permit_cost = [False]
    publishers = {
        robot.robot_id: peer.create_publisher(
            RobotState, robot.namespace + "/factory/robot_state", 10
        )
        for robot in config.robots
    }

    def estimate(request, response):
        costs.append(request.mission_id)
        response.feasible = permit_cost[0]
        response.path_cost = 1.0
        response.predicted_final_battery = 60.0
        return response

    actions = []

    def accepted(handle):
        goals.append(handle.request)
        handles.append(handle)

    for robot in config.robots:
        peer.create_service(
            EstimateMissionCost,
            robot.namespace + "/factory/estimate_mission_cost",
            estimate,
        )
        actions.append(
            ActionServer(
                peer,
                ExecuteFactoryMission,
                robot.namespace + "/factory/execute_mission",
                lambda _: ExecuteFactoryMission.Result(
                    success=False, final_state="FAILED", error_code="STOPPED"
                ),
                handle_accepted_callback=accepted,
            )
        )
    acquire = peer.create_client(AcquireResource, "/factory/resources/acquire")
    renew = peer.create_client(RenewResource, "/factory/resources/renew")
    processes, logs = [], []

    def read():
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
            return connection.execute(
                "SELECT state, assigned_robot_id, payload_ownership FROM missions WHERE mission_id='old'"
            ).fetchone()

    def spin(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait(predicate, timeout=8, require_alive=True):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if require_alive:
                assert processes[-1].poll() is None, "fleet process exited unexpectedly"
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "bounded restart predicate timed out"

    def start():
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))
        manager_source = os.environ.get(
            "TASK14_MANAGER_SOURCE", str(ROOT / "src/fleet_manager")
        )
        env["PYTHONPATH"] = manager_source + os.pathsep + env.get("PYTHONPATH", "")
        log = (tmp_path / f"manager-{len(processes)}.log").open("w")
        logs.append(log)
        name = f"restart_manager_{len(processes)}"
        process = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "import faulthandler, signal; faulthandler.register(signal.SIGUSR1); from fleet_manager.main import main; main()",
                "--ros-args",
                "-p",
                "fleet_file:=" + str(fleet_file),
                "-p",
                "journal_path:=" + str(path),
                "-p",
                "use_sim_time:=true",
                "-p",
                "reconciliation_timeout_sec:="
                + ("5.0" if mode == "OFFLINE" else "20.0"),
                "-r",
                "__node:=" + name,
            ],
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        processes.append(process)
        wait(
            lambda: (
                name in peer.get_node_names()
                and len(peer.get_subscriber_names_and_types_by_node(name, "/")) >= 2
                and all(
                    publisher.get_subscription_count() >= 1
                    for publisher in publishers.values()
                )
            )
        )
        # Real process is killed at the exact persisted boundary before fresh
        # observations. Paused /clock must not bypass the startup barrier.
        spin(0.1)
        assert read()[0] == (seed_state if len(processes) == 1 else state)
        return process

    def heartbeat(robot_id):
        first = robot_id == "amr_01"
        observed_mode = mode
        if live:
            observed_mode = (
                "EXECUTING"
                if phase[0]
                and mode == "LIVE_AFTER"
                or phase[0] == 1
                and mode == "LIVE_WRITE_FAIL"
                else "AVAILABLE"
            )
        loaded = (
            first
            and ownership in ("PICKED_UP", "UNKNOWN")
            and (not live or phase[0] > 0)
            and not (mode == "LIVE_WRITE_FAIL" and phase[0] == 2)
        )
        message = RobotState(
            robot_id=robot_id,
            mode=observed_mode if first else "AVAILABLE",
            frame_id=robot_id + "/map",
            battery_percent=80.0,
            payload_state=("LOADED" if ownership == "PICKED_UP" else "UNKNOWN")
            if loaded
            else "EMPTY",
            mission_id="old"
            if first and observed_mode in ("EXECUTING", "WAITING_FOR_RESOURCE")
            else "",
        )
        # ROS stamp deliberately remains 0 with a paused clock. Freshness comes
        # from DDS publish evidence, never fabricated advancing simulation time.
        message.pose.position.x, message.pose.position.y = map(
            # Both robots must clear the configured expanded aisle region. The
            # old second pose (1, -3) was only 0.403 m outside, below the 0.45 m
            # conservative footprint bound, and correctly prevented recovery.
            float,
            position if first else (1.5, -3.0),
        )
        message.pose.orientation.w = 1.0
        publishers[robot_id].publish(message)

    def call(client, request):
        wait(client.service_is_ready)
        future = client.call_async(request)
        wait(future.done)
        return future.result()

    def observe_robot(robot_id):
        deadline = time.monotonic() + 6
        boundary = time.time_ns()
        while time.monotonic() < deadline:
            heartbeat(robot_id)
            spin(0.05)
            with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
                latest = connection.execute(
                    "SELECT MAX(source_time_ns) FROM robot_observations WHERE robot_id=?",
                    (robot_id,),
                ).fetchone()[0]
                if (latest or 0) > boundary:
                    return
        processes[-1].send_signal(signal.SIGUSR1)
        spin(0.1)
        pytest.fail("fresh DDS heartbeat was not persisted within startup deadline")

    try:
        first = start()
        if live:
            permit_cost[0] = True
            observe_robot("amr_01")
            observe_robot("amr_02")
            wait(lambda: len(handles) == 1)
            assert goals[0].robot_id == "amr_01"
            handles[0].publish_feedback(
                ExecuteFactoryMission.Feedback(state="NAVIGATING_TO_PICKUP")
            )
            wait(lambda: read()[0] == "EXECUTING")
            if mode == "LIVE_AFTER":
                handles[0].publish_feedback(
                    ExecuteFactoryMission.Feedback(state="NAVIGATING_TO_DROPOFF")
                )
                wait(lambda: read()[2] == "PICKED_UP")
            elif mode == "LIVE_WRITE_FAIL":
                # Real SQLite failure preserves the correlated observation but
                # rolls back the mission transition. The fresh empty robot after
                # restart must not erase this durable carrying evidence.
                with sqlite3.connect(path) as connection:
                    connection.execute(
                        "CREATE TRIGGER fail_pickup BEFORE UPDATE ON missions "
                        "WHEN NEW.payload_ownership = 'PICKED_UP' "
                        "BEGIN SELECT RAISE(ABORT, 'pickup write failed'); END"
                    )
                phase[0] = 1
                observe_robot("amr_01")
                spin(0.1)
                assert read()[2] == "NOT_PICKED_UP"
                with sqlite3.connect(path) as connection:
                    connection.execute("DROP TRIGGER fail_pickup")
        first.kill()
        first.wait(timeout=3)
        if live:
            phase[0] = 2 if mode == "LIVE_WRITE_FAIL" else 1
            permit_cost[0] = False
            costs.clear()
            if mode == "LIVE_BEFORE":
                handles[0].execute()
                spin(0.1)
        goals_before = len(goals)
        start()
        if mode != "OFFLINE":
            observe_robot("amr_01")
        spin(0.2)
        blocked = call(
            acquire,
            AcquireResource.Request(
                robot_id="amr_01", mission_id="probe", resource_id="inspection"
            ),
        )
        assert not blocked.granted and blocked.reason == "fleet reconciling"
        assert costs == [] and len(goals) == goals_before
        observe_robot("amr_02")
        wait(lambda: len(costs) > 0 or read()[0] == want and want != "QUEUED")
        assert read()[0] == want
        if want == "RECOVERY_REQUIRED":
            assert read()[1] == "amr_01"
            if ownership == "PICKED_UP":
                assert read()[2] == "PICKED_UP"
        if old_lease:
            stale = call(
                renew,
                RenewResource.Request(
                    robot_id="amr_01",
                    mission_id=old_lease.mission_id,
                    resource_id=held,
                    lease_id=old_lease.lease_id,
                ),
            )
            assert not stale.renewed
            other = call(
                acquire,
                AcquireResource.Request(
                    robot_id="amr_02", mission_id="probe", resource_id=held
                ),
            )
            assert not other.granted
            with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
                evidence = connection.execute(
                    "SELECT evidence_json FROM resource_snapshots WHERE resource_id=?",
                    (held,),
                ).fetchone()[0]
                assert old_lease.lease_id in evidence
        if mode == "WAITING_FOR_RESOURCE" and held is None:
            overlap = call(
                acquire,
                AcquireResource.Request(
                    robot_id="amr_02", mission_id="probe", resource_id="assembly"
                ),
            )
            assert not overlap.granted
            with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
                evidence = connection.execute(
                    "SELECT evidence_json FROM resource_snapshots WHERE resource_id='assembly'"
                ).fetchone()[0]
                assert '"robot_id":"amr_01"' in evidence
                assert '"lease":null' in evidence
        assert len(goals) == goals_before
        if mode == "OFFLINE":
            assert read()[2] == "UNKNOWN"
            denied = call(
                acquire,
                AcquireResource.Request(
                    robot_id="amr_02", mission_id="probe", resource_id="central_aisle"
                ),
            )
            assert not denied.granted
        if want == "QUEUED" and mode in ("AVAILABLE", "LIVE_BEFORE"):
            permit_cost[0] = True
            # Refresh both agents throughout paced cost retries.
            deadline = time.monotonic() + 5
            while len(goals) == goals_before and time.monotonic() < deadline:
                heartbeat("amr_01")
                heartbeat("amr_02")
                spin(0.1)
            assert len(goals) == goals_before + 1 and goals[-1].mission_id == "old"
            spin(0.2)
            assert len(goals) == goals_before + 1 and read()[0] == "ASSIGNED"
            assert read()[2] == "NOT_PICKED_UP"
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        for log in logs:
            log.close()
        for action in actions:
            action.destroy()
        executor.shutdown()
        peer.destroy_node()
        context.shutdown()
