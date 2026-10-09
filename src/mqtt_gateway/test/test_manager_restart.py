"""Keep real MQTT and DDS peers alive while only the manager process restarts."""

import importlib
import json
import os
from pathlib import Path
import signal
import socket
import sqlite3
import subprocess
import sys
import threading
import time
import uuid

import paho.mqtt.client as mqtt
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from factory_interfaces.action import ExecuteFactoryMission
from factory_interfaces.msg import RobotState
from factory_interfaces.srv import EstimateMissionCost
from fleet_manager.config import load_fleet_config
from mqtt_gateway.node import MqttGatewayNode

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tests/system"))
Domain = importlib.import_module("fleet_isolation").Domain


@pytest.mark.parametrize("initial", ["QUEUED", "EXECUTING"])
def test_live_gateway_observes_manager_only_restart_without_reexecution(
    tmp_path, initial
):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    reservation = Domain()
    owned_broker = importlib.import_module("fleet_driver").Owned(
        tmp_path, time.monotonic() + 60
    )
    context = Context()
    broker_id, gateway, executor, thread, probe = None, None, None, None, None
    processes, logs, servers = [], [], []
    stop = threading.Event()
    peer = None
    path = tmp_path / "restart.sqlite3"
    config_file = ROOT / "src/factory_bringup/config/fleet.yaml"
    config = load_fleet_config(config_file)
    statuses, executed, active = [], [], {}
    eligible = [initial == "EXECUTING"]
    finish = [False]

    def wait(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        assert predicate(), "bounded MQTT manager-restart predicate timed out"

    def read_state():
        if not path.exists():
            return None
        with sqlite3.connect(path) as connection:
            try:
                row = connection.execute(
                    "SELECT state FROM missions WHERE mission_id='manager-restart'"
                ).fetchone()
            except sqlite3.OperationalError:
                return None
        return row[0] if row else None

    def start_manager():
        log = (tmp_path / f"manager-{len(processes)}.log").open("w")
        logs.append(log)
        process = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "from fleet_manager.main import main; main()",
                "--ros-args",
                "-p",
                f"fleet_file:={config_file}",
                "-p",
                f"journal_path:={path}",
                "-r",
                f"__node:=mqtt_restart_manager_{len(processes)}",
            ],
            cwd=ROOT,
            env=dict(os.environ, ROS_DOMAIN_ID=str(reservation.id)),
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        processes.append(process)
        return process

    try:
        owned_broker.create_broker(port)
        broker_id = owned_broker.container
        assert len(broker_id) == 64 and all(c in "0123456789abcdef" for c in broker_id)
        rclpy.init(context=context, domain_id=reservation.id)
        peer = rclpy.create_node("mqtt_restart_agents", context=context)
        publishers = {}

        def accepted(handle, robot_id):
            executed.append(handle.request.mission_id)
            active[robot_id] = handle
            handle.publish_feedback(
                ExecuteFactoryMission.Feedback(
                    state="NAVIGATING_TO_DROPOFF", station="inspection"
                )
            )

        def execute(handle):
            handle.succeed()
            return ExecuteFactoryMission.Result(success=True, final_state="COMPLETED")

        def cost(request, response):
            response.feasible = eligible[0]
            response.path_cost = 1.0
            response.predicted_final_battery = 80.0
            return response

        for robot in config.robots:
            base = robot.namespace + "/factory/"
            publishers[robot.robot_id] = peer.create_publisher(
                RobotState, base + "robot_state", 10
            )
            peer.create_service(
                EstimateMissionCost, base + "estimate_mission_cost", cost
            )
            servers.append(
                ActionServer(
                    peer,
                    ExecuteFactoryMission,
                    base + "execute_mission",
                    execute,
                    handle_accepted_callback=lambda handle, robot_id=robot.robot_id: (
                        accepted(handle, robot_id)
                    ),
                )
            )

        def heartbeat():
            for robot in config.robots:
                handle = active.get(robot.robot_id)
                if handle is not None and finish[0]:
                    del active[robot.robot_id]
                    handle.execute()
                    handle = None
                state = RobotState(
                    robot_id=robot.robot_id,
                    mode="EXECUTING" if handle else "AVAILABLE",
                    frame_id=robot.frame_prefix + "map",
                    battery_percent=80.0,
                    payload_state="LOADED" if handle else "EMPTY",
                    mission_id=handle.request.mission_id if handle else "",
                )
                state.pose.position.x, state.pose.position.y = (
                    robot.spawn.x,
                    robot.spawn.y,
                )
                state.pose.orientation.w = 1.0
                publishers[robot.robot_id].publish(state)

        peer.create_timer(0.1, heartbeat)
        gateway = MqttGatewayNode(
            context=context,
            parameter_overrides=[
                Parameter("broker_port", value=port),
                Parameter("fleet_file", value=str(config_file)),
            ],
        )
        executor = SingleThreadedExecutor(context=context)
        for node in (peer, gateway):
            executor.add_node(node)

        def spin():
            while not stop.is_set():
                executor.spin_once(timeout_sec=0.05)

        thread = threading.Thread(target=spin)
        thread.start()
        probe = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="restart-probe-" + uuid.uuid4().hex,
        )
        subscribed = threading.Event()
        probe.on_connect = lambda client, *args: client.subscribe(
            "factory/missions/manager-restart/status", qos=1
        )
        probe.on_subscribe = lambda *args: subscribed.set()
        probe.on_message = lambda client, userdata, message: statuses.append(
            json.loads(message.payload)
        )
        probe.connect("127.0.0.1", port, 20)
        probe.loop_start()
        wait(lambda: subscribed.is_set() and gateway.connected)
        manager = start_manager()
        wait(gateway.action.server_is_ready)
        raw = json.dumps(
            dict(
                mission_id="manager-restart",
                pickup="assembly",
                dropoff="inspection",
                part="motor",
            )
        )
        probe.publish("factory/missions/request", raw, qos=1).wait_for_publish(
            timeout=3
        )
        wait(lambda: read_state() == initial)
        wait(lambda: any(status["state"] == initial for status in statuses))
        assert len(executed) == (1 if initial == "EXECUTING" else 0)
        os.kill(manager.pid, signal.SIGKILL)
        manager.wait(timeout=5)
        assert manager.returncode == -signal.SIGKILL
        start_manager()
        wait(
            lambda: (
                gateway.action.server_is_ready()
                and gateway.mission_query.service_is_ready()
            )
        )
        if initial == "QUEUED":
            eligible[0], finish[0] = True, True
        wanted = "COMPLETED" if initial == "QUEUED" else "RECOVERY_REQUIRED"
        wait(lambda: read_state() == wanted)
        count = len(statuses)
        probe.publish("factory/missions/request", raw, qos=1).wait_for_publish(
            timeout=3
        )
        wait(lambda: any(status["state"] == wanted for status in statuses[count:]))
        assert len(executed) == 1
        assert gateway.context is context and processes[-1].poll() is None
        inspect = subprocess.run(
            ["docker", "inspect", "--format", "{{.Id}}", broker_id],
            capture_output=True,
            text=True,
            check=True,
            timeout=10,
        )
        assert inspect.stdout.strip() == broker_id
        if initial == "EXECUTING":
            finish[0] = (
                True  # Late old robot completion cannot change durable recovery.
            )
            wait(lambda: not active)
            assert read_state() == wanted
        probe.publish(
            "factory/missions/request", raw.replace('"motor"', '"gear"'), qos=1
        ).wait_for_publish(timeout=3)
        wait(
            lambda: any(
                status.get("error_code") == "MISSION_ID_CONFLICT" for status in statuses
            )
        )
        assert len(executed) == 1 and read_state() == wanted
        (tmp_path / "mqtt-statuses.json").write_text(
            json.dumps(statuses, indent=2) + "\n"
        )
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        if probe is not None:
            probe.disconnect()
            probe.loop_stop()
        stop.set()
        if executor is not None:
            executor.wake()
        if thread is not None:
            thread.join(timeout=5)
            assert not thread.is_alive()
        if executor is not None:
            executor.shutdown(timeout_sec=5)
        for server in servers:
            server.destroy()
        if gateway is not None:
            gateway.destroy_node()
        if peer is not None:
            peer.destroy_node()
        if context.ok():
            context.shutdown()
        for log in logs:
            log.close()
        try:
            owned_broker.close()
        finally:
            reservation.close()
