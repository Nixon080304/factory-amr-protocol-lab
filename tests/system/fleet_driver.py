# SPDX-License-Identifier: Apache-2.0
"""Bounded production fleet driver with owned process groups and real evidence."""

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import sqlite3
import subprocess
import sys
import tempfile
import time
import traceback
from urllib.request import urlopen
import uuid
import xml.etree.ElementTree as ET

import paho.mqtt.client as mqtt
import yaml

from fleet_isolation import Domain
from test_successful_mission import launch_child_receipt

ROOT = Path(__file__).resolve().parents[2]


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def mission_ingress_ready(snapshot, evidence):
    return (
        evidence["subscribed"]
        and evidence["availability"] == "online"
        and snapshot["fleet_state"] == "RUNNING"
        and all(r["health"] == "ONLINE" for r in snapshot["robots"])
    )


def wait_dispatch_endpoints(endpoints, spin, check, deadline):
    """Discover exact cost/action endpoints within the existing startup budget."""
    while True:
        check()
        ready = {name: available() for name, available in endpoints.items()}
        if all(ready.values()):
            return ready
        assert time.monotonic() < deadline, (
            "dispatch endpoint startup deadline: "
            + ", ".join(name for name, available in ready.items() if not available)
        )
        spin()


def manager_dispatch_ready(robots, configured_ids):
    return {robot["robot_id"] for robot in robots} == configured_ids and all(
        robot.get("cost_service_ready") is True
        and robot.get("mission_action_ready") is True
        for robot in robots
    )


def update_charging_proof(proofs, states, contacts, inside_dock, resources, dock_id):
    for rid, proof in proofs.items():
        proof["contact"] |= contacts.get(rid) is True
        proof["reached_80"] |= states[rid].battery_percent >= 80.0
        proof["verified_clearance"] |= (
            proof["reached_80"]
            and states[rid].mode == "AVAILABLE"
            and rid not in inside_dock
            and all(r["resource_id"] != dock_id or r["owner"] != rid for r in resources)
        )


def charging_roles(config, initial_battery):
    low, eligible = [], []
    for robot in config.robots:
        battery = initial_battery.get(robot.robot_id, robot.battery_start_percent)
        group = low if battery < config.energy.charge_below_percent else eligible
        group.append(robot.robot_id)
    assert len(low) == len(eligible) == 1, (
        "charging proof requires one low and one eligible robot"
    )
    return low[0], eligible[0]


def record_world_sample(world_log, samples, row, *, robot_radius):
    # Retain the failing physical observation, not only the preceding safe pose.
    samples.append(row)
    world_log.write(json.dumps(row) + "\n")
    world_log.flush()
    robots = list(row["robots"].items())
    for index, (first, first_pose) in enumerate(robots):
        for second, second_pose in robots[index + 1 :]:
            distance = math.hypot(
                first_pose["x"] - second_pose["x"],
                first_pose["y"] - second_pose["y"],
            )
            assert distance >= 2 * robot_radius, (
                f"physical footprint overlap {first}/{second}: {distance:.3f}m"
            )


def world_pose_evidence(pose, *, receipt_monotonic):
    p, q = pose.position, pose.orientation
    return dict(
        x=p.x,
        y=p.y,
        z=p.z,
        quaternion=dict(x=q.x, y=q.y, z=q.z, w=q.w),
        yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        receipt_monotonic=receipt_monotonic,
    )


def navigation_plan_evidence(robot_id, message, *, receipt_monotonic):
    return dict(
        robot_id=robot_id,
        frame_id=message.header.frame_id,
        source_stamp=dict(
            sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec
        ),
        pose_count=len(message.poses),
        receipt_monotonic=receipt_monotonic,
        final_pose=(
            world_pose_evidence(
                message.poses[-1].pose, receipt_monotonic=receipt_monotonic
            )
            if message.poses
            else None
        ),
    )


def free_port():
    with socket.socket() as connection:
        connection.bind(("127.0.0.1", 0))
        return connection.getsockname()[1]


def cost_winner(costs, available):
    candidates = [
        (cost["path_cost"], robot_id)
        for robot_id, cost in costs.items()
        if robot_id in available
        and cost["feasible"]
        and math.isfinite(cost["path_cost"])
        and cost["path_cost"] >= 0
        and cost["predicted_final_battery"] >= 20.0
    ]
    assert candidates, "no eligible production cost candidate"
    return min(candidates)[1]


def read_cost(service, request, spin, check, deadline):
    # The production cost service intentionally returns immediately while its
    # asynchronous Nav2 cache is pending. Only that response permits a retry.
    while time.monotonic() < deadline:
        check()
        future = service.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            check()
            spin()
        assert future.done(), "production cost evidence deadline expired"
        response = future.result()
        if response.reason != "path pending":
            return response
        spin()
    raise AssertionError("production cost evidence deadline expired")


def audit_leases(rows, *, clearances=(), samples=(), bounds=None, physical_radius=0.15):
    """Prove exact capacity-one identities, not merely a grant count."""
    current, identities, grants = {}, {}, {}
    for row in rows:
        resource, evidence = row["resource_id"], row["evidence"]
        lease = evidence.get("lease")
        former = evidence.get("former_leases") or (
            [evidence["former_lease"]] if evidence.get("former_lease") else []
        )
        assert not (lease and former), f"authority overlaps quarantine: {resource}"
        token = lease["lease_id"] if lease else None
        previous = current.get(resource)
        if previous not in (None, token) and token is not None:
            identity = identities[previous]
            assert (*identity, previous) in clearances, (
                f"atomic handoff without exact release: {resource} {previous}"
            )
            before = [
                s for s in samples if 0 <= row["timestamp"] - s["monotonic"] <= 1.0
            ]
            assert before and bounds and resource in bounds, (
                "missing fresh pre-handoff world sample"
            )
            pose = before[-1]["robots"][identity[1]]
            left, bottom, right, top = bounds[resource]
            distance = math.hypot(
                max(left - pose["x"], 0, pose["x"] - right),
                max(bottom - pose["y"], 0, pose["y"] - top),
            )
            assert distance > physical_radius, (
                f"physical owner has not cleared {resource}"
            )
        for claim in ([lease] if lease else []) + list(former):
            identity = tuple(
                claim[k] for k in ("resource_id", "robot_id", "mission_id")
            )
            assert identity[0] == resource and all(identity) and claim["lease_id"]
            assert identities.setdefault(claim["lease_id"], identity) == identity, (
                "lease token reused for another exact identity"
            )
        if token and token != previous:
            grants.setdefault(resource, []).append(token)
        current[resource] = token
    return grants


class Owned:
    def __init__(self, output, deadline):
        self.output, self.deadline, self.children = output, deadline, []
        self.container = None
        self.broker_name = None
        self.broker_owner = None
        self.cleanup = []

    @staticmethod
    def exact_container(value):
        return len(value) == 64 and all(c in "0123456789abcdef" for c in value)

    def create_broker(self, port):
        # Record an unguessable ownership label before the interruptible call.
        # A timeout can occur after Docker creates the container but before the
        # client returns its ID. Cleanup must resolve that exact owned object.
        self.broker_owner = uuid.uuid4().hex
        self.broker_name = "factory-fleet-" + self.broker_owner
        result = subprocess.run(
            [
                "docker",
                "run",
                "-d",
                "--name",
                self.broker_name,
                "--label",
                "factory.fleet.owner=" + self.broker_owner,
                "-p",
                f"127.0.0.1:{port}:1883",
                "-v",
                str(ROOT / "docker/mosquitto.conf")
                + ":/mosquitto/config/mosquitto.conf:ro",
                "eclipse-mosquitto:2.0.22",
            ],
            capture_output=True,
            text=True,
            timeout=min(60, max(0.1, self.deadline - time.monotonic())),
        )
        (self.output / "broker.log").write_text(result.stdout + result.stderr)
        assert result.returncode == 0, "owned Mosquitto startup failed"
        candidate = result.stdout.strip()
        assert self.exact_container(candidate), (
            "missing exact owned broker container ID"
        )
        self.container = candidate

    def resolve_broker(self):
        if not self.broker_name or self.container:
            return
        result = subprocess.run(
            ["docker", "inspect", self.broker_name],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            self.cleanup.append(dict(container_name=self.broker_name, absent=True))
            return
        objects = json.loads(result.stdout)
        if (
            len(objects) != 1
            or objects[0].get("Config", {}).get("Labels", {}).get("factory.fleet.owner")
            != self.broker_owner
            or not self.exact_container(objects[0].get("Id", ""))
        ):
            raise RuntimeError("broker ownership mismatch; cleanup refused")
        self.container = objects[0]["Id"]

    @staticmethod
    def group_alive(group):
        # A dead wrapper does not prove that its process group is clear. Linux
        # may retain reparented zombies; those cannot perform work or hold ROS.
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry / "stat").read_text().rsplit(") ", 1)[1].split()
                if int(fields[2]) == group and fields[0] != "Z":
                    return True
            except (FileNotFoundError, ProcessLookupError):
                continue
        return False

    def start(self, name, command, environment):
        handle = (self.output / (name + ".log")).open("w")
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.children.append((name, process, handle))
        return process

    def close(self):
        previous = {
            sig: signal.signal(sig, signal.SIG_IGN)
            for sig in (signal.SIGINT, signal.SIGTERM)
        }
        try:
            self._close()
        finally:
            write_json(self.output / "cleanup.json", self.cleanup)
            for sig, handler in previous.items():
                signal.signal(sig, handler)

    def _close(self):
        for name, process, handle in reversed(self.children):
            escalated = False
            group_alive = self.group_alive(process.pid)
            if group_alive:
                for sig, bound in (
                    (signal.SIGINT, 12),
                    (signal.SIGTERM, 5),
                    (signal.SIGKILL, 2),
                ):
                    try:
                        if (
                            name == "launch"
                            and sig == signal.SIGINT
                            and process.poll() is None
                        ):
                            # ros2 launch propagates its first interrupt itself.
                            # Group-wide SIGINT here would interrupt every child
                            # twice during its ROS cleanup. Escalation still owns
                            # the exact group, including orphaned descendants.
                            process.send_signal(sig)
                        else:
                            os.killpg(process.pid, sig)
                    except ProcessLookupError:
                        break
                    end = time.monotonic() + bound
                    while self.group_alive(process.pid) and time.monotonic() < end:
                        process.poll()
                        time.sleep(0.05)
                    if not self.group_alive(process.pid):
                        process.wait(timeout=1)
                        break
                    escalated = True
            handle.close()
            receipt = dict(
                name=name,
                pid=process.pid,
                process_group=process.pid,
                exit_code=process.poll(),
                escalated=escalated,
                group_clear=not self.group_alive(process.pid),
            )
            if name == "launch":
                receipt.update(
                    launch_child_receipt(
                        (self.output / "launch.log").read_text(), process.poll()
                    )
                )
            self.cleanup.append(receipt)
        self.resolve_broker()
        if self.container:
            logs = subprocess.run(
                ["docker", "logs", self.container],
                capture_output=True,
                text=True,
                timeout=10,
            )
            (self.output / "broker-runtime.log").write_text(logs.stdout + logs.stderr)
            result = subprocess.run(
                ["docker", "rm", "-f", self.container],
                capture_output=True,
                text=True,
                timeout=15,
            )
            self.cleanup.append(
                dict(
                    container=self.container,
                    exit_code=result.returncode,
                    output=result.stdout + result.stderr,
                )
            )

    def check(self):
        if time.monotonic() >= self.deadline:
            raise TimeoutError("fleet run deadline expired")
        for name, child, _ in self.children:
            if child.poll() is not None:
                raise RuntimeError(
                    f"owned {name} exited early with {child.returncode}; inspect {name}.log"
                )


def prepare(config, output, scenario):
    source = ROOT / "src/factory_bringup/config/fleet.yaml"
    value = yaml.safe_load(source.read_text())
    for robot in value["robots"]:
        robot["battery_start_percent"] = scenario.get("initial_battery", {}).get(
            robot["robot_id"], robot["battery_start_percent"]
        )
    fleet = output / "fleet.yaml"
    fleet.write_text(yaml.safe_dump(value))
    world = ET.parse(ROOT / "src/factory_simulation/worlds/factory_floor.world")
    root = world.getroot().find("world")
    template = next(
        item
        for item in root.findall("include")
        if item.findtext("name") == "factory_part"
    )
    root.remove(template)
    for robot in config.robots:
        item = copy.deepcopy(template)
        item.find("name").text = robot.robot_id + "_part"
        root.append(item)
    # The V1 obstacle starts at the V2 dock. Preserve the obstacle, put it in
    # an unused upper corner, clear of dock exit bays; retain all collision geometry.
    root.find("model[@name='dynamic_obstacle']/pose").text = "5 3 0.25 0 0 0"
    # A visible simulation-only pad marks independently sensed dock geometry.
    dock = config.docks[config.energy.dock_id].charging_pose
    pad = ET.fromstring(
        '<model name="charging_dock"><static>true</static><pose>0 0 0.01 0 0 0</pose><link name="pad"><visual name="pad"><geometry><box><size>0.6 0.6 0.02</size></box></geometry><material><ambient>0.1 0.8 0.25 1</ambient><diffuse>0.1 0.8 0.25 1</diffuse></material></visual></link></model>'
    )
    pad.find("pose").text = f"{dock.x} {dock.y} 0.01 0 0 0"
    root.append(pad)
    gui = root.find("gui")
    if gui is None:
        gui = ET.SubElement(root, "gui")
    camera = ET.SubElement(gui, "camera", name="fleet_evidence_camera")
    ET.SubElement(camera, "pose").text = "0 -8 9 0 0.85 1.5707963267948966"
    ET.SubElement(camera, "view_controller").text = "orbit"
    world_path = output / "fleet.world"
    world.write(world_path, encoding="utf-8", xml_declaration=True)
    return fleet, world_path


def gazebo_scenario(name, output, headless, deadline):
    from factory_interfaces.action import ExecuteFactoryMission
    from factory_interfaces.msg import RobotState
    from factory_interfaces.srv import EstimateMissionCost
    from factory_simulation.entity_probe import WorldPoseProbe
    from fleet_manager.config import load_fleet_config
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Path as NavigationPath
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool, String

    scenario = yaml.safe_load((ROOT / f"tests/scenarios/fleet_{name}.yaml").read_text())
    config = load_fleet_config(ROOT / "src/factory_bringup/config/fleet.yaml")
    if name == "charging":
        low_robot, worker = charging_roles(config, scenario["initial_battery"])
    output.mkdir()
    domain = Domain()
    broker_port, plc_port, control_port, gazebo_port, dashboard_port = [
        free_port() for _ in range(5)
    ]
    assert len({broker_port, plc_port, control_port, gazebo_port, dashboard_port}) == 5
    environment = {
        **os.environ,
        "ROS_DOMAIN_ID": str(domain.id),
        "ROS_LOCALHOST_ONLY": "1",
        "PYTHONNOUSERSITE": "1",
        "GAZEBO_MASTER_URI": f"http://127.0.0.1:{gazebo_port}",
        "DISPLAY": os.environ.get("DISPLAY", ":0"),
    }
    owned = Owned(output, min(deadline, time.monotonic() + scenario["timeout_sec"]))
    # entity_probe owns a temporary global-executor spin. Use the matching
    # default context; each scenario shuts it down before creating the next.
    rclpy.init(domain_id=domain.id)
    node = rclpy.create_node("fleet_acceptance_probe")
    world_probe = WorldPoseProbe(node)
    states, contacts, payloads, statuses, samples, snapshots = {}, {}, [], [], [], []
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, client_id="fleet-proof-" + uuid.uuid4().hex
    )
    ingress = dict(subscribed=False, availability=None, subscription_mid=None)

    def mqtt_connected(c, userdata, flags, reason, properties):
        ingress.update(subscribed=False, availability=None)
        if reason == 0:
            result, mid = c.subscribe(
                [("factory/missions/+/status", 1), ("factory/fleet/availability", 1)]
            )
            ingress["subscription_mid"] = (
                mid if result == mqtt.MQTT_ERR_SUCCESS else None
            )

    def mqtt_subscribed(c, userdata, mid, reasons, properties):
        if mid == ingress["subscription_mid"]:
            ingress["subscribed"] = len(reasons) == 2 and all(
                not reason.is_failure for reason in reasons
            )

    def mqtt_message(c, userdata, message):
        if message.topic == "factory/fleet/availability":
            ingress["availability"] = message.payload.decode("ascii")
        else:
            statuses.append(json.loads(message.payload))

    client.on_connect = mqtt_connected
    client.on_subscribe = mqtt_subscribed
    client.on_message = mqtt_message
    client.on_disconnect = lambda *_: ingress.update(
        subscribed=False, availability=None
    )
    subscriptions = []
    plan_log = (output / "navigation-plans.jsonl").open("w")

    def observe_plan(robot_id, message):
        row = navigation_plan_evidence(
            robot_id, message, receipt_monotonic=time.monotonic()
        )
        plan_log.write(json.dumps(row) + "\n")
        plan_log.flush()

    for robot in config.robots:
        subscriptions.append(
            node.create_subscription(
                NavigationPath,
                robot.namespace + "/plan",
                lambda m, rid=robot.robot_id: observe_plan(rid, m),
                10,
            )
        )
        subscriptions.append(
            node.create_subscription(
                RobotState,
                robot.namespace + "/factory/robot_state",
                lambda m, rid=robot.robot_id: states.__setitem__(rid, m),
                qos_profile_sensor_data,
            )
        )
        subscriptions.append(
            node.create_subscription(
                Bool,
                robot.namespace + "/factory/dock_contact",
                lambda m, rid=robot.robot_id: contacts.__setitem__(rid, m.data),
                10,
            )
        )
    from rclpy.qos import DurabilityPolicy, QoSProfile

    subscriptions.append(
        node.create_subscription(
            String,
            "/factory/payload_state",
            lambda m: payloads.append(json.loads(m.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
    )
    lifecycle_clients = [
        node.create_client(GetState, robot.namespace + "/" + server + "/get_state")
        for robot in config.robots
        for server in (
            "map_server",
            "amcl",
            "controller_server",
            "smoother_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "velocity_smoother",
        )
    ]
    endpoint = f"http://127.0.0.1:{dashboard_port}"
    connected = False
    receipt = None
    initial_costs, expected_first_robot = {}, None
    started = time.monotonic()
    try:
        fleet_file, world = prepare(config, output, scenario)
        owned.create_broker(broker_port)
        owned.start(
            "plc",
            [
                sys.executable,
                "-m",
                "plc_simulator.server",
                "--port",
                str(plc_port),
                "--fault-control-port",
                str(control_port),
            ],
            environment,
        )
        owned.start(
            "launch",
            [
                "ros2",
                "launch",
                "factory_bringup",
                "fleet_demo.launch.py",
                f"fleet_file:={fleet_file}",
                f"world:={world}",
                f"gui:={'false' if headless else 'true'}",
                "rviz:=false",
                f"broker_port:={broker_port}",
                f"plc_port:={plc_port}",
                f"control_port:={control_port}",
                f"output_dir:={output}",
                f"journal_path:={output / 'fleet.sqlite3'}",
                f"dashboard_port:={dashboard_port}",
            ],
            environment,
        )
        write_json(
            output / "run.json",
            dict(
                scenario=name,
                ros_domain_id=domain.id,
                ports=dict(
                    broker=broker_port,
                    plc=plc_port,
                    control=control_port,
                    gazebo=gazebo_port,
                    dashboard=dashboard_port,
                ),
                dashboard_url=endpoint,
                config_sha256=hashlib.sha256(
                    (ROOT / "src/factory_bringup/config/fleet.yaml").read_bytes()
                ).hexdigest(),
                headless=headless,
            ),
        )
        print(
            f"{name}: {output}; dashboard {endpoint}; ROS domain {domain.id}",
            flush=True,
        )
        ready = set()
        startup_deadline = min(owned.deadline, started + 100)
        while len(ready) < len(lifecycle_clients) or len(states) < len(config.robots):
            owned.check()
            assert time.monotonic() < startup_deadline, (
                f"startup deadline: active={len(ready)}/18 states={list(states)}"
            )
            rclpy.spin_once(node, timeout_sec=0.02)
            for index, service in enumerate(lifecycle_clients):
                if index in ready or not service.service_is_ready():
                    continue
                future = service.call_async(GetState.Request())
                end = min(startup_deadline, time.monotonic() + 1)
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)
                if future.done() and future.result().current_state.id == 3:
                    ready.add(index)
        while not connected:
            owned.check()
            try:
                client.connect("127.0.0.1", broker_port, 20)
                client.loop_start()
                connected = True
            except OSError:
                rclpy.spin_once(node, timeout_sec=0.1)
        # Wait for the MQTT subscription acknowledgement and reconciled fleet.
        while True:
            owned.check()
            assert time.monotonic() < startup_deadline, (
                "mission ingress startup deadline"
            )
            rclpy.spin_once(node, timeout_sec=0.02)
            try:
                with urlopen(endpoint + "/api/snapshot", timeout=1) as response:
                    snapshot = json.load(response)
                if mission_ingress_ready(snapshot, ingress) and manager_dispatch_ready(
                    snapshot["robots"], {robot.robot_id for robot in config.robots}
                ):
                    break
            except OSError:
                pass
        write_json(output / "startup-snapshot.json", snapshot)
        # Nav2 ACTIVE and cost feasibility do not prove that a coordinator's
        # robot-local mission server has reached DDS discovery. The production
        # manager correctly excludes that robot until both endpoints are ready.
        dispatch_clients, dispatch_actions, endpoints = [], [], {}
        try:
            for robot in config.robots:
                cost_name = robot.namespace + "/factory/estimate_mission_cost"
                action_name = robot.namespace + "/factory/execute_mission"
                service = node.create_client(EstimateMissionCost, cost_name)
                action = ActionClient(node, ExecuteFactoryMission, action_name)
                dispatch_clients.append(service)
                dispatch_actions.append(action)
                endpoints[cost_name] = service.service_is_ready
                endpoints[action_name] = action.server_is_ready
            write_json(
                output / "dispatch-endpoints.json",
                wait_dispatch_endpoints(
                    endpoints,
                    lambda: rclpy.spin_once(node, timeout_sec=0.02),
                    owned.check,
                    startup_deadline,
                ),
            )
        finally:
            for action in dispatch_actions:
                action.destroy()
            for service in dispatch_clients:
                node.destroy_client(service)
        if name == "nominal":
            mission = scenario["missions"][0]
            for robot in config.robots:
                service = node.create_client(
                    EstimateMissionCost,
                    robot.namespace + "/factory/estimate_mission_cost",
                )
                end = min(owned.deadline, time.monotonic() + 10)
                while not service.service_is_ready() and time.monotonic() < end:
                    owned.check()
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert service.service_is_ready(), (
                    "production cost endpoint unavailable"
                )
                cost = read_cost(
                    service,
                    EstimateMissionCost.Request(
                        mission_id=mission["mission_id"],
                        pickup_station=mission["pickup"],
                        dropoff_station=mission["dropoff"],
                        part=mission["part"],
                    ),
                    lambda: rclpy.spin_once(node, timeout_sec=0.02),
                    owned.check,
                    end,
                )
                initial_costs[robot.robot_id] = dict(
                    feasible=cost.feasible,
                    path_cost=cost.path_cost,
                    predicted_final_battery=float(cost.predicted_final_battery),
                    reason=cost.reason,
                )
                node.destroy_client(service)
            available = {
                r["robot_id"]
                for r in snapshot["robots"]
                if r["health"] == "ONLINE"
                and r["mode"] == "AVAILABLE"
                and r["payload_state"] == "EMPTY"
                and not r["mission_id"]
            }
            write_json(output / "initial-costs.json", initial_costs)
            expected_first_robot = cost_winner(initial_costs, available)
        for mission in scenario["missions"]:
            message = client.publish(
                "factory/missions/request", json.dumps(mission), qos=1
            )
            message.wait_for_publish(timeout=2)
        with (
            (output / "world-poses.jsonl").open("w") as world_log,
            (output / "fleet-snapshots.jsonl").open("w") as snapshot_log,
        ):
            charged, contacted, cleared = False, False, False
            charging_robots = {
                r.robot_id: dict(
                    contact=False, reached_80=False, verified_clearance=False
                )
                for r in config.robots
            }
            post_mission_low = False
            last_sample = 0
            while True:
                owned.check()
                rclpy.spin_once(node, timeout_sec=0.02)
                if time.monotonic() - last_sample < 0.2:
                    continue
                last_sample = time.monotonic()
                poses, physical = {}, {}
                for robot in config.robots:
                    pose = world_probe.pose(robot.robot_id, timeout_sec=1)
                    poses[robot.robot_id] = pose
                    physical[robot.robot_id] = world_pose_evidence(
                        pose,
                        receipt_monotonic=time.monotonic(),
                    )
                row = dict(
                    monotonic=time.monotonic(),
                    robots=physical,
                    contact=dict(contacts),
                    battery={
                        rid: float(s.battery_percent) for rid, s in states.items()
                    },
                )
                # Planned clearance includes arrival error. Actual sampled
                # non-overlap uses physical footprint only (0.30 m supplied).
                record_world_sample(
                    world_log,
                    samples,
                    row,
                    robot_radius=max(d.robot_radius for d in config.docks.values()),
                )
                for resource, bounds in {
                    **config.traffic_bounds,
                    **config.resource_bounds,
                }.items():
                    inside = [
                        rid
                        for rid, p in poses.items()
                        if bounds[0] - 0.15 <= p.position.x <= bounds[2] + 0.15
                        and bounds[1] - 0.15 <= p.position.y <= bounds[3] + 0.15
                    ]
                    assert len(inside) <= 1, (
                        f"physical capacity-one violation {resource}: {inside}"
                    )
                dock = config.docks[config.energy.dock_id]
                inside_dock = [
                    rid
                    for rid, p in poses.items()
                    if math.hypot(
                        p.position.x - dock.charging_pose.x,
                        p.position.y - dock.charging_pose.y,
                    )
                    <= dock.arrival_tolerance + 0.15
                ]
                assert len(inside_dock) <= 1, f"physical dock overlap: {inside_dock}"
                with urlopen(endpoint + "/api/snapshot", timeout=1) as response:
                    snapshot = json.load(response)
                snapshots.append(snapshot)
                snapshot_log.write(json.dumps(snapshot) + "\n")
                snapshot_log.flush()
                terminal = {
                    s["mission_id"]: s
                    for s in statuses
                    if s["state"] in ("COMPLETED", "FAILED", "RECOVERY_REQUIRED")
                }
                if any(s["state"] != "COMPLETED" for s in terminal.values()):
                    raise AssertionError(f"unexpected mission failure: {terminal}")
                if name == "charging":
                    update_charging_proof(
                        charging_robots,
                        states,
                        contacts,
                        inside_dock,
                        snapshot["resources"],
                        config.energy.dock_id,
                    )
                    post_mission_low |= (
                        "fleet-charge-work" in terminal
                        and not states[worker].mission_id
                        and states[worker].battery_percent < 30.0
                    )
                    contacted = all(p["contact"] for p in charging_robots.values())
                    charged = all(p["reached_80"] for p in charging_robots.values())
                    cleared = all(
                        p["verified_clearance"] for p in charging_robots.values()
                    )
                if len(terminal) == len(scenario["missions"]) and (
                    name != "charging" or (contacted and charged and cleared)
                ):
                    break
        assigned = {mid: s["robot_id"] for mid, s in terminal.items()}
        if name in ("nominal", "contention"):
            assert len(set(assigned.values())) == 2
        if name == "nominal":
            assert (
                assigned[scenario["missions"][0]["mission_id"]] == expected_first_robot
            ), "automatic assignment differs from eligible production cost ordering"
        if name == "contention":
            assert assigned["M-001"] == "amr_01"
        if name == "charging":
            assert assigned["fleet-charge-work"] == worker
        for mid, rid in assigned.items():
            assert any(
                p["mission_id"] == mid
                and p["robot_id"] == rid
                and p["state"] == "IN_TRANSIT"
                for p in payloads
            ), f"missing confirmed carrying {mid}"
            assert any(
                p["mission_id"] == mid
                and p["robot_id"] == rid
                and p["state"] == "AT_INSPECTION"
                for p in payloads
            ), f"missing confirmed delivery {mid}"
            part = world_probe.pose(rid + "_part", timeout_sec=2)
            assert abs(part.position.x - 3) < 0.2 and abs(part.position.y - 2) < 0.3
        with sqlite3.connect(
            f"file:{output / 'fleet.sqlite3'}?mode=ro", uri=True
        ) as database:
            leases = [
                dict(
                    sequence=sequence,
                    resource_id=resource,
                    evidence=json.loads(evidence),
                    timestamp=stamp,
                )
                for sequence, resource, evidence, stamp in database.execute(
                    "SELECT sequence,resource_id,evidence_json,timestamp FROM resource_events ORDER BY sequence"
                )
            ]
            clearances = list(
                database.execute(
                    "SELECT resource_id,robot_id,mission_id,evidence_id FROM resource_claim_resolutions"
                )
            )
        assert leases and any(row["evidence"].get("lease") for row in leases), (
            "missing exact lease journal"
        )
        write_json(output / "lease-history.json", leases)
        lease_grants = audit_leases(
            leases,
            clearances=clearances,
            samples=samples,
            bounds={
                **config.traffic_bounds,
                **config.resource_bounds,
                **{
                    key: (
                        dock.charging_pose.x - dock.arrival_tolerance,
                        dock.charging_pose.y - dock.arrival_tolerance,
                        dock.charging_pose.x + dock.arrival_tolerance,
                        dock.charging_pose.y + dock.arrival_tolerance,
                    )
                    for key, dock in config.docks.items()
                },
            },
            physical_radius=max(d.robot_radius for d in config.docks.values()),
        )
        write_json(output / "exact-clearances.json", clearances)
        if name == "charging":
            assert post_mission_low, (
                "working robot never becomes charge-eligible after delivery"
            )
            dock_grants = lease_grants.get(config.energy.dock_id, [])
            assert len(dock_grants) == 2 and len(set(dock_grants)) == 2, (
                "missing exact two-robot dock handoff"
            )
            dock_owners = [
                row["evidence"]["lease"]["robot_id"]
                for row in leases
                if row["resource_id"] == config.energy.dock_id
                and row["evidence"].get("lease")
                and row["evidence"]["lease"]["lease_id"] in dock_grants
            ]
            assert list(dict.fromkeys(dock_owners)) == [low_robot, worker]
        write_json(output / "mqtt-statuses.json", statuses)
        write_json(output / "payload-states.json", payloads)
        assert (output / "protocol_events.jsonl").stat().st_size > 0
        receipt = dict(
            scenario=name,
            success=True,
            assignments=assigned,
            initial_costs=initial_costs,
            expected_first_robot=expected_first_robot,
            active_nav2_nodes=len(ready),
            world_samples=len(samples),
            exact_lease_grants=lease_grants,
            charging=dict(
                contact=contacted,
                reached_80=charged,
                verified_clearance=cleared,
                robots=charging_robots,
                post_mission_below_30=post_mission_low,
            ),
            elapsed_sec=time.monotonic() - started,
            assertions=scenario["assertions"],
        )
        write_json(output / "receipt.json", receipt)
    finally:
        write_json(output / "mqtt-statuses.json", statuses)
        write_json(output / "payload-states.json", payloads)
        if connected:
            client.disconnect()
            client.loop_stop()
        world_probe.close()
        plan_log.close()
        node.destroy_node()
        rclpy.try_shutdown()
        owned.close()
        domain.close()
    assert receipt is not None
    launched = next(r for r in owned.cleanup if r.get("name") == "launch")
    assert launched["all_children_clean"], f"unclean launch teardown: {launched}"
    assert all(r.get("group_clear", True) for r in owned.cleanup), (
        "owned process group remains live"
    )
    return receipt


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("mode", choices=("demo", "acceptance"))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--scenario",
        choices=("all", "nominal", "contention", "robot_failure", "charging", "scale"),
        default="all",
    )
    parser.add_argument("--timeout", type=int, default=900)
    parser.add_argument("--output", type=Path)
    options = parser.parse_args()
    root = options.output or ROOT / "artifacts/fleet"
    root.mkdir(parents=True, exist_ok=True)
    output = Path(
        tempfile.mkdtemp(
            prefix=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-")
            + options.mode
            + "-",
            dir=root,
        )
    ).resolve()
    print(f"Fleet artifacts: {output}", flush=True)
    deadline = time.monotonic() + options.timeout
    receipts = []

    def interrupted(signum, frame):
        raise InterruptedError(f"fleet run interrupted by signal {signum}")

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        scenarios = (
            ["charging", "nominal"]
            if options.mode == "demo"
            else (
                ["nominal", "contention", "robot_failure", "charging", "scale"]
                if options.scenario == "all"
                else [options.scenario]
            )
        )
        for scenario in scenarios:
            if scenario in ("scale", "robot_failure"):
                destination = output / scenario
                destination.mkdir()
                script = (
                    "fleet_scale_probe.py"
                    if scenario == "scale"
                    else "fleet_failure_probe.py"
                )
                owned = Owned(destination, min(deadline, time.monotonic() + 65))
                run = owned.start(
                    "probe",
                    [
                        sys.executable,
                        str(ROOT / "tests/system" / script),
                        str(destination),
                    ],
                    dict(os.environ),
                )
                try:
                    run.wait(timeout=max(0.1, owned.deadline - time.monotonic()))
                finally:
                    owned.close()
                assert all(r.get("group_clear") for r in owned.cleanup), (
                    f"{scenario} owned process group remains live"
                )
                assert run.returncode == 0, (
                    f"{scenario} proof failed; inspect {destination}"
                )
                path = destination / (
                    "ten-agent-receipt.json"
                    if scenario == "scale"
                    else "failure-receipt.json"
                )
                assert path.is_file(), f"missing {scenario} receipt"
                receipts.append(json.loads(path.read_text()))
            else:
                receipts.append(
                    gazebo_scenario(
                        scenario, output / scenario, options.headless, deadline
                    )
                )
        write_json(output / "summary.json", dict(success=True, receipts=receipts))
        print(f"PASS fleet proof: {output}", flush=True)
    except BaseException as error:
        write_json(
            output / "summary.json",
            dict(success=False, error=str(error), receipts=receipts),
        )
        traceback.print_exc()
        return (
            130
            if isinstance(error, InterruptedError)
            else 124
            if isinstance(error, (TimeoutError, subprocess.TimeoutExpired))
            else 1
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
