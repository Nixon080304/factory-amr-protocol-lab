# SPDX-License-Identifier: Apache-2.0
"""Real rendered Gazebo, Nav2, MQTT and PLC acceptance test; no fake peers."""

import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
import uuid

import paho.mqtt.client as mqtt
from pymodbus.client import ModbusTcpClient


ROOT = Path(__file__).resolve().parents[2]


def free_port():
    with socket.socket() as connection:
        connection.bind(("127.0.0.1", 0))
        return connection.getsockname()[1]


def stamp(message):
    return message.sec + message.nanosec / 1e9


def yaw(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y**2 + quaternion.z**2),
    )


def test_successful_factory_mission():
    assert (ROOT / "scripts/run_demo.sh").is_file(), "full demo script missing"
    assert (ROOT / "src/factory_bringup/launch/demo.launch.py").is_file(), (
        "full bringup missing"
    )
    # Imports occur after the environment is isolated, before initializing DDS.
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("FACTORY_SCENARIO_DOMAIN", "80")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    from factory_interfaces.msg import ProtocolEvent, StationDetection
    import cv2
    from cv_bridge import CvBridge
    from factory_simulation.entity_probe import get_entity_pose
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker

    output = (
        Path(os.environ["FACTORY_SCENARIO_OUTPUT"])
        if "FACTORY_SCENARIO_OUTPUT" in os.environ
        else (
            ROOT
            / ".superpowers/sdd/2026-10-02-factory-amr-02-simulation-autonomy/evidence"
            / ("system-" + uuid.uuid4().hex)
        )
    )
    output.mkdir(parents=True, exist_ok=True)
    ports = set()
    while len(ports) < 3:
        ports.add(free_port())
    broker_port, plc_port, gazebo_port = ports
    environment = {
        **os.environ,
        "FACTORY_ROS_DOMAIN_ID": os.environ["ROS_DOMAIN_ID"],
        "ROS_LOCALHOST_ONLY": "1",
        "FACTORY_MQTT_PORT": str(broker_port),
        "FACTORY_PLC_PORT": str(plc_port),
        "FACTORY_COMPOSE_PROJECT": "factory-system-" + uuid.uuid4().hex,
        "GAZEBO_MASTER_URI": f"http://127.0.0.1:{gazebo_port}",
        "FACTORY_OUTPUT_DIR": str(output),
        "DISPLAY": os.environ.get("DISPLAY", ":0"),
    }
    preexisting_containers = set(
        subprocess.check_output(["docker", "ps", "-aq"], text=True).split()
    )
    events, detections, payloads, poses, states, images, statuses, availability = (
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
    )
    goals, markers = [], []
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, client_id="system-" + uuid.uuid4().hex
    )

    def on_connect(client, userdata, flags, reason, properties):
        client.subscribe("factory/missions/+/status", qos=1)
        client.subscribe("factory/robots/amr_01/availability", qos=1)

    def on_message(client, userdata, message):
        if message.topic.endswith("/status"):
            statuses.append(json.loads(message.payload))
        else:
            availability.append(message.payload.decode())

    client.on_connect, client.on_message = on_connect, on_message
    rclpy.init()
    node = rclpy.create_node(
        "factory_system_probe",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    localization_samples = []
    rendered_images = {}
    bridge = CvBridge()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = (
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "DetectorParameters")
        else cv2.aruco.DetectorParameters_create()
    )
    parameters.minMarkerDistanceRate = 0.03

    def camera(message):
        source = stamp(message.header.stamp)
        images.append(source)
        assert (message.width, message.height) == (640, 480)
        assert message.header.frame_id == "camera_optical_frame"
        pixels = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        _, ids, _ = cv2.aruco.detectMarkers(pixels, dictionary, parameters=parameters)
        if ids is not None:
            rendered_images[source] = (
                set(int(value) for value in ids.reshape(-1)),
                pixels,
            )

    def localization(message):
        poses.append(message)
        localization_samples.append(
            {
                "stamp": stamp(message.header.stamp),
                "clock": node.get_clock().now().nanoseconds / 1e9,
                "x": message.pose.pose.position.x,
                "y": message.pose.pose.position.y,
                "yaw": yaw(message.pose.pose.orientation),
            }
        )

    _subscriptions = [
        node.create_subscription(
            ProtocolEvent, "/factory/protocol_events", events.append, 100
        ),
        node.create_subscription(
            StationDetection,
            "/factory/station_detection",
            detections.append,
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            String,
            "/factory/payload_state",
            lambda message: payloads.append(json.loads(message.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ),
        node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            localization,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ),
        node.create_subscription(
            String,
            "/factory/mission_state",
            lambda message: states.append(message.data),
            100,
        ),
        node.create_subscription(
            Image,
            "/camera/image_raw",
            camera,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
        ),
        node.create_subscription(
            PoseStamped,
            "/factory/navigation_goal",
            goals.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ),
        node.create_subscription(
            Marker, "/factory/station_markers", markers.append, 10
        ),
    ]
    log = (output / "demo.log").open("w")
    if "FACTORY_SCENARIO_OUTPUT" in os.environ:
        from test_protocol_faults import record_owned_resource, record_owned_process

        existing = subprocess.check_output(
            [
                "docker",
                "ps",
                "-aq",
                "--filter",
                "label=com.docker.compose.project="
                + environment["FACTORY_COMPOSE_PROJECT"],
            ],
            text=True,
        )
        assert not existing.strip(), "fresh scenario Compose project already exists"
        record_owned_resource(
            "compose_projects", environment["FACTORY_COMPOSE_PROJECT"]
        )
        for port in (broker_port, plc_port, gazebo_port):
            record_owned_resource("ports", port)
    process = subprocess.Popen(
        [str(ROOT / "scripts/run_demo.sh"), "--headless"],
        cwd=ROOT,
        env=environment,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    if "FACTORY_SCENARIO_OUTPUT" in os.environ:
        record_owned_process(process)
    deadline = time.monotonic() + 270
    previous_termination = signal.getsignal(signal.SIGTERM)

    def terminate_owned_harness(signum, frame):
        raise TimeoutError("outer deadline terminated the owned system harness")

    signal.signal(signal.SIGTERM, terminate_owned_harness)

    def wait(predicate, seconds, description):
        end = min(deadline, time.monotonic() + seconds)
        while not predicate() and time.monotonic() < end:
            assert process.poll() is None, (
                f"demo exited {process.returncode}; log={output / 'demo.log'}"
            )
            rclpy.spin_once(node, timeout_sec=0.05)
        assert predicate(), f"{description}; log={output / 'demo.log'}"

    def plc_snapshot():
        with ModbusTcpClient("127.0.0.1", port=plc_port, timeout=2) as plc:
            values = []
            for unit in (1, 2):
                registers = plc.read_holding_registers(0, count=4, device_id=unit)
                coils = plc.read_coils(0, count=5, device_id=unit)
                assert not registers.isError() and not coils.isError()
                values.append((registers.registers, coils.bits[:5]))
            return values

    def reset_scenario_controls():
        if "FACTORY_SCENARIO_OUTPUT" not in os.environ:
            return
        reset = node.create_client(Trigger, "/factory/faults/reset")
        try:
            wait(reset.service_is_ready, 10, "fault reset service missing")
            future = reset.call_async(Trigger.Request())
            wait(future.done, 10, "fault reset acknowledgement missing")
            assert future.result().success, future.result().message
        finally:
            node.destroy_client(reset)

    try:
        # Connect only once Compose exposes the owned broker, without touching
        # another project's broker or PLC.
        connected = False
        end = time.monotonic() + 90
        while not connected and time.monotonic() < end:
            try:
                client.connect("127.0.0.1", broker_port)
                connected = True
            except OSError:
                time.sleep(0.1)
        assert connected, f"broker missing; log={output / 'demo.log'}"
        client.loop_start()
        wait(
            lambda: "online" in availability and poses and images and payloads,
            75,
            "full system readiness missing",
        )
        for name in (
            "map_server",
            "amcl",
            "controller_server",
            "planner_server",
            "smoother_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "velocity_smoother",
        ):
            lifecycle = node.create_client(GetState, f"/{name}/get_state")
            try:
                wait(lifecycle.service_is_ready, 10, f"{name} service missing")

                def active():
                    future = lifecycle.call_async(GetState.Request())
                    end = min(deadline, time.monotonic() + 1)
                    while not future.done() and time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    if not future.done():
                        future.cancel()
                        return False
                    return (
                        future.result() is not None
                        and future.result().current_state.id == 3
                    )

                wait(active, 20, f"{name} inactive")
            finally:
                node.destroy_client(lifecycle)
        reset_scenario_controls()
        assert payloads[-1]["state"] == "AT_ASSEMBLY"
        before = plc_snapshot()
        assert [entry[0][2] for entry in before] == [0, 0]
        assert node.count_publishers("/odom") == 1
        assert node.count_publishers("/cmd_vel") == 1
        for topic in ("/camera/image_raw", "/camera/camera_info", "/scan"):
            endpoints = node.get_publishers_info_by_topic(topic)
            assert len(endpoints) == 1
            profile = endpoints[0].qos_profile
            assert profile.reliability == ReliabilityPolicy.BEST_EFFORT, topic
            assert profile.durability == DurabilityPolicy.VOLATILE, topic
        result = subprocess.run(
            [str(ROOT / "scripts/send_demo_mission.sh")],
            env=environment,
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=15,
        )
        assert result.returncode == 0, result.stderr
        assert json.loads(result.stdout) == {
            "mission_id": "M-001",
            "robot_id": "amr_01",
            "pickup": "assembly",
            "dropoff": "inspection",
            "part": "motor",
        }
        mission_deadline = min(deadline, time.monotonic() + 170)

        def phase_image_stamps(leg, station, marker):
            starts = [
                event for event in events if event.event == f"perception_{leg}_started"
            ]
            finishes = [
                event for event in events if event.event == f"perception_{leg}_finished"
            ]
            if len(starts) != 1 or len(finishes) != 1:
                return []
            return sorted(
                {
                    stamp(item.header.stamp)
                    for item in detections
                    if stamp(starts[0].stamp)
                    < stamp(item.header.stamp)
                    <= stamp(finishes[0].stamp)
                    and item.station_id == station
                    and item.marker_id == marker
                }
            )

        def rendered_overlap(times, marker):
            return [
                value
                for value in times[:5]
                if value in rendered_images and marker in rendered_images[value][0]
            ]

        # Independent best-effort subscribers need not receive identical subsets.
        # Correlate a decoded rendered frame exactly within the five-source window.
        for leg, station, marker in [
            ("pickup", "assembly", 10),
            ("dropoff", "inspection", 20),
        ]:
            wait(
                lambda: (
                    any(event.event == f"perception_{leg}_finished" for event in events)
                    or any(item["state"] == "FAILED" for item in statuses)
                ),
                max(0, mission_deadline - time.monotonic()),
                f"{station}: verification did not finish",
            )
            if any(item["state"] == "FAILED" for item in statuses):
                break

            def source_evidence_delivered():
                times = phase_image_stamps(leg, station, marker)
                return (
                    len(times) >= 5
                    and times[4] - times[0] <= 1.5
                    and bool(rendered_overlap(times, marker))
                )

            wait(
                source_evidence_delivered,
                2,
                f"{station}: five fresh detections need a decoded exact rendered source frame",
            )
        wait(
            lambda: any(item["state"] in ("COMPLETED", "FAILED") for item in statuses),
            max(0, mission_deadline - time.monotonic()),
            "mission did not terminate",
        )
        terminal = [
            item for item in statuses if item["state"] in ("COMPLETED", "FAILED")
        ]
        if not terminal or terminal[0]["state"] != "COMPLETED":
            diagnostic = {
                "localization": localization_samples,
                "states": states,
                "statuses": statuses,
            }
            robot = get_entity_pose(node, "factory_amr")
            diagnostic["world_pose"] = {
                "x": robot.position.x,
                "y": robot.position.y,
                "yaw": yaw(robot.orientation),
            }
            (output / "failure-proof.json").write_text(
                json.dumps(diagnostic, indent=2) + "\n"
            )
            print(json.dumps(diagnostic, indent=2))
        assert [item["state"] for item in terminal] == ["COMPLETED"], terminal
        assert terminal[0]["error_code"] is None
        wait(
            lambda: payloads[-1]["state"] == "AT_INSPECTION",
            10,
            "payload did not unload",
        )
        wait(
            lambda: (
                len([event for event in events if event.event == "mission_finished"])
                == 1
            ),
            5,
            "reliable terminal event missing",
        )
        expected = [
            "RECEIVED",
            "NAVIGATING_TO_PICKUP",
            "VERIFYING_PICKUP",
            "LOADING",
            "NAVIGATING_TO_DROPOFF",
            "VERIFYING_DROPOFF",
            "UNLOADING",
            "COMPLETED",
        ]
        assert states == expected, states
        assert [item["state"] for item in payloads] == [
            "AT_ASSEMBLY",
            "IN_TRANSIT",
            "AT_INSPECTION",
        ]
        assert sum(event.event == "mission_started" for event in events) == 1
        assert (
            sum(
                event.event == "mission_finished" and event.outcome == "COMPLETED"
                for event in events
            )
            == 1
        )
        wait(
            lambda: len(goals) == 2 and {marker.id for marker in markers} == {10, 20},
            5,
            "RViz goals or station identities missing",
        )
        assert [(goal.pose.position.x, goal.pose.position.y) for goal in goals] == [
            (-3.0, 0.8),
            (3.0, 0.8),
        ]
        evidence = {}
        for leg, station, marker in [
            ("pickup", "assembly", 10),
            ("dropoff", "inspection", 20),
        ]:
            start = [
                event for event in events if event.event == f"perception_{leg}_started"
            ]
            finish = [
                event for event in events if event.event == f"perception_{leg}_finished"
            ]
            assert len(start) == len(finish) == 1 and finish[0].outcome == "SUCCEEDED"
            observations = [
                item
                for item in detections
                if stamp(start[0].stamp)
                < stamp(item.header.stamp)
                <= stamp(finish[0].stamp)
                and item.station_id == station
                and item.marker_id == marker
            ]
            times = sorted(set(stamp(item.header.stamp) for item in observations))
            assert len(times) >= 5 and times[4] - times[0] <= 1.5, (station, times)
            wait(
                lambda: bool(rendered_overlap(times, marker)),
                2,
                f"{station}: confirmed window lacks a decoded exact rendered source frame",
            )
            overlaps = rendered_overlap(times, marker)
            image_path = output / f"{station}_{marker}_rendered.png"
            assert cv2.imwrite(str(image_path), rendered_images[overlaps[0]][1])
            transfer = [
                event for event in events if event.event == f"modbus_{leg}_finished"
            ]
            assert (
                len(transfer) == 1
                and transfer[0].protocol == "MODBUS"
                and transfer[0].outcome == "SUCCEEDED"
            )
            detail = json.loads(transfer[0].detail)
            assert detail == {
                "station_id": station,
                "transfer_kind": "LOADING" if leg == "pickup" else "UNLOADING",
                "cycle_counter": 1,
            }
            for phase in (
                "mqtt_acceptance",
                f"navigation_{leg}",
                f"perception_{leg}",
                f"modbus_{leg}",
            ):
                assert sum(event.event == phase + "_started" for event in events) == 1
                assert sum(event.event == phase + "_finished" for event in events) == 1
            evidence[station] = {
                "marker": marker,
                "fresh_detection_count": len(times),
                "image_stamps": times,
                "received_rendered_source_stamps": overlaps,
                "rendered_png": str(image_path),
                "window_sec": times[4] - times[0],
                "transfer": detail,
            }
        after = plc_snapshot()
        assert [entry[0][2] for entry in after] == [1, 1]
        assert all(
            registers[1] == 1 and not coils[1] and not coils[2]
            for registers, coils in after
        )
        # Let the final conveyor animation settle and verify independent world poses.
        wait(
            lambda: (
                node.get_clock().now().nanoseconds / 1e9 > stamp(events[-1].stamp) + 1.5
            ),
            5,
            "settling clock",
        )
        robot = get_entity_pose(node, "factory_amr")
        part = get_entity_pose(node, "factory_part")
        assert math.hypot(part.position.x - 3, part.position.y - 2) < 0.01
        assert abs(part.position.z - 0.65) < 0.01
        final = {}
        for name, pose in [("gazebo", robot), ("amcl", poses[-1].pose.pose)]:
            position_error = math.hypot(pose.position.x - 3, pose.position.y - 0.8)
            yaw_error = abs(
                math.atan2(
                    math.sin(yaw(pose.orientation) - math.pi / 2),
                    math.cos(yaw(pose.orientation) - math.pi / 2),
                )
            )
            assert position_error < 0.25 and yaw_error < 0.25, (
                name,
                position_error,
                yaw_error,
            )
            final[name] = {
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": yaw(pose.orientation),
                "position_error": position_error,
                "yaw_error": yaw_error,
            }
        trace = output / "protocol_events.jsonl"
        report = output / ("mission_" + hashlib.sha256(b"M-001").hexdigest() + ".md")
        wait(
            lambda: (
                report.exists()
                and "COMPLETED" in report.read_text()
                and "not available" not in report.read_text()
            ),
            5,
            "complete terminal observer report missing",
        )
        records = [json.loads(line) for line in trace.read_text().splitlines()]
        assert sum(record["event"] == "mission_started" for record in records) == 1
        assert sum(record["event"] == "mission_finished" for record in records) == 1
        assert [record["sequence"] for record in records] == list(
            range(1, len(records) + 1)
        )
        assert all(
            f"Modbus handshake: {leg}" in report.read_text()
            for leg in ("pickup", "dropoff")
        )
        # Re-deliver the identical standard JSON. The broker status replay must
        # not cause another action, PLC cycle, or payload transition.
        subprocess.run(
            [str(ROOT / "scripts/send_demo_mission.sh")],
            env=environment,
            cwd=ROOT,
            check=True,
            timeout=15,
        )
        wait(
            lambda: sum(item["state"] == "COMPLETED" for item in statuses) == 2,
            5,
            "duplicate did not replay final state",
        )
        wait(
            lambda: any(event.event == "mqtt_duplicate" for event in events),
            5,
            "duplicate event missing",
        )
        settle = time.monotonic() + 2
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert sum(event.event == "mission_started" for event in events) == 1
        assert plc_snapshot() == after
        assert len(payloads) == 3
        second = {
            "mission_id": "M-002",
            "robot_id": "amr_01",
            "pickup": "assembly",
            "dropoff": "inspection",
            "part": "motor",
        }
        client.publish(
            "factory/missions/request", json.dumps(second), qos=1
        ).wait_for_publish(timeout=5)
        wait(
            lambda: any(
                item["mission_id"] == "M-002" and item["state"] == "FAILED"
                for item in statuses
            ),
            5,
            "consumed payload did not require restart",
        )
        refused = [item for item in statuses if item["mission_id"] == "M-002"]
        assert [item["state"] for item in refused] == ["RECEIVED", "FAILED"], refused
        assert refused[-1]["error_code"] == "RESTART_REQUIRED"
        wait(
            lambda: any(
                event.mission_id == "M-002"
                and event.event == "mission_rejected"
                and event.detail == "RESTART_REQUIRED"
                for event in events
            ),
            5,
            "explicit lifecycle rejection missing",
        )
        assert sum(event.event == "mission_started" for event in events) == 1
        assert len(goals) == 2 and plc_snapshot() == after and len(payloads) == 3
        proof = {
            "output_dir": str(output),
            "states": states,
            "stations": evidence,
            "plc_before": before,
            "plc_after": after,
            "payloads": payloads,
            "final_pose": final,
            "part_world_pose": {
                "x": part.position.x,
                "y": part.position.y,
                "z": part.position.z,
            },
            "odom_publishers": node.count_publishers("/odom"),
            "cmd_vel_publishers": node.count_publishers("/cmd_vel"),
            "action_executions": 1,
            "mqtt_completed_before_duplicate": 1,
            "mqtt_completed_after_duplicate": 2,
            "next_mission": refused,
            "trace": str(trace),
            "report": str(report),
        }
        (output / "system-proof.json").write_text(json.dumps(proof, indent=2) + "\n")
        print(json.dumps(proof, indent=2))
        reset_scenario_controls()
    finally:
        # Signal the owned script, allowing its trap to stop its own project and
        # launch children. Escalate only the process group created above.
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        client.disconnect()
        client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        signal.signal(signal.SIGTERM, previous_termination)
    owned = subprocess.check_output(
        [
            "docker",
            "compose",
            "-f",
            "docker/compose.yaml",
            "-p",
            environment["FACTORY_COMPOSE_PROJECT"],
            "ps",
            "-aq",
        ],
        cwd=ROOT,
        env=environment,
        text=True,
    )
    assert not owned.strip(), "owned Compose containers survived cleanup"
    remaining = set(subprocess.check_output(["docker", "ps", "-aq"], text=True).split())
    assert preexisting_containers <= remaining, "a preexisting container was removed"
    launched_pids = [
        int(pid)
        for pid in re.findall(
            r"process started with pid \[(\d+)\]", (output / "demo.log").read_text()
        )
    ]
    assert launched_pids
    assert not any(Path(f"/proc/{pid}").exists() for pid in launched_pids), (
        "owned launch child survived cleanup"
    )
    print(
        f"Owned cleanup verified: {len(launched_pids)} launch children exited; Compose project removed; "
        f"{len(preexisting_containers)} preexisting containers preserved"
    )
    if "FACTORY_SCENARIO_OUTPUT" in os.environ:
        return dict(
            final_state=terminal[0]["state"],
            error_code=terminal[0]["error_code"],
            mission_id="M-001",
            action_executions=sum(event.event == "mission_started" for event in events),
            events=events,
            proof=str(output / "system-proof.json"),
        )
