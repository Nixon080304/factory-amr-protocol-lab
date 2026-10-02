"""Bounded autonomy faults against the rendered demo and real protocol peers."""

import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid

import cv2
from cv_bridge import CvBridge
from factory_interfaces.msg import ProtocolEvent, StationDetection
from factory_interfaces.srv import SetFault
from factory_simulation.entity_probe import get_entity_pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import paho.mqtt.client as mqtt
from pymodbus.client import ModbusTcpClient
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from test_protocol_faults import (
    ROOT,
    free_port,
    services,
    record_owned_process,
    record_owned_resource,
)
from fault_injector.models import FaultRequest
from trace_assertions import assert_successful_transfer_phases


def stamp(value):
    return value.sec + value.nanosec / 1e9


@pytest.mark.parametrize(
    "name,error",
    [
        ("wrong_marker", "STATION_NOT_CONFIRMED"),
        ("nav_reject_once", None),
        ("nav_reject_twice", "NAVIGATION_FAILED"),
    ],
)
def test_autonomy_fault_outcome_recovery_and_trace(name, error):
    # Match the owned launch's loopback transport before creating the probe.
    # Restore the caller's environment even if setup or assertions fail.
    with pytest.MonkeyPatch.context() as environment:
        environment.setenv("ROS_LOCALHOST_ONLY", "1")
        return autonomy_fault_outcome_recovery_and_trace(name, error)


def autonomy_fault_outcome_recovery_and_trace(name, error):
    # Missing validation/routing used to make these scenarios unavailable.
    FaultRequest(
        name, "M-autonomy", activation_point="navigation_start", duration=120.0
    )
    output = (
        Path(os.environ["FACTORY_SCENARIO_OUTPUT"])
        if "FACTORY_SCENARIO_OUTPUT" in os.environ
        else (
            ROOT
            / ".superpowers/sdd/2026-10-02-factory-amr-03-reliability-release/evidence"
            / (name + "-" + uuid.uuid4().hex)
        )
    )
    output.mkdir(parents=True, exist_ok=True)
    # A fresh domain per case also prevents lingering DDS discovery from the
    # preceding launch from being mistaken for a new lifecycle service peer.
    domain = int(
        os.environ.get(
            "FACTORY_SCENARIO_DOMAIN",
            {"wrong_marker": 91, "nav_reject_once": 94, "nav_reject_twice": 95}[name],
        )
    )
    rclpy.init(domain_id=domain)
    node = rclpy.create_node(
        "autonomy_fault_probe",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    events, detections, frames, statuses, poses = [], [], {}, [], []
    bridge = CvBridge()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.minMarkerDistanceRate = 0.03

    def camera(message):
        pixels = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        _, ids, _ = cv2.aruco.detectMarkers(pixels, dictionary, parameters=parameters)
        if ids is not None:
            frames[stamp(message.header.stamp)] = (
                set(map(int, ids.reshape(-1))),
                pixels,
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
            Image, "/camera/image_raw", camera, qos_profile_sensor_data
        ),
        node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            poses.append,
            qos_profile_sensor_data,
        ),
    ]
    configure = node.create_client(SetFault, "/factory/faults/set")
    reset = node.create_client(Trigger, "/factory/faults/reset")
    process = None
    with services() as (broker_port, plc):
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id="autonomy-" + uuid.uuid4().hex
        )
        subscribed = []
        client.on_connect = lambda client, *args: client.subscribe(
            "factory/missions/+/status", 1
        )
        client.on_subscribe = lambda *args: subscribed.append(True)
        client.on_message = lambda client, userdata, message: statuses.append(
            json.loads(message.payload)
        )
        client.connect("127.0.0.1", broker_port)
        client.loop_start()
        gazebo_port = free_port()
        environment = {
            **os.environ,
            "ROS_DOMAIN_ID": str(domain),
            "ROS_LOCALHOST_ONLY": "1",
            "GAZEBO_MASTER_URI": f"http://127.0.0.1:{gazebo_port}",
            "GAZEBO_MODEL_DATABASE_URI": "",
            "PYTHONNOUSERSITE": "1",
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "DISPLAY": os.environ.get("DISPLAY", ":0"),
        }
        log = (output / "demo.log").open("w")
        deadline = time.monotonic() + 240

        def wait(predicate, seconds=10):
            end = min(deadline, time.monotonic() + seconds)
            while not predicate() and time.monotonic() < end:
                assert process.poll() is None, (
                    f"launch exited; log={output / 'demo.log'}"
                )
                rclpy.spin_once(node, timeout_sec=0.05)
            assert predicate(), f"bounded scenario deadline; log={output / 'demo.log'}"

        def call(client, request):
            future = client.call_async(request)
            wait(future.done)
            return future.result()

        try:
            process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "factory_bringup",
                    "demo.launch.py",
                    "gui:=false",
                    "rviz:=false",
                    f"broker_port:={broker_port}",
                    f"plc_port:={plc.port}",
                    f"output_dir:={output}",
                ],
                env=environment,
                cwd=ROOT,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            record_owned_process(process)
            record_owned_resource("ports", gazebo_port)
            wait(
                lambda: (
                    subscribed
                    and poses
                    and configure.service_is_ready()
                    and reset.service_is_ready()
                ),
                80,
            )
            ready = subprocess.run(
                ["ros2", "run", "factory_bringup", "factory_wait_ready"],
                env=environment,
                capture_output=True,
                text=True,
                timeout=65,
            )
            assert ready.returncode == 0, ready.stderr
            assert call(reset, Trigger.Request()).success
            response = call(
                configure,
                SetFault.Request(
                    json=json.dumps(
                        dict(
                            name=name,
                            mission_id="M-autonomy",
                            station="assembly",
                            activation_point="navigation_start",
                            duration=120.0,
                        )
                    )
                ),
            )
            assert response.accepted, response.message
            client.publish(
                "factory/missions/request",
                json.dumps(
                    dict(
                        mission_id="M-autonomy",
                        robot_id="amr_01",
                        pickup="assembly",
                        dropoff="inspection",
                        part="motor",
                    )
                ),
                qos=1,
            ).wait_for_publish(timeout=2)
            wait(
                lambda: any(
                    item["state"] in ("COMPLETED", "FAILED") for item in statuses
                ),
                170,
            )
            terminal = [
                item for item in statuses if item["state"] in ("COMPLETED", "FAILED")
            ]
            assert [(item["state"], item["error_code"]) for item in terminal] == [
                ("FAILED" if error else "COMPLETED", error)
            ]
            wait(lambda: any(event.event == "mission_finished" for event in events))
            mission = [event for event in events if event.mission_id == "M-autonomy"]
            names = [event.event for event in mission]
            assert (
                names.index("mission_started")
                < names.index("navigation_pickup_started")
                < names.index("mission_finished")
            )
            faults = [event.event for event in mission if event.protocol == "FAULT"]
            assert faults[:2] == ["fault_activated", "fault_consumed"]
            goals = [
                json.loads(event.detail)
                for event in mission
                if event.event == "navigation_goal_requested"
            ]
            assert goals[0]["station"] == "assembly" and goals[0]["pose"] == [
                -3.0,
                0.8,
                1.5707963267948966,
            ]
            if name.startswith("nav_reject"):
                assert goals[:2] == [goals[0], goals[0]], (
                    "retry must keep the exact failed station pose"
                )
                clears = [
                    json.loads(event.detail)["service"]
                    for event in mission
                    if event.event == "costmap_cleared"
                ]
                assert sorted(clears) == [
                    "/global_costmap/clear_entirely_global_costmap",
                    "/local_costmap/clear_entirely_local_costmap",
                ]
                assert names.count("retry") == 1
                recovery = names.index("navigation_recovery_started")
                retry = names.index("retry")
                assert names.index("navigation_pickup_finished") < recovery < retry
                assert all(
                    recovery < index < retry
                    for index, value in enumerate(names)
                    if value == "costmap_cleared"
                )
                assert names[retry:].index("navigation_pickup_started") < names[
                    retry:
                ].index("navigation_pickup_finished")
            if name == "wrong_marker":
                start = next(
                    event
                    for event in mission
                    if event.event == "perception_pickup_started"
                )
                finish = next(
                    event
                    for event in mission
                    if event.event == "perception_pickup_finished"
                )
                assert (
                    finish.outcome == "FAILED"
                    and finish.detail == "STATION_NOT_CONFIRMED"
                )
                assert (
                    names.index("fault_activated")
                    < names.index("wrong_marker_pose_applied")
                    < names.index("perception_pickup_started")
                )
                assert 10.0 <= stamp(finish.stamp) - stamp(start.stamp) < 10.2
                wrong = [
                    stamp(item.header.stamp)
                    for item in detections
                    if item.marker_id == 20
                    and item.station_id == "inspection"
                    and stamp(start.stamp)
                    <= stamp(item.header.stamp)
                    <= stamp(finish.stamp)
                ]
                overlap = [
                    value
                    for value in wrong
                    if value in frames and 20 in frames[value][0]
                ]
                assert len(set(wrong)) >= 5 and overlap, (
                    "wrong identity must come from independently decoded rendered camera frames"
                )
                pose = get_entity_pose(node, "wrong_marker_station")
                assert (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ) == pytest.approx((-3.0, 1.57, 0.35))
                assert cv2.imwrite(
                    str(output / "wrong-marker-rendered.png"), frames[overlap[0]][1]
                )
                assert (
                    names.index("navigation_pickup_finished")
                    < names.index("perception_pickup_started")
                    < names.index("perception_pickup_finished")
                    < names.index("mission_finished")
                )
            if not error:
                assert_successful_transfer_phases(mission)
                for leg, station, marker in [
                    ("pickup", "assembly", 10),
                    ("dropoff", "inspection", 20),
                ]:
                    start = next(
                        event
                        for event in mission
                        if event.event == f"perception_{leg}_started"
                    )
                    finish = next(
                        event
                        for event in mission
                        if event.event == f"perception_{leg}_finished"
                    )
                    sources = sorted(
                        {
                            stamp(item.header.stamp)
                            for item in detections
                            if item.station_id == station
                            and item.marker_id == marker
                            and stamp(start.stamp)
                            <= stamp(item.header.stamp)
                            <= stamp(finish.stamp)
                        }
                    )
                    assert len(sources) >= 5 and sources[4] - sources[0] <= 1.5
                    assert any(
                        value in frames and marker in frames[value][0]
                        for value in sources[:5]
                    )
            with ModbusTcpClient("127.0.0.1", port=plc.port, timeout=1) as connection:
                counters = [
                    connection.read_holding_registers(
                        0, count=4, device_id=unit
                    ).registers[2]
                    for unit in (1, 2)
                ]
            assert counters == ([0, 0] if error else [1, 1])
            if error:
                assert not any(event.protocol == "MODBUS" for event in mission), (
                    "failed autonomy must never dispatch PLC transfer"
                )
            assert call(reset, Trigger.Request()).success
            wait(
                lambda: (
                    any(event.event == "fault_reset" for event in mission)
                    or any(
                        event.event == "fault_reset"
                        and event.mission_id == "M-autonomy"
                        for event in events
                    )
                )
            )
            parked = get_entity_pose(node, "wrong_marker_station")
            assert parked.position.z == pytest.approx(-10.0)
            if name == "wrong_marker":
                reset_stamp = node.get_clock().now().nanoseconds / 1e9
                wait(
                    lambda: any(
                        item.marker_id == 10
                        and item.station_id == "assembly"
                        and stamp(item.header.stamp) > reset_stamp
                        and stamp(item.header.stamp) in frames
                        and 10 in frames[stamp(item.header.stamp)][0]
                        for item in detections
                    )
                )
            records = [
                dict(
                    event=event.event,
                    protocol=event.protocol,
                    outcome=event.outcome,
                    detail=event.detail,
                    stamp=stamp(event.stamp),
                )
                for event in events
            ]
            (output / "scenario-proof.json").write_text(
                json.dumps(
                    dict(
                        name=name,
                        terminal=terminal,
                        counters=counters,
                        goals=goals,
                        rendered_frame_count=len(frames),
                        events=records,
                    ),
                    indent=2,
                )
                + "\n"
            )
            print(
                f"{name}: {terminal[0]['state']}/{error}; cycles={counters}; evidence={output}"
            )
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
            client.disconnect()
            client.loop_stop()
            node.destroy_node()
            rclpy.shutdown()
            log.close()
            children = re.findall(
                r"process started with pid \[(\d+)\]", (output / "demo.log").read_text()
            )
            assert not any(Path(f"/proc/{pid}").exists() for pid in children)
            print(
                f"cleanup: {len(children)} owned launch children stopped; Gazebo master={gazebo_port}"
            )
    if "FACTORY_SCENARIO_OUTPUT" in os.environ:
        return dict(
            final_state=terminal[0]["state"],
            error_code=terminal[0]["error_code"],
            mission_id="M-autonomy",
            action_executions=names.count("mission_started"),
            events=events,
            proof=str(output / "scenario-proof.json"),
        )
