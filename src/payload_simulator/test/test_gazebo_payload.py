# SPDX-License-Identifier: Apache-2.0
"""Measure actual Gazebo motion after real successful ProtocolEvent messages."""

import json
import os
from pathlib import Path
import socket
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


STATE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@pytest.mark.launch_test
def generate_test_description():
    domain = int(os.environ.get("FACTORY_PAYLOAD_TEST_DOMAIN_ID", "75"))
    if not 1 <= domain <= 232:
        raise ValueError(
            "FACTORY_PAYLOAD_TEST_DOMAIN_ID must be an isolated domain from 1 to 232"
        )
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        port = reserved.getsockname()[1]
    os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{port}"
    share = Path(get_package_share_directory("factory_simulation"))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/simulation.launch.py")),
        launch_arguments={"gui": "false"}.items(),
    )
    payload = Node(
        package="payload_simulator",
        executable="payload_simulator",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    return LaunchDescription(
        [simulation, payload, launch_testing.actions.ReadyToTest()]
    )


def event(
    *, kind="LOADING", mission_id="payload_test", counter=10, outcome="SUCCEEDED"
):
    from factory_interfaces.msg import ProtocolEvent

    message = ProtocolEvent()
    message.mission_id = mission_id
    message.protocol = "MODBUS"
    message.direction = "RECEIVE"
    message.event = (
        "modbus_pickup_finished" if kind == "LOADING" else "modbus_dropoff_finished"
    )
    message.outcome = outcome
    message.detail = json.dumps(
        {
            "station_id": "assembly" if kind == "LOADING" else "inspection",
            "transfer_kind": kind,
            "cycle_counter": counter,
        }
    )
    return message


class TestGazeboPayload(unittest.TestCase):
    CARRY_OFFSET = (-0.03, 0.0, 0.28)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            "payload_pose_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=5):
        deadline = min(self.deadline, time.monotonic() + timeout)
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(predicate(), "bounded payload condition did not arrive")

    def pose(self, name):
        from factory_simulation.entity_probe import get_entity_pose

        self.assertLess(time.monotonic(), self.deadline)
        return get_entity_pose(
            self.node, name, timeout_sec=min(2, self.deadline - time.monotonic())
        )

    def assert_pose(self, name, xyz):
        actual = self.pose(name)
        for value, expected in zip(
            (actual.position.x, actual.position.y, actual.position.z), xyz
        ):
            self.assertAlmostEqual(value, expected, delta=0.005)
        return actual

    def publish(self, publisher, message):
        message.stamp = self.node.get_clock().now().to_msg()
        publisher.publish(message)

    def payload_is_on_robot(self, tolerance=0.015):
        robot = self.pose("factory_amr")
        part = self.pose("factory_part")
        return all(
            abs((part_value - robot_value) - offset) <= tolerance
            for part_value, robot_value, offset in zip(
                (part.position.x, part.position.y, part.position.z),
                (robot.position.x, robot.position.y, robot.position.z),
                self.CARRY_OFFSET,
            )
        )

    def assert_payload_on_robot(self):
        self.assertTrue(
            self.payload_is_on_robot(),
            "factory_part must remain visibly mounted on factory_amr",
        )
        return self.pose("factory_part")

    def observe_animation(self, conveyor, expected_part=None):
        offsets = []
        visible_positions = []
        visible_link_positions = []
        deadline = min(self.deadline, time.monotonic() + 5)
        while time.monotonic() < deadline:
            belt = self.pose(conveyor)
            offsets.append(belt.position.y - 2.0)
            part = self.pose("factory_part")
            xyz = (part.position.x, part.position.y, part.position.z)
            link = self.pose("factory_part::part_link")
            station_x = -3.0 if conveyor == "assembly_conveyor" else 3.0
            if (
                abs(link.position.x - station_x) < 0.005
                and abs(link.position.z - 0.65) < 0.005
            ):
                visible_link_positions.append(link.position.y)
            if (
                abs(part.position.x - station_x) < 0.005
                and abs(part.position.z - 0.65) < 0.005
            ):
                visible_positions.append(part.position.y)
            if conveyor == "inspection_conveyor" and abs(offsets[-1]) >= 0.05:
                self.assertAlmostEqual(part.position.x, 3.0, delta=0.005)
                self.assertAlmostEqual(part.position.z, 0.65, delta=0.005)
                self.assertGreaterEqual(part.position.y, 1.8 - 0.005)
                self.assertLessEqual(part.position.y, 2.0 + 0.005)
            at_final_pose = (
                self.payload_is_on_robot()
                if expected_part is None
                else all(
                    abs(value - expected) < 0.005
                    for value, expected in zip(xyz, expected_part)
                )
            )
            if max(offsets) >= 0.05 and abs(offsets[-1]) < 0.005 and at_final_pose:
                break
        self.assertGreaterEqual(max(offsets), 0.05, "actual conveyor must visibly move")
        self.assertGreaterEqual(
            max(visible_positions) - min(visible_positions),
            0.14,
            "actual part must travel visibly on conveyor before final placement",
        )
        self.assertTrue(any(1.84 < value < 1.96 for value in visible_positions))
        self.assertGreaterEqual(
            max(visible_link_positions) - min(visible_link_positions), 0.14
        )
        self.assertTrue(any(1.84 < value < 1.96 for value in visible_link_positions))
        self.assertAlmostEqual(offsets[-1], 0, delta=0.005)
        if expected_part is None:
            self.assert_payload_on_robot()
        else:
            self.assert_pose("factory_part", expected_part)
            self.assert_pose("factory_part::part_link", expected_part)
        print(
            f"Actual {conveyor} offsets: min={min(offsets):.3f}, max={max(offsets):.3f}, "
            f"final={offsets[-1]:.3f}; factory_part={xyz}"
        )
        print(
            f"Visible part travel: {min(visible_positions):.3f}..{max(visible_positions):.3f}"
        )

    def assert_no_replay(self, publisher, messages, conveyor, part_xyz):
        from factory_simulation.entity_probe import set_entity_pose

        # Leave an independent sentinel pose: any replay returns the conveyor to
        # its animation origin and is observable even if sampling misses motion.
        sentinel = Pose()
        sentinel.position.x = -3.0 if conveyor == "assembly_conveyor" else 3.0
        sentinel.position.y = 2.03
        sentinel.position.z = 0.603
        sentinel.orientation.w = 1.0
        set_entity_pose(self.node, conveyor, sentinel, timeout_sec=2)
        for message in messages:
            self.publish(publisher, message)
        deadline = min(self.deadline, time.monotonic() + 1.4)
        samples = 0
        while time.monotonic() < deadline:
            self.assert_pose(conveyor, (sentinel.position.x, 2.03, 0.603))
            self.assert_pose("factory_part", part_xyz)
            samples += 1
        self.assertGreater(samples, 2)
        print(
            f"No replay: {conveyor} sentinel y=2.030 retained across {samples} actual pose samples"
        )
        sentinel.position.y = 2.0
        set_entity_pose(self.node, conveyor, sentinel, timeout_sec=2)

    def test_successful_events_animate_once_and_retain_state(self):
        from factory_interfaces.msg import ProtocolEvent

        self.deadline = time.monotonic() + 70
        states = []
        subscription = self.node.create_subscription(
            String,
            "/factory/payload_state",
            lambda message: states.append(json.loads(message.data)),
            STATE_QOS,
        )
        publisher = self.node.create_publisher(
            ProtocolEvent,
            "/factory/protocol_events",
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.spin_until(
            lambda: bool(states) and publisher.get_subscription_count() == 1, 30
        )
        self.assertEqual(states[-1]["state"], "AT_ASSEMBLY")
        self.assert_pose("factory_part", (-3, 2, 0.65))
        self.assert_pose("assembly_station", (-3, 2, 0))
        self.assert_pose("inspection_station", (3, 2, 0))
        rejected = [
            event(outcome="FAILED"),
            event(kind="UNLOADING"),
            event(mission_id="bad/id"),
        ]
        malformed = event()
        malformed.detail = (
            '{"station_id":"assembly","transfer_kind":"LOADING","cycle_counter":true}'
        )
        rejected.append(malformed)
        wrong_phase = event()
        wrong_phase.event = "modbus_pickup_started"
        rejected.append(wrong_phase)
        self.assert_no_replay(publisher, rejected, "assembly_conveyor", (-3, 2, 0.65))
        self.assertEqual([state["state"] for state in states], ["AT_ASSEMBLY"])

        loaded = event()
        self.publish(publisher, loaded)
        self.spin_until(lambda: states[-1]["state"] == "IN_TRANSIT")
        self.observe_animation("assembly_conveyor")
        before = self.assert_payload_on_robot()
        from factory_simulation.entity_probe import set_entity_pose

        moved_robot = Pose()
        moved_robot.position.x = 1.0
        moved_robot.position.y = -1.0
        moved_robot.position.z = 0.01
        moved_robot.orientation.w = 1.0
        set_entity_pose(self.node, "factory_amr", moved_robot, timeout_sec=2)
        self.spin_until(self.payload_is_on_robot, 3)
        after = self.assert_payload_on_robot()
        self.assertGreater(
            abs(after.position.x - before.position.x)
            + abs(after.position.y - before.position.y),
            1.0,
            "factory_part must travel with a moving factory_amr",
        )
        carried_xyz = (
            after.position.x,
            after.position.y,
            after.position.z,
        )
        self.assertEqual(
            states[-1],
            {
                "state": "IN_TRANSIT",
                "mission_id": "payload_test",
                "transfer_kind": "LOADING",
                "cycle_counter": 10,
            },
        )
        self.assert_no_replay(
            publisher,
            [
                loaded,
                event(kind="UNLOADING", mission_id="other"),
                event(kind="UNLOADING", outcome="FAILED"),
            ],
            "assembly_conveyor",
            carried_xyz,
        )
        self.assertEqual(len(states), 2)

        unloaded = event(kind="UNLOADING", counter=11)
        self.publish(publisher, unloaded)
        self.spin_until(lambda: states[-1]["state"] == "AT_INSPECTION")
        self.observe_animation("inspection_conveyor", (3, 2, 0.65))
        self.assert_no_replay(
            publisher,
            [unloaded, loaded, event(mission_id="next")],
            "inspection_conveyor",
            (3, 2, 0.65),
        )
        self.assertEqual(
            [state["state"] for state in states],
            ["AT_ASSEMBLY", "IN_TRANSIT", "AT_INSPECTION"],
        )
        late_states = []
        late = self.node.create_subscription(
            String,
            "/factory/payload_state",
            lambda message: late_states.append(json.loads(message.data)),
            STATE_QOS,
        )
        self.spin_until(lambda: bool(late_states))
        self.assertEqual(late_states, [states[-1]])
        self.assert_pose("assembly_station", (-3, 2, 0))
        self.assert_pose("inspection_station", (3, 2, 0))
        self.node.destroy_subscription(late)
        self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(publisher)

    def test_missing_visual_service_does_not_block_logical_state(self):
        from payload_simulator.node import PayloadSimulatorNode

        self.deadline = time.monotonic() + 10
        payload = PayloadSimulatorNode(
            cli_args=[
                "--ros-args",
                "-r",
                "/gazebo/set_entity_state:=/payload_test/unavailable",
                "-r",
                "/factory/payload_state:=/payload_test/state",
                "-r",
                "/factory/protocol_events:=/payload_test/events",
            ]
        )
        received = []
        subscription = self.node.create_subscription(
            String,
            "/payload_test/state",
            lambda message: received.append(json.loads(message.data)),
            STATE_QOS,
        )
        try:
            payload._on_event(event())
            deadline = time.monotonic() + 0.5
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(payload, timeout_sec=0.01)
                rclpy.spin_once(self.node, timeout_sec=0.01)
            self.assertTrue(received)
            self.assertEqual(received[-1]["state"], "IN_TRANSIT")
            # Continue through the bounded visual timeout and accept unloading.
            deadline = time.monotonic() + 5.5
            while time.monotonic() < deadline:
                rclpy.spin_once(payload, timeout_sec=0.02)
            payload._on_event(event(kind="UNLOADING", counter=11))
            self.spin_until(lambda: received[-1]["state"] == "AT_INSPECTION")
            print(
                "Missing Gazebo service: logical IN_TRANSIT published within 0.5 s; "
                "visual timeout does not block AT_INSPECTION"
            )
        finally:
            self.node.destroy_subscription(subscription)
            payload.destroy_node()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
