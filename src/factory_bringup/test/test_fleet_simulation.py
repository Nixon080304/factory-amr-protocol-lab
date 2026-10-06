# SPDX-License-Identifier: Apache-2.0
"""Mandatory bounded proof of two Gazebo entities and active isolated Nav2 stacks."""

import os
from pathlib import Path
import socket
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from factory_simulation.entity_probe import get_entity_pose
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    domain = int(os.environ.get("FACTORY_FLEET_TEST_DOMAIN_ID", "74"))
    if not 1 <= domain <= 232:
        raise ValueError("FACTORY_FLEET_TEST_DOMAIN_ID must be isolated from 1 to 232")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    uri = os.environ.get("FACTORY_FLEET_GAZEBO_URI")
    if uri is None:
        with socket.socket() as reserved:
            reserved.bind(("127.0.0.1", 0))
            uri = f"http://127.0.0.1:{reserved.getsockname()[1]}"
    os.environ["GAZEBO_MASTER_URI"] = uri
    share = Path(get_package_share_directory("factory_bringup"))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/fleet.launch.py")),
        launch_arguments={"gui": "false", "rviz": "false"}.items(),
    )
    return LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestFleetSimulation(unittest.TestCase):
    def test_two_descriptions_entities_topics_tf_and_nav2_lifecycles(self):
        rclpy.init()
        node = rclpy.create_node("fleet_simulation_probe")
        messages = {}
        transforms = {"amr_01": set(), "amr_02": set()}
        subscriptions = []
        clients = []

        def receive(topic, message):
            messages[topic] = message

        def receive_tf(robot, message):
            transforms[robot].update(
                (entry.header.frame_id, entry.child_frame_id)
                for entry in message.transforms
            )

        try:
            for robot in ("amr_01", "amr_02"):
                for name, message_type in (
                    ("robot_description", String),
                    ("odom", Odometry),
                    ("scan", LaserScan),
                    ("camera/image_raw", Image),
                ):
                    topic = f"/{robot}/{name}"
                    qos = (
                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                        if name == "robot_description"
                        else qos_profile_sensor_data
                    )
                    subscriptions.append(
                        node.create_subscription(
                            message_type,
                            topic,
                            lambda message, topic=topic: receive(topic, message),
                            qos,
                        )
                    )
                subscriptions.append(
                    node.create_subscription(
                        TFMessage,
                        f"/{robot}/tf",
                        lambda message, robot=robot: receive_tf(robot, message),
                        qos_profile_sensor_data,
                    )
                )
            deadline = time.monotonic() + 100

            def ready():
                return len(messages) == 8 and all(
                    {
                        (f"{robot}/map", f"{robot}/odom"),
                        (f"{robot}/odom", f"{robot}/base_footprint"),
                    }
                    <= transforms[robot]
                    for robot in ("amr_01", "amr_02")
                )

            while not ready() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertEqual(
                len(messages),
                8,
                f"missing robot description/sensor evidence: {sorted(messages)}",
            )
            for index, robot in enumerate(("amr_01", "amr_02")):
                self.assertIn(
                    f"{robot}/base_footprint",
                    messages[f"/{robot}/robot_description"].data,
                )
                odom = messages[f"/{robot}/odom"]
                self.assertEqual(
                    (odom.header.frame_id, odom.child_frame_id),
                    (f"{robot}/odom", f"{robot}/base_footprint"),
                )
                self.assertEqual(
                    messages[f"/{robot}/scan"].header.frame_id, f"{robot}/base_scan"
                )
                self.assertEqual(
                    messages[f"/{robot}/camera/image_raw"].header.frame_id,
                    f"{robot}/camera_optical_frame",
                )
                self.assertIn((f"{robot}/map", f"{robot}/odom"), transforms[robot])
                self.assertIn(
                    (f"{robot}/odom", f"{robot}/base_footprint"), transforms[robot]
                )
                self.assertGreater(
                    deadline - time.monotonic(), 0, "fleet startup deadline expired"
                )
                pose = get_entity_pose(
                    node, robot, timeout_sec=min(5.0, deadline - time.monotonic())
                )
                self.assertAlmostEqual(pose.position.x, float(index), delta=0.1)
                self.assertAlmostEqual(pose.position.y, -3.0, delta=0.1)
                for name in (
                    "map_server",
                    "amcl",
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ):
                    client = node.create_client(GetState, f"/{robot}/{name}/get_state")
                    clients.append(client)
                    self.assertTrue(
                        client.wait_for_service(
                            timeout_sec=max(0.0, min(2.0, deadline - time.monotonic()))
                        ),
                        name,
                    )
                    state = None
                    while (
                        state != State.PRIMARY_STATE_ACTIVE
                        and time.monotonic() < deadline
                    ):
                        future = client.call_async(GetState.Request())
                        while not future.done() and time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.05)
                        self.assertTrue(future.done(), name)
                        state = future.result().current_state.id
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertEqual(
                        state, State.PRIMARY_STATE_ACTIVE, f"{robot}/{name}"
                    )
            print(
                "Two descriptions, two entities, isolated sensors/TF, and 18 active Nav2 lifecycle nodes verified"
            )
        finally:
            for client in clients:
                node.destroy_client(client)
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
