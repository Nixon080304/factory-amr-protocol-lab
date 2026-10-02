# SPDX-License-Identifier: Apache-2.0
"""Exercise actual Gazebo sensors, TF ownership, and bounded physical motion."""

import math
import os
from pathlib import Path
import socket
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    # Never inherit a user's robot domain. The explicit test override is for CI workers.
    domain = int(os.environ.get("FACTORY_SIM_TEST_DOMAIN_ID", "72"))
    if not 1 <= domain <= 232:
        raise ValueError("FACTORY_SIM_TEST_DOMAIN_ID must be an isolated domain from 1 to 232")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        port = reserved.getsockname()[1]
    os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{port}"
    package = Path(get_package_share_directory("factory_simulation"))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package / "launch/simulation.launch.py")),
        launch_arguments={"gui": "false"}.items())
    return LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestSimulation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("factory_simulation_probe")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), "condition did not become true within bounded timeout")

    def entity_pose(self, client):
        request = GetEntityState.Request()
        request.name = "factory_amr"
        request.reference_frame = "world"
        future = client.call_async(request)
        self.spin_until(future.done, 3)
        response = future.result()
        self.assertTrue(response.success)
        return response.state.pose

    def test_sensors_tf_and_physical_motion(self):
        messages = {}
        transforms = set()
        subscriptions = []
        topics = {
            "/clock": Clock, "/scan": LaserScan, "/camera/image_raw": Image,
            "/camera/camera_info": CameraInfo, "/imu": Imu, "/odom": Odometry,
            "/joint_states": JointState, "/tf": TFMessage,
        }

        def receive(topic, message):
            messages[topic] = message
            if topic == "/tf":
                transforms.update((entry.header.frame_id, entry.child_frame_id) for entry in message.transforms)

        for topic, message_type in topics.items():
            subscriptions.append(self.node.create_subscription(
                message_type, topic, lambda message, topic=topic: receive(topic, message), qos_profile_sensor_data))
        publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        client = self.node.create_client(GetEntityState, "/gazebo/get_entity_state")
        try:
            deadline = time.monotonic() + 30
            while set(messages) != set(topics) and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertEqual(set(messages), set(topics), f"missing topics: {set(topics) - set(messages)}")
            clock = messages["/clock"].clock
            self.assertGreater(clock.sec * 1_000_000_000 + clock.nanosec, 0)
            scan = messages["/scan"]
            self.assertEqual(scan.header.frame_id, "base_scan")
            self.assertEqual(len(scan.ranges), 360)
            self.assertTrue(any(math.isfinite(value) and scan.range_min < value < scan.range_max for value in scan.ranges))
            image = messages["/camera/image_raw"]
            self.assertEqual((image.width, image.height), (640, 480))
            self.assertEqual(image.header.frame_id, "camera_optical_frame")
            self.assertGreater(len(image.data), 0)
            self.assertGreater(max(image.data), min(image.data), "camera must render scene geometry")
            info = messages["/camera/camera_info"]
            self.assertEqual((info.width, info.height), (640, 480))
            self.assertGreater(info.k[0], 0)
            self.assertEqual(messages["/imu"].header.frame_id, "imu_link")
            self.assertEqual(set(messages["/joint_states"].name), {"wheel_left_joint", "wheel_right_joint"})
            self.assertEqual(self.node.count_publishers("/odom"), 1)
            odometry = messages["/odom"]
            self.assertEqual((odometry.header.frame_id, odometry.child_frame_id), ("odom", "base_footprint"))
            self.spin_until(lambda: ("odom", "base_footprint") in transforms, 3)
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            initial_pose = self.entity_pose(client)
            initial_odom = messages["/odom"].pose.pose.position
            self.assertLess(math.hypot(initial_odom.x, initial_odom.y), 0.02,
                            "wheel odometry starts at spawn, independent of Gazebo world pose")
            self.spin_until(lambda: publisher.get_subscription_count() == 1, 3)
            command = Twist()
            command.linear.x = 0.12
            motion_deadline = time.monotonic() + 3
            try:
                while time.monotonic() < motion_deadline:
                    publisher.publish(command)
                    rclpy.spin_once(self.node, timeout_sec=0.05)
            finally:
                # The plugin holds its last command. Always stop before any assertion or shutdown.
                stop_deadline = time.monotonic() + 0.5
                while time.monotonic() < stop_deadline:
                    publisher.publish(Twist())
                    rclpy.spin_once(self.node, timeout_sec=0.05)
            final_pose = self.entity_pose(client)
            final_odom = messages["/odom"].pose.pose.position
            physical_distance = math.hypot(final_pose.position.x - initial_pose.position.x,
                                           final_pose.position.y - initial_pose.position.y)
            odom_distance = math.hypot(final_odom.x - initial_odom.x, final_odom.y - initial_odom.y)
            self.assertGreater(physical_distance, 0.05)
            self.assertLess(physical_distance, 0.6)
            self.assertAlmostEqual(odom_distance, physical_distance, delta=0.03)
            print(f"Gazebo physical motion={physical_distance:.3f} m; odometry motion={odom_distance:.3f} m; all 8 topics received")
        finally:
            publisher.publish(Twist())
            self.node.destroy_publisher(publisher)
            self.node.destroy_client(client)
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
