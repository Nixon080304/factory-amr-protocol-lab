# SPDX-License-Identifier: Apache-2.0
"""Prove station identity from actual rendered Gazebo images at both map poses."""

import math
import os
from pathlib import Path
import socket
import time
import unittest
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from factory_interfaces.msg import StationDetection
from factory_simulation.entity_probe import get_entity_pose, set_entity_pose
from geometry_msgs.msg import Pose
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

from station_perception.confirmation_window import ConfirmationWindow
from station_perception.detector import ArucoStationDetector
from station_perception.node import StationDetectorNode


STATIONS_FILE = Path(__file__).resolve().parents[2] / "factory_bringup/config/stations.yaml"


@pytest.mark.launch_test
def generate_test_description():
    domain = int(os.environ.get("FACTORY_CAMERA_TEST_DOMAIN_ID", "74"))
    if not 1 <= domain <= 232:
        raise ValueError("FACTORY_CAMERA_TEST_DOMAIN_ID must be an isolated domain from 1 to 232")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        port = reserved.getsockname()[1]
    os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{port}"
    share = Path(get_package_share_directory("factory_simulation"))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/simulation.launch.py")),
        launch_arguments={"gui": "false"}.items())
    perception = Node(package="station_perception", executable="station_detector",
                      parameters=[{"use_sim_time": True, "stations_file": str(STATIONS_FILE)}], output="screen")
    return LaunchDescription([simulation, perception, launch_testing.actions.ReadyToTest()])


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


class TestCamera(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("station_camera_probe",
                                   parameter_overrides=[Parameter("use_sim_time", value=True)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_both_configured_verification_poses(self):
        self.deadline = time.monotonic() + 60
        images = {}
        detections = []
        image_subscription = self.node.create_subscription(
            Image, "/camera/image_raw",
            lambda image: images.update({stamp_seconds(image.header): image}), qos_profile_sensor_data)
        detection_subscription = self.node.create_subscription(
            StationDetection, "/factory/station_detection", detections.append, 10)
        bridge = CvBridge()
        stations = yaml.safe_load(STATIONS_FILE.read_text())["stations"]
        evidence = Path(os.environ.get("FACTORY_CAMERA_EVIDENCE_DIR", "/tmp/factory_camera_evidence"))
        evidence.mkdir(parents=True, exist_ok=True)
        try:
            self.spin_until(lambda: bool(images), 30, "rendered camera image did not arrive")
            for topic in ("/camera/image_raw", "/camera/camera_info", "/scan"):
                endpoints = self.node.get_publishers_info_by_topic(topic)
                self.assertEqual(len(endpoints), 1)
                profile = endpoints[0].qos_profile
                self.assertEqual(profile.reliability, ReliabilityPolicy.BEST_EFFORT, topic)
                self.assertEqual(profile.durability, DurabilityPolicy.VOLATILE, topic)
            for station_id, station in stations.items():
                pose = Pose()
                pose.position.x = station["x"]
                pose.position.y = station["y"]
                pose.position.z = 0.01
                pose.orientation.z = math.sin(station["yaw"] / 2)
                pose.orientation.w = math.cos(station["yaw"] / 2)
                set_entity_pose(self.node, "factory_amr", pose, timeout_sec=5)
                actual = get_entity_pose(self.node, "factory_amr", timeout_sec=5)
                self.assertAlmostEqual(actual.position.x, station["x"], delta=0.02)
                self.assertAlmostEqual(actual.position.y, station["y"], delta=0.02)
                actual_yaw = math.atan2(2 * actual.orientation.w * actual.orientation.z,
                                        1 - 2 * actual.orientation.z ** 2)
                self.assertAlmostEqual(actual_yaw, station["yaw"], delta=0.02)
                # Drain images rendered before repositioning, then require new stamps.
                threshold = self.node.get_clock().now().nanoseconds / 1e9 + 0.2
                detections.clear()
                window = ConfirmationWindow(station_id)
                confirmed = False
                processed = 0
                station_deadline = min(self.deadline, time.monotonic() + 15)
                matched = None
                confirmation_messages = []
                while not confirmed and time.monotonic() < station_deadline:
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                    for message in detections[processed:]:
                        processed += 1
                        stamp = stamp_seconds(message.header)
                        if stamp <= threshold:
                            continue
                        self.assertEqual(message.station_id, station_id)
                        self.assertEqual(message.marker_id, station["marker_id"])
                        self.assertEqual(message.header.frame_id, "camera_optical_frame")
                        self.assertGreater(message.confidence, 0.005)
                        self.assertLess(message.confidence, 0.1)
                        confirmation_messages.append(message)
                        confirmed = window.observe(message.station_id, stamp)
                        if confirmed:
                            matched = message
                            break
                if not confirmed:
                    image = images[max(images)]
                    cv2.imwrite(str(evidence / f"{station_id}_failure.png"),
                                bridge.imgmsg_to_cv2(image, desired_encoding="bgr8"))
                self.assertTrue(confirmed, f"{station_id}: no five rendered marker detections at configured pose")
                # Best-effort subscribers can receive different subsets. Require
                # exact source correlation within the confirmed five-image window.
                confirmation_messages = confirmation_messages[-5:]
                self.spin_until(lambda: any(stamp_seconds(message.header) in images
                                           for message in confirmation_messages), 2,
                                "confirmation needs an actual rendered source image")
                matched = next(message for message in confirmation_messages
                               if stamp_seconds(message.header) in images)
                image = images[stamp_seconds(matched.header)]
                pixels = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                pure = ArucoStationDetector().detect(pixels)[0]
                self.assertEqual(pure.marker_id, station["marker_id"])
                self.assertAlmostEqual(pure.confidence, matched.confidence, places=6)
                path = evidence / f"{station_id}_{station['marker_id']}_rendered.png"
                self.assertTrue(cv2.imwrite(str(path), pixels))
                print(f"Rendered {station_id}: marker={matched.marker_id}, area={matched.confidence:.6f}, "
                      f"stamp={stamp_seconds(matched.header):.9f}, world=({actual.position.x:.3f}, "
                      f"{actual.position.y:.3f}, {actual_yaw:.6f}), evidence={path}")
        finally:
            self.node.destroy_subscription(image_subscription)
            self.node.destroy_subscription(detection_subscription)

    def spin_until(self, predicate, timeout, message):
        deadline = min(self.deadline, time.monotonic() + timeout)
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), message)


class TestNodeConfiguration(unittest.TestCase):
    def test_default_node_does_not_require_bringup_package(self):
        prefixes = [prefix for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
                    if Path(prefix).name != "factory_bringup"]
        # Alter real package discovery inputs; no dependency implementation is mocked.
        with patch.dict(os.environ, {"AMENT_PREFIX_PATH": os.pathsep.join(prefixes)}):
            rclpy.init()
            node = None
            try:
                node = StationDetectorNode()
                self.assertEqual(node.get_parameter("stations_file").value, "")
                self.assertTrue(node.get_parameter("use_sim_time").value)
            finally:
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
