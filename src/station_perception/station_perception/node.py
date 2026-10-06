# SPDX-License-Identifier: Apache-2.0
"""Publish the largest configured marker from each camera image.

The coordinator owns confirmation and freshness. Every published header is the
source image header, keeping station observations in the simulation time domain.
"""

from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
from factory_interfaces.msg import StationDetection
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

from station_perception.detector import ArucoStationDetector


class StationDetectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("station_detector", **kwargs)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        elif not self.get_parameter("use_sim_time").value:
            self.set_parameters([Parameter("use_sim_time", value=True)])
        config_path = self.declare_parameter("stations_file", "").value
        mapping = None
        if config_path:
            stations = yaml.safe_load(Path(config_path).read_text())["stations"]
            mapping = {}
            for station_id, values in stations.items():
                marker_id = values["marker_id"]
                if (
                    type(marker_id) is not int
                    or not 0 <= marker_id < 50
                    or marker_id in mapping
                ):
                    raise ValueError(
                        "Station marker IDs must be unique DICT_4X4_50 integers"
                    )
                mapping[marker_id] = station_id
        self._detector = ArucoStationDetector(mapping)
        self._bridge = CvBridge()
        self._publisher = self.create_publisher(
            StationDetection, "factory/station_detection", 10
        )
        self._subscription = self.create_subscription(
            Image, "camera/image_raw", self._on_image, qos_profile_sensor_data
        )

    def _on_image(self, image: Image):
        try:
            detections = self._detector.detect(
                self._bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            )
        except (CvBridgeError, cv2.error) as error:
            self.get_logger().warning(f"Cannot decode station camera image: {error}")
            return
        if not detections:
            return
        winner = detections[0]
        message = StationDetection()
        message.header = image.header
        message.station_id = winner.station_id
        message.marker_id = winner.marker_id
        message.confidence = winner.confidence
        self._publisher.publish(message)
        self.get_logger().info(
            f"station={winner.station_id} marker={winner.marker_id} "
            f"normalized_area={winner.confidence:.6f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = StationDetectorNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError as error:
        # Humble can race signal shutdown while creating its next wait set.
        if rclpy.ok() or not any(
            message in str(error)
            for message in ("context is invalid", "context is not valid")
        ):
            raise
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
