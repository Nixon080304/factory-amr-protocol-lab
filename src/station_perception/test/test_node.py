# SPDX-License-Identifier: Apache-2.0
"""Exercise the real detector constructor across a DDS-free transport boundary."""

from pathlib import Path
from types import SimpleNamespace
import sys

from cv_bridge import CvBridge
import cv2
import numpy as np
import pytest
from rclpy.expand_topic_name import expand_topic_name
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from station_perception.node import StationDetectorNode


def test_namespaced_camera_and_detection_publish_real_marker(monkeypatch):
    # Only ROS middleware creation is replaced. Detection, image conversion,
    # message construction, source headers, and endpoint arguments stay real.
    endpoints, publications = [], []
    namespace = "/amr_01"
    monkeypatch.setattr(Node, "__init__", lambda self, *args, **kwargs: None)
    monkeypatch.setattr(Node, "has_parameter", lambda self, name: False)
    monkeypatch.setattr(
        Node,
        "declare_parameter",
        lambda self, name, value: SimpleNamespace(value=value),
    )
    monkeypatch.setattr(
        Node, "get_logger", lambda self: SimpleNamespace(info=lambda text: None)
    )

    def publisher(self, kind, topic, qos):
        resolved = expand_topic_name(topic, "station_detector", namespace)
        endpoints.append(resolved)
        return SimpleNamespace(publish=lambda message: publications.append(message))

    def subscription(self, kind, topic, callback, qos):
        endpoints.append(expand_topic_name(topic, "station_detector", namespace))
        return SimpleNamespace()

    monkeypatch.setattr(Node, "create_publisher", publisher)
    monkeypatch.setattr(Node, "create_subscription", subscription)
    generate_marker = (
        cv2.aruco.generateImageMarker
        if hasattr(cv2.aruco, "generateImageMarker")
        else cv2.aruco.drawMarker
    )
    pixels = np.full((300, 300, 3), 255, dtype=np.uint8)
    marker = generate_marker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 10, 180
    )
    pixels[60:240, 60:240] = marker[:, :, None]
    for robot_id in ("amr_01", "amr_02"):
        namespace = f"/{robot_id}"
        node = StationDetectorNode()
        image = CvBridge().cv2_to_imgmsg(pixels, encoding="bgr8")
        image.header.frame_id = f"{robot_id}/camera_optical_frame"
        image.header.stamp.sec = 42
        node._on_image(image)
    assert endpoints == [
        "/amr_01/factory/station_detection",
        "/amr_01/camera/image_raw",
        "/amr_02/factory/station_detection",
        "/amr_02/camera/image_raw",
    ]
    assert [(message.station_id, message.marker_id) for message in publications] == [
        ("assembly", 10),
        ("assembly", 10),
    ]
    assert [message.header.frame_id for message in publications] == [
        "amr_01/camera_optical_frame",
        "amr_02/camera_optical_frame",
    ]
    assert [message.header.stamp.sec for message in publications] == [42, 42]


@pytest.mark.parametrize("namespace", ["/amr_01", "/amr_02"])
def test_real_dds_detector_endpoints(namespace):
    import rclpy
    from rclpy.context import Context

    context = Context()
    rclpy.init(context=context, domain_id=74)
    node = None
    try:
        node = StationDetectorNode(context=context, namespace=namespace)
        assert node._publisher.topic_name == f"{namespace}/factory/station_detection"
        assert node._subscription.topic_name == f"{namespace}/camera/image_raw"
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
