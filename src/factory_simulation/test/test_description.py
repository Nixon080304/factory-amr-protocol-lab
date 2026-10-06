# SPDX-License-Identifier: Apache-2.0
"""Expanded robot assets must isolate frames and plugin endpoints."""

import importlib
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


@pytest.mark.parametrize(
    "namespace,prefix", [("/amr_01", "amr_01/"), ("/warehouse/cart", "floor/cart/")]
)
def test_xacro_prefixes_every_link_joint_and_sensor_frame(namespace, prefix):
    # Removing any xacro prefix creates colliding TF or joint identities.
    path = Path(__file__).resolve().parents[1] / "urdf/factory_amr.urdf.xacro"
    root = ET.fromstring(
        xacro.process_file(
            str(path), mappings={"namespace": namespace, "frame_prefix": prefix}
        ).toxml()
    )
    for node in root.findall("link") + root.findall("joint"):
        assert node.get("name").startswith(prefix), node.get("name")
    for node in root.findall(".//parent") + root.findall(".//child"):
        assert node.get("link").startswith(prefix)
    for plugin in root.findall(".//plugin"):
        assert plugin.findtext("ros/namespace") == namespace
        for tag in (
            "frame_name",
            "odometry_frame",
            "robot_base_frame",
            "left_joint",
            "right_joint",
            "joint_name",
        ):
            for frame in plugin.findall(tag):
                assert frame.text.startswith(prefix), frame.text
        for remap in plugin.findall("ros/remapping"):
            target = remap.text.split(":=", 1)[1]
            assert not target.startswith("/"), target


def test_production_description_helper_expands_installed_xacro():
    # Bypassing mappings in the production helper must break generated sensor frames.
    module = importlib.import_module("factory_simulation.description")
    root = ET.fromstring(module.robot_description("/amr_02", "amr_02/"))
    assert root.find("link[@name='amr_02/base_footprint']") is not None
    assert (
        root.findtext(".//plugin[@name='camera_ros']/frame_name")
        == "amr_02/camera_optical_frame"
    )
    assert root.findtext(".//plugin[@name='diff_drive']/ros/namespace") == "/amr_02"
