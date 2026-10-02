# SPDX-License-Identifier: Apache-2.0
"""Catch invalid robot trees, duplicate odometry, and intersecting spawn geometry."""

from itertools import combinations
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


PACKAGE = Path(__file__).resolve().parents[1]


def robot():
    path = PACKAGE / "urdf/factory_amr.urdf.xacro"
    assert path.is_file(), "sensor-equipped AMR asset is missing"
    return ET.fromstring(xacro.process_file(str(path)).toxml())


def world():
    path = PACKAGE / "worlds/factory_floor.world"
    assert path.is_file(), "factory world asset is missing"
    return ET.parse(path).getroot().find("world")


def test_robot_has_connected_unique_links_and_joints():
    root = robot()
    links = [node.get("name") for node in root.findall("link")]
    joints = root.findall("joint")
    assert len(links) == len(set(links))
    assert len(joints) == len({node.get("name") for node in joints})
    assert {node.find("child").get("link") for node in joints} == set(links) - {"base_footprint"}
    assert len(joints) == len(links) - 1
    reached = {"base_footprint"}
    for _ in joints:
        reached.update(node.find("child").get("link") for node in joints
                       if node.find("parent").get("link") in reached)
    assert reached == set(links)
    assert {"base_link", "base_scan", "imu_link", "camera_optical_frame"} <= reached


def test_robot_has_exactly_one_of_each_sensor_and_one_odometry_source():
    root = robot()
    assert sorted(node.get("type") for node in root.findall(".//sensor")) == ["camera", "imu", "ray"]
    plugins = root.findall(".//plugin")
    drive = [node for node in plugins if node.get("filename") == "libgazebo_ros_diff_drive.so"]
    assert len(drive) == 1
    assert [node for node in plugins if node.findtext("publish_odom") == "true"] == drive
    assert drive[0].findtext("publish_odom_tf") == "true"
    assert drive[0].findtext("publish_wheel_tf") == "false"
    assert drive[0].findtext("odometry_frame") == "odom"
    assert drive[0].findtext("robot_base_frame") == "base_footprint"
    assert drive[0].findtext("odometry_source") == "0", "wheel encoder odometry must not use Gazebo ground truth"
    assert float(drive[0].findtext("wheel_separation")) == pytest.approx(0.160)
    assert float(drive[0].findtext("wheel_diameter")) == pytest.approx(0.066)
    assert sum(node.get("filename") == "libgazebo_ros_joint_state_publisher.so" for node in plugins) == 1
    for node in root.findall(".//sensor"):
        assert node.findtext("always_on") == "true"
        assert float(node.findtext("update_rate")) > 0
    camera = root.find(".//sensor[@type='camera']")
    assert camera.findtext("camera/image/format") == "R8G8B8"
    assert camera.findtext("plugin/frame_name") == "camera_optical_frame"
    origin = root.find("joint[@name='camera_joint']/origin")
    assert float(origin.get("xyz").split()[2]) > 0.25


def test_sensor_publisher_configuration_uses_sensor_data_qos():
    # DDS discovery reports reliability/durability, but Humble does not expose
    # remote history/depth. Verify those configured policies in generated URDF.
    root = robot()
    publishers = {}
    for topic in root.findall(".//sensor/plugin/ros/qos/topic"):
        publishers[topic.get("name")] = topic.find("publisher")
    assert set(publishers) == {"camera/image_raw", "camera/camera_info", "scan"}
    for topic, publisher in publishers.items():
        assert publisher.findtext("reliability") == "best_effort", topic
        assert publisher.findtext("durability") == "volatile", topic
        assert publisher.findtext("history") == "keep_last", topic
        assert int(publisher.find("history").get("depth")) == 5, topic


def models_with_poses():
    root = world()
    result = []
    for include in root.findall("include"):
        uri = include.findtext("uri")
        assert uri.startswith("model://")
        name = uri.removeprefix("model://")
        directory = PACKAGE / "models" / name
        config = ET.parse(directory / "model.config").getroot()
        model = ET.parse(directory / config.findtext("sdf")).getroot().find("model")
        result.append((include.findtext("name"), model, tuple(map(float, include.findtext("pose").split()))))
    for model in root.findall("model"):
        result.append((model.get("name"), model, tuple(map(float, model.findtext("pose", "0 0 0 0 0 0").split()))))
    return result


def test_factory_contains_stations_part_shelving_and_dynamic_obstacle():
    models = {name: model for name, model, _ in models_with_poses()}
    assert {"assembly_station", "inspection_station", "factory_part", "shelf_north", "shelf_south", "dynamic_obstacle"} <= set(models)
    assert models["dynamic_obstacle"].findtext("static") == "false"
    assert models["factory_part"].findtext("static") == "false"
    for name in ("assembly_station", "inspection_station"):
        assert models[name].findtext("static") == "true"
        assert models[name].find("link/visual[@name='marker_plane']") is not None
        assert models[name].find("link/collision[@name='conveyor']") is not None


def test_initial_collision_boxes_do_not_overlap_and_spawn_is_clear():
    boxes = []
    for name, model, pose in models_with_poses():
        assert pose[3:] == (0.0, 0.0, 0.0), "geometry check requires axis-aligned world models"
        for link in model.findall("link"):
            link_pose = tuple(map(float, link.findtext("pose", "0 0 0 0 0 0").split()))
            for collision in link.findall("collision"):
                size = collision.findtext("geometry/box/size")
                if size is None:
                    continue
                offset = tuple(map(float, collision.findtext("pose", "0 0 0 0 0 0").split()))
                xyz = tuple(pose[i] + link_pose[i] + offset[i] for i in range(3))
                bounds = tuple((xyz[i] - d / 2, xyz[i] + d / 2) for i, d in enumerate(map(float, size.split())))
                boxes.append((name, bounds))
    boxes.append(("factory_amr", ((-0.15, 0.15), (-3.15, -2.85), (0.0, 0.4))))
    for (left_name, left), (right_name, right) in combinations(boxes, 2):
        if left_name == right_name:
            continue
        overlap = all(min(a[1], b[1]) - max(a[0], b[0]) > 1e-6 for a, b in zip(left, right))
        assert not overlap, f"initial collisions overlap: {left_name}, {right_name}"
