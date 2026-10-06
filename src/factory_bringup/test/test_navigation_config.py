# SPDX-License-Identifier: Apache-2.0
"""Catch map/world drift, unsafe stations, and conflicting Nav2 interfaces."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def load(relative):
    path = PACKAGE / relative
    assert path.is_file(), f"required navigation asset missing: {path}"
    return yaml.safe_load(path.read_text())


def map_data():
    metadata = load("maps/factory_map.yaml")
    path = PACKAGE / "maps" / metadata["image"]
    assert path.is_file(), "committed map image missing"
    with path.open("rb") as stream:
        assert stream.readline().strip() == b"P5"
        width, height = map(int, stream.readline().split())
        assert stream.readline().strip() == b"255"
        pixels = stream.read()
    assert len(pixels) == width * height
    assert (width, height) == (248, 168)
    assert metadata["resolution"] == 0.05
    assert metadata["origin"] == [-6.2, -4.2, 0.0]
    return metadata, width, height, pixels


def pixel_at(data, x, y):
    metadata, width, height, pixels = data
    column = math.floor((x - metadata["origin"][0]) / metadata["resolution"])
    row = math.floor((y - metadata["origin"][1]) / metadata["resolution"])
    assert 0 <= column < width and 0 <= row < height
    return pixels[(height - 1 - row) * width + column]


def test_map_tracks_static_world_geometry():
    data = map_data()
    world = ET.parse(PACKAGE.parent / "factory_simulation/worlds/factory_floor.world")
    # Every floor-level static collision must appear in the occupancy raster.
    models = world.findall(".//world/model")
    for include in world.findall(".//world/include"):
        name = include.findtext("uri").removeprefix("model://")
        model = (
            ET.parse(PACKAGE.parent / f"factory_simulation/models/{name}/model.sdf")
            .getroot()
            .find("model")
        )
        ET.SubElement(model, "pose").text = include.findtext("pose")
        models.append(model)
    for model in models:
        if model.findtext("static") != "true" or model.attrib["name"] == "floor":
            continue
        pose = [float(value) for value in model.findtext("pose", "0 0 0 0 0 0").split()]
        for collision in model.findall("link/collision"):
            size = collision.findtext("geometry/box/size")
            if size is None:
                continue
            local = [
                float(value)
                for value in collision.findtext("pose", "0 0 0 0 0 0").split()
            ]
            sx, sy, sz = map(float, size.split())
            if pose[2] + local[2] - sz / 2 > 0.18:
                continue
            assert pixel_at(data, pose[0] + local[0], pose[1] + local[1]) == 0, (
                model.attrib["name"]
            )
    for x, y in [(0, -3), (-2, 0), (2, 0), (-3, 0.8), (3, 0.8), (4, -2)]:
        assert pixel_at(data, x, y) == 254


def test_stations_are_free_and_face_marker_planes():
    data = map_data()
    stations = load("config/stations.yaml")
    assert stations["frame_id"] == "map"
    for name, x, marker_id in [("assembly", -3.0, 10), ("inspection", 3.0, 20)]:
        pose = stations["stations"][name]
        assert pose["marker_id"] == marker_id
        assert pose["x"] == x and pose["y"] == 0.8
        assert (
            abs(
                math.atan2(
                    math.sin(pose["yaw"] - math.pi / 2),
                    math.cos(pose["yaw"] - math.pi / 2),
                )
            )
            < 1e-9
        )
        for dx, dy in [(0, 0), (0.22, 0), (-0.22, 0), (0, 0.22), (0, -0.22)]:
            assert pixel_at(data, pose["x"] + dx, pose["y"] + dy) == 254


def test_nav2_uses_simulation_frames_and_one_velocity_smoother():
    params = load("config/nav2_params.yaml")

    def check_time(value):
        for key, child in value.items():
            if key == "ros__parameters":
                assert child["use_sim_time"] is True
            elif isinstance(child, dict):
                check_time(child)

    check_time(params)
    amcl = params["amcl"]["ros__parameters"]
    assert (amcl["global_frame_id"], amcl["odom_frame_id"], amcl["base_frame_id"]) == (
        "map",
        "odom",
        "base_footprint",
    )
    assert amcl["initial_pose"] == {"x": 0.0, "y": -3.0, "z": 0.0, "yaw": 0.0}
    assert (
        params["bt_navigator"]["ros__parameters"]["robot_base_frame"]
        == "base_footprint"
    )
    for name, frame in [("local_costmap", "odom"), ("global_costmap", "map")]:
        costmap = params[name][name]["ros__parameters"]
        assert (costmap["global_frame"], costmap["robot_base_frame"]) == (
            frame,
            "base_footprint",
        )
        assert costmap["obstacle_layer"]["scan"]["topic"] == "scan"
        assert costmap["obstacle_layer"]["scan"]["marking"] is True
        assert costmap["obstacle_layer"]["scan"]["clearing"] is True
    assert (
        params["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
        == "nav2_navfn_planner/NavfnPlanner"
    )
    assert (
        params["controller_server"]["ros__parameters"]["FollowPath"]["plugin"]
        == "dwb_core::DWBLocalPlanner"
    )
    assert "velocity_smoother" in params
