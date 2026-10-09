# SPDX-License-Identifier: Apache-2.0
"""Ten real DDS agent contracts with bounded external navigation drivers."""

import os
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def test_public_scale_timeout_is_nonzero_and_cleans_owned_group():
    result = subprocess.run(
        [
            str(ROOT / "scripts/run_fleet_acceptance.sh"),
            "--headless",
            "--scenario",
            "scale",
            "--timeout",
            "1",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=25,
    )
    assert result.returncode == 124, result.stdout + result.stderr
    artifact = Path(
        next(
            line.removeprefix("Fleet artifacts: ")
            for line in result.stdout.splitlines()
            if line.startswith("Fleet artifacts: ")
        )
    )
    summary = json.loads((artifact / "summary.json").read_text())
    assert summary["success"] is False
    cleanup = json.loads((artifact / "scale/cleanup.json").read_text())
    assert cleanup and all(item["group_clear"] for item in cleanup)


def test_concurrent_public_scale_drivers_isolate_domains_and_artifacts():
    commands = [
        subprocess.Popen(
            [
                str(ROOT / "scripts/run_fleet_acceptance.sh"),
                "--headless",
                "--scenario",
                "scale",
                "--timeout",
                "60",
            ],
            cwd=ROOT,
            env={**os.environ, "ROS_DOMAIN_ID": "9999"},
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        for _ in range(2)
    ]
    paths, domains = [], []
    try:
        for command in commands:
            output, _ = command.communicate(timeout=75)
            assert command.returncode == 0, output
            artifact = Path(
                next(
                    line.removeprefix("Fleet artifacts: ")
                    for line in output.splitlines()
                    if line.startswith("Fleet artifacts: ")
                )
            )
            paths.append(artifact)
            summary = json.loads((artifact / "summary.json").read_text())
            assert summary["success"] is True
            receipt = json.loads(
                (artifact / "scale/ten-agent-receipt.json").read_text()
            )
            domains.append(receipt["ros_domain_id"])
            cleanup = json.loads((artifact / "scale/cleanup.json").read_text())
            assert cleanup and all(item["group_clear"] for item in cleanup)
    finally:
        for command in commands:
            if command.poll() is None:
                command.terminate()
                command.wait(timeout=25)
    assert len(set(paths)) == len(set(domains)) == 2
    assert 9999 not in domains


def test_ten_synthetic_agents_register_heartbeat_cost_schedule_and_complete(tmp_path):
    probe = ROOT / "tests/system/fleet_scale_probe.py"
    assert probe.is_file(), "ten-agent DDS proof driver is missing"
    result = subprocess.run(
        [str(ROOT / ".venv/bin/python3"), str(probe), str(tmp_path)],
        cwd=ROOT,
        env={**os.environ, "ROS_LOCALHOST_ONLY": "1"},
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    import json

    receipt = json.loads((tmp_path / "ten-agent-receipt.json").read_text())
    assert (
        receipt["registered"]
        == receipt["costed"]
        == receipt["assigned"]
        == receipt["completed"]
        == 10
    )
    assert receipt["assignment_order"] == [f"synthetic_{i:02}" for i in range(10)]
    assert receipt["production_config_unchanged"] is True
    assert receipt["production_code_unchanged"] is True
    assert set(receipt["cost_requests"]) == {f"synthetic_{i:02}" for i in range(10)}
    assert all(count > 0 for count in receipt["cost_requests"].values())
    # DDS scale fixtures must still clear every initial route and committed-map
    # footprint through the production validator; this is not ten-body Nav2 proof.
    from fleet_manager.config import load_fleet_config
    from PIL import Image
    import yaml

    config = load_fleet_config(tmp_path / "ten-agent-fleet.yaml")
    assert len(config.robots) == 10
    maps = ROOT / "src/factory_bringup/maps"
    metadata = yaml.safe_load((maps / "factory_map.yaml").read_text())
    raster = Image.open(maps / metadata["image"])
    resolution = metadata["resolution"]
    origin_x, origin_y, _ = metadata["origin"]
    for index, robot in enumerate(config.robots):
        assert all(
            math.hypot(robot.spawn.x - other.spawn.x, robot.spawn.y - other.spawn.y)
            >= 0.60
            for other in config.robots[:index]
        )
        for dx in range(-6, 7):
            for dy in range(-6, 7):
                if math.hypot(dx * resolution, dy * resolution) > 0.30:
                    continue
                column = math.floor((robot.spawn.x - origin_x) / resolution) + dx
                row = (
                    raster.height
                    - 1
                    - math.floor((robot.spawn.y - origin_y) / resolution)
                    - dy
                )
                assert 0 <= column < raster.width and 0 <= row < raster.height
                assert raster.getpixel((column, row)) >= 254
