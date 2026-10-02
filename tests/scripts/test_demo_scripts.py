# SPDX-License-Identifier: Apache-2.0
"""Reject unsafe demo overrides before starting any service or robot."""

import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("name,value,error", [
    ("FACTORY_ROS_DOMAIN_ID", "0", "FACTORY_ROS_DOMAIN_ID"),
    ("FACTORY_ROS_DOMAIN_ID", "233", "FACTORY_ROS_DOMAIN_ID"),
    ("FACTORY_ROS_DOMAIN_ID", "eight", "FACTORY_ROS_DOMAIN_ID"),
    ("FACTORY_MQTT_PORT", "65536", "FACTORY_MQTT_PORT"),
    ("FACTORY_PLC_PORT", "-1", "FACTORY_PLC_PORT"),
    ("FACTORY_COMPOSE_PROJECT", "Wrong/Project", "FACTORY_COMPOSE_PROJECT"),
])
def test_demo_rejects_invalid_overrides(name, value, error):
    environment = {key: value for key, value in os.environ.items() if not key.startswith("FACTORY_")}
    environment[name] = value
    result = subprocess.run([str(ROOT / "scripts/run_demo.sh"), "--headless"], cwd=ROOT,
                            env=environment, capture_output=True, text=True, timeout=15)
    assert result.returncode != 0
    assert error in result.stderr


def test_demo_does_not_inherit_unrelated_robot_domain():
    environment = {key: value for key, value in os.environ.items() if not key.startswith("FACTORY_")}
    environment["ROS_DOMAIN_ID"] = "0"
    result = subprocess.run([str(ROOT / "scripts/run_demo.sh"), "--invalid-option"], cwd=ROOT,
                            env=environment, capture_output=True, text=True, timeout=15)
    # The valid explicit default passes validation, even when the inherited
    # domain is invalid for this script. Argument handling occurs before Docker.
    assert result.returncode == 2
    assert "Usage:" in result.stderr
    assert "FACTORY_ROS_DOMAIN_ID" not in result.stderr


def test_mission_publisher_rejects_invalid_port():
    result = subprocess.run([str(ROOT / "scripts/send_demo_mission.sh")], cwd=ROOT,
                            env={**os.environ, "FACTORY_MQTT_PORT": "0"},
                            capture_output=True, text=True, timeout=15)
    assert result.returncode != 0
    assert "FACTORY_MQTT_PORT" in result.stderr
