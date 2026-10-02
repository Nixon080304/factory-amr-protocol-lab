# SPDX-License-Identifier: Apache-2.0
"""Reject unsafe demo overrides before starting any service or robot."""

import os
from pathlib import Path
import subprocess
import signal
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    "name,value,error",
    [
        ("FACTORY_ROS_DOMAIN_ID", "0", "FACTORY_ROS_DOMAIN_ID"),
        ("FACTORY_ROS_DOMAIN_ID", "233", "FACTORY_ROS_DOMAIN_ID"),
        ("FACTORY_ROS_DOMAIN_ID", "eight", "FACTORY_ROS_DOMAIN_ID"),
        ("FACTORY_MQTT_PORT", "65536", "FACTORY_MQTT_PORT"),
        ("FACTORY_PLC_PORT", "-1", "FACTORY_PLC_PORT"),
        ("FACTORY_COMPOSE_PROJECT", "Wrong/Project", "FACTORY_COMPOSE_PROJECT"),
    ],
)
def test_demo_rejects_invalid_overrides(name, value, error):
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith("FACTORY_")
    }
    environment[name] = value
    result = subprocess.run(
        [str(ROOT / "scripts/run_demo.sh"), "--headless"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode != 0
    assert error in result.stderr


def test_demo_does_not_inherit_unrelated_robot_domain():
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith("FACTORY_")
    }
    environment["ROS_DOMAIN_ID"] = "0"
    result = subprocess.run(
        [str(ROOT / "scripts/run_demo.sh"), "--invalid-option"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=15,
    )
    # The valid explicit default passes validation, even when the inherited
    # domain is invalid for this script. Argument handling occurs before Docker.
    assert result.returncode == 2
    assert "Usage:" in result.stderr
    assert "FACTORY_ROS_DOMAIN_ID" not in result.stderr


def test_mission_publisher_rejects_invalid_port():
    result = subprocess.run(
        [str(ROOT / "scripts/send_demo_mission.sh")],
        cwd=ROOT,
        env={**os.environ, "FACTORY_MQTT_PORT": "0"},
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode != 0
    assert "FACTORY_MQTT_PORT" in result.stderr


@pytest.mark.parametrize("leader_exits", [False, True])
def test_cleanup_escalates_owned_group_even_after_leader_exits(leader_exits):
    script = (ROOT / "scripts/run_demo.sh").read_text()
    cleanup = (
        "cleanup() {"
        + script.split("cleanup() {", 1)[1].split("\ntrap cleanup EXIT", 1)[0]
    )
    leader = subprocess.Popen(
        [
            "bash",
            "-c",
            "trap '' INT TERM; sleep 300 & echo $!; "
            + ("exit" if leader_exits else "wait"),
        ],
        stdout=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    child = int(leader.stdout.readline())
    try:
        if leader_exits:
            leader.wait(timeout=3)
        assert (
            subprocess.run(
                ["bash", "-c", f"kill -0 -- -{leader.pid}"], capture_output=True
            ).returncode
            == 0
        )
        started = time.monotonic()
        result = subprocess.run(
            [
                "bash",
                "-c",
                f"launch_pid={leader.pid}; readiness_pid=''; owns_project=false; "
                + cleanup
                + "\ncleanup",
            ],
            capture_output=True,
            text=True,
            timeout=22,
        )
        assert result.returncode == 0, result.stderr
        assert time.monotonic() - started < 20
        status = Path(f"/proc/{child}/status")

        def child_running():
            try:
                return "State:\tZ" not in status.read_text()
            except FileNotFoundError:
                return False

        stopped_deadline = time.monotonic() + 1
        while child_running() and time.monotonic() < stopped_deadline:
            time.sleep(0.01)
        assert not child_running(), "owned descendant still running"
    finally:
        try:
            os.killpg(leader.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        leader.wait(timeout=3)
        leader.stdout.close()
