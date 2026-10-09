# SPDX-License-Identifier: Apache-2.0
"""Reject unsafe demo overrides before starting any service or robot."""

import os
from pathlib import Path
import subprocess
import signal
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("kind", ["relative", "missing", "unreadable", "empty"])
def test_demo_rejects_invalid_explicit_install_before_services(tmp_path, kind):
    setup = tmp_path / "setup.bash"
    setup.write_text("export DEMO_SELECTED_INSTALL=unexpected\n")
    value = str(setup)
    if kind == "relative":
        value = "install/setup.bash"
    elif kind == "missing":
        value = str(tmp_path / "missing.bash")
    elif kind == "unreadable":
        setup.chmod(0)
        assert not os.access(setup, os.R_OK)
    else:
        value = ""
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith("FACTORY_")
    }
    environment["FACTORY_INSTALL_SETUP"] = value
    result = subprocess.run(
        [str(ROOT / "scripts/run_demo.sh"), "--invalid-option"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 2
    assert "FACTORY_INSTALL_SETUP requires an absolute readable setup file" in (
        result.stderr
    )
    assert "Usage:" not in result.stderr


@pytest.mark.parametrize("explicit", [False, True])
def test_demo_sources_exact_install_and_builds_only_default(tmp_path, explicit):
    # Execute the production install-selection block without starting Docker or
    # robots. Only the external build command is intercepted; source is real.
    script = (ROOT / "scripts/run_demo.sh").read_text()
    block = (
        "# Incremental colcon builds changed inputs."
        + script.split("# Incremental colcon builds changed inputs.", 1)[1].split(
            '\nmkdir -p "$project_root/artifacts"', 1
        )[0]
    )
    default = tmp_path / "install/setup.bash"
    default.parent.mkdir()
    default.write_text("export DEMO_SELECTED_INSTALL=default\n")
    fresh = tmp_path / "fresh install/setup.bash"
    fresh.parent.mkdir()
    fresh.write_text("export DEMO_SELECTED_INSTALL=fresh\n")
    environment = {
        key: value
        for key, value in os.environ.items()
        if key not in ("FACTORY_INSTALL_SETUP", "DEMO_SELECTED_INSTALL")
    }
    if explicit:
        environment["FACTORY_INSTALL_SETUP"] = str(fresh)
    result = subprocess.run(
        [
            "bash",
            "-c",
            'set -eu; colcon() { printf "BUILD:%s\\n" "$*"; }; '
            + block
            + '\nprintf "INSTALL:%s\\n" "$DEMO_SELECTED_INSTALL"',
        ],
        cwd=tmp_path,
        env=environment,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == (
        ["INSTALL:fresh"]
        if explicit
        else ["BUILD:build --symlink-install --base-paths src", "INSTALL:default"]
    )


@pytest.mark.parametrize(
    "name,value",
    [
        ("FACTORY_DASHBOARD_PORT", "0"),
        ("FACTORY_DASHBOARD_PORT", "65536"),
        ("FACTORY_DASHBOARD_PORT", "wrong"),
        ("FACTORY_JOURNAL_PATH", "relative.sqlite3"),
        ("FACTORY_JOURNAL_PATH", ""),
    ],
)
def test_demo_rejects_invalid_owned_observer_options(name, value):
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith("FACTORY_")
    }
    environment[name] = value
    result = subprocess.run(
        [str(ROOT / "scripts/run_demo.sh"), "--invalid-option"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 2
    assert name in result.stderr
    assert "Usage:" not in result.stderr


@pytest.mark.parametrize("explicit", [False, True])
def test_demo_forwards_exact_owned_observer_options(tmp_path, explicit):
    script = (ROOT / "scripts/run_demo.sh").read_text()
    # Execute the real launch argument boundary without external ROS processes.
    block = script.split("setsid ros2 launch factory_bringup demo.launch.py", 1)
    assert len(block) == 2
    prefix = block[0].split("# Demo launch arguments\n")
    assert len(prefix) == 2, "owned observer launch argument selection is missing"
    launch = (
        prefix[1]
        + "setsid ros2 launch factory_bringup demo.launch.py"
        + block[1].split("\nlaunch_pid=$!", 1)[0]
        + "\nwait"
    )
    environment = {
        key: value
        for key, value in os.environ.items()
        if key not in ("FACTORY_DASHBOARD_PORT", "FACTORY_JOURNAL_PATH")
    }
    journal = str(tmp_path / "fresh journal.sqlite3")
    if explicit:
        environment.update(FACTORY_DASHBOARD_PORT="31701", FACTORY_JOURNAL_PATH=journal)
    result = subprocess.run(
        [
            "bash",
            "-c",
            "set -eu; gui=false; rviz=false; FACTORY_MQTT_PORT=1883; "
            "FACTORY_PLC_PORT=1502; output_dir=/tmp/evidence; "
            'setsid() { printf "%s\\n" "$@"; }; ' + launch,
        ],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        "ros2",
        "launch",
        "factory_bringup",
        "demo.launch.py",
        "gui:=false",
        "rviz:=false",
        "broker_port:=1883",
        "plc_port:=1502",
        "output_dir:=/tmp/evidence",
        *(["dashboard_port:=31701", f"journal_path:={journal}"] if explicit else []),
    ]


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
