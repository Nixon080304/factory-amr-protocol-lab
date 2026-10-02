# SPDX-License-Identifier: Apache-2.0
"""Exercise environment checks against controlled executable boundaries."""
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "verify_environment.sh"


@pytest.fixture
def commands(tmp_path):
    def write(name, body):
        path = tmp_path / name
        path.write_text("#!/bin/bash\n" + body + "\n")
        path.chmod(0o755)

    write("ros2", '[[ "$1" == "--help" ]]')
    write("colcon", '[[ "$1" == "--help" ]]')
    write("rosdep", '[[ "$1" == "--version" ]]')
    write("docker", '[[ "$*" == "info" || "$*" == "compose version" ]]')
    # Execute at the original venv path so Python finds its pyvenv.cfg.
    write("python3", f'exec {sys.executable} "$@"')
    return tmp_path, write


def run_script(path, **environment):
    env = {**os.environ, "PATH": str(path), "ROS_DISTRO": "humble", **environment}
    return subprocess.run(
        ["/bin/bash", str(SCRIPT)], env=env, capture_output=True, text=True,
        timeout=20,
    )


@pytest.mark.parametrize("command,code", [
    ("ros2", 10), ("python3", 12), ("docker", 14),
    ("colcon", 17), ("rosdep", 18),
])
def test_missing_commands_have_distinct_exit_codes(commands, command, code):
    path, _ = commands
    (path / command).unlink()
    result = run_script(path)
    assert result.returncode == code, result.stderr


@pytest.mark.parametrize("distro", ["", "jazzy"])
def test_requires_sourced_humble(commands, distro):
    path, _ = commands
    assert run_script(path, ROS_DISTRO=distro).returncode == 11


@pytest.mark.parametrize("body,code", [
    ('[[ "$*" == "compose version" ]]', 15),
    ('[[ "$*" == "info" ]]', 16),
])
def test_daemon_and_compose_failures_are_distinct(commands, body, code):
    path, write = commands
    write("docker", body)
    assert run_script(path).returncode == code


def test_rejects_python_older_than_310(commands):
    path, write = commands
    (path / "python3").unlink()
    write("python3", '[[ "$1" == "--version" ]]')
    assert run_script(path).returncode == 13


def test_dependency_failure_is_not_reported_as_ready(commands):
    path, write = commands
    (path / "python3").unlink()
    write("python3", f'[[ "$1" != "-" ]] || exit 1\nexec {sys.executable} "$@"')
    assert run_script(path).returncode == 19


def test_complete_environment_is_ready(commands):
    path, _ = commands
    result = run_script(path)
    assert result.returncode == 0, result.stderr + result.stdout


def test_real_target_environment_is_ready():
    result = subprocess.run(
        ["/bin/bash", str(SCRIPT)], capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stderr + result.stdout
