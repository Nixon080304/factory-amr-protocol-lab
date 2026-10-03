# SPDX-License-Identifier: Apache-2.0
"""Protect the package metadata that makes colcon discover pytest tests."""

import os
from pathlib import Path
import subprocess
import sys

from colcon_core.task.python.test.pytest import PytestPythonTestingStep
from colcon_python_setup_py.package_identification.python_setup_py import (
    get_setup_information,
)
import pytest


ROOT = Path(__file__).resolve().parents[2]


def rosdep_check_command():
    command = ["rosdep", "check", "--from-paths", "src", "--ignore-src"]
    skip_keys = os.environ.get("FACTORY_AMR_ROSDEP_SKIP_KEYS")
    if skip_keys:
        command.extend(["--skip-keys", skip_keys])
    return command


@pytest.mark.parametrize(
    "package", ["mqtt_gateway", "modbus_gateway", "protocol_observer"]
)
def test_colcon_selects_pytest_for_ros_python_package(package):
    # Execute real setup metadata, then ask the installed runner to select it.
    metadata = get_setup_information(ROOT / "src" / package / "setup.py")
    assert PytestPythonTestingStep().match(None, os.environ, metadata)


def test_colcon_launcher_uses_the_project_python(tmp_path):
    # Observe the real entry point's interpreter without changing its behavior.
    (tmp_path / "sitecustomize.py").write_text(
        "import sys\nprint('COLCON_PREFIX:' + sys.prefix)\n"
    )
    environment = dict(os.environ)
    environment["PYTHONPATH"] = (
        str(tmp_path) + os.pathsep + environment.get("PYTHONPATH", "")
    )
    result = subprocess.run(
        ["colcon", "--help"],
        env=environment,
        capture_output=True,
        text=True,
        check=True,
        timeout=20,
    )
    assert result.stdout.splitlines()[0] == "COLCON_PREFIX:" + sys.prefix


def test_ros_package_dependencies_resolve_on_target():
    result = subprocess.run(
        rosdep_check_command(),
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr + result.stdout


def test_rosdep_check_uses_same_explicit_ci_skip(monkeypatch):
    monkeypatch.setenv("FACTORY_AMR_ROSDEP_SKIP_KEYS", "nav2_bringup")
    assert rosdep_check_command() == [
        "rosdep",
        "check",
        "--from-paths",
        "src",
        "--ignore-src",
        "--skip-keys",
        "nav2_bringup",
    ]
