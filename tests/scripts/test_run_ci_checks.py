# SPDX-License-Identifier: Apache-2.0
"""Exercise the real gate runner with isolated, observable tool boundaries."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
STAGES = [
    "Python formatting",
    "Python lint",
    "C++ formatting",
    "C++ lint",
    "ROS build",
    "ROS tests",
    "ROS test results",
    "Pure tests",
    "Contracts and schemas",
    "Real protocol integration",
]


@pytest.fixture
def runner(tmp_path):
    script = ROOT / "scripts/run_ci_checks.sh"
    (tmp_path / "scripts").mkdir()
    if script.is_file():
        shutil.copy2(script, tmp_path / "scripts/run_ci_checks.sh")
    tools = tmp_path / ".venv/bin"
    tools.mkdir(parents=True)
    (tools / "activate").write_text(f'export PATH="{tools}:$PATH"\n')
    source = tmp_path / "src/mission_coordinator/src"
    source.mkdir(parents=True)
    (source / "main.cpp").write_text("int main() { return 0; }\n")
    (tmp_path / "build").mkdir()
    (tmp_path / "install").mkdir()
    (tmp_path / "install/setup.bash").write_text("")
    log = tmp_path / "commands.jsonl"
    fake = """#!/usr/bin/python3
import json
import os
from pathlib import Path
import sys

name = Path(sys.argv[0]).name
if name == 'python3' and sys.argv[1] == '-c':
    sys.exit(0)
log = Path(os.environ['CI_FAKE_LOG'])
previous = log.read_text().splitlines() if log.exists() else []
with log.open('a') as output:
    output.write(json.dumps(dict(command=name, args=sys.argv[1:],
        localhost=os.environ.get('ROS_LOCALHOST_ONLY'),
        usersite=os.environ.get('PYTHONNOUSERSITE'))) + '\\n')
if len(previous) == int(os.environ.get('CI_FAIL_INDEX', '-1')):
    sys.exit(int(os.environ.get('CI_FAIL_CODE', '37')))
"""
    for name in ("ruff", "clang-format", "cppcheck", "colcon", "python3"):
        executable = tools / name
        executable.write_text(fake)
        executable.chmod(0o755)

    def run(fail_index=-1, code=37):
        environment = {
            **os.environ,
            "CI_FAKE_LOG": str(log),
            "CI_FAIL_INDEX": str(fail_index),
            "CI_FAIL_CODE": str(code),
            "ROS_LOCALHOST_ONLY": "0",
            "PYTHONNOUSERSITE": "0",
        }
        result = subprocess.run(
            ["bash", str(tmp_path / "scripts/run_ci_checks.sh")],
            cwd=tmp_path,
            env=environment,
            capture_output=True,
            text=True,
            timeout=20,
        )
        calls = (
            [json.loads(line) for line in log.read_text().splitlines()]
            if log.exists()
            else []
        )
        return result, calls

    return run


@pytest.mark.parametrize("index", range(10))
@pytest.mark.parametrize("code", [1, 37, 124])
def test_failed_stage_stops_later_stages_and_preserves_exit_code(runner, index, code):
    result, calls = runner(index, code)
    assert result.returncode == code, result.stdout + result.stderr
    assert len(calls) == index + 1
    assert f"[FAIL] {STAGES[index]} (exit {code})" in result.stderr
    assert "All non-Gazebo checks PASS" not in result.stdout
    for later in STAGES[index + 1 :]:
        assert f"[RUN] {later}" not in result.stdout


def test_success_requires_every_stage_and_forces_ros_and_python_isolation(runner):
    result, calls = runner()
    assert result.returncode == 0, result.stdout + result.stderr
    assert [call["command"] for call in calls] == [
        "ruff",
        "ruff",
        "clang-format",
        "cppcheck",
        "colcon",
        "colcon",
        "colcon",
        "python3",
        "python3",
        "python3",
    ]
    for stage in STAGES:
        assert f"[RUN] {stage}" in result.stdout
        assert f"[PASS] {stage}" in result.stdout
    assert result.stdout.rstrip().endswith("All non-Gazebo checks PASS")
    assert all(call["localhost"] == call["usersite"] == "1" for call in calls)
    entrypoints = [
        "src/factory_bringup/scripts/factory_visualization",
        "src/factory_bringup/scripts/factory_wait_ready",
    ]
    assert calls[0]["args"] == ["format", "--check", "src", "tests", *entrypoints]
    assert calls[1]["args"] == ["check", "src", "tests", *entrypoints]
    assert "test" in calls[5]["args"]
    assert "test_simulation_topics|test_navigation_goal" in calls[5]["args"]
    assert calls[4]["args"][:2] == ["build", "--symlink-install"]
    assert calls[6]["args"][:3] == ["test-result", "--verbose", "--test-result-base"]
    assert calls[5]["args"][1] == "--return-code-on-test-failure"
    assert calls[5]["args"][3] == calls[6]["args"][3]
    assert "tests/system/test_protocol_faults.py" in calls[-1]["args"]
    assert "tests/system/test_qos_behavior.py" in calls[-1]["args"]
    assert not any(
        argument.endswith(("test_successful_mission.py", "test_autonomy_faults.py"))
        for call in calls
        for argument in call["args"]
    )
