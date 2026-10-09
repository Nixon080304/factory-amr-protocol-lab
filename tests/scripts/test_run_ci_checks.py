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
    "Fleet bounded system",
    "Fleet dashboard browser",
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
        usersite=os.environ.get('PYTHONNOUSERSITE'),
        overlays={key: os.environ.get(key, '') for key in (
            'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
            'PYTHONPATH', 'LD_LIBRARY_PATH')})) + '\\n')
if len(previous) == int(os.environ.get('CI_FAIL_INDEX', '-1')):
    sys.exit(int(os.environ.get('CI_FAIL_CODE', '37')))
if name == 'colcon' and 'build' in sys.argv and '--install-base' in sys.argv:
    install = Path(sys.argv[sys.argv.index('--install-base') + 1])
    install.mkdir(parents=True)
    (install / 'setup.bash').write_text('')
"""
    for name in ("ruff", "clang-format", "cppcheck", "colcon", "python3", "node"):
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
            **{
                key: str(tmp_path / "historical-install")
                for key in (
                    "AMENT_PREFIX_PATH",
                    "CMAKE_PREFIX_PATH",
                    "COLCON_PREFIX_PATH",
                    "PYTHONPATH",
                    "LD_LIBRARY_PATH",
                )
            },
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


@pytest.mark.parametrize("index", range(len(STAGES)))
@pytest.mark.parametrize("code", [1, 37, 124])
def test_failed_stage_stops_later_stages_and_preserves_exit_code(runner, index, code):
    result, calls = runner(index, code)
    assert result.returncode == code, result.stdout + result.stderr
    assert len(calls) == index + 1
    assert f"[FAIL] {STAGES[index]} (exit {code})" in result.stderr
    assert "All non-Gazebo checks PASS" not in result.stdout
    for later in STAGES[index + 1 :]:
        assert f"[RUN] {later}" not in result.stdout


def test_pure_stage_runs_public_document_acceptance_checks(runner):
    result, calls = runner()
    assert result.returncode == 0
    pure = calls[7]
    assert pure["command"] == "python3"
    assert pure["args"][:3] == ["-m", "pytest", "-q"]
    assert "tests/docs" in pure["args"]


def test_each_gate_uses_fresh_artifact_roots_and_preserves_historical_results(
    runner, tmp_path
):
    historical = tmp_path / "build/ci-results.historical/package/pytest.xml"
    historical.parent.mkdir(parents=True)
    evidence = (
        '<testsuite tests="1" failures="1"><testcase><failure/></testcase></testsuite>'
    )
    historical.write_text(evidence)
    roots = []
    for _ in range(2):
        result, all_calls = runner()
        assert result.returncode == 0, result.stdout + result.stderr
        calls = all_calls[-len(STAGES) :]
        tests, results = calls[5]["args"], calls[6]["args"]
        test_root = Path(tests[tests.index("--test-result-base") + 1])
        assert test_root.name == "test-results"
        assert test_root.is_relative_to(tmp_path / "artifacts/ci")
        assert results[results.index("--test-result-base") + 1] == str(test_root)
        run_root = test_root.parent
        roots.append(run_root)
        build = calls[4]["args"]
        assert build[build.index("--build-base") + 1] == str(run_root / "build")
        assert build[build.index("--install-base") + 1] == str(run_root / "install")
        # Ament CMake resolves its XML destination at build/configure time.
        # A test-only override relocates CTest metadata, not package JUnit XML.
        assert build[build.index("--test-result-base") + 1] == str(test_root)
        assert tests[tests.index("--build-base") + 1] == str(run_root / "build")
        assert tests[tests.index("--install-base") + 1] == str(run_root / "install")
        for call in calls[4:7]:
            args = call["args"]
            assert args[args.index("--log-base") + 1] == str(run_root / "log")
        assert historical.read_text() == evidence
    assert roots[0] != roots[1]


def test_gate_does_not_load_inherited_workspace_overlays(runner, tmp_path):
    result, calls = runner()
    assert result.returncode == 0, result.stdout + result.stderr
    inherited = str(tmp_path / "historical-install")
    assert all(
        inherited not in value for call in calls for value in call["overlays"].values()
    )


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
        "node",
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
    assert (
        "test_simulation_topics|test_navigation_goal|test_fleet_simulation"
        in calls[5]["args"]
    )
    build, tests, results = (call["args"] for call in calls[4:7])
    assert build[2:4] == ["build", "--symlink-install"]
    assert results[2:5] == ["test-result", "--verbose", "--test-result-base"]
    assert tests[2:4] == ["test", "--return-code-on-test-failure"]
    assert "--executor" in tests, "DDS package tests must not share parallel domains"
    assert tests[tests.index("--executor") + 1] == "sequential"
    assert tests[tests.index("--test-result-base") + 1] == results[5]
    assert "tests/system/test_protocol_faults.py" in calls[-1]["args"]
    assert "tests/system/test_qos_behavior.py" in calls[-1]["args"]
    assert "tests/system/test_two_robot_fleet.py" in calls[9]["args"]
    assert "tests/system/test_ten_robot_scale.py" in calls[9]["args"]
    assert "tests/system/test_legacy_readiness.py" in calls[9]["args"]
    assert calls[10]["args"] == ["tests/browser/test_fleet_dashboard.mjs"]
    assert not any(
        argument.endswith(("test_successful_mission.py", "test_autonomy_faults.py"))
        for call in calls
        for argument in call["args"]
    )
