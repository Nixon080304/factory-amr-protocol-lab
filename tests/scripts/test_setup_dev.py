# SPDX-License-Identifier: Apache-2.0
"""Verify setup keeps full local dependencies and allows a narrow CI skip."""

from pathlib import Path
import os
import shutil
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]


def run_setup(tmp_path: Path, skip_keys: str | None = None):
    project = tmp_path / "project"
    (project / "scripts").mkdir(parents=True)
    shutil.copy2(ROOT / "scripts/setup_dev.sh", project / "scripts/setup_dev.sh")
    (project / "src/plc_simulator").mkdir(parents=True)
    (project / "requirements-dev.txt").write_text("")
    (project / "requirements.txt").write_text("")

    log = tmp_path / "commands.log"
    tools = tmp_path / "tools"
    tools.mkdir()
    fake = f"""#!/bin/bash
printf '%s\\n' \"$0 $*\" >> {log}
if [[ $(basename \"$0\") == python3 && $1 == -m && $2 == venv ]]; then
    mkdir -p \"$4/bin\"
    cp \"$0\" \"$4/bin/python\"
fi
"""
    for name in ("python3", "rosdep"):
        path = tools / name
        path.write_text(fake)
        path.chmod(0o755)

    environment = {**os.environ, "PATH": f"{tools}:{os.environ['PATH']}"}
    environment.pop("FACTORY_AMR_ROSDEP_SKIP_KEYS", None)
    if skip_keys is not None:
        environment["FACTORY_AMR_ROSDEP_SKIP_KEYS"] = skip_keys
    result = subprocess.run(
        ["bash", str(project / "scripts/setup_dev.sh")],
        cwd=project,
        env=environment,
        capture_output=True,
        text=True,
        timeout=20,
    )
    return result, log.read_text()


def test_setup_installs_all_rosdeps_by_default(tmp_path):
    result, commands = run_setup(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "rosdep install --from-paths src --ignore-src -r -y\n" in commands
    assert "--skip-keys" not in commands


def test_setup_accepts_explicit_ci_rosdep_skip_keys(tmp_path):
    result, commands = run_setup(tmp_path, "nav2_bringup")
    assert result.returncode == 0, result.stdout + result.stderr
    assert (
        "rosdep install --from-paths src --ignore-src -r -y --skip-keys nav2_bringup\n"
    ) in commands


def test_hosted_ci_declares_only_nav2_bringup_skip():
    workflow = yaml.safe_load((ROOT / ".github/workflows/ci.yml").read_text())
    quality_job = workflow["jobs"]["quality"]
    assert quality_job["env"]["FACTORY_AMR_ROSDEP_SKIP_KEYS"] == "nav2_bringup"

    setup_step = next(
        step
        for step in quality_job["steps"]
        if step.get("name") == "Set up project dependencies"
    )
    assert "FACTORY_AMR_ROSDEP_SKIP_KEYS" not in setup_step.get("env", {})
