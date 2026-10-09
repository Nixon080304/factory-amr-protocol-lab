# SPDX-License-Identifier: Apache-2.0
"""Public fleet wrappers must not replace an explicitly selected fresh install."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ("run_fleet_acceptance.sh", "run_fleet_demo.sh")


@pytest.fixture
def wrapper(tmp_path):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    for name in SCRIPTS:
        shutil.copy2(ROOT / "scripts" / name, scripts / name)
    tools = tmp_path / ".venv/bin"
    tools.mkdir(parents=True)
    (tools / "activate").write_text(f'export PATH="{tools}:$PATH"\n')
    executable = tools / "python3"
    executable.write_text(
        "#!/usr/bin/python3\nimport json,os,sys\n"
        "print(json.dumps(dict(marker=os.environ.get('SELECTED_FLEET_INSTALL'),"
        "setup=os.environ.get('FACTORY_INSTALL_SETUP'),args=sys.argv[1:])))\n"
    )
    executable.chmod(0o755)

    def run(name, setup=None):
        environment = {
            key: value
            for key, value in os.environ.items()
            if key not in ("FACTORY_INSTALL_SETUP", "SELECTED_FLEET_INSTALL")
        }
        if setup is not None:
            environment["FACTORY_INSTALL_SETUP"] = setup
        return subprocess.run(
            [str(scripts / name), "--headless"],
            cwd=tmp_path,
            env=environment,
            capture_output=True,
            text=True,
            timeout=10,
        )

    return run


@pytest.mark.parametrize("name", SCRIPTS)
@pytest.mark.parametrize("default_exists", [False, True])
def test_nested_fleet_driver_uses_fresh_setup_not_missing_or_stale_default(
    wrapper, tmp_path, name, default_exists
):
    if default_exists:
        default = tmp_path / "install/setup.bash"
        default.parent.mkdir()
        default.write_text("export SELECTED_FLEET_INSTALL=stale\n")
    fresh = tmp_path / "fresh install/setup.bash"
    fresh.parent.mkdir()
    fresh.write_text("export SELECTED_FLEET_INSTALL=fresh\n")
    result = wrapper(name, str(fresh))
    assert result.returncode == 0, result.stdout + result.stderr
    observed = json.loads(result.stdout)
    assert observed["marker"] == "fresh"
    assert observed["setup"] == str(fresh)
    assert observed["args"][:2] == [
        "tests/system/fleet_driver.py",
        "acceptance" if name == SCRIPTS[0] else "demo",
    ]
    assert "--headless" in observed["args"]


@pytest.mark.parametrize("name", SCRIPTS)
@pytest.mark.parametrize("kind", ["relative", "missing", "unreadable", "empty"])
def test_fleet_driver_rejects_invalid_setup_before_driver_launch(
    wrapper, tmp_path, name, kind
):
    setup = tmp_path / "setup.bash"
    setup.write_text("export SELECTED_FLEET_INSTALL=unexpected\n")
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
    result = wrapper(name, value)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "FACTORY_INSTALL_SETUP requires an absolute readable setup file" in (
        result.stderr
    )
    assert result.stdout == ""


@pytest.mark.parametrize("name", SCRIPTS)
def test_fleet_driver_retains_default_install_when_override_unset(
    wrapper, tmp_path, name
):
    default = tmp_path / "install/setup.bash"
    default.parent.mkdir()
    default.write_text("export SELECTED_FLEET_INSTALL=default\n")
    result = wrapper(name)
    assert result.returncode == 0, result.stdout + result.stderr
    assert json.loads(result.stdout)["marker"] == "default"
