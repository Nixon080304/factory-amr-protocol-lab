# SPDX-License-Identifier: Apache-2.0
"""Exercise hosted resource provisioning without changing installed dependencies."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def provision(tmp_path):
    from ament_index_python.packages import get_package_share_directory

    installed = Path(get_package_share_directory("nav2_bringup"))
    content = tmp_path / "official-resource-copy"
    shutil.copytree(installed, content / "share/nav2_bringup")
    marker = Path("share/ament_index/resource_index/packages/nav2_bringup")
    (content / marker).parent.mkdir(parents=True)
    shutil.copy2(installed.parent.parent / marker, content / marker)
    tools = tmp_path / "tools"
    tools.mkdir()
    log, environment_file = tmp_path / "commands.jsonl", tmp_path / "github-env"
    environment_file.touch()
    external = """#!/usr/bin/python3
import json
import os
from pathlib import Path
import shutil
import sys
name = Path(sys.argv[0]).name
with open(os.environ['TEST_NAV2_LOG'], 'a') as stream:
    stream.write(json.dumps([name, *sys.argv[1:]]) + '\\n')
if name == 'apt-get':
    status = int(os.environ.get('TEST_NAV2_APT_STATUS', '0'))
    if status:
        sys.exit(status)
    Path('ros-humble-nav2-bringup_contract.deb').write_bytes(b'contract-only')
elif '--extract' in sys.argv:
    shutil.copytree(os.environ['TEST_NAV2_CONTENT'], Path(sys.argv[-1]) / 'opt/ros/humble')
elif sys.argv[-1] == 'Package':
    print(os.environ.get('TEST_NAV2_PACKAGE', 'ros-humble-nav2-bringup'))
else:
    print('Contract boundary: ros-humble-nav2-bringup')
"""
    for name in ("apt-get", "dpkg-deb"):
        executable = tools / name
        executable.write_text(external)
        executable.chmod(0o755)

    def run(**extra):
        result = subprocess.run(
            ["bash", str(ROOT / "scripts/provision_hosted_nav2_resources.sh")],
            env={
                **os.environ,
                "PATH": f"{tools}:{os.environ['PATH']}",
                "RUNNER_TEMP": str(tmp_path),
                "GITHUB_ENV": str(environment_file),
                "TEST_NAV2_LOG": str(log),
                "TEST_NAV2_CONTENT": str(content),
                **extra,
            },
            capture_output=True,
            text=True,
            timeout=10,
        )
        calls = (
            [json.loads(line) for line in log.read_text().splitlines()]
            if log.exists()
            else []
        )
        return result, calls, environment_file.read_text()

    return run, installed


def test_provision_downloads_only_nav2_and_exports_owned_resource_prefix(
    provision, tmp_path
):
    run, installed = provision
    result, calls, exported = run()
    assert result.returncode == 0, result.stdout + result.stderr
    assert calls[0] == ["apt-get", "download", "ros-humble-nav2-bringup"]
    prefix = Path(exported.strip().removeprefix("FACTORY_AMR_NAV2_RESOURCE_PREFIX="))
    assert prefix.is_relative_to(tmp_path)
    assert (
        prefix / "share/nav2_bringup/launch/localization_launch.py"
    ).read_bytes() == (installed / "launch/localization_launch.py").read_bytes()
    assert not (prefix / "lib").exists() and not (prefix / "bin").exists()
    assert not any(
        call[0] in ("sudo", "rosdep") or "--install" in call for call in calls
    )


@pytest.mark.parametrize(
    "extra,status",
    [({"TEST_NAV2_APT_STATUS": "37"}, 37), ({"TEST_NAV2_PACKAGE": "other"}, 2)],
)
def test_provision_preserves_download_and_package_validation_failures(
    provision, extra, status
):
    run, _ = provision
    result, calls, exported = run(**extra)
    assert result.returncode == status
    assert exported == ""
    assert not any("--extract" in call for call in calls)
