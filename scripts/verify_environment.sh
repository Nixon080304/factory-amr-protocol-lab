#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

fail() {
    printf '%s\n' "$2" >&2
    exit "$1"
}

command -v ros2 >/dev/null || fail 10 'Missing ros2; install ROS 2 Humble and source /opt/ros/humble/setup.bash.'
[[ "${ROS_DISTRO:-}" == humble ]] || fail 11 'ROS_DISTRO must be humble; source /opt/ros/humble/setup.bash.'
ros2 --help >/dev/null || fail 10 'ros2 cannot start; check the sourced ROS environment.'
printf 'ROS 2 Humble: ready\n'

command -v python3 >/dev/null || fail 12 'Missing python3; install Python 3.10 or newer.'
python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' || fail 13 'Python 3.10 or newer is required.'
python3 --version

command -v docker >/dev/null || fail 14 'Missing docker; install Docker Engine and the Compose plugin.'
docker info >/dev/null 2>&1 || fail 15 'Docker daemon is unavailable; check daemon status and user socket permissions.'
docker compose version || fail 16 'Docker Compose plugin is unavailable; install Docker Compose v2.'
printf 'Docker daemon: ready\n'

command -v colcon >/dev/null || fail 17 'Missing colcon; install python3-colcon-common-extensions.'
colcon --help >/dev/null || fail 17 'colcon cannot start; check its Python dependencies.'
command -v rosdep >/dev/null || fail 18 'Missing rosdep; install python3-rosdep.'
rosdep --version >/dev/null || fail 18 'rosdep cannot start; check its Python dependencies.'

project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
python3 - "$project_root" <<'PY' || fail 19 'Project dependencies are unhealthy; activate .venv and run scripts/setup_dev.sh.'
from importlib import import_module, metadata
from pathlib import Path
import sys

from packaging.requirements import Requirement

root = Path(sys.argv[1])
for filename in ("requirements.txt", "requirements-dev.txt"):
    for line in (root / filename).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        requirement = Requirement(line)
        version = metadata.version(requirement.name)
        if version not in requirement.specifier:
            raise RuntimeError(f"{requirement.name}=={version} does not satisfy {requirement.specifier}")
for module in ("rclpy", "jsonschema", "paho.mqtt.client", "pymodbus", "pytest", "plc_simulator.server"):
    import_module(module)
print("Project Python dependencies and ROS imports: ready")
PY
printf 'Environment: ready\n'
