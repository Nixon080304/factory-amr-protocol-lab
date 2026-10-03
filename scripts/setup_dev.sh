#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
export PYTHONNOUSERSITE=1

project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
cd "$project_root"
[[ -f /opt/ros/humble/setup.bash ]] || {
    printf 'Install ROS 2 Humble before running setup_dev.sh.\n' >&2
    exit 1
}
python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'
python3 -m venv --system-site-packages .venv
# Install the recorded build tools before the standalone editable package.
.venv/bin/python -m pip install -r requirements-dev.txt
# System site packages can satisfy this requirement without creating a local
# entry point. Reinstall only colcon-core inside the venv to create that script.
.venv/bin/python -m pip install --force-reinstall --no-deps colcon-core==0.20.1
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip install --no-build-isolation -e src/plc_simulator
# ROS setup scripts can reference unset variables.
set +u
source /opt/ros/humble/setup.bash
set -u
rosdep_args=(install --from-paths src --ignore-src -r -y)
if [[ -n "${FACTORY_AMR_ROSDEP_SKIP_KEYS:-}" ]]; then
    read -r -a rosdep_skip_keys <<< "$FACTORY_AMR_ROSDEP_SKIP_KEYS"
    rosdep_args+=(--skip-keys "${rosdep_skip_keys[@]}")
fi
rosdep "${rosdep_args[@]}"
printf 'Setup complete. Source .venv/bin/activate and /opt/ros/humble/setup.bash.\n'
