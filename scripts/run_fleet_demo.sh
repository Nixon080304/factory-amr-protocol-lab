#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
headless=false
output=''
for argument in "$@"; do
    case "$argument" in
        --headless) [[ $headless == false ]] || { echo 'Duplicate --headless' >&2; exit 2; }; headless=true ;;
        --*) echo 'Usage: scripts/run_fleet_demo.sh [--headless] [output-directory]' >&2; exit 2 ;;
        *) [[ -z $output ]] || { echo 'Only one output-directory is allowed' >&2; exit 2; }; output=$argument ;;
    esac
done
cd "$project_root"
export PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY=1
set +u
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
set -u
arguments=(demo --timeout 900)
[[ $headless == false ]] || arguments+=(--headless)
[[ -z $output ]] || arguments+=(--output "$output")
exec python3 tests/system/fleet_driver.py "${arguments[@]}"
