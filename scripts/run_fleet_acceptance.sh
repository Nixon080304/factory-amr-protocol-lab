#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
headless=false
scenario=all
deadline=900
seen_scenario=false
seen_timeout=false
while (($#)); do
    case "$1" in
        --headless) [[ $headless == false ]] || { echo 'Duplicate --headless' >&2; exit 2; }; headless=true; shift ;;
        --scenario)
            [[ $seen_scenario == false && $# -ge 2 ]] || { echo 'Missing or duplicate --scenario' >&2; exit 2; }
            case "$2" in all|nominal|contention|robot_failure|charging|scale) ;; *) echo 'Unknown fleet scenario' >&2; exit 2 ;; esac
            scenario=$2; seen_scenario=true; shift 2 ;;
        --timeout)
            [[ $seen_timeout == false && $# -ge 2 && $2 =~ ^[1-9][0-9]*$ && ${#2} -le 6 ]] || { echo '--timeout requires a positive integer <= 999999' >&2; exit 2; }
            deadline=$2; seen_timeout=true; shift 2 ;;
        *) echo 'Usage: scripts/run_fleet_acceptance.sh [--headless] [--scenario all|nominal|contention|robot_failure|charging|scale] [--timeout seconds]' >&2; exit 2 ;;
    esac
done
cd "$project_root"
export PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY=1
set +u
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
set -u
arguments=(acceptance --scenario "$scenario" --timeout "$deadline")
[[ $headless == false ]] || arguments+=(--headless)
exec python3 tests/system/fleet_driver.py "${arguments[@]}"
