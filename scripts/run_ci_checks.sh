#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
export PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY=1

project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
cd "$project_root"
# ROS and colcon overlays can reference unset variables.
set +u
source /opt/ros/humble/setup.bash
source .venv/bin/activate
set -u
python3 -c 'from importlib.metadata import version; from pathlib import Path; import shutil; from packaging.version import Version; assert version("colcon-core") == "0.20.1"; assert version("jsonschema") == "4.26.0"; assert version("paho-mqtt") == "2.1.0"; assert version("pymodbus") == "3.15.0"; assert Version(version("setuptools")) < Version("80"); assert Path(shutil.which("colcon")).absolute() == Path(".venv/bin/colcon").absolute()'

run_stage() {
    local label=$1 status
    shift
    printf '[RUN] %s\n' "$label"
    if "$@"; then
        printf '[PASS] %s\n' "$label"
    else
        status=$?
        printf '[FAIL] %s (exit %s)\n' "$label" "$status" >&2
        exit "$status"
    fi
}

mapfile -d '' cpp_files < <(find src/mission_coordinator -type f \( -name '*.cpp' -o -name '*.hpp' \) -print0)
python_entrypoints=(src/factory_bringup/scripts/factory_visualization src/factory_bringup/scripts/factory_wait_ready)
run_stage 'Python formatting' ruff format --check src tests "${python_entrypoints[@]}"
run_stage 'Python lint' ruff check src tests "${python_entrypoints[@]}"
run_stage 'C++ formatting' clang-format --dry-run --Werror "${cpp_files[@]}"
run_stage 'C++ lint' cppcheck --enable=warning,portability --error-exitcode=1 \
    --std=c++17 --inline-suppr '-DTEST(suite,name)=void suite##_##name()' \
    '-DTEST_F(suite,name)=void suite##_##name()' -Isrc/mission_coordinator/include \
    src/mission_coordinator/src src/mission_coordinator/test
run_stage 'ROS build' colcon build --symlink-install --event-handlers console_direct+
set +u
source install/setup.bash
set -u
# A unique result base excludes retained local Gazebo XML from this gate.
# Keep results for inspection; no existing build/test artifacts are deleted.
result_base=$(mktemp -d "$project_root/build/ci-results.XXXXXX")
printf 'Fresh non-Gazebo colcon results: %s\n' "$result_base"
run_stage 'ROS tests' colcon test --return-code-on-test-failure \
    --test-result-base "$result_base" --event-handlers console_direct+ \
    --ctest-args ' -E' 'test_simulation_topics|test_navigation_goal' \
    --pytest-args test " --ignore=$project_root/src/station_perception/test/test_camera_integration.py" \
    " --ignore=$project_root/src/payload_simulator/test/test_gazebo_payload.py"
run_stage 'ROS test results' colcon test-result --verbose --test-result-base "$result_base"
run_stage 'Pure tests' python3 -m pytest -q tests/scripts tests/scenarios src/plc_simulator/test
run_stage 'Contracts and schemas' python3 -m pytest -q tests/contracts src/mqtt_gateway/test/test_validator.py
# Existing reviewed fixtures own ephemeral loopback Mosquitto/Modbus peers and
# verify their cleanup. The protocol driver replaces navigation, not services.
run_stage 'Real protocol integration' python3 -m pytest -q -s tests/integration \
    tests/system/test_protocol_faults.py tests/system/test_qos_behavior.py
printf 'All non-Gazebo checks PASS\n'
