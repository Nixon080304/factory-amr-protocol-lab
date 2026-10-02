#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
export PYTHONNOUSERSITE=1
project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
cd "$project_root"
[[ -f .venv/bin/activate ]] || { printf 'Run scripts/setup_dev.sh first.\n' >&2; exit 1; }
set +u
source .venv/bin/activate
source /opt/ros/humble/setup.bash
set -u

# Do not inherit the caller's robot domain. Explicit overrides must be isolated.
export ROS_DOMAIN_ID=${FACTORY_ROS_DOMAIN_ID:-80}
export ROS_LOCALHOST_ONLY=1
export FACTORY_MQTT_PORT=${FACTORY_MQTT_PORT:-1883}
export FACTORY_PLC_PORT=${FACTORY_PLC_PORT:-1502}
export FACTORY_COMPOSE_PROJECT=${FACTORY_COMPOSE_PROJECT:-factory-demo-$$}
python3 - <<'PY'
import os
import re
domain = os.environ['ROS_DOMAIN_ID']
if not domain.isascii() or not domain.isdigit() or not 1 <= int(domain) <= 232:
    raise SystemExit('FACTORY_ROS_DOMAIN_ID must be an isolated integer from 1 to 232')
for name in ('FACTORY_MQTT_PORT', 'FACTORY_PLC_PORT'):
    value = os.environ[name]
    if not value.isascii() or not value.isdigit() or not 1 <= int(value) <= 65535:
        raise SystemExit(f'{name} must be an integer from 1 to 65535')
if os.environ['FACTORY_MQTT_PORT'] == os.environ['FACTORY_PLC_PORT']:
    raise SystemExit('MQTT and PLC ports must differ')
if not re.fullmatch('[a-z0-9][a-z0-9_-]*', os.environ['FACTORY_COMPOSE_PROJECT']):
    raise SystemExit('FACTORY_COMPOSE_PROJECT must be a lowercase Compose project name')
PY
if [[ -z ${GAZEBO_MASTER_URI:-} ]]; then
    GAZEBO_MASTER_URI=$(python3 - <<'PY'
import socket
with socket.socket() as connection:
    connection.bind(('127.0.0.1', 0))
    print(f'http://127.0.0.1:{connection.getsockname()[1]}')
PY
)
    export GAZEBO_MASTER_URI
fi
python3 - <<'PY'
import os
import socket
from urllib.parse import urlparse
uri = urlparse(os.environ['GAZEBO_MASTER_URI'])
try:
    port = uri.port
except ValueError:
    raise SystemExit('GAZEBO_MASTER_URI must use a valid local TCP port')
if uri.scheme != 'http' or uri.hostname != '127.0.0.1' or port is None or not 1 <= port <= 65535 or uri.path not in ('', '/'):
    raise SystemExit('GAZEBO_MASTER_URI must be http://127.0.0.1:<port>')
with socket.socket() as connection:
    try:
        connection.bind(('127.0.0.1', port))
    except OSError:
        raise SystemExit('GAZEBO_MASTER_URI port is already in use; choose a fresh local master')
PY
gui=true
rviz=true
case "${1:-}" in
    '') ;;
    --headless) gui=false; rviz=false ;;
    *) printf 'Usage: scripts/run_demo.sh [--headless]\n' >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { printf 'Unexpected arguments.\n' >&2; exit 2; }
[[ -n ${DISPLAY:-} ]] || { printf 'Set DISPLAY to a working display; RGB rendering needs it even headless.\n' >&2; exit 1; }
scripts/verify_environment.sh
compose=(docker compose -f docker/compose.yaml -p "$FACTORY_COMPOSE_PROJECT")
owns_project=false
launch_pid=''
readiness_pid=''
cleanup() {
    trap - EXIT INT TERM
    for owned_pid in "$readiness_pid" "$launch_pid"; do
      if [[ -n $owned_pid ]] && kill -0 "$owned_pid" 2>/dev/null; then
        kill -INT -- "-$owned_pid" 2>/dev/null || true
        for ((attempt=0; attempt<100; attempt++)); do
            kill -0 "$owned_pid" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$owned_pid" 2>/dev/null; then
            kill -TERM -- "-$owned_pid" 2>/dev/null || true
            for ((attempt=0; attempt<50; attempt++)); do
                kill -0 "$owned_pid" 2>/dev/null || break
                sleep 0.1
            done
        fi
        if kill -0 "$owned_pid" 2>/dev/null; then
            kill -KILL -- "-$owned_pid" 2>/dev/null || true
        fi
        wait "$owned_pid" 2>/dev/null || true
      fi
    done
    if [[ $owns_project == true ]]; then
        "${compose[@]}" down --timeout 5 || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
existing=$("${compose[@]}" ps -aq)
if [[ -z $existing ]]; then
    owns_project=true
    "${compose[@]}" up -d --build --wait --wait-timeout 60
else
    # Reused projects are read-only. Never restart, remove, or reconfigure a
    # preexisting service. Refuse partial/unhealthy projects rather than mutate.
    "${compose[@]}" ps --format json | python3 -c '
import json, os, sys
rows = [json.loads(line) for line in sys.stdin if line.strip()]
if len(rows) != 2 or {row["Service"] for row in rows} != {"mosquitto", "plc-simulator"} or any(
        row.get("State") != "running" or row.get("Health") != "healthy" for row in rows):
    raise SystemExit("Reused Compose project must have both services running and healthy")
for row in rows:
    target, variable = (1883, "FACTORY_MQTT_PORT") if row["Service"] == "mosquitto" else (1502, "FACTORY_PLC_PORT")
    if not any(port.get("URL") == "127.0.0.1" and port.get("TargetPort") == target and
               port.get("PublishedPort") == int(os.environ[variable]) for port in row.get("Publishers", [])):
        raise SystemExit("Reused Compose ports must match the requested localhost ports")'
fi
# Incremental colcon builds changed inputs. Fail before sourcing stale installs.
colcon build --symlink-install
set +u
source install/setup.bash
set -u
mkdir -p "$project_root/artifacts"
output_dir=${FACTORY_OUTPUT_DIR:-$(mktemp -d "$project_root/artifacts/demo-XXXXXXXX")}
mkdir -p "$output_dir"
printf 'ROS domain: %s; Compose project: %s; trace directory: %s\n' "$ROS_DOMAIN_ID" "$FACTORY_COMPOSE_PROJECT" "$output_dir"
setsid ros2 launch factory_bringup demo.launch.py "gui:=$gui" "rviz:=$rviz" \
    "broker_port:=$FACTORY_MQTT_PORT" "plc_port:=$FACTORY_PLC_PORT" "output_dir:=$output_dir" &
launch_pid=$!
setsid ros2 run factory_bringup factory_wait_ready &
readiness_pid=$!
wait "$readiness_pid"
readiness_pid=''
printf 'Publish standard mission in another terminal: FACTORY_MQTT_PORT=%s scripts/send_demo_mission.sh\n' "$FACTORY_MQTT_PORT"
wait "$launch_pid"
