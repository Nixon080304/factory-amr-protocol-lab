# Factory AMR Protocol Lab

A ROS 2 lab for factory material transfer, with MQTT at the dispatcher boundary
and Modbus TCP at the station boundary.

## Milestone status

The protocol core provides shared ROS interfaces, a C++ mission state machine,
MQTT validation/deduplication/reconnect buffering, a standalone PLC simulator,
Modbus transfer handshakes, and JSONL traces with mission reports. The core is
tested without Gazebo. ROS node adapters, navigation, perception, and the full
simulated mission workflow follow in later milestones.

## Development setup

Target Ubuntu 22.04, ROS 2 Humble, and Python 3.10+. Install ROS 2 Humble,
`python3-venv`, `python3-colcon-common-extensions`, `python3-rosdep`, Docker
Engine, and Docker Compose v2. Initialize rosdep once with `sudo rosdep init`
if needed, then run `rosdep update`. Your user must be able to access Docker.

The setup script creates `.venv` with access to system ROS Python packages,
installs the recorded dependencies and editable standalone PLC package, then
runs rosdep. It also installs a venv-local colcon launcher so Python package
tests use the pinned dependencies. Its pip commands always use `.venv/bin/python`.

```bash
scripts/setup_dev.sh
source .venv/bin/activate
source /opt/ros/humble/setup.bash
scripts/verify_environment.sh
colcon build --symlink-install
source install/setup.bash
ros2 interface show factory_interfaces/action/ExecuteFactoryMission
ros2 interface show factory_interfaces/srv/TransferPart
```

Environment verification checks Humble, Python 3.10+, Docker daemon access,
Compose, colcon, rosdep, recorded dependency versions, and ROS/PLC imports.
Its distinct nonzero exit codes identify missing tools or unhealthy dependencies.
It checks this project's requirements rather than unrelated inherited system
packages.

## Local services

```bash
docker compose -f docker/compose.yaml config --quiet
docker compose -f docker/compose.yaml up -d --build --wait --wait-timeout 60
docker compose -f docker/compose.yaml ps
# After finishing, stop only this Compose project.
docker compose -f docker/compose.yaml down
```

Mosquitto publishes host `127.0.0.1:1883`; the PLC publishes `127.0.0.1:1502`.
Startup waits for a broker publish acknowledgment and real PLC reads on both
station unit IDs. See [protocol contracts](docs/protocols.md) for topics,
payloads, registers, retry bounds, errors, and observer events.

## Protocol-core tests

With `.venv` activated and ROS Humble sourced:

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 -m pytest -q tests src/mqtt_gateway/test src/plc_simulator/test src/modbus_gateway/test src/protocol_observer/test
git diff --check
```

The integration suite starts and stops its own local PyModbus servers. Compose
startup above separately verifies the repeatable broker and PLC deployment.

Licensed under Apache-2.0. See [LICENSE](LICENSE).
