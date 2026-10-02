# Factory AMR Protocol Lab

A ROS 2 Humble simulation of a mobile robot carrying a motor part from assembly
to inspection. Nav2 and AMCL navigate on a committed map, a rendered RGB camera
confirms ArUco markers, MQTT accepts missions, and real Modbus TCP handshakes
control the simulated PLC. Gazebo animates conveyors and payload. The observer
writes a JSONL trace and a mission report.

## Setup and demo

Use Ubuntu 22.04, ROS 2 Humble, Python 3.10+, Gazebo Classic, Nav2, TurtleBot3,
OpenCV with ArUco, Docker Engine, and Docker Compose v2. Install `python3-venv`,
`python3-colcon-common-extensions`, and `python3-rosdep`. Initialize rosdep once
with `sudo rosdep init` if needed, run `rosdep update`, and allow your user to
access Docker. Set up the recorded project dependencies:

Ubuntu's `python3-opencv` 4.5.4 provides the required ArUco module. Detection and
marker generation support its API and newer OpenCV releases. The demo disables
Python user-site packages and uses the system ROS/OpenCV packages plus the owned
`.venv` dependencies; no user OpenCV wheel is required.

```bash
scripts/setup_dev.sh
```

Start the demo from the repository root with a working display:

```bash
DISPLAY=:0 scripts/run_demo.sh
```

The script verifies the environment, starts healthy Compose services, builds
incrementally with `--symlink-install`, and launches Gazebo, RViz, navigation,
perception, and all protocol adapters. Publish in another terminal using the
command printed by the script:

```bash
scripts/send_demo_mission.sh
```

Mission `M-001` loads at assembly marker 10 and unloads at inspection marker 20.
The complete reliable state stream is `RECEIVED`, `NAVIGATING_TO_PICKUP`,
`VERIFYING_PICKUP`, `LOADING`, `NAVIGATING_TO_DROPOFF`, `VERIFYING_DROPOFF`,
`UNLOADING`, `COMPLETED`. Each station requires five fresh detections within
1.5 seconds, current navigation success, and agreeing AMCL localization before
transfer. Payload states are `AT_ASSEMBLY`, `IN_TRANSIT`, `AT_INSPECTION`.
Sending the same mission again replays its final MQTT status without another
execution or PLC cycle. Humble can omit initial action feedback; reliable state
and protocol streams retain the complete order.

Press Ctrl+C to stop. Cleanup stops only the script's launch process group and
its newly created Compose project. A supplied preexisting project is reused
only when both services are healthy and is preserved on exit. Host ports bind
to localhost and default to MQTT 1883 and Modbus 1502. For concurrent demos,
set distinct `FACTORY_MQTT_PORT`, `FACTORY_PLC_PORT`, and
`FACTORY_COMPOSE_PROJECT`; use the same MQTT port with the mission publisher.
The script explicitly defaults to isolated ROS domain 80 regardless of the
caller's `ROS_DOMAIN_ID`. `FACTORY_ROS_DOMAIN_ID` permits an explicit integer
from 1 to 232. Concurrent robots must use distinct domains. Each run chooses a
local Gazebo master and a fresh trace directory, printed at startup.

Use `DISPLAY=:0 scripts/run_demo.sh --headless` to omit GUI windows. RGB
rendering still requires a working display. `FACTORY_OUTPUT_DIR` selects the
observer directory; give each observer its own directory. The trace is
`protocol_events.jsonl`; report names use the full SHA256 of the UTF-8 mission
ID. See [architecture](docs/architecture.md) and [protocol contracts](docs/protocols.md)
for components, RViz topics, schemas, registers, retries, and limitations.

## Verification

Run the same non-Gazebo gates locally and in GitHub Actions:

```bash
scripts/run_ci_checks.sh
```

Run `scripts/setup_dev.sh` once to install the pinned Ruff 0.15.7 and
clang-format 14.0.6 tools in `.venv`. Install Ubuntu 22.04's `cppcheck` package
(2.7), and provide a working Docker daemon. The runner sources ROS 2 Humble and
the project environment, requires `.venv` colcon 0.20.1 and the recorded runtime
pins, and explicitly sets `ROS_LOCALHOST_ONLY=1` and `PYTHONNOUSERSITE=1`.
Each stage prints RUN/PASS/FAIL. The first failed stage stops the runner and
preserves its exit code; the success line appears only after every stage passes.

The gates check Python formatting and lint, C++ formatting and correctness/
portability lint, a symlink build of all 10 ROS packages, package tests and fresh
colcon results, pure Python tests, ROS interface contracts, MQTT JSON schemas,
and real Mosquitto/Modbus integration fixtures. C++ performance suggestions are
advisory and are not part of the blocking lint categories. Colcon restricts
Python discovery to each package's `test` directory and excludes the rendered
camera and Gazebo payload tests; CTest excludes the simulation-topic and
navigation-goal launch tests. A unique result directory under `build/` retains
only this run's colcon results for inspection. Existing local result files are
preserved. The protocol fixtures own ephemeral loopback services and verify
cleanup. Their bounded navigation driver is not physical autonomy coverage.
The additional QoS/reset tests use actual DDS and ROS service endpoints, without
starting Gazebo.

The workflow uses Ubuntu 22.04, provisions ROS/platform dependencies, and then
calls `scripts/setup_dev.sh` and this exact runner. These provisioning steps are
CI-only setup; they do not add another test selection. GitHub success requires
an observed workflow run on the pushed commit and is not implied by local PASS.

Full Gazebo, physical success/autonomy tests, and the complete scenario matrix
remain **local-only**. With a working display, run:

```bash
set -e
source .venv/bin/activate
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY=1
colcon build --symlink-install
source install/setup.bash
DISPLAY=:0 colcon test --event-handlers console_direct+
colcon test-result --verbose
DISPLAY=:0 python3 -m pytest -q tests src/*/test
DISPLAY=:0 timeout 300s python3 -m pytest -q tests/system/test_successful_mission.py -s
git diff --check
```

The system test runs actual Gazebo motion, AMCL/Nav2, rendered camera detection,
an owned broker and PLC, and MQTT mission delivery. It checks exactly one
execution, both PLC counters, payload transitions, final independent world and
localization poses, duplicate suppression, and observer artifacts. Domain 80
is reserved for this test; package tests use 72–79. Services, ports, Gazebo
master, and output directory are isolated per run.

## Repeatable fault scenarios

Run one scenario or the complete 12-case matrix after setup and a successful
`colcon build --symlink-install`:

```bash
DISPLAY=:0 scripts/run_scenario.sh success
scripts/run_scenario.sh mqtt_disconnect
scripts/run_scenario.sh modbus_timeout
DISPLAY=:0 scripts/run_scenario.sh wrong_marker
DISPLAY=:0 scripts/run_scenario.sh nav_retry
DISPLAY=:0 scripts/run_all_scenarios.sh
```

Each case writes actual behavior, correlated trace assertions, and command logs
under `reports/<run-id>/<scenario>/`. Exit zero means both the expected final
state and trace assertions match. The default deadline is 300 seconds per case;
`FACTORY_SCENARIO_TIMEOUT` sets a positive number of seconds. Timeout returns
124, interruption returns 130 or 143, and a failed scenario command preserves
its exit code. Cleanup can take up to 65 additional seconds after a deadline.
`FACTORY_REPORT_ROOT` selects a report root. Matrix runs create `summary.json`
and `summary.md` and continue after unexpected outcomes.

Success, wrong marker, navigation retry, and navigation failure use real Gazebo.
Protocol fault cases use actual MQTT/ROS/Modbus peers and a bounded navigation
action driver. QoS mismatch is a separate DDS experiment. Every case starts
fresh resources and resets its controls. See [measured fault scenarios](docs/fault-scenarios.md)
for names, evidence, and transport lessons. Full Gazebo verification requires
a local rendering display; it is not claimed as CI coverage.

Vanished or hung transfer gateways can leave
a stopped robot with a pending mission and unknown PLC state. No bound on
every mission or crash recovery is claimed. Restart the simulation to reset
the single-part lifecycle. After successful pickup, another valid mission
returns `FAILED/RESTART_REQUIRED` without moving or transferring, even if the
first mission later fails or is canceled. Failures before successful pickup
permit a new mission; identical MQTT IDs still replay their recorded status.
The guard is process-local: restart the full demo rather than only the coordinator.

Licensed under Apache-2.0. See [LICENSE](LICENSE).
